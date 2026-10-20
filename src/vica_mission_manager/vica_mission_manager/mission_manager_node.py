#!/usr/bin/env python3
"""vica_mission_manager — VICA 음성→주행 통합의 유일한 신규 노드 (진행순서 ②)."""
from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Optional
from uuid import UUID

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from nav2_msgs.msg import BehaviorTreeLog, SpeedLimit
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.exceptions import InvalidHandle
from rclpy.executors import (ExternalShutdownException, MultiThreadedExecutor, ShutdownException,
                             SingleThreadedExecutor)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import Log
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from vica_interfaces.msg import EmergencyEvent, RobotState, VicaIntent
from vica_interfaces.msg import PersonDetection, SmartHandleState
from vica_interfaces.srv import (
    DeleteHome,
    GetHome,
    MissionCommand,
    RequestApproach,
    RequestDestination,
    SaveHome,
)

from .approach_speed import DEFAULT_APPROACH_STAGES, stages_from_lists
from .destinations import load_destinations, load_home, load_map_bounds
from .approach_geometry import approach_goal
from .home_storage import HomeStorage, build_home
from .ledger import (Ledger, LedgerStore, apply_goal_event, confirm_abort_name, place_here,
                     state_fields)
from .map_meta import load_map_meta
from .mission_logic import (
    APPROACH_TURN_YAW_DEG,
    DIALOG_PAUSED_HANDLE,
    GRIP_ENTER_WINDOW_SEC,
    GRIP_HINT_PULSE_SEC,
    GRIP_RATIO,
    GRIP_RELEASE_GRACE_SEC,
    GRIP_RESUME_WINDOW_SEC,
    GRIP_WAIT_TIMEOUT_SEC,
    HANDLE_LOST_GIVE_UP_SEC,
    HANDLE_LOST_REPEAT_SEC,
    HANDLE_SIDE_MIN_YAW_RAD,
    HANDLE_STATE_STALE_SEC,
    HOME_BEACON_INTERVAL_SEC,
    Haptic,
    MSG_APPROACH_ONBOARDING,
    MSG_APPROACH_REASK,
    MSG_DEST_RETRY,
    NEAR_CALL_MAX_M,
    NEAR_CALL_NO_SPIN_M,
    PERSON_APPROACH_SPEED_PERCENT,
    RETURN_RESUME_SEC,
    DEST_RETRY_RETURN_SEC,
    APPROACH_BT_FILE,
    GUIDED_BT_FILE,
    GUIDED_NO_RAIL_BT_FILE,
    NAV_TREE_GUIDED,
    NAV_TREE_WAIT,
    WAIT_SPOT_BT_FILE,
    ApproachRequest,
    GoalEvent,
    HelpEvent,
    CancelNav,
    Destination,
    GateReason,
    IntentData,
    MissionLogic,
    Navigate,
    NavStatus,
    Say,
    StopSpeech,
    WakeReply,
    SetNavSpeedLimit,
    SpinInPlace,
    Pose2D,
    State,
    check_gate,
    doa_to_spin_yaw,
    nav_behavior_tree,
    yaw_deg_to_quaternion,
)
from .obstacle_inputs import (
    BASE_FRAME,
    LASER_DEFAULT,
    Guard,
    OdomTrail,
    depth_frame_ok,
    format_decision,
    load_grid,
    scan_points,
    take_all,
    yaw_of,
)
from .obstacle_judge import ObstacleJudge, cm_what, goal_event, parse_state

BT_SWITCH_WAIT_SEC = 2.0

NAV_POLL_WAIT_SEC = 0.03

CLEAR_BEFORE_DEPARTURE_TIMEOUT_SEC = 1.0

HOME_DESTINATION_ID = "__home__"

try:
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
except ImportError:
    BasicNavigator = None
    TaskResult = None

if BasicNavigator is not None:
    class MissionNavigator(BasicNavigator):
        """isTaskComplete 의 기다림만 넘겨받을 수 있게 한 BasicNavigator."""

        def isTaskComplete(self, timeout_sec: float = 0.10) -> bool:  # noqa: N802 - 원본 이름
            if not self.result_future:
                return True
            rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=timeout_sec)
            result = self.result_future.result()
            if not result:
                return False
            self.status = result.status
            return True
else:
    MissionNavigator = None


class MissionManagerNode(Node):
    """LLM intent를 안전 게이트로 심사하고 승인된 Nav2 작업만 실행한다."""

    def __init__(self) -> None:
        """설정과 목적지를 읽고 intent·긴급정지·Nav2·TTS 토픽을 연결한다."""
        super().__init__("vica_mission_manager")

        self.declare_parameter("destinations_yaml", "")
        self.declare_parameter("map_id", "")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("auto_return_home", False)
        self.declare_parameter("confirm_timeout_sec", 30.0)
        self.declare_parameter("approach_turn_yaw_deg", APPROACH_TURN_YAW_DEG)
        self.declare_parameter("approach_bt_xml", "auto")
        self.declare_parameter("guided_bt_xml", "auto")
        self.declare_parameter("guided_no_rail_bt_xml", "auto")
        self.declare_parameter("wait_spot_bt_xml", "auto")
        self.declare_parameter("wake_doa_sign", 1.0)
        self.declare_parameter("seek_look_sec", 8.0)
        self.declare_parameter("near_call_max_m", NEAR_CALL_MAX_M)
        self.declare_parameter("near_call_no_spin_m", NEAR_CALL_NO_SPIN_M)
        self.declare_parameter(
            "handle_side_min_yaw_deg", math.degrees(HANDLE_SIDE_MIN_YAW_RAD))
        self.declare_parameter("return_resume_sec", RETURN_RESUME_SEC)
        self.declare_parameter("dest_retry_return_sec", DEST_RETRY_RETURN_SEC)
        self.declare_parameter(
            "person_approach_speed_percent", PERSON_APPROACH_SPEED_PERCENT)
        self.declare_parameter("estop_release_grace_sec", 1.0)
        self.declare_parameter("nav_retry_limit", 2)
        self.declare_parameter("nav_retry_delay_sec", 3.0)
        self.declare_parameter(
            "approach_slowdown_distances_m",
            [distance for distance, _ in DEFAULT_APPROACH_STAGES],
        )
        self.declare_parameter(
            "approach_speed_limit_percents",
            [percent for _, percent in DEFAULT_APPROACH_STAGES],
        )
        self.declare_parameter("tick_hz", 5.0)
        self.declare_parameter("current_floor", -1)
        self.declare_parameter("current_building", "")
        self.declare_parameter("arrival_dialog", True)
        self.declare_parameter("grip_enter_window_sec", GRIP_ENTER_WINDOW_SEC)
        self.declare_parameter("grip_ratio", GRIP_RATIO)
        self.declare_parameter("grip_wait_timeout_sec", GRIP_WAIT_TIMEOUT_SEC)
        self.declare_parameter("grip_assume_held", False)
        self.declare_parameter("home_beacon_interval_sec", HOME_BEACON_INTERVAL_SEC)
        self.declare_parameter("obstacle_narration", True)
        self.declare_parameter("grip_hint_pulse_sec", GRIP_HINT_PULSE_SEC)
        self.declare_parameter("grip_release_grace_sec", GRIP_RELEASE_GRACE_SEC)
        self.declare_parameter("grip_resume_window_sec", GRIP_RESUME_WINDOW_SEC)
        self.declare_parameter("handle_lost_repeat_sec", HANDLE_LOST_REPEAT_SEC)
        self.declare_parameter("handle_lost_give_up_sec", HANDLE_LOST_GIVE_UP_SEC)
        self.declare_parameter("handle_state_stale_sec", HANDLE_STATE_STALE_SEC)

        dest_path = str(self.get_parameter("destinations_yaml").value)
        if not dest_path:
            raise RuntimeError("destinations_yaml parameter 가 비어 있습니다 (launch 에서 절대경로 지정)")
        self._destinations_path = str(Path(dest_path).expanduser())
        configured_map_id = str(self.get_parameter("map_id").value).strip()
        self._map_id = configured_map_id or Path(self._destinations_path).parent.name
        self.destinations = load_destinations(self._destinations_path)

        self._map_meta = load_map_meta(self._destinations_path)
        self._ledger_store = LedgerStore(self._destinations_path)
        self._ledger: Ledger = self._ledger_store.read()
        self._ledger_prev_confirming: Optional[str] = None
        self._pose_cov_xy = 0.0
        self.get_logger().info(
            f"대장: 건물 {self._map_meta.building or '?'} {self._map_meta.floor}층, "
            f"직전 도착 {self._ledger.last_destination or '-'}, 파일 {self._ledger_store.path}")

        if Path(self._destinations_path).exists():
            self.get_logger().info(
                f"목적지 {len(self.destinations)}개 로드: {self._destinations_path}"
            )
        else:
            self.get_logger().warn(
                f"목적지 catalog가 없어 빈 목록으로 시작합니다: {self._destinations_path}"
            )

        self._home_storage = HomeStorage(Path(self._destinations_path).parent.parent)

        map_yaml = str(self.get_parameter("map_yaml").value)
        if map_yaml:
            self.map_bounds = load_map_bounds(map_yaml)
            self.get_logger().info(f"지도 경계: {self.map_bounds}")
        else:
            self.map_bounds = None
            self.get_logger().warn(
                "map_yaml 미지정 — 게이트 ⑤의 지도 경계 검증이 생략됩니다. "
                "실기 운용 전 반드시 지정할 것."
            )

        try:
            approach_stages = stages_from_lists(
                self.get_parameter("approach_slowdown_distances_m").value,
                self.get_parameter("approach_speed_limit_percents").value,
            )
        except (TypeError, ValueError) as exc:
            raise RuntimeError(f"접근 감속 단계 파라미터가 잘못되었습니다: {exc}") from exc

        retry_limit = int(self.get_parameter("nav_retry_limit").value)
        retry_delay = float(self.get_parameter("nav_retry_delay_sec").value)
        arrival_dialog = bool(self.get_parameter("arrival_dialog").value)
        home = self._load_home_destination()
        self.logic = MissionLogic(
            confirm_timeout_sec=float(self.get_parameter("confirm_timeout_sec").value),
            approach_turn_yaw_rad=math.radians(
                float(self.get_parameter("approach_turn_yaw_deg").value)),
            wake_doa_sign=float(self.get_parameter("wake_doa_sign").value),
            seek_look_sec=float(self.get_parameter("seek_look_sec").value),
            near_call_max_m=float(self.get_parameter("near_call_max_m").value),
            near_call_no_spin_m=float(
                self.get_parameter("near_call_no_spin_m").value),
            return_resume_sec=float(self.get_parameter("return_resume_sec").value),
            handle_side_min_yaw_rad=math.radians(
                float(self.get_parameter("handle_side_min_yaw_deg").value)),
            dest_retry_return_sec=float(
                self.get_parameter("dest_retry_return_sec").value),
            estop_release_grace_sec=float(self.get_parameter("estop_release_grace_sec").value),
            approach_stages=approach_stages,
            nav_retry_limit=retry_limit,
            nav_retry_delay_sec=retry_delay,
            arrival_dialog=arrival_dialog,
            return_destination=home,
            auto_return_home=bool(self.get_parameter("auto_return_home").value),
            person_approach_speed_percent=float(
                self.get_parameter("person_approach_speed_percent").value),
            **{name: float(self.get_parameter(name).value) for name in (
                "grip_enter_window_sec", "grip_ratio", "grip_wait_timeout_sec",
                "grip_hint_pulse_sec", "grip_release_grace_sec",
                "grip_resume_window_sec", "handle_lost_repeat_sec",
                "handle_lost_give_up_sec", "handle_state_stale_sec")},
            grip_assume_held=bool(self.get_parameter("grip_assume_held").value),
            home_beacon_interval_sec=float(
                self.get_parameter("home_beacon_interval_sec").value),
        )
        self.get_logger().info(
            "손잡이: 잡기 "
            f"{self.logic.grip_enter_window_sec:.1f}s 중 {self.logic.grip_ratio:.0%} · "
            f"대기 {self.logic.grip_wait_timeout_sec:.0f}s · "
            f"놓침 {self.logic.grip_release_grace_sec:.2f}s 뒤 정지")
        if self.logic.grip_assume_held:
            self.get_logger().warn(
                f"손잡이 시연 스위치 켜짐: 대기 {self.logic.grip_wait_timeout_sec:.0f}s 뒤 "
                "잡은 것으로 넘어가고 손 놓침 정지를 쓰지 않는다(grip_assume_held)")
        if arrival_dialog:
            self.get_logger().info(
                f"도착 후 대화: 켜짐 · 홈={'있음' if home else '없음(제자리 대기)'}")
        if home is not None and self.logic.auto_return_home:
            self.get_logger().warn(
                "접근 뒤 자동 홈 복귀: 켜짐 — 사람이 부르지 않아도 로봇이 홈까지 달립니다."
            )
        elif home is not None:
            self.get_logger().info(
                "접근 뒤 자동 홈 복귀: 꺼짐(기본) — 접근을 마친 자리에 섭니다. "
                "앱의 홈 복귀 버튼은 그대로 동작합니다."
            )
        if retry_limit > 0:
            self.get_logger().info(
                f"주행 실패 시 자동 재시도: 최대 {retry_limit}회 · {retry_delay:.1f}초 간격"
            )
        else:
            self.get_logger().info("주행 실패 시 자동 재시도: 꺼짐")
        if approach_stages:
            ladder = ", ".join(
                f"{distance:.2f}m이하 {percent:.0f}%"
                for distance, percent in approach_stages
            )
            self.get_logger().info(f"접근 감속 단계: {ladder}")
        else:
            self.get_logger().warn(
                "접근 감속 단계가 비어 있습니다 — 목적지까지 최대속도로 접근합니다."
            )

        if BasicNavigator is None:
            raise RuntimeError("nav2_simple_commander 를 import 할 수 없습니다")
        self.navigator = MissionNavigator("vica_mission_navigator")
        self._nav_lock = threading.Lock()
        self._nav_lock_timeout_sec = 2.0
        self._nav_active = False
        self._approach_bt = self._resolve_bt(
            str(self.get_parameter("approach_bt_xml").value).strip(),
            APPROACH_BT_FILE, "사람 접근 트리(레일 없이 자유주행)")
        self._tree_files = {
            NAV_TREE_GUIDED: self._resolve_bt(
                str(self.get_parameter("guided_bt_xml").value).strip(),
                GUIDED_BT_FILE, "사용자 안내 트리(위치만 판정)"),
            NAV_TREE_WAIT: self._resolve_bt(
                str(self.get_parameter("wait_spot_bt_xml").value).strip(),
                WAIT_SPOT_BT_FILE, "대기 장소 트리(레일 없음, 위치·방향)"),
        }
        self._guided_no_rail_bt = self._resolve_bt(
            str(self.get_parameter("guided_no_rail_bt_xml").value).strip(),
            GUIDED_NO_RAIL_BT_FILE, "사용자 안내 트리(레일 없는 지도, 위치만 판정)")
        self._task_bt = ""
        self._nav_gen = 0
        self._clear_before_next_nav = True
        self._nav_task_is_spin = False
        self._delivery_dest_id: Optional[str] = None

        self._main_group = MutuallyExclusiveCallbackGroup()
        self._emergency_group = MutuallyExclusiveCallbackGroup()

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(
            VicaIntent, "/vica/intent", self._on_intent, 10, callback_group=self._main_group
        )
        self.create_subscription(
            EmergencyEvent,
            "/vica/emergency",
            self._on_emergency,
            reliable_qos,
            callback_group=self._emergency_group,
        )
        self._estop_active = False
        self.create_subscription(
            Bool,
            "/vica/tipped",
            self._on_tipped,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self._emergency_group,
        )
        self.create_subscription(
            String,
            "/app_estop_state",
            self._on_app_estop_state,
            10,
            callback_group=self._emergency_group,
        )
        self.create_subscription(
            Bool,
            "/emergency_stop",
            self._on_estop,
            10,
            callback_group=self._emergency_group,
        )

        self.create_subscription(
            String,
            "/vica/tts_done",
            self._on_tts_done,
            10,
            callback_group=self._main_group,
        )

        self.create_subscription(
            String, "/vica/wake", self._on_wake, 10,
            callback_group=self._main_group,
        )
        self.create_subscription(
            Float32, "/vica/wake_doa", self._on_wake_doa, 10,
            callback_group=self._main_group,
        )
        self.create_subscription(
            PersonDetection, "/vica/person_detection", self._on_person_detection, 10,
            callback_group=self._main_group,
        )
        self.create_subscription(
            String, "/vica/listen_state",
            lambda msg: self._run_actions(
                self.logic.on_listen_state(msg.data, self._now())),
            10,
            callback_group=self._main_group,
        )
        self.create_subscription(
            Bool, "/vica/thinking",
            lambda msg: self.logic.on_llm_thinking(msg.data, self._now()),
            10,
            callback_group=self._main_group,
        )

        self.create_subscription(
            SmartHandleState, "/vica/smart_handle_state",
            lambda msg: self.logic.on_handle_state(
                msg.user_contact, msg.uplink_fresh, self._now()),
            10,
            callback_group=self._main_group,
        )

        self.pub_tts = self.create_publisher(String, "/vica/tts_request", 10)
        self.pub_listen_request = self.create_publisher(Bool, "/vica/listen_request", 10)
        self.pub_wake_reply = self.create_publisher(String, "/vica/wake_reply", 10)
        self._wake_reply_seq = ""
        self.pub_haptic = self.create_publisher(String, "/vica/haptic_request", 10)
        self.pub_state = self.create_publisher(RobotState, "/vica/robot_state", 10)
        self.pub_goal_event = self.create_publisher(String, "/vica_goal_event", 10)
        self.pub_speed_limit = self.create_publisher(
            SpeedLimit,
            "/speed_limit",
            10,
        )
        self._publish_nav_speed_limit(0.0)
        self.create_service(
            RequestDestination,
            "/vica/mission/request_destination",
            self._on_destination_request,
            callback_group=self._main_group,
        )
        self.create_service(
            RequestDestination,
            "/vica/mission/request_delivery",
            self._on_delivery_request,
            callback_group=self._main_group,
        )
        self.create_service(
            RequestDestination,
            "/vica/mission/request_wait_spot",
            self._on_wait_spot_request,
            callback_group=self._main_group,
        )
        self.create_service(
            Trigger,
            "/vica/mission/reload_destinations",
            self._on_reload_destinations,
            callback_group=self._main_group,
        )
        self.create_service(
            MissionCommand,
            "/vica/mission/cancel_destination",
            self._on_cancel_request,
            callback_group=self._main_group,
        )
        self.create_service(
            MissionCommand,
            "/vica/mission/pause_navigation",
            self._on_pause_request,
            callback_group=self._main_group,
        )
        self.create_service(
            MissionCommand,
            "/vica/mission/resume_navigation",
            self._on_resume_request,
            callback_group=self._main_group,
        )
        self.create_service(
            MissionCommand,
            "/vica/mission/help_ack",
            self._on_help_ack,
            callback_group=self._main_group,
        )
        self.create_service(
            RequestApproach,
            "/vica/mission/request_approach",
            self._on_approach_request,
            callback_group=self._main_group,
        )
        self.create_service(
            MissionCommand,
            "/vica/mission/cancel_approach",
            self._on_approach_cancel,
            callback_group=self._main_group,
        )
        self.create_service(
            SaveHome,
            "/vica/home/save",
            self._on_save_home,
            callback_group=self._main_group,
        )
        self.create_service(
            GetHome,
            "/vica/home/get",
            self._on_get_home,
            callback_group=self._main_group,
        )
        self.create_service(
            DeleteHome,
            "/vica/home/delete",
            self._on_delete_home,
            callback_group=self._main_group,
        )
        self.create_service(
            MissionCommand,
            "/vica/mission/return_home",
            self._on_return_home,
            callback_group=self._main_group,
        )
        self._robot_pose = None
        amcl_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_amcl_pose,
            amcl_qos,
            callback_group=self._main_group,
        )

        self._obstacle: Optional[ObstacleJudge] = None
        self._obstacle_guard = Guard(self._on_obstacle_error)
        self._obstacle_cues: deque = deque(maxlen=8)
        if bool(self.get_parameter("obstacle_narration").value):
            self._setup_obstacle_narration(map_yaml)
        else:
            self.get_logger().info("장애물 안내: 꺼짐(obstacle_narration=false)")

        tick_hz = float(self.get_parameter("tick_hz").value)
        self.create_timer(1.0 / tick_hz, self._tick, callback_group=self._main_group)
        self.create_timer(1.0, self._publish_robot_state, callback_group=self._main_group)

        self.get_logger().info("vica_mission_manager 시작 (상태: idle)")

    def _load_home(self):
        try:
            return load_home(self._destinations_path)
        except Exception as exc:
            self.get_logger().warn(f"홈 로드 실패(제자리 대기로 폴백): {exc}")
            return None

    def _on_intent(self, msg: VicaIntent) -> None:
        """음성 요청. 판단은 로직(on_voice_intent)이 하고 여기선 실행·기록만 한다(2026-10-08)."""
        intent = IntentData(
            intent=msg.intent,
            matched_destination_id=msg.matched_destination_id,
            need_confirm=msg.need_confirm,
            safety_flag=msg.safety_flag,
            wait_minutes=int(getattr(msg, "wait_minutes", -1)),
            reply=msg.reply,
        )
        before = self.logic.state
        actions = self.logic.on_voice_intent(
            intent, self._now(), self._lookup_destination,
            bounds=self.map_bounds, nav_ready=self._nav2_ready())
        self._run_actions(actions)
        said = [a.text for a in actions if isinstance(a, Say)]
        self.get_logger().info(
            f"음성 요청 intent={msg.intent} dest={msg.matched_destination_id or '-'} "
            f"confirm={msg.need_confirm}: {before.value} -> {self.logic.state.value}"
            + (f" 말={said}" if said else " (말 없음)")
            + (f" — {self.logic.voice_note}" if self.logic.voice_note else ""))

    def _lookup_destination(self, dest_id: str) -> Optional[Destination]:
        if not dest_id:
            return None
        return self.destinations.get(dest_id) or None

    def _on_tts_done(self, msg: String) -> None:
        """TTS 가 끊기지 않고 끝까지 재생한 문장. 접근 질문일 때만 시계를 켠다."""
        if MSG_APPROACH_REASK in msg.data:
            self.logic.on_approach_question_spoken(self._now())
        self._run_actions(self.logic.on_approach_reply_spoken(msg.data, self._now()))
        if MSG_APPROACH_ONBOARDING in msg.data or MSG_DEST_RETRY in msg.data:
            self._run_actions(self.logic.on_dest_prompt_spoken(self._now()))
        self.logic.on_arrival_question_spoken(self._now())
        self._run_actions(self.logic.on_wait_speech_spoken(msg.data, self._now()))

    def _on_wake(self, msg: String) -> None:
        """/vica/wake — 호출 반응표대로 대답할지 정한다(2026-10-07, 판단은 로직)."""
        kind, _, seq = (msg.data or "").strip().partition(":")
        rescue = kind == "rescue"
        self._wake_reply_seq = seq if seq.isdigit() else ""
        now = self._now()
        before = self.logic.state
        actions: list = []
        if self.logic.state == State.RETURNING:
            actions.extend(self.logic.on_return_brake(now))
            if actions:
                self.get_logger().info("복귀 중 '비카야' — 복귀 취소하고 응대")
        if rescue:
            actions.extend(self.logic.on_wake(now))
        else:
            actions.extend(self.logic.on_wake_call(now))
        self._run_actions(actions)
        reply = next((a for a in actions if isinstance(a, WakeReply)), None)
        verdict = "창 안 구제" if rescue else ("들음" if reply and reply.listen else "대답 안 함")
        self.get_logger().info(
            f"'비카야'({verdict}): {before.value} -> {self.logic.state.value}")

    def _on_wake_doa(self, msg: Float32) -> None:
        """/vica/wake_doa — 호출 방향으로 고개를 돌린다 (IDLE 에서만)."""
        now = self._now()
        before = self.logic.state
        nav_ready = self._nav2_ready()
        actions = self.logic.on_wake_doa(float(msg.data), nav_ready, now)
        self._run_actions(actions)
        if State.SEEKING in (before, self.logic.state) and before != self.logic.state:
            self._publish_robot_state()
        yaw_rad = doa_to_spin_yaw(float(msg.data), self.logic.wake_doa_sign)
        if any(isinstance(a, SpinInPlace) for a in actions):
            verdict = "회전 시작"
        elif before == State.IDLE and self.logic.state == State.AWAITING_USER:
            verdict = "핸들 쪽 — 회전 없이 곧바로 질문"
        elif before != State.IDLE:
            verdict = f"거절(대기 중 아님, state={before.value})"
        elif self.logic.estop_active:
            verdict = "거절(E-stop)"
        elif not nav_ready:
            verdict = "거절(nav 미준비)"
        elif self.logic.return_interrupted:
            verdict = "거절(복귀 대기 중)"
        elif self.logic.wake_guard_active(now):
            verdict = "거절(방금 wake 소비 직후로 추정)"
        elif self.logic.user_attached_guard_active(now):
            verdict = "거절(안내 시작 직후)"
        else:
            verdict = "생략(10도 미만, 창만 유지)"
        self.get_logger().info(
            f"'비카야' 방향 doa={msg.data:.0f}° yaw={math.degrees(yaw_rad):.0f}°: "
            f"{before.value} -> {self.logic.state.value} ({verdict})")

    def _on_person_detection(self, msg: PersonDetection) -> None:
        """/vica/person_detection 원본 결과 (근접 호출, 2026-09-10 확장)."""
        if self.logic.state == State.APPROACHING:
            self._recheck_approach(msg)
            return
        before = self.logic.state
        actions = self.logic.on_person_detection(
            track_id=msg.track_id,
            distance_m=msg.distance_m,
            stable=msg.stable,
            approachable=msg.approachable,
            now=self._now(),
            person=self._detection_pose(msg),
        )
        if before == self.logic.state:
            return
        self._run_actions(actions)
        self.get_logger().info(
            f"근접 호출: track={msg.track_id} dist={msg.distance_m:.2f}m "
            f"{before.value} -> {self.logic.state.value}"
        )

    @staticmethod
    def _detection_pose(msg: PersonDetection) -> Optional[Pose2D]:
        """검출의 사람 자리(map). 좌표가 NaN 이면 None — 자리로 판단하지 않는다."""
        px, py = msg.pose.position.x, msg.pose.position.y
        if not (math.isfinite(px) and math.isfinite(py)):
            return None
        return Pose2D(x=px, y=py, yaw_deg=0.0, frame_id=msg.header.frame_id or "map")

    def _recheck_approach(self, msg: PersonDetection) -> None:
        """접근 중 검출 → 중간 재측정(2026-10-10, 설계 docs/superpowers/specs/2026-10-10-…)."""
        person = self._detection_pose(msg)
        if person is None:
            return
        recheck = self.logic.approach_recheck
        before = recheck.phase
        new_person = self.logic.on_approach_detection(person, msg.confidence, self._now())
        after = recheck.phase
        if after != before:
            self.get_logger().info(
                f"중간 재측정: {before.value} -> {after.value} (처음 거리 "
                f"{recheck.start_distance_m:.2f} m, 시점 {recheck.trigger_m:.2f} m, "
                f"표본 {recheck.sample_count}개)")
        if new_person is None or self._robot_pose is None:
            return
        old = self.logic.approach_person
        shift = (math.hypot(new_person.x - old.x, new_person.y - old.y)
                 if old is not None else float("nan"))
        goal = approach_goal(new_person, self._robot_pose)
        actions, moved = self.logic.on_approach_recheck_goal(
            new_person, goal, self.map_bounds, self._now())
        self._run_actions(actions)
        self.get_logger().info(
            f"중간 재측정: 사람 자리 {shift:.2f} m 고침 → 목표 {moved:.2f} m "
            f"{'다시 보냄' if actions else '그대로(12 cm 미만이거나 지도 밖)'} · 이후 목표 고정")

    def _on_cancel_request(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
    ) -> MissionCommand.Response:
        return self._handle_mission_command(request, response, "cancel")

    def _on_pause_request(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
    ) -> MissionCommand.Response:
        return self._handle_mission_command(request, response, "pause")

    def _on_resume_request(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
    ) -> MissionCommand.Response:
        return self._handle_mission_command(request, response, "resume")

    def _handle_mission_command(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
        command: str,
    ) -> MissionCommand.Response:
        try:
            request_id = str(UUID(request.request_id))
        except ValueError:
            response.accepted = False
            response.message = "request_id는 UUID여야 합니다."
            return response
        if request_id != request.request_id.lower():
            response.accepted = False
            response.message = "request_id가 canonical UUID 형식이 아닙니다."
            return response

        before = self.logic.state
        actions, reason = self._run_mission_command(command)
        if reason != GateReason.OK:
            self._run_actions(actions)
            response.accepted = False
            response.message = f"{command} 요청 거부: {reason.value}"
            self.get_logger().warn(
                f"{command} 요청 거부: state={before.value} reason={reason.value}"
            )
            return response

        self._run_actions(actions)
        response.accepted = True
        response.message = f"{command} 요청을 처리했습니다."
        self.get_logger().info(
            f"{command} 처리: {before.value} -> {self.logic.state.value}"
        )
        return response

    def _on_help_ack(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
    ) -> MissionCommand.Response:
        """관리자가 앱에서 도움 요청을 확인했다. request_id = 확인할 help_id(uuid4)."""
        try:
            request_id = str(UUID(request.request_id))
        except ValueError:
            response.accepted = False
            response.message = "request_id는 UUID여야 합니다."
            return response
        if request_id != request.request_id.lower():
            response.accepted = False
            response.message = "request_id가 canonical UUID 형식이 아닙니다."
            return response
        actions, accepted = self.logic.on_help_ack(request_id, self._now())
        self._run_actions(actions)
        response.accepted = accepted
        if accepted:
            response.message = "ok"
        elif self.logic.help_id is None:
            response.message = "no_active_help"
        else:
            response.message = "id_mismatch"
        self.get_logger().info(f"도움 요청 확인: {request_id} → {response.message}")
        return response

    def _load_home_destination(self):
        """home.yaml 을 읽어 MissionLogic 이 쓸 Destination 으로 만든다."""
        map_yaml = str(self.get_parameter("map_yaml").value)
        if map_yaml:
            before = self._home_storage.read(self._map_id)
            home = self._home_storage.invalidate_if_map_is_newer(
                self._map_id, map_yaml
            )
            if (
                before is not None
                and home is not None
                and before.visited_ok
                and not home.visited_ok
            ):
                self.get_logger().warn(
                    "지도가 홈보다 새로 저장되어 홈 확인 상태를 되돌렸습니다. "
                    "앱에서 '홈으로 가보기'로 다시 확인하세요."
                )
        else:
            home = self._home_storage.read(self._map_id)
        if home is None:
            return None
        return Destination(
            id=HOME_DESTINATION_ID,
            name=home.label or "홈",
            pose=Pose2D(x=home.x, y=home.y, yaw_deg=home.yaw, frame_id="map"),
        )

    def _refresh_home(self) -> None:
        """저장·삭제 직후 MissionLogic 이 쓰는 홈을 새로 읽어 맞춘다."""
        self.logic.return_destination = self._load_home_destination()

    def _on_save_home(
        self,
        request: SaveHome.Request,
        response: SaveHome.Response,
    ) -> SaveHome.Response:
        if request.map_id != self._map_id:
            response.accepted = False
            response.visited_ok = False
            response.message = (
                f"다른 지도의 홈은 저장할 수 없습니다. "
                f"현재={self._map_id}, 요청={request.map_id}"
            )
            return response

        try:
            home = build_home(
                map_id=request.map_id,
                x=request.x,
                y=request.y,
                yaw=request.yaw,
                source=request.source,
                score=request.score,
                label=request.label,
            )
            saved = self._home_storage.write(home)
        except (ValueError, OSError) as exc:
            response.accepted = False
            response.visited_ok = False
            response.message = f"홈 저장 실패: {exc}"
            self.get_logger().warn(response.message)
            return response

        self._refresh_home()
        response.accepted = True
        response.visited_ok = saved.visited_ok
        response.message = (
            "홈을 저장했습니다. 아직 가 본 적이 없으니 '홈으로 가보기'로 확인하세요."
        )
        self.get_logger().info(
            f"홈 저장: ({saved.x:.2f}, {saved.y:.2f}, {saved.yaw:.0f}도) "
            f"source={saved.source} score={saved.score:.1f}"
        )
        return response

    def _on_get_home(
        self,
        request: GetHome.Request,
        response: GetHome.Response,
    ) -> GetHome.Response:
        map_id = request.map_id or self._map_id
        try:
            home = self._home_storage.read(map_id)
        except ValueError as exc:
            response.exists = False
            response.message = f"map_id 오류: {exc}"
            return response

        if home is None:
            response.exists = False
            response.message = "홈이 아직 지정되지 않았습니다."
            return response

        response.exists = True
        response.message = ""
        response.x = home.x
        response.y = home.y
        response.yaw = home.yaw
        response.source = home.source
        response.score = home.score
        response.label = home.label
        response.visited_ok = home.visited_ok
        response.saved_at = home.saved_at
        return response

    def _on_delete_home(
        self,
        request: DeleteHome.Request,
        response: DeleteHome.Response,
    ) -> DeleteHome.Response:
        if request.map_id != self._map_id:
            response.accepted = False
            response.message = (
                f"다른 지도의 홈은 지울 수 없습니다. "
                f"현재={self._map_id}, 요청={request.map_id}"
            )
            return response

        try:
            removed = self._home_storage.delete(request.map_id)
        except (ValueError, OSError) as exc:
            response.accepted = False
            response.message = f"홈 삭제 실패: {exc}"
            return response

        self._refresh_home()
        response.accepted = True
        response.message = (
            "홈을 지웠습니다. 자동 복귀가 꺼집니다."
            if removed
            else "지울 홈이 없었습니다."
        )
        self.get_logger().info(response.message)
        return response

    def _on_return_home(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
    ) -> MissionCommand.Response:
        """관리자가 앱에서 홈 복귀를 눌렀다."""
        try:
            request_id = str(UUID(request.request_id))
        except ValueError:
            response.accepted = False
            response.message = "request_id는 UUID여야 합니다."
            return response
        if request_id != request.request_id.lower():
            response.accepted = False
            response.message = "request_id가 canonical UUID 형식이 아닙니다."
            return response

        before = self.logic.state
        accepted, reason, actions = self.logic.on_return_home_request(
            self._nav2_ready(), self._now()
        )
        if not accepted:
            response.accepted = False
            response.message = f"홈 복귀 거부: {reason.value}"
            self.get_logger().warn(
                f"홈 복귀 거부: state={before.value} reason={reason.value}"
            )
            return response

        self._run_actions(actions)
        home = self.logic.return_destination
        if home is not None:
            self._publish_goal_event("return_home_sent", home)
        response.accepted = True
        response.message = "홈으로 복귀합니다."
        self.get_logger().info(
            f"홈 복귀 시작: {before.value} -> {self.logic.state.value}"
        )
        return response

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        q = msg.pose.pose.orientation
        yaw_deg = math.degrees(
            math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        self._robot_pose = Pose2D(
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            yaw_deg=yaw_deg,
            frame_id=msg.header.frame_id or "map",
        )
        self.logic.robot_yaw_deg = yaw_deg
        self.logic.robot_pose = self._robot_pose
        cov = msg.pose.covariance
        self._pose_cov_xy = float(cov[0] + cov[7]) if len(cov) >= 8 else 0.0

    def destroy_node(self) -> bool:
        executor = getattr(self, "_obs_executor", None)
        if executor is not None:
            executor.shutdown(timeout_sec=1.0)
        node = getattr(self, "_obs_node", None)
        if node is not None:
            node.destroy_node()
        return super().destroy_node()

    def _setup_obstacle_narration(self, map_yaml: str) -> None:
        grid, why = load_grid(map_yaml)
        if grid is None:
            self.get_logger().warn(f"장애물 안내: 지도를 못 읽어 끕니다 — {why}")
            return
        try:
            self._obstacle = ObstacleJudge(grid)
            self._laser_offset: Optional[tuple] = None
            self._tf_buffer = tf2_ros.Buffer()
            self._obs_amcl: Optional[tuple] = None
            self._obs_trail = OdomTrail()
            self._obs_subs: list = []
            self._obs_on = False
            node = rclpy.create_node("vica_mission_obstacle")
            self._obs_node = node
            group = ReentrantCallbackGroup()
            self._obs_group = group
            wrap = self._obstacle_guard.wrap
            kept = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
            static = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            node.create_subscription(PathMsg, "/rail_plan", wrap(self._obs_rail), 5, callback_group=group)
            node.create_subscription(String, "/vica_goal_event", wrap(self._obs_goal), 10, callback_group=group)
            node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", wrap(self._obs_amcl_msg), kept,
                                     callback_group=group)
            node.create_subscription(TFMessage, "/tf_static", wrap(self._obs_static), static, callback_group=group)
            self._obs_timer = node.create_timer(0.1, wrap(self._obs_tick), callback_group=group)
            self._obs_timer.cancel()
            self._obs_executor = SingleThreadedExecutor()
            self._obs_executor.add_node(self._obs_node)
            threading.Thread(target=self._obs_spin, name="obstacle_inputs", daemon=True).start()
        except Exception as exc:  # noqa: BLE001
            self._obstacle_guard.enabled = False
            self._on_obstacle_error(exc)
            return
        self.get_logger().info(f"장애물 안내: 켜짐 (지도 {map_yaml}) — 입력은 안내 주행 중에만 받습니다")

    def _obs_spin(self) -> None:
        while rclpy.ok():
            try:
                self._obs_executor.spin_once()
            except InvalidHandle:
                continue
            except (ExternalShutdownException, ShutdownException):
                return
            except Exception as exc:  # noqa: BLE001 — 보조 실행기가 죽으면 안내만 끈다
                if rclpy.ok():
                    self._obstacle_guard.enabled = False
                    self._on_obstacle_error(exc)
                return

    def _obstacle_inputs_follow(self) -> None:
        """장애물 입력은 안내 주행(NAVIGATING) 중에만 받는다 — 안내 멘트도 그때만 나간다(obstacle_cue)."""
        if self._obstacle is None or not self._obstacle_guard.enabled:
            if getattr(self, "_obs_on", False):
                try:
                    self._obs_stop()
                except Exception:  # noqa: BLE001 — 이미 꺼진 안내다. 닫다 난 오류는 더 할 일이 없다
                    pass
            return
        want = self.logic.dialog_state == State.NAVIGATING.value
        if want == self._obs_on:
            return
        try:
            if want:
                self._obs_start()
            else:
                self._obs_stop()
        except Exception as exc:  # noqa: BLE001 — 열고 닫다 난 오류도 안내만 끈다
            self._obstacle_guard.enabled = False
            self._on_obstacle_error(exc)

    def _obs_start(self) -> None:
        self._obstacle.clear_inputs()
        self._obs_trail.clear()
        node, group = self._obs_node, self._obs_group
        latest = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.BEST_EFFORT)
        wrap = self._obstacle_guard.wrap
        self._obs_on = True
        self._obs_subs = []
        self._obs_timer.reset()
        subs = self._obs_subs
        subs.append(node.create_subscription(Odometry, "/odom", wrap(self._obs_odom_msg), 10, callback_group=group))
        subs.append(node.create_subscription(LaserScan, "/scan", wrap(self._obs_scan), latest, callback_group=group))
        subs.append(node.create_subscription(LaserScan, "/camera/depth_scan", wrap(self._obs_depth), latest,
                                             callback_group=group))
        subs.append(node.create_subscription(String, "/vcc/state", wrap(self._obs_vcc), 10, callback_group=group))
        subs.append(node.create_subscription(BehaviorTreeLog, "/behavior_tree_log", wrap(self._obs_bt), 50,
                                             callback_group=group))
        subs.append(node.create_subscription(Log, "/rosout", wrap(self._obs_rosout), 50, callback_group=group))
        self.get_logger().info("장애물 안내: 입력 받기 시작(안내 주행)")

    def _obs_stop(self) -> None:
        self._obs_on = False
        self._obs_timer.cancel()
        for sub in self._obs_subs:
            self._obs_node.destroy_subscription(sub)
        self._obs_subs = []
        self.get_logger().info("장애물 안내: 입력 멈춤(안내 주행 아님)")

    def _on_obstacle_error(self, exc: BaseException) -> None:
        self._obstacle = None
        self.get_logger().error(f"장애물 안내: 오류로 끕니다 — 안내 주행은 계속됩니다 ({exc!r})")

    @staticmethod
    def _stamp(msg) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _obs_amcl_msg(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        self._obs_amcl = (self._stamp(msg), p.position.x, p.position.y, yaw_of(p.orientation))

    def _obs_odom_msg(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._obs_trail.add(self._stamp(msg), p.position.x, p.position.y, yaw_of(p.orientation))

    def _obs_static(self, msg: TFMessage) -> None:
        for tr in msg.transforms:
            self._tf_buffer.set_transform_static(tr, "default_authority")

    def _obs_pose(self, t: float) -> None:
        pose = self._obs_trail.pose_since(self._obs_amcl)
        if pose is None:
            return
        self._obstacle.on_pose(t, *pose)

    def _obs_laser_offset(self) -> tuple:
        if self._laser_offset is None:
            try:
                tr = self._tf_buffer.lookup_transform(BASE_FRAME, "laser_frame", rclpy.time.Time())
            except tf2_ros.TransformException:
                return LASER_DEFAULT
            self._laser_offset = (tr.transform.translation.x, tr.transform.translation.y)
        return self._laser_offset

    def _obs_scan(self, msg: LaserScan) -> None:
        t = self._now()
        self._obs_pose(t)
        self._obstacle.on_points("scan", t, *scan_points(msg, self._obs_laser_offset()))

    def _obs_depth(self, msg: LaserScan) -> None:
        if depth_frame_ok(msg.header.frame_id):
            self._obstacle.on_points("depth", self._now(), *scan_points(msg))

    def _obs_vcc(self, msg: String) -> None:
        st = parse_state(msg.data)
        if st is not None:
            t = self._now()
            self._obstacle.on_dialog(t, self.logic.dialog_state)
            self._obstacle.on_vcc(t, st)

    def _obs_bt(self, msg: BehaviorTreeLog) -> None:
        for e in msg.event_log:
            self._obstacle.on_bt(e.timestamp.sec + e.timestamp.nanosec * 1e-9, e.node_name, e.current_status)

    def _obs_rosout(self, msg: Log) -> None:
        if msg.name == "collision_monitor":
            what = cm_what(msg.msg)
            if what:
                self._obstacle.on_cm(self._now(), what)

    def _obs_rail(self, msg: PathMsg) -> None:
        self._obstacle.on_rail(self._now(), [(p.pose.position.x, p.pose.position.y) for p in msg.poses])

    def _obs_goal(self, msg: String) -> None:
        ev = goal_event(msg.data)
        if ev:
            self._obstacle.on_goal(self._now(), ev["event"], ev["x"], ev["y"], ev["loc"])

    def _obs_tick(self) -> None:
        t = self._now()
        self._obs_pose(t)
        for d in self._obstacle.tick(t):
            if d["decision"] == "announce":
                self._obstacle_cues.append((d["phrase"], d["t"]))
            self.get_logger().info("장애물: " + format_decision(d))

    def _drain_obstacle_cues(self) -> None:
        """판정 줄이 넘긴 후보를 대화 줄에서 말할지 정한다(_tick 끝에서 부른다)."""
        for phrase, onset in take_all(self._obstacle_cues):
            actions, why = self.logic.obstacle_cue(phrase, onset, self._now())
            if why:
                self.get_logger().info(f"장애물 안내 뺌: {why}")
            self._run_actions(actions)

    def _on_approach_request(
        self,
        request: RequestApproach.Request,
        response: RequestApproach.Response,
    ) -> RequestApproach.Response:
        """사람 접근 요청. 검증 -> goal 계산 -> 상태 기계 순서로 거른다."""
        try:
            request_id = str(UUID(request.request_id))
        except ValueError:
            response.accepted = False
            response.message = "request_id는 UUID여야 합니다."
            return response
        if request_id != request.request_id.lower():
            response.accepted = False
            response.message = "request_id가 canonical UUID 형식이 아닙니다."
            return response

        target = request.target
        if request.track_id != target.track_id:
            response.accepted = False
            response.message = "track_id가 target.track_id와 다릅니다."
            return response
        if target.track_id == PersonDetection.TRACK_ID_NONE:
            response.accepted = False
            response.message = "추적 id가 없는 탐지는 접근 대상이 아닙니다."
            return response

        px, py = target.pose.position.x, target.pose.position.y
        if not (math.isfinite(px) and math.isfinite(py)):
            response.accepted = False
            response.message = "탐지 좌표가 유효하지 않습니다."
            return response
        if self._robot_pose is None:
            response.accepted = False
            response.message = "로봇 위치(/amcl_pose)를 아직 받지 못했습니다."
            return response

        person = Pose2D(x=px, y=py, yaw_deg=0.0,
                        frame_id=target.header.frame_id or "map")
        goal = approach_goal(person, self._robot_pose)
        approach = ApproachRequest(
            goal=goal,
            track_id=target.track_id,
            approachable=target.approachable,
            person=person,
        )

        before = self.logic.state
        actions, reason = self.logic.on_approach_request(
            approach, self.map_bounds, self._nav2_ready(), self._now()
        )
        if reason != GateReason.OK:
            response.accepted = False
            response.message = f"접근 요청 거부: {reason.value}"
            self.get_logger().warn(
                f"접근 거부: track={target.track_id} state={before.value} "
                f"reason={reason.value}"
            )
            return response

        self._run_actions(actions)
        response.accepted = True
        response.message = "접근을 시작합니다."
        self.get_logger().info(
            f"접근 승인: track={target.track_id} dist={target.distance_m:.2f}m "
            f"{before.value} -> {self.logic.state.value}"
        )
        return response

    def _on_approach_cancel(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
    ) -> MissionCommand.Response:
        try:
            request_id = str(UUID(request.request_id))
        except ValueError:
            response.accepted = False
            response.message = "request_id는 UUID여야 합니다."
            return response
        if request_id != request.request_id.lower():
            response.accepted = False
            response.message = "request_id가 canonical UUID 형식이 아닙니다."
            return response

        before = self.logic.state
        actions, reason = self.logic.on_approach_cancel_request(self._now())
        if reason != GateReason.OK:
            response.accepted = False
            response.message = f"접근 취소 거부: {reason.value}"
            return response
        self._run_actions(actions)
        response.accepted = True
        response.message = "접근을 취소했습니다."
        self.get_logger().info(
            f"접근 취소: {before.value} -> {self.logic.state.value}"
        )
        return response

    def _run_mission_command(self, command: str) -> tuple:
        """command 이름에 맞는 로직 핸들러를 부른다. (actions, GateReason)"""
        now = self._now()
        if command == "cancel":
            was_idle = self.logic.state is State.IDLE
            result = self.logic.on_app_cancel(now)
            if was_idle:
                self._publish_goal_event(
                    "state_idle", None, "취소할 주행이 없습니다(대기 중)."
                )
            return result
        if command == "pause":
            return self.logic.on_pause_request(now)
        if command == "resume":
            return self.logic.on_resume_request(self._nav2_ready(), now)
        return [], GateReason.NOT_NAVIGATE

    def _on_destination_request(
        self,
        request: RequestDestination.Request,
        response: RequestDestination.Response,
    ) -> RequestDestination.Response:
        """앱·CLI 요청을 UUID로 검증하고 기존 Mission gate를 통과시킨다."""
        return self._handle_destination_request(
            request, response, allow_private=False, label="공개 목적지"
        )

    def _on_delivery_request(
        self,
        request: RequestDestination.Request,
        response: RequestDestination.Response,
    ) -> RequestDestination.Response:
        """물류 배송 요청. 공개 요청과 같되 private 목적지를 허용한다."""
        return self._handle_destination_request(
            request, response, allow_private=True, label="배송 목적지",
            is_delivery=True,
        )

    def _check_destination_request(
        self,
        request: RequestDestination.Request,
        response: RequestDestination.Response,
    ):
        """요청 모양(UUID·지도)을 검사한다. 통과하면 목적지 id, 아니면 None"""
        try:
            request_id = str(UUID(request.request_id))
            destination_id = str(UUID(request.destination_id))
        except ValueError:
            response.accepted = False
            response.message = "request_id와 destination_id는 UUID여야 합니다."
            return None
        if request_id != request.request_id.lower():
            response.accepted = False
            response.message = "request_id가 canonical UUID 형식이 아닙니다."
            return None
        parsed_destination = UUID(destination_id)
        if (
            parsed_destination.version != 4
            or destination_id != request.destination_id.lower()
        ):
            response.accepted = False
            response.message = "destination_id는 canonical UUID v4여야 합니다."
            return None
        if request.map_id != self._map_id:
            response.accepted = False
            response.message = (
                f"현재 지도와 요청 지도가 다릅니다: "
                f"current={self._map_id}, requested={request.map_id}"
            )
            return None
        return destination_id

    def _handle_destination_request(
        self,
        request: RequestDestination.Request,
        response: RequestDestination.Response,
        allow_private: bool,
        label: str,
        is_delivery: bool = False,
    ) -> RequestDestination.Response:
        destination_id = self._check_destination_request(request, response)
        if destination_id is None:
            return response
        destination = self.destinations.get(destination_id)
        before = self.logic.state
        actions, reason = self.logic.on_app_destination(
            destination, self.map_bounds, self._nav2_ready(), self._now(),
            allow_private=allow_private, is_delivery=is_delivery,
        )
        if reason != GateReason.OK:
            response.accepted = False
            response.message = f"목적지 요청 거부: {reason.value}"
            self.get_logger().warn(
                f"{label} 요청 거부: map_id={request.map_id} "
                f"id={destination_id} reason={reason.value}"
            )
            return response
        if before != State.IDLE:
            self.get_logger().info(
                f"앱 선점 주행: {before.value} 를 취소하고 새 목적지로")
        self._run_actions(actions)
        self._delivery_dest_id = (
            destination_id if is_delivery and self._nav_active else None)
        response.accepted = self._nav_active
        response.message = (
            f"목적지 요청을 수락했습니다: {destination.name}"
            if response.accepted and destination is not None
            else "Nav2가 목적지 요청을 수락하지 않았습니다."
        )
        return response

    def _on_wait_spot_request(
        self,
        request: RequestDestination.Request,
        response: RequestDestination.Response,
    ) -> RequestDestination.Response:
        """대기 장소로 가보기. 안내 중(IDLE 아님)에는 거절한다 — 로직이 판단한다."""
        destination_id = self._check_destination_request(request, response)
        if destination_id is None:
            return response
        destination = self.destinations.get(destination_id)
        actions, reason = self.logic.on_app_wait_spot(
            destination, self.map_bounds, self._nav2_ready(), self._now())
        if reason != GateReason.OK:
            response.accepted = False
            response.message = f"대기 장소 가보기 거부: {reason.value}"
            self.get_logger().warn(
                f"대기 장소 가보기 거부: id={destination_id} reason={reason.value}")
            return response
        self._run_actions(actions)
        response.accepted = self._nav_active
        response.message = (
            f"대기 장소로 이동합니다: {destination.name}-대기"
            if response.accepted and destination is not None
            else "Nav2가 대기 장소 요청을 수락하지 않았습니다."
        )
        return response

    def _on_reload_destinations(
        self,
        _: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """새 catalog 전체를 검증한 뒤에만 현재 메모리 목록을 교체한다."""
        try:
            destinations = load_destinations(self._destinations_path)
        except (OSError, ValueError, TypeError) as exc:
            response.success = False
            response.message = f"목적지 reload 실패: {exc}"
            self.get_logger().error(response.message)
            return response
        self.destinations = destinations
        response.success = True
        response.message = f"목적지 {len(destinations)}개를 다시 불러왔습니다."
        self.get_logger().info(response.message)
        return response

    def _on_emergency(self, msg: EmergencyEvent) -> None:
        self.get_logger().warn(f"긴급어 수신: '{msg.keyword}' (원문: {msg.source_text})")
        actions = self.logic.on_emergency(msg.keyword, self._now())
        self._run_actions(actions)

    def _on_tipped(self, msg: Bool) -> None:
        tipped = bool(msg.data)
        was_tipped = self.logic.tipped
        actions = self.logic.on_tipped(tipped, self._now())
        if tipped and not was_tipped:
            self.get_logger().error(
                f"넘어짐 감지 -> state={self.logic.state.value} (세울 때까지 멘트 반복)")
        elif was_tipped and not tipped:
            self.get_logger().warn(
                f"넘어짐 풀림(세움) -> state={self.logic.state.value} (해제는 관리자 앱)")
        else:
            self.get_logger().info(f"넘어짐 신호: {'넘어짐' if tipped else '서 있음'}(지금 상태)")
        self._run_actions(actions)

    def _on_app_estop_state(self, msg: String) -> None:
        try:
            app_active = bool(json.loads(msg.data).get("app_active", False))
        except (ValueError, AttributeError):
            return
        self.logic.on_app_estop(app_active)

    def _on_estop(self, msg: Bool) -> None:
        active = bool(msg.data)
        changed = active != self._estop_active
        self._estop_active = active
        actions = self.logic.on_estop(self._estop_active, self._now())
        if actions:
            self.get_logger().warn(
                f"중앙 estop={self._estop_active} -> state={self.logic.state.value}"
            )
        elif changed:
            self.get_logger().info(
                f"중앙 estop={self._estop_active} -> state={self.logic.state.value}"
            )
        self._run_actions(actions)

    def _tick(self) -> None:
        before = self.logic.state
        status = self._poll_nav_status()
        distance = (
            self._nav_distance_remaining() if status == NavStatus.RUNNING else None
        )
        nav_ready = (self._nav2_ready()
                     if self.logic.dialog_state == DIALOG_PAUSED_HANDLE else True)
        dialog_before = self.logic.dialog_state
        actions = self.logic.on_tick(self._now(), status, distance, nav_ready=nav_ready)
        confirming = self.logic.confirming_dest_id
        active_dest_id = self.logic.active_destination.id if self.logic.active_destination else None
        aborted_name = confirm_abort_name(
            self._ledger_prev_confirming, confirming, before, self.logic.state,
            active_dest_id, self.destinations,
        )
        if aborted_name is not None:
            self._ledger.aborted_destination = aborted_name
            self._save_ledger()
        self._ledger_prev_confirming = confirming
        self._run_actions(actions)
        self._drain_obstacle_cues()
        self._obstacle_inputs_follow()
        if State.SEEKING in (before, self.logic.state) and before != self.logic.state:
            self._publish_robot_state()
        elif dialog_before != self.logic.dialog_state:
            self.get_logger().info(
                f"손잡이: {dialog_before} → {self.logic.dialog_state} "
                f"(활성={self.logic.handle_active})")
            self._publish_robot_state()

    def _publish_robot_state(self) -> None:
        msg = RobotState()
        floor_param = int(self.get_parameter("current_floor").value)
        building_param = str(self.get_parameter("current_building").value)
        msg.current_floor = floor_param if floor_param >= 0 else self._map_meta.floor
        msg.current_building = building_param or self._map_meta.building
        msg.is_moving = self.logic.state in (
            State.NAVIGATING, State.SEEKING,
            State.MOVING_TO_WAIT_SPOT, State.MOVING_BACK_TO_DEST)
        msg.is_paused = self.logic.state == State.PAUSED
        fields = state_fields(
            self._ledger, self.logic.dialog_state, self._robot_pose, self._pose_cov_xy,
            self.destinations, now_epoch=time.time(),
            wait_minutes=self.logic.wait_minutes_requested(),
            wait_left_sec=self.logic.wait_left_sec(self._now()),
            door_side=self.logic.door_side,
            wait_place=self.logic.wait_place,
        )
        for key, value in fields.items():
            setattr(msg, key, value)
        self.pub_state.publish(msg)

    def _run_actions(self, actions) -> None:
        for action in actions:
            if isinstance(action, Say):
                out = String()
                out.data = f"{action.priority}:{action.text}"
                self.pub_tts.publish(out)
                self.get_logger().info(f"TTS[{action.priority}]: {action.text}")
                if getattr(action, "expects_reply", False):
                    self.pub_listen_request.publish(Bool(data=True))
                    self.logic.note_question_asked(action.text, self._now())
                    self.get_logger().info("질문 발화 — 재청취 요청 발행")
            elif isinstance(action, StopSpeech):
                out = String()
                out.data = "control:stop"
                self.pub_tts.publish(out)
                self.get_logger().info("발화 큐 청소 (취소·선점)")
            elif isinstance(action, WakeReply):
                verdict = "listen" if action.listen else "ignore"
                seq = self._wake_reply_seq
                self.pub_wake_reply.publish(String(data=f"{verdict}:{seq}" if seq else verdict))
            elif isinstance(action, CancelNav):
                self._cancel_nav(action.destination, action.event)
            elif isinstance(action, Navigate):
                self._start_nav(action)
            elif isinstance(action, SpinInPlace):
                self._start_spin(action)
            elif isinstance(action, SetNavSpeedLimit):
                self._publish_nav_speed_limit(action.percent)
            elif isinstance(action, GoalEvent):
                self._publish_goal_event(
                    action.event, action.destination, action.reason,
                    extra={"wait_place": action.wait_place,
                           "wait_minutes": action.wait_minutes})
                self.get_logger().info(f"앱 알림: {action.event} {action.reason}")
            elif isinstance(action, HelpEvent):
                self._publish_help_event(action)
            elif isinstance(action, Haptic):
                out = String()
                out.data = action.pattern
                self.pub_haptic.publish(out)
                self.get_logger().info(f"진동 요청: {action.pattern}")

    def _publish_help_event(self, action: HelpEvent) -> None:
        """도움 요청 사건을 /vica_goal_event 로 낸다(목적지 칸은 빈 값). help_requested 는 보낼"""
        extra: dict = {"help_id": action.help_id}
        if action.event == "help_requested":
            pose = self._robot_pose
            place, _ = place_here(pose, self.destinations, self._pose_cov_xy)
            extra.update({
                "place": place,
                "robot_x": pose.x if pose is not None else None,
                "robot_y": pose.y if pose is not None else None,
            })
        self._publish_goal_event(action.event, None, extra=extra)
        self.get_logger().info(f"앱 알림: {action.event} {action.help_id} {extra.get('place', '')}")

    def _publish_nav_speed_limit(self, percent: float) -> None:
        msg = SpeedLimit()
        msg.percentage = True
        msg.speed_limit = float(percent)
        self.pub_speed_limit.publish(msg)
        if percent == 0.0:
            self.get_logger().info("Nav2 접근 속도 제한 해제")
        else:
            self.get_logger().info(f"Nav2 접근 속도 제한: 최대속도의 {percent:.1f}%")

    def _resolve_bt(self, value: str, file_name: str, label: str) -> str:
        """*_bt_xml 파라미터를 실제 파일 경로로 바꾼다. 못 쓰면 "" (끔 = 기본 트리)."""
        if not value:
            self.get_logger().info(f"{label}: 끔 — 기본 트리(레일)를 쓴다")
            return ""
        if value == "auto":
            try:
                from ament_index_python.packages import get_package_share_directory
                value = str(Path(get_package_share_directory("vica_nav2"))
                            / "behavior_trees" / file_name)
            except Exception as exc:  # noqa: BLE001 - 패키지가 없어도 노드는 뜬다
                self.get_logger().warn(
                    f"{label}를 못 찾아 끔 — 기본 트리(레일)를 쓴다: {exc}")
                return ""
        if not Path(value).is_file():
            self.get_logger().warn(
                f"{label} 파일이 없어 끔 — 기본 트리(레일)를 쓴다: {value}")
            return ""
        self.get_logger().info(f"{label}: {value}")
        return value

    def _finish_task_for_bt_switch(self) -> None:
        """트리가 바뀌는 goal 을 보내기 전에 앞 task 를 끝낸다. _nav_lock 안에서 부른다."""
        try:
            if self.navigator.isTaskComplete():
                return
            self.get_logger().info("트리가 바뀌는 goal — 앞 task 를 먼저 취소한다")
            self.navigator.cancelTask()
            deadline = time.monotonic() + BT_SWITCH_WAIT_SEC
            while not self.navigator.isTaskComplete():
                if time.monotonic() > deadline:
                    self.get_logger().warn(
                        f"앞 task 가 {BT_SWITCH_WAIT_SEC:.1f}초 안에 안 끝났다 — 그대로 보낸다")
                    return
        except Exception as exc:  # noqa: BLE001 - 보내기 자체는 막지 않는다
            self.get_logger().warn(f"트리 전환 전 취소 실패: {exc}")

    def _clear_costmaps_before_departure(self, dest_id: str) -> None:
        """새 출발 직전에 local·global costmap 을 한 번 비운다. _nav_lock 안에서 부른다."""
        try:
            if not self.navigator.isTaskComplete():
                self.get_logger().warn(
                    f"출발 전 costmap 비우기 건너뜀 — 앞 task 가 아직 돈다 ({dest_id})")
                return
            sent = []
            for name, client in (("local", self.navigator.clear_costmap_local_srv),
                                 ("global", self.navigator.clear_costmap_global_srv)):
                if not client.service_is_ready():
                    self.get_logger().warn(
                        f"출발 전 {name} costmap 비우기 건너뜀 — 서비스 없음 ({dest_id})")
                    continue
                sent.append((name, client, client.call_async(ClearEntireCostmap.Request())))
            deadline = time.monotonic() + CLEAR_BEFORE_DEPARTURE_TIMEOUT_SEC
            for name, client, future in sent:
                left = deadline - time.monotonic()
                if not future.done() and left > 0.0:
                    rclpy.spin_until_future_complete(self.navigator, future, timeout_sec=left)
                if not future.done():
                    client.remove_pending_request(future)
                    self.get_logger().warn(
                        f"출발 전 {name} costmap 비우기 응답 없음 "
                        f"({CLEAR_BEFORE_DEPARTURE_TIMEOUT_SEC:.1f} s) — 그대로 출발 ({dest_id})")
                    continue
                self.get_logger().info(f"출발 전 {name} costmap 비움 ({dest_id})")
        except Exception as exc:  # noqa: BLE001 - 보내기 자체는 막지 않는다
            self.get_logger().warn(f"출발 전 costmap 비우기 실패: {exc}")

    def _start_nav(self, action: Navigate) -> None:
        dest = action.destination
        if self._delivery_dest_id is not None and dest.id != self._delivery_dest_id:
            self._delivery_dest_id = None
        goal = PoseStamped()
        goal.header.frame_id = dest.pose.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = dest.pose.x
        goal.pose.position.y = dest.pose.y
        qx, qy, qz, qw = yaw_deg_to_quaternion(dest.pose.yaw_deg)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        if not self._nav2_ready():
            if not self._quiet_goal_events():
                self._publish_goal_event("goal_rejected", dest, "Nav2 가 준비되지 않았습니다.")
            self.get_logger().error(f"NavigateToPose 전송 보류: Nav2 미준비 ({dest.id})")
            self._run_actions(self.logic.on_tick(self._now(), NavStatus.FAILED))
            return
        if not self._nav_lock.acquire(timeout=self._nav_lock_timeout_sec):
            self._publish_goal_event(
                "goal_rejected", dest, "이전 goal 취소가 아직 처리 중입니다."
            )
            self.get_logger().error(
                f"NavigateToPose 전송 보류: 이전 취소가 "
                f"{self._nav_lock_timeout_sec:.1f}초 안에 끝나지 않았다 ({dest.id})"
            )
            self._run_actions(self.logic.on_tick(self._now(), NavStatus.FAILED))
            return
        bt = (nav_behavior_tree(dest.id, self._approach_bt)
              or self._tree_files.get(action.tree, ""))
        if action.tree == NAV_TREE_GUIDED and bt and not self._route_server_up():
            bt = self._guided_no_rail_bt
        quiet = self._quiet_goal_events()
        try:
            if bt != self._task_bt:
                self._finish_task_for_bt_switch()
            if self._clear_before_next_nav:
                self._clear_costmaps_before_departure(dest.id)
            accepted = self.navigator.goToPose(goal, behavior_tree=bt)
            self._nav_active = bool(accepted)
            if accepted:
                self._nav_gen += 1
                self._task_bt = bt
                self._clear_before_next_nav = False
                self._nav_task_is_spin = False
        finally:
            self._nav_lock.release()
        if accepted:
            if not quiet:
                self._publish_goal_event("goal_sent", dest)
                self._publish_goal_event("goal_accepted", dest)
            self.get_logger().info(
                f"NavigateToPose 전송: {dest.id} ({dest.pose.x:.2f}, {dest.pose.y:.2f}, "
                f"{dest.pose.yaw_deg:.1f}deg) 트리={Path(bt).name if bt else '기본'}"
            )
        else:
            if not quiet:
                self._publish_goal_event("goal_rejected", dest, "Nav2 goal rejected")
            self.get_logger().error(f"NavigateToPose goal 거부됨: {dest.id}")
            self._run_actions(self.logic.on_tick(self._now(), NavStatus.FAILED))

    def _start_spin(self, action: SpinInPlace) -> None:
        """제자리 회전. BasicNavigator.spin() 은 behavior server 의 Spin 을 부른다."""
        with self._nav_lock:
            accepted = self.navigator.spin(spin_dist=action.yaw_rad)
            self._nav_active = bool(accepted)
            if accepted:
                self._nav_gen += 1
                self._nav_task_is_spin = True
        if accepted:
            self.get_logger().info(
                f"제자리 회전 시작: {action.yaw_rad:+.2f} rad "
                f"({math.degrees(action.yaw_rad):+.0f}°) — {action.reason}"
            )
        else:
            self.get_logger().error(f"Spin 거부됨 ({action.reason}) - 회전 없이 넘어간다")
            self._run_actions(self.logic.on_tick(self._now(), NavStatus.FAILED))

    def _cancel_nav(self, destination=None, event: str = "goal_canceled") -> None:
        """goal 취소를 알리고, Nav2 취소 호출은 별도 스레드에 맡긴다."""
        if destination is not None:
            reason = (
                "일시정지 요청으로 목적지를 보관하고 멈췄습니다."
                if event == "goal_paused"
                else "비상정지 또는 Mission 요청으로 목적지가 취소되었습니다."
            )
            self._publish_goal_event(event, destination, reason)

        if not self._nav_active:
            return
        self._nav_active = False

        threading.Thread(
            target=self._cancel_nav_blocking,
            args=(self._nav_gen,),
            name="vica_nav_cancel",
            daemon=True,
        ).start()
        self.get_logger().warn(
            "Nav2 goal 취소 요청 (보조 경로 — 모터 정지 권위는 래치 체인)"
        )

    def _cancel_nav_blocking(self, gen: int) -> None:
        """콜백 밖에서 navigator.cancelTask() 를 부른다. 예외를 삼키지 않고 남긴다."""
        with self._nav_lock:
            if self._nav_gen != gen:
                self.get_logger().info(
                    "Nav2 취소 건너뜀 — 새 goal 이 이전 goal 을 이미 대체했다"
                )
                return
            try:
                self.navigator.cancelTask()
            except Exception as exc:  # noqa: BLE001 - 스레드가 조용히 죽지 않게 한다
                self.get_logger().error(f"Nav2 goal 취소 호출 실패: {exc}")
            else:
                self.get_logger().info("Nav2 goal 취소 완료")

    def _poll_nav_status(self) -> NavStatus:
        if not self._nav_active:
            return NavStatus.NONE
        with self._nav_lock:
            if not self.navigator.isTaskComplete(timeout_sec=NAV_POLL_WAIT_SEC):
                return NavStatus.RUNNING
            self._nav_active = False
            result = self.navigator.getResult()
        if not self._nav_task_is_spin and result == TaskResult.SUCCEEDED:
            self._clear_before_next_nav = True
        returning = self.logic.state == State.RETURNING
        if self._quiet_goal_events():
            if result == TaskResult.SUCCEEDED:
                return NavStatus.SUCCEEDED
            if result == TaskResult.CANCELED:
                return NavStatus.CANCELED
            return NavStatus.FAILED
        if result == TaskResult.SUCCEEDED:
            if returning:
                self._record_home_visit(True)
            if self.logic.active_destination is not None:
                self._publish_goal_event(
                    "return_home_succeeded" if returning else "goal_succeeded",
                    self.logic.active_destination,
                )
            return NavStatus.SUCCEEDED
        if result == TaskResult.CANCELED:
            if self.logic.active_destination is not None:
                self._publish_goal_event(
                    "return_home_canceled" if returning else "goal_canceled",
                    self.logic.active_destination,
                )
            return NavStatus.CANCELED
        if returning:
            self._record_home_visit(False)
        if self.logic.active_destination is not None:
            self._publish_goal_event(
                "return_home_failed" if returning else "goal_failed",
                self.logic.active_destination,
                "Nav2 task failed",
            )
        return NavStatus.FAILED

    def _route_server_up(self) -> bool:
        """레일 서버(compute_route 액션)가 떠 있는가. 모르면 False — 레일 없는 트리는"""
        try:
            from rclpy.action.graph import get_action_names_and_types
            names = get_action_names_and_types(self)
        except Exception:  # noqa: BLE001 - 그래프 조회 실패는 '없음'으로 본다
            return False
        return any(name.rstrip("/").endswith("compute_route") for name, _ in names)

    def _quiet_goal_events(self) -> bool:
        """지금 주행이 앱에 일반 주행 알림을 내지 않는 주행인가(대기 장소로 혼자 가는 길)."""
        return self.logic.state in (State.MOVING_TO_WAIT_SPOT, State.MOVING_BACK_TO_DEST)

    def _record_home_visit(self, visited_ok: bool) -> None:
        """복귀 결과를 home.yaml 에 남긴다. 실패해도 주행 판정을 막지 않는다."""
        try:
            self._home_storage.mark_visited(self._map_id, visited_ok)
        except (ValueError, OSError) as exc:
            self.get_logger().warn(f"홈 확인 상태를 기록하지 못했습니다: {exc}")

    def _nav_distance_remaining(self):
        """Nav2 feedback 의 남은 거리(m). 아직 없으면 None."""
        with self._nav_lock:
            feedback = self.navigator.getFeedback()
        distance = getattr(feedback, "distance_remaining", None)
        if distance is None:
            return None
        distance = float(distance)
        return distance if distance > 0.0 else None

    def _nav2_ready(self) -> bool:
        """Nav2 미준비 시 goal 거부용. goToPose 내부의 무한 대기를 피하려고"""
        try:
            return bool(self.navigator.nav_to_pose_client.server_is_ready())
        except AttributeError:
            return False

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_goal_event(self, event: str, destination, reason: str = "",
                            extra: Optional[dict] = None) -> None:
        """goal 사건을 앱에 알린다. destination 이 None 이면 좌표 없이 보낸다."""
        dest_id = destination.id if destination else ""
        dest_name = destination.name if destination else ""
        if apply_goal_event(self._ledger, event, dest_id, dest_name, time.time()):
            self._save_ledger()

        delivery_extra = {}
        if dest_id and dest_id == self._delivery_dest_id:
            if event == "goal_succeeded":
                delivery_extra = {"delivery": True}
            if event in ("goal_succeeded", "goal_failed", "goal_canceled", "goal_rejected"):
                self._delivery_dest_id = None

        msg = String()
        msg.data = json.dumps(
            {
                "event": event,
                "map_id": self._map_id,
                "location_id": destination.id if destination else "",
                "destination_id": destination.id if destination else "",
                "name": destination.name if destination else "",
                "x": destination.pose.x if destination else 0.0,
                "y": destination.pose.y if destination else 0.0,
                "yaw": destination.pose.yaw_deg if destination else 0.0,
                "reason": reason,
                "timestamp": datetime.now().isoformat(timespec="seconds"),
                **(extra or {}),
                **delivery_extra,
            },
            ensure_ascii=False,
        )
        self.pub_goal_event.publish(msg)

    def _save_ledger(self) -> None:
        """대장 파일 갱신. 실패는 경고만 — 파일 오류로 노드가 죽지 않는다."""
        if not self._ledger_store.write(self._ledger):
            self.get_logger().warning(f"대장 파일 쓰기 실패: {self._ledger_store.path}")


def main(args=None) -> None:
    """긴급 콜백이 일반 처리에 막히지 않도록 다중 스레드 executor로 실행한다."""
    rclpy.init(args=args)
    node = MissionManagerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
