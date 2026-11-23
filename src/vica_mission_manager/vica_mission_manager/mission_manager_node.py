#!/usr/bin/env python3
"""vica_mission_manager — VICA 음성→주행 통합의 유일한 신규 노드 (진행순서 ②)."""
from __future__ import annotations

import json
import math
import re
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
try:
    from vica_interfaces.msg import VicaQuestion
except ImportError:
    VicaQuestion = None
from vica_interfaces.srv import (
    DeleteHome,
    GetHome,
    MissionCommand,
    RequestApproach,
    RequestDestination,
    SaveHome,
)

try:
    from vica_interfaces.srv import GetPatrolRoute, SavePatrolRoute
except ImportError:
    GetPatrolRoute = SavePatrolRoute = None

from .approach_speed import DEFAULT_APPROACH_STAGES, stages_from_lists
from .destinations import load_destinations, load_home, load_map_bounds
from .approach_geometry import approach_goal
from .dock_flow import (CALL_CANCEL, CALL_START, CALL_UNDOCK, DOCK_FAR_COV, DOCK_FAR_M, DockFlow,
                        classify_start, parse_node_state, staging_relation)
from .dock_spot import DOCK_DESTINATION_ID, dock_yaml_path, read_dock_spot, staging_destination
from .home_storage import HOME_UNREACHABLE_MESSAGE, HomeStorage, build_home, near_home
from .ledger import (COV_UNKNOWN, Ledger, LedgerStore, apply_goal_event, confirm_abort_name,
                     place_here, state_fields)
from .map_meta import load_map_meta
from .mission_logic import (
    APPROACH_BT_FILE,
    APPROACH_TURN_YAW_DEG,
    ApproachRequest,
    CancelNav,
    DEST_RETRY_RETURN_SEC,
    DIALOG_PAUSED_HANDLE,
    Destination,
    GRIP_ENTER_WINDOW_SEC,
    GRIP_HINT_PULSE_SEC,
    GRIP_RATIO,
    GRIP_RELEASE_GRACE_SEC,
    GRIP_RESUME_WINDOW_SEC,
    GRIP_WAIT_TIMEOUT_SEC,
    GUIDED_BT_FILE,
    GUIDED_NO_RAIL_BT_FILE,
    GateReason,
    GoalEvent,
    HANDLE_LOST_GIVE_UP_SEC,
    HANDLE_LOST_REPEAT_SEC,
    HANDLE_SIDE_MIN_YAW_RAD,
    HANDLE_STATE_STALE_SEC,
    HOME_BEACON_INTERVAL_SEC,
    Haptic,
    HelpEvent,
    IntentData,
    MSG_APPROACH_ONBOARDING,
    MSG_APPROACH_REASK,
    MSG_DEST_RETRY,
    MSG_DOCK_GO,
    MissionLogic,
    NAV_TREE_GUIDED,
    NAV_TREE_WAIT,
    NEAR_CALL_MAX_M,
    NEAR_CALL_NO_SPIN_M,
    NavStatus,
    Navigate,
    PATROL_ALARM_KINDS,
    PATROL_ALARM_RESEND_SEC,
    PATROL_ARMING_TIMEOUT_SEC,
    PATROL_DESTINATION_PREFIX,
    PATROL_DETECTOR_TIMEOUT_SEC,
    PATROL_DWELL_SEC,
    PATROL_INTRUDER_REPEAT_SEC,
    PATROL_LIGHT_REPEAT_SEC,
    PATROL_MAX_LEG_FAILURES,
    PATROL_REJECT_TEXT,
    PATROL_SPEED_MPS,
    PATROL_START_REJECT_TEXT,
    PATROL_WATCH_KINDS,
    PERSON_APPROACH_SPEED_PERCENT,
    PatrolEvent,
    PatrolRecordClip,
    Pose2D,
    RETURN_RESUME_SEC,
    Say,
    SetNavSpeedCap,
    SetNavSpeedLimit,
    SpinInPlace,
    State,
    StopSpeech,
    WAIT_SPOT_BT_FILE,
    WakeReply,
    check_gate,
    doa_to_spin_yaw,
    nav_behavior_tree,
    switch_bundle_warning,
    yaw_deg_to_quaternion,
)
from .question_record import QuestionRecord, QuestionTracker, answer_for
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
from .patrol_storage import (
    ROUTE_IO_ERROR,
    ROUTE_NOTHING_WATCHED,
    ROUTE_OK,
    ROUTE_TOO_FEW,
    ROUTE_UNKNOWN_MAP,
    PatrolEventLog,
    PatrolRouteStorage,
    alarm_record,
    check_route_ids,
    clean_watch_off,
    clip_record,
    nearest_place_name,
)

PATROL_ALARM_ID_PATTERN = re.compile(r"^\d{8}_\d{6}_[a-z_]+$")

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

    _dock: Optional[DockFlow] = None

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
        self.declare_parameter("grip_release_stop", False)
        self.declare_parameter("speaker_gate", False)
        self.declare_parameter("spk_joined_threshold", 0.50)
        self.declare_parameter("speaker_gate_grip", False)
        self.declare_parameter("reask_ladder", False)
        self.declare_parameter("wake_revive", False)
        self.declare_parameter("answer_field", False)
        self.declare_parameter("home_beacon_interval_sec", HOME_BEACON_INTERVAL_SEC)
        self.declare_parameter("obstacle_narration", True)
        self.declare_parameter("grip_hint_pulse_sec", GRIP_HINT_PULSE_SEC)
        self.declare_parameter("grip_release_grace_sec", GRIP_RELEASE_GRACE_SEC)
        self.declare_parameter("grip_resume_window_sec", GRIP_RESUME_WINDOW_SEC)
        self.declare_parameter("handle_lost_repeat_sec", HANDLE_LOST_REPEAT_SEC)
        self.declare_parameter("handle_lost_give_up_sec", HANDLE_LOST_GIVE_UP_SEC)
        self.declare_parameter("handle_state_stale_sec", HANDLE_STATE_STALE_SEC)
        self.declare_parameter("patrol_dwell_sec", PATROL_DWELL_SEC)
        self.declare_parameter("patrol_intruder_repeat_sec", PATROL_INTRUDER_REPEAT_SEC)
        self.declare_parameter("patrol_light_repeat_sec", PATROL_LIGHT_REPEAT_SEC)
        self.declare_parameter("patrol_alarm_resend_sec", PATROL_ALARM_RESEND_SEC)
        self.declare_parameter("patrol_detector_timeout_sec", PATROL_DETECTOR_TIMEOUT_SEC)
        self.declare_parameter("patrol_arming_timeout_sec", PATROL_ARMING_TIMEOUT_SEC)
        self.declare_parameter("patrol_speed_mps", PATROL_SPEED_MPS)
        self.declare_parameter("patrol_max_leg_failures", PATROL_MAX_LEG_FAILURES)
        self.declare_parameter("patrol_media_dir", "~/workspaces/isaac_ros-dev/vica_patrol")

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
        self._home_watch: Optional[tuple[float, float]] = None

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
            grip_release_stop=bool(self.get_parameter("grip_release_stop").value),
            speaker_gate=bool(self.get_parameter("speaker_gate").value),
            spk_joined_threshold=float(self.get_parameter("spk_joined_threshold").value),
            speaker_gate_grip=bool(self.get_parameter("speaker_gate_grip").value),
            reask_ladder=bool(self.get_parameter("reask_ladder").value),
            wake_revive=bool(self.get_parameter("wake_revive").value),
            answer_field=bool(self.get_parameter("answer_field").value),
            home_beacon_interval_sec=float(
                self.get_parameter("home_beacon_interval_sec").value),
            **{name: float(self.get_parameter(name).value) for name in (
                "patrol_dwell_sec", "patrol_intruder_repeat_sec", "patrol_light_repeat_sec",
                "patrol_alarm_resend_sec", "patrol_detector_timeout_sec",
                "patrol_arming_timeout_sec", "patrol_speed_mps")},
            patrol_max_leg_failures=int(self.get_parameter("patrol_max_leg_failures").value),
        )
        self._patrol_routes = PatrolRouteStorage(Path(self._destinations_path).parent.parent)
        self._patrol_log = PatrolEventLog(str(self.get_parameter("patrol_media_dir").value))
        self.get_logger().info(
            "손잡이: 잡기 "
            f"{self.logic.grip_enter_window_sec:.1f}s 중 {self.logic.grip_ratio:.0%} · "
            f"대기 {self.logic.grip_wait_timeout_sec:.0f}s · "
            + (f"놓침 {self.logic.grip_release_grace_sec:.2f}s 뒤 정지" if self.logic.grip_release_stop
               else "주행 중 놓침 정지 꺼짐(터치는 처음 잡기만)"))
        if self.logic.grip_assume_held:
            self.get_logger().warn(
                f"손잡이 시연 스위치 켜짐: 대기 {self.logic.grip_wait_timeout_sec:.0f}s 뒤 "
                "잡은 것으로 넘어가고 손 놓침 정지를 쓰지 않는다(grip_assume_held)")
        bundle_warning = switch_bundle_warning(
            self.logic.answer_field, self.logic.reask_ladder, self.logic.wake_revive)
        if bundle_warning:
            self.get_logger().warn(bundle_warning)
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
            String, "/vica/listen_state", self._on_listen_state, 10,
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
        self.create_subscription(
            String, "/vica/speaker_mode", self._on_speaker_mode,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self._main_group,
        )

        self.pub_tts = self.create_publisher(String, "/vica/tts_request", 10)
        self.pub_listen_request = self.create_publisher(Bool, "/vica/listen_request", 10)
        self._questions = QuestionTracker(prefix=f"q{int(time.time())}")
        self._question_lock = threading.Lock()
        self.pub_question = None
        if VicaQuestion is not None:
            self.pub_question = self.create_publisher(
                VicaQuestion, "/vica/question",
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
        else:
            self.get_logger().warn("질문 기록 없음(옛 vica_interfaces) — /vica/question 발행 안 함")
        self._publish_question(None)
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
        self.create_subscription(
            String, "/vica/patrol/event", self._on_patrol_event, 10,
            callback_group=self._main_group,
        )
        self._pub_patrol_watch = self.create_publisher(
            String, "/vica/patrol/watch",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._publish_patrol_watch()
        for name, handler in (("patrol_start", self._on_patrol_start),
                              ("patrol_stop", self._on_patrol_stop),
                              ("patrol_ack", self._on_patrol_ack),
                              ("patrol_records_clear", self._on_patrol_records_clear)):
            self.create_service(
                MissionCommand, f"/vica/mission/{name}", handler,
                callback_group=self._main_group,
            )
        self._dock = DockFlow()
        self._dock_lock = threading.RLock()
        self._dock_far_check_at = 0.0
        self._dock_clients = {
            name: self.create_client(Trigger, f"/vica/dock/{name}", callback_group=self._main_group)
            for name in (CALL_START, CALL_UNDOCK, CALL_CANCEL)
        }
        self.create_subscription(
            String, "/vica/dock/state", self._on_dock_state,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self._main_group,
        )
        for name, handler in (("dock_start", self._on_dock_start),
                              ("dock_undock", self._on_dock_undock),
                              ("dock_cancel", self._on_dock_cancel)):
            self.create_service(
                MissionCommand, f"/vica/mission/{name}", handler,
                callback_group=self._main_group,
            )
        if SavePatrolRoute is not None:
            self.create_service(
                SavePatrolRoute, "/vica/patrol/route/save", self._on_save_patrol_route,
                callback_group=self._main_group,
            )
            self.create_service(
                GetPatrolRoute, "/vica/patrol/route/get", self._on_get_patrol_route,
                callback_group=self._main_group,
            )
        else:
            self.get_logger().warn(
                "순찰 경로 서비스 없음 — vica_interfaces 를 다시 빌드해야 한다(SavePatrolRoute·GetPatrolRoute)")
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

    def _on_speaker_mode(self, msg: String) -> None:
        before = self.logic.speaker_gate
        self.logic.set_speaker_mode(msg.data)
        if self.logic.speaker_gate != before:
            self.get_logger().info(f"[화자] 음성 모드 {msg.data!r} — 대기 중 본인 확인 "
                                   + ("켬" if self.logic.speaker_gate else "끔"))

    def _on_intent(self, msg: VicaIntent) -> None:
        """음성 요청. 판단은 로직(on_voice_intent)이 하고 여기선 실행·기록만 한다(2026-10-08)."""
        open_q = self._questions.current
        answer = answer_for(open_q.id if open_q else "", str(getattr(msg, "question_id", "")),
                            str(getattr(msg, "answer", "")))
        answer_value = str(getattr(msg, "answer_value", ""))
        answer_kind = open_q.kind if answer and open_q else ""
        answer_options = tuple(option[0] for option in open_q.options) if answer and open_q else ()
        intent = IntentData(
            intent=msg.intent,
            matched_destination_id=msg.matched_destination_id,
            need_confirm=msg.need_confirm,
            safety_flag=msg.safety_flag,
            wait_minutes=int(getattr(msg, "wait_minutes", -1)),
            reply=msg.reply,
            speaker_verdict=str(getattr(msg, "speaker_verdict", "")),
            speaker_score=float(getattr(msg, "speaker_score", -1.0)),
            speaker_joined=float(getattr(msg, "speaker_score_joined", -1.0)),
            ask_text=str(getattr(msg, "ask_text", "")),
            ask_kind=str(getattr(msg, "ask_kind", "")),
            ask_option_ids=tuple(getattr(msg, "ask_option_ids", ())),
            answer=answer, answer_value=answer_value if answer else "",
            answer_kind=answer_kind, answer_options=answer_options,
        )
        before = self.logic.state
        self._question_safely(self._close_question, "answer")
        actions = self.logic.on_voice_intent(
            intent, self._now(), self._lookup_destination,
            bounds=self.map_bounds, nav_ready=self._nav2_ready())
        self._run_actions(actions)
        said = [a.text for a in actions if isinstance(a, Say)]
        self.get_logger().info(
            f"음성 요청 intent={msg.intent} dest={msg.matched_destination_id or '-'} "
            f"confirm={msg.need_confirm}: {before.value} -> {self.logic.state.value}"
            + (f" answer={answer}" if answer else "")
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
        patrol = before.value.startswith("patrol")
        actions, reason = self._run_mission_command(command)
        if reason != GateReason.OK:
            self._run_actions(actions)
            response.accepted = False
            response.message = f"{command} 요청 거부: {reason.value}"
            if patrol and reason in PATROL_REJECT_TEXT:
                response.message = PATROL_REJECT_TEXT[reason]
            self.get_logger().warn(
                f"{command} 요청 거부: state={before.value} reason={reason.value}"
            )
            return response

        self._run_actions(actions)
        home_message = self._patrol_stop_follow_up() if patrol and command == "cancel" else ""
        if patrol and self.logic.state != before:
            self._publish_robot_state()
        response.accepted = True
        response.message = home_message or f"{command} 요청을 처리했습니다."
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

    def _patrol_follow(self, before: str, why: str) -> None:
        """순찰 상태가 바뀌었으면 한 줄 남기고 /vica/robot_state 를 곧바로 낸다."""
        after = self.logic.dialog_state
        if after == before:
            return
        self.get_logger().info(f"순찰({why}): {before} -> {after}")
        self._publish_robot_state()

    def _on_patrol_event(self, msg: String) -> None:
        """/vica/patrol/event — 순찰이 아니면 JSON 도 풀지 않는다."""
        if not self.logic.patrol_active:
            return
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(data, dict):
            return
        before = self.logic.dialog_state
        self._run_actions(self.logic.on_patrol_event(data, self._now()))
        self._patrol_follow(before, f"감지 {data.get('kind')}")

    def _load_patrol_route(self) -> tuple:
        """지금 지도의 순찰 경로 → (장소 목록 또는 None, 못 쓰는 이유 한 줄, 꺼 둔 감시 항목)."""
        try:
            ids, watch_off = self._patrol_routes.read(self._map_id)
        except ValueError:
            ids, watch_off = None, []
        if not ids:
            return None, "", watch_off
        code, message = check_route_ids(ids, self.destinations)
        if code == ROUTE_TOO_FEW:
            return None, "", watch_off
        if code != ROUTE_OK:
            return None, message, watch_off
        return [self.destinations[dest_id] for dest_id in ids], "", watch_off

    def _publish_patrol_watch(self, watch_off=None) -> None:
        """꺼 둔 감시 항목을 /vica/patrol/watch 로 알린다. None 이면 지금 지도의 파일에서 읽는다."""
        if watch_off is None:
            try:
                watch_off = self._patrol_routes.read(self._map_id)[1]
            except ValueError:
                watch_off = []
        self._pub_patrol_watch.publish(String(data=json.dumps({"off": list(watch_off)})))

    def _patrol_reply(self, response, reason: GateReason, ok_message: str, what: str):
        """순찰 서비스 응답. 거절 사유는 한국어 한 줄로 돌려준다."""
        response.accepted = reason == GateReason.OK
        response.message = (ok_message if response.accepted
                            else PATROL_REJECT_TEXT.get(reason, reason.value))
        if response.accepted:
            self.get_logger().info(f"{what}: {response.message}")
        else:
            self.get_logger().warn(f"{what} 거부: {reason.value}")
        return response

    def _on_patrol_start(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
    ) -> MissionCommand.Response:
        """경비 시작. patrol.yaml 을 읽어 로직에 넘긴다 — 수락하면 감지 노드 준비부터 기다린다."""
        if self._dock_refuse(response, "경비 시작"):
            return response
        route, problem, watch_off = self._load_patrol_route()
        pose_known = self._robot_pose is not None and self._pose_cov_xy <= COV_UNKNOWN
        before = self.logic.dialog_state
        actions, reason = self.logic.on_patrol_start(
            route, self.map_bounds, self._nav2_ready(), pose_known, self._now(), watch_off)
        if reason == GateReason.OK:
            self._publish_patrol_watch(watch_off)
            if watch_off:
                self.get_logger().info(f"경비 시작 — 꺼 둔 감시 항목: {', '.join(watch_off)}")
        self._run_actions(actions)
        self._patrol_follow(before, "시작")
        self._patrol_reply(response, reason, "경비를 시작합니다.", "경비 시작")
        if reason in PATROL_START_REJECT_TEXT:
            response.message = PATROL_START_REJECT_TEXT[reason]
        if reason == GateReason.NO_PATROL_ROUTE and problem:
            response.message = problem
        return response

    def _on_patrol_stop(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
    ) -> MissionCommand.Response:
        """경비 종료 → 홈으로 복귀(2026-11-14 사용자 결정). 홈이 없거나 복귀를 시작할 수 없으면"""
        before = self.logic.dialog_state
        actions, reason = self.logic.on_patrol_stop(self._now(), self._nav2_ready())
        self._run_actions(actions)
        home_message = self._patrol_stop_follow_up() if reason == GateReason.OK else ""
        self._patrol_follow(before, "종료")
        self._patrol_reply(response, reason, "경비를 종료했습니다.", "경비 종료")
        if home_message:
            response.message = home_message
        return response

    def _patrol_stop_follow_up(self) -> str:
        """관리자 경비 종료 뒤의 홈 복귀 마무리(로직 on_patrol_stop 의 행동을 실행한 다음 부른다)."""
        reason = self.logic.patrol_stop_home_reason
        self.logic.patrol_stop_home_reason = None
        if reason is None:
            return ""
        if reason != GateReason.OK:
            self.get_logger().warn(f"경비 종료 뒤 홈 복귀 거부: reason={reason.value}")
            return f"홈 복귀 거부: {reason.value}"
        home = self.logic.return_destination
        if home is not None:
            self._publish_goal_event("return_home_sent", home)
        self.get_logger().info("경비 종료 — 홈으로 복귀합니다.")
        return ""

    def _on_patrol_ack(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
    ) -> MissionCommand.Response:
        """경보 확인(음성 반복·재전송만 멈춘다). request_id 가 경보 id 모양이면 그 경보만,"""
        raw = str(request.request_id or "").strip()
        alarm_id = raw if PATROL_ALARM_ID_PATTERN.fullmatch(raw) else ""
        actions, reason = self.logic.on_patrol_ack(alarm_id, self._now())
        self._run_actions(actions)
        return self._patrol_reply(response, reason, "경보를 확인했습니다.", "경보 확인")

    def _on_patrol_records_clear(
        self,
        request: MissionCommand.Request,
        response: MissionCommand.Response,
    ) -> MissionCommand.Response:
        """경비 기록·영상·사진을 모두 지운다(앱 '알림 및 로그 > 경비 기록'의 삭제, 2026-11-16 사용자 요청)."""
        if self.logic.patrol_active:
            response.accepted = False
            response.message = "경비 중에는 기록을 지울 수 없습니다."
            self.get_logger().warn("경비 기록 삭제 거부: 경비 중")
            return response
        try:
            removed = self._patrol_log.clear()
        except OSError as exc:
            response.accepted = False
            response.message = "경비 기록을 지우지 못했습니다."
            self.get_logger().warn(f"경비 기록 삭제 실패: {exc}")
            return response
        response.accepted = True
        response.message = "경비 기록을 지웠습니다."
        self.get_logger().info(f"경비 기록 삭제: {removed}개 ({self._patrol_log.path.parent})")
        return response

    def _patrol_catalog(self, map_id: str):
        """그 지도의 목적지 catalog. 지금 지도면 메모리의 것, 다른 지도면 파일을 읽는다. 없으면 None."""
        if map_id == self._map_id:
            return self.destinations
        try:
            path = self._patrol_routes.path_for(map_id).with_name("destinations.yaml")
            return load_destinations(str(path)) if path.exists() else None
        except Exception:  # noqa: BLE001 — 깨진 catalog 는 '모르는 지도'로 본다
            return None

    def _on_save_patrol_route(self, request, response):
        """순찰 경로와 꺼 둔 감시 항목 저장. 장소가 빈 배열이면 경로를 지운다. reason 코드는 SPEC 3.2."""
        map_id = str(request.map_id or "").strip() or self._map_id
        ids = [str(dest_id).strip().lower() for dest_id in request.destination_ids]
        watch_off = clean_watch_off(list(request.watch_off))
        catalog = self._patrol_catalog(map_id)
        if catalog is None:
            code, message = ROUTE_UNKNOWN_MAP, f"지도를 찾을 수 없습니다: {map_id}"
        elif len(watch_off) == len(PATROL_WATCH_KINDS):
            code, message = ROUTE_NOTHING_WATCHED, "감시 항목은 하나 이상 켜 두어야 합니다."
        else:
            try:
                if not ids:
                    had_route = bool(self._patrol_routes.read(map_id)[0])
                    if watch_off:
                        self._patrol_routes.write(map_id, [], watch_off)
                    else:
                        self._patrol_routes.delete(map_id)
                    code = ROUTE_OK
                    message = "순찰 경로를 지웠습니다." if had_route else "지울 순찰 경로가 없었습니다."
                else:
                    code, message = check_route_ids(ids, catalog)
                    if code == ROUTE_OK:
                        self._patrol_routes.write(map_id, ids, watch_off)
                        message = f"순찰 경로를 저장했습니다(장소 {len(ids)}곳)."
            except (OSError, ValueError) as exc:
                code, message = ROUTE_IO_ERROR, f"순찰 경로 저장 실패: {exc}"
        response.accepted = code == ROUTE_OK
        response.reason = code
        response.message = message
        if response.accepted and map_id == self._map_id and not self.logic.patrol_active:
            self._publish_patrol_watch(watch_off)
        self.get_logger().info(
            f"순찰 경로 저장({map_id}, {len(ids)}곳, 끈 항목 {watch_off or '없음'}): {code} — {message}")
        return response

    def _on_get_patrol_route(self, request, response):
        """순찰 경로 읽기. 이름은 catalog 에서 찾고, 지워진 장소는 이름을 "" 로 둔다."""
        map_id = str(request.map_id or "").strip() or self._map_id
        response.found = False
        response.map_id = map_id
        response.destination_ids = []
        response.names = []
        response.watch_off = []
        try:
            ids, response.watch_off = self._patrol_routes.read(map_id)
        except ValueError:
            response.message = f"지도를 찾을 수 없습니다: {map_id}"
            return response
        if not ids:
            response.message = "저장된 순찰 경로가 없습니다."
            return response
        catalog = self._patrol_catalog(map_id) or {}
        names = [catalog[dest_id].name if dest_id in catalog else "" for dest_id in ids]
        response.found = True
        response.destination_ids = ids
        response.names = names
        response.message = "" if all(names) else "경로에 지워진 장소가 있습니다. 경로를 다시 저장하세요."
        return response

    def _patrol_record(self, record: dict) -> None:
        """경보 기록 파일에 한 줄 덧붙인다. 실패는 경고만 — 순찰은 계속한다."""
        if not self._patrol_log.append(record):
            self.get_logger().warn(f"경보 기록 쓰기 실패: {self._patrol_log.path}")

    def _publish_patrol_event(self, action: PatrolEvent) -> None:
        """순찰 사건을 /vica_goal_event 로 낸다(SPEC 3.3)."""
        extra: dict = {}
        if action.event == "patrol_started":
            extra["route"] = list(action.route)
        elif action.event == "patrol_waypoint":
            extra.update({"index": action.index, "lap": action.lap, "count": action.count})
        elif action.event == "patrol_acked":
            extra["alarm_id"] = action.alarm_id
        elif action.event == "patrol_stopped":
            extra["return_home"] = action.return_home
        elif action.kind in PATROL_ALARM_KINDS:
            pose = self._robot_pose
            place = nearest_place_name(pose, self.destinations)
            robot_x = pose.x if pose is not None else None
            robot_y = pose.y if pose is not None else None
            extra.update({"alarm_id": action.alarm_id, "place": place,
                          "robot_x": robot_x, "robot_y": robot_y, "resend": action.resend})
            if action.clip_id:
                extra["clip_id"] = action.clip_id
            if action.photo:
                extra["photo"] = action.photo
            if action.level_db is not None:
                extra["level_db"] = action.level_db
            if not action.resend:
                self._patrol_record(alarm_record(
                    action.alarm_id, action.kind, action.time, place, robot_x, robot_y,
                    self._map_id, action.clip_id, action.photo, action.level_db))
        self._publish_goal_event(action.event, action.destination, action.reason, extra=extra)
        self.get_logger().info(
            f"앱 알림: {action.event} {action.alarm_id or action.reason}"
            + (" (재전송)" if action.resend else ""))

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
        self._home_watch = (
            (home.x, home.y)
            if home is not None and (not home.visited_ok or home.reach_failed)
            else None
        )
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
        response.message = HOME_UNREACHABLE_MESSAGE if home.reach_failed else ""
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
        if self._dock_refuse(response, "홈 복귀"):
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
        self._check_home_reached()

    def _robot_near_home(self) -> bool:
        """위치를 제대로 잡은 채 홈 30 cm 안에 있는가."""
        home = self.logic.return_destination
        pose = self._robot_pose
        if home is None or pose is None or self._pose_cov_xy > COV_UNKNOWN:
            return False
        return near_home(home.pose.x, home.pose.y, pose.x, pose.y)

    def _check_home_reached(self) -> None:
        """홈 30 cm 안에 들어오면 '가 봄'으로 적는다(11-19 사용자 — 주행에서 어떻게든 홈에 한 번"""
        watch = self._home_watch
        pose = self._robot_pose
        if watch is None or pose is None or self._pose_cov_xy > COV_UNKNOWN:
            return
        if near_home(watch[0], watch[1], pose.x, pose.y):
            self._home_watch = None
            self.get_logger().info("홈 30 cm 안에 들어와 '가 봄'으로 기록합니다.")
            self._record_home_visit(True)

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
        if self._dock is not None and (self._dock.mode or self._dock.undock_pending
                                       or self._dock.near_dock):
            response.accepted = False
            response.message = "도킹 중이라 접근하지 않습니다."
            return response
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
        if command == "cancel" and self.logic.state.value.startswith("patrol"):
            return self.logic.on_patrol_stop(now, self._nav2_ready())
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
        if self._dock_refuse(response, label):
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
        sent = self._nav_active or self._dock_holding()
        self._delivery_dest_id = destination_id if is_delivery and sent else None
        response.accepted = sent
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
        if self._dock_refuse(response, "대기 장소 가보기"):
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
        response.accepted = self._nav_active or self._dock_holding()
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
        if self._dock is not None and active and self._dock.phase == "going":
            if self._dock_lock.acquire(blocking=False):
                try:
                    self._dock_apply(self._dock.on_estop(True))
                finally:
                    self._dock_lock.release()
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
        self._question_safely(self._tick_question)
        before = self.logic.state
        status = self._poll_nav_status()
        if self._dock is not None and self._dock.owns_nav():
            with self._dock_lock:
                self._dock_apply(self._dock.on_nav_status(status.value))
            status = NavStatus.NONE
        if self._dock is not None and self._estop_active and self._dock.phase == "going":
            with self._dock_lock:
                self._dock_apply(self._dock.on_estop(True))
        if self._dock is not None and self._now() >= self._dock_far_check_at:
            self._dock_far_check_at = self._now() + 1.0
            self._dock_far_check()
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
        elif "patrol" in (dialog_before[:6], self.logic.dialog_state[:6]):
            self._patrol_follow(dialog_before, "틱")
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
            State.MOVING_TO_WAIT_SPOT, State.MOVING_BACK_TO_DEST, State.PATROLLING)
        msg.is_paused = self.logic.state in (State.PAUSED, State.PATROL_PAUSED)
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

    def _on_listen_state(self, msg: String) -> None:
        """음성의 듣기 결과. 빈손으로 끝났으면 열린 질문을 먼저 닫는다 — 뒤이은 되묻기가 새 질문을 연다."""
        self._question_safely(self._listen_question, msg.data)
        self._run_actions(self.logic.on_listen_state(msg.data, self._now()))

    def _question_safely(self, fn, *args) -> None:
        """질문 기록은 기록일 뿐이다 — 여기서 난 예외가 미션의 말·동작을 막거나 노드를 죽이지 않게 한다."""
        try:
            fn(*args)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"[질문] 기록 실패 — 미션은 계속: {exc!r}")

    def _listen_question(self, state: str) -> None:
        with self._question_lock:
            closed = self._questions.on_listen_state(state, self._now())
        if closed is not None:
            self._close_question("silence", closed)

    def _tick_question(self) -> None:
        """상태가 바뀌었거나 음성에서 아무 신호 없이 오래 열려 있으면 닫는다."""
        with self._question_lock:
            closed = self._questions.tick(self.logic.state.value, self._now())
        if closed is not None:
            self._close_question(closed[1], closed[0])

    def _publish_question(self, record: Optional[QuestionRecord]) -> None:
        """기록을 /vica/question 으로 낸다. None 이면 빈 기록(열린 질문 없음)."""
        if self.pub_question is None:
            return
        msg = VicaQuestion()
        if record is not None:
            msg.id, msg.kind, msg.text = record.id, record.kind, record.text
            msg.option_ids = [o[0] for o in record.options]
            msg.option_names = [o[1] for o in record.options]
            msg.step, msg.on_silence = record.step, record.on_silence
            msg.opened_at = self.get_clock().now().to_msg()
        self.pub_question.publish(msg)

    def _open_question(self, say: Say) -> None:
        if not say.kind:
            self.get_logger().warn(f"[질문] kind 없음 — open 으로 냄: {say.text}")
        with self._question_lock:
            old, record = self._questions.open(say, self.logic.state.value, self._now())
        if old is not None:
            self._log_question_closed(old, "replaced")
        self._publish_question(record)
        self.get_logger().info(
            f"[질문] 열림 id={record.id} kind={record.kind} 후보={len(record.options)} "
            f"요청={'y' if say.expects_reply else 'n'} 글자={record.text}"
            + (f" 되묻기={record.step}" if record.step else ""))

    def _close_question(self, reason: str, record: Optional[QuestionRecord] = None) -> None:
        """열린 질문을 닫고 빈 기록을 낸다. record 를 주면 이미 닫힌 것을 알리기만 한다."""
        if record is None:
            with self._question_lock:
                record = self._questions.close(reason)
        if record is None:
            return
        self._log_question_closed(record, reason)
        self._publish_question(None)

    def _log_question_closed(self, record: QuestionRecord, reason: str) -> None:
        text = f"[질문] 닫힘 id={record.id} 사유={reason} 열린 시간={self._now() - record.opened_at:.1f}s"
        if reason == "no_signal":
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    def _run_actions(self, actions) -> None:
        actions = list(actions)
        for index, action in enumerate(actions):
            if isinstance(action, Say):
                out = String()
                out.data = f"{action.priority}:{action.text}"
                self.pub_tts.publish(out)
                self.get_logger().info(f"TTS[{action.priority}]: {action.text}")
                if action.kind or getattr(action, "expects_reply", False):
                    self._question_safely(self._open_question, action)
                if getattr(action, "expects_reply", False):
                    self.pub_listen_request.publish(Bool(data=True))
                    self.logic.note_question_asked(action.question_text or action.text, self._now())
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
                if self._dock is not None and self._dock.held is not None:
                    follows = any(isinstance(a, (Navigate, SpinInPlace)) for a in actions[index + 1:])
                    if self._dock_lock.acquire(timeout=0.2):
                        try:
                            self._dock_apply(self._dock.on_mission_cancel(keep_leaving=follows))
                        finally:
                            self._dock_lock.release()
                    else:
                        self.get_logger().warn("도킹 잠금을 0.2 s 안에 못 잡아 잡아 둔 주행 취소를 건너뛴다")
                self._cancel_nav(action.destination, action.event)
            elif isinstance(action, Navigate):
                if not self._dock_hold(action):
                    self._start_nav(action)
            elif isinstance(action, SpinInPlace):
                if not self._dock_hold(action):
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
            elif isinstance(action, PatrolEvent):
                self._publish_patrol_event(action)
            elif isinstance(action, PatrolRecordClip):
                self._patrol_record(clip_record(action.alarm_id, action.clip_id))
            elif isinstance(action, SetNavSpeedCap):
                self._publish_nav_speed_cap(action.mps)
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

    def _publish_nav_speed_cap(self, mps: float) -> None:
        """순찰 구간 속도 상한(m/s). 같은 /speed_limit 을 절대값으로 낸다 — 푸는 것은 0.0(위)."""
        msg = SpeedLimit()
        msg.percentage = False
        msg.speed_limit = float(mps)
        self.pub_speed_limit.publish(msg)
        self.get_logger().info(f"Nav2 속도 상한(순찰): {mps:.2f} m/s")

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

    def _patrol_leg_stale(self) -> bool:
        """순찰 구간을 보내려는데 순찰이 이미 끝났나(비상정지 스레드가 그 사이 끼어든 경우)."""
        return self.logic.state != State.PATROLLING or self.logic.estop_active

    def _start_nav(self, action: Navigate) -> None:
        dest = action.destination
        patrol_leg = dest.id.startswith(PATROL_DESTINATION_PREFIX)
        dock_nav = dest.id == DOCK_DESTINATION_ID
        if patrol_leg and self._patrol_leg_stale():
            self.get_logger().warn(f"순찰 구간 전송 안 함 — 그 사이 순찰이 끝났다 ({dest.id})")
            return
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
            if not (patrol_leg or dock_nav or self._quiet_goal_events()):
                self._publish_goal_event("goal_rejected", dest, "Nav2 가 준비되지 않았습니다.")
            self.get_logger().error(f"NavigateToPose 전송 보류: Nav2 미준비 ({dest.id})")
            self._nav_send_failed(dock_nav)
            return
        if not self._nav_lock.acquire(timeout=self._nav_lock_timeout_sec):
            if not (patrol_leg or dock_nav):
                self._publish_goal_event(
                    "goal_rejected", dest, "이전 goal 취소가 아직 처리 중입니다."
                )
            self.get_logger().error(
                f"NavigateToPose 전송 보류: 이전 취소가 "
                f"{self._nav_lock_timeout_sec:.1f}초 안에 끝나지 않았다 ({dest.id})"
            )
            self._nav_send_failed(dock_nav)
            return
        bt = (nav_behavior_tree(dest.id, self._approach_bt)
              or self._tree_files.get(action.tree, ""))
        if action.tree == NAV_TREE_GUIDED and bt and not self._route_server_up():
            bt = self._guided_no_rail_bt
        quiet = patrol_leg or dock_nav or self._quiet_goal_events()
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
        if accepted and patrol_leg and self._patrol_leg_stale():
            self.get_logger().warn(f"순찰 구간 전송 직후 취소 — 그 사이 순찰이 끝났다 ({dest.id})")
            self._cancel_nav(None)
            return
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
            self._nav_send_failed(dock_nav)

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

    _DOCK_REPLY = {
        "ok": "도킹을 시작합니다.",
        "estop": "비상정지 중에는 도킹할 수 없습니다.",
        "already_docked": "이미 도크에 들어가 있습니다.",
        "busy": "다른 일을 하는 중이라 지금은 도킹할 수 없습니다.",
        "no_dock": "도크 위치가 저장되지 않았습니다. 지도 설정에서 먼저 저장하세요.",
        "nav_not_ready": "Nav2 가 준비되지 않았습니다.",
        "not_docked": "도크에 들어가 있지 않습니다.",
        "idle": "멈출 도킹이 없습니다.",
    }

    def _nav_send_failed(self, dock_nav: bool) -> None:
        """goal 을 못 보냈다 — 다음 tick 이 아니라 지금 실패로 넘긴다."""
        if dock_nav:
            self._dock_apply(self._dock.on_nav_status("failed"))
        else:
            self._run_actions(self.logic.on_tick(self._now(), NavStatus.FAILED))

    def _dock_apply(self, cmds: list) -> None:
        """도킹 흐름이 낸 명령을 실행하고, 로직에 도킹 단계를 알린다(dialog_state·음성 막기)."""
        with self._dock_lock:
            before = self.logic.dock_mode
            self.logic.dock_mode = self._dock.mode
            for c in cmds:
                if c.kind == "call":
                    self._dock_call(c.arg)
                elif c.kind == "nav":
                    self._start_nav(Navigate(c.arg))
                elif c.kind == "restage":
                    staging = self._dock_staging()
                    in_zone, near_spin = self._dock_near(staging)
                    self.get_logger().info("도킹 재시도: 도크 위치를 다시 읽어 준비 위치로")
                    self._dock_apply(self._dock.restage(staging, near_spin, in_zone))
                elif c.kind == "spin":
                    self._start_spin(SpinInPlace(c.arg, reason="도킹: 제자리에서 도크 보기"))
                    if not self._nav_active:
                        self._dock_apply(self._dock.on_nav_status("failed"))
                elif c.kind == "cancel_nav":
                    self._cancel_nav(None)
                elif c.kind == "release_held" and self._estop_active:
                    self.get_logger().warn("도크에서 나왔지만 비상정지 중 — 잡아 둔 주행을 취소로 끝낸다")
                    self._run_actions(self.logic.on_dock_held_dropped(self._now()))
                elif c.kind == "release_held":
                    self.get_logger().info("도크에서 나왔다 — 잡아 둔 주행을 보낸다")
                    if isinstance(c.arg, SpinInPlace):
                        self._start_spin(c.arg)
                    else:
                        self._start_nav(c.arg)
                elif c.kind == "drop_held":
                    self.get_logger().warn("도크에서 나오지 못해 잡아 둔 주행을 취소로 끝낸다")
                    self._run_actions(self.logic.on_dock_held_dropped(self._now()))
                elif c.kind == "event":
                    self._publish_goal_event(c.arg, self._dock.staging, c.message)
                    self.get_logger().info(f"도킹 사건 {c.arg} {c.message}")
            self.logic.dock_mode = self._dock.mode
            self.logic.dock_pending = self._dock.undock_pending and not self._dock.mode
            self.logic.dock_near = (self._dock.near_dock and not self._dock.mode
                                    and not self._dock.undock_pending)
            if self.logic.dock_mode != before:
                self.get_logger().info(f"도킹 단계: '{before}' → '{self.logic.dock_mode}'")
                self._publish_robot_state()

    def _dock_hold(self, action) -> bool:
        """미션 주행이 나가려 한다 — 도킹 흐름이 가로챌지 정한다. 가로챘으면(잡아 둠·버림) True."""
        if self._dock is None:
            return False
        if isinstance(action, Navigate) and action.destination.id == DOCK_DESTINATION_ID:
            return False
        is_spin = isinstance(action, SpinInPlace)
        with self._dock_lock:
            verdict, cmds = self._dock.intercept(action, is_spin)
            if verdict == "pass":
                if self._dock.undock_pending and not self._dock.busy():
                    self.get_logger().info("나오다 멈춘 자리에서 원격 주행 — 그대로 보낸다")
                    self._dock_call(CALL_CANCEL)
                return False
            if verdict == "held":
                self.get_logger().info("도크 안·나오는 중 — 도크에서 나온 뒤 보낸다")
                self._dock_apply(cmds)
                return True
            phase = self._dock.phase or "도크 앞"
        self.get_logger().warn(
            f"도킹 관련 중({phase})이라 {'제자리 회전' if is_spin else '주행'}을 보내지 않는다")
        self._run_actions(self.logic.on_tick(self._now(), NavStatus.FAILED) if is_spin
                          else self.logic.on_dock_held_dropped(self._now()))
        return True

    def _dock_holding(self) -> bool:
        """도킹 흐름이 미션 주행을 잡아 두었는가(도크에서 나온 뒤 보낸다) — 요청은 받아들인 것이다."""
        return self._dock is not None and self._dock.held is not None

    _DOCK_REFUSE_DOCKING = "도킹 중입니다. '도킹 중지'를 누른 뒤 다시 보내 주세요."
    _DOCK_REFUSE_STOPPING = "도크에서 나오다 멈추는 중입니다. 잠시 뒤 다시 보내 주세요."

    def _dock_refuse(self, response, label: str) -> bool:
        """도킹 노드가 로봇을 움직이는 중이라 원격 주행을 받을 수 없으면 거절 응답을 채우고 True."""
        d = self._dock
        if d is None:
            return False
        if d.phase in ("going", "entering"):
            message = self._DOCK_REFUSE_DOCKING
        elif d.phase == "leaving" and d.canceling:
            message = self._DOCK_REFUSE_STOPPING
        else:
            return False
        response.accepted = False
        response.message = message
        self.get_logger().warn(f"{label} 요청 거부: 도킹 흐름 {d.phase}")
        return True

    def _dock_call(self, name: str) -> None:
        cli = self._dock_clients[name]
        if not cli.service_is_ready():
            self.get_logger().error(f"/vica/dock/{name} 없음 — vica_docking 노드가 떠 있지 않다")
            self._dock_apply(self._dock.on_call_result(name, False, "dock_node_down"))
            return
        fut = cli.call_async(Trigger.Request())
        fut.add_done_callback(lambda f, n=name: self._on_dock_call_done(n, f))

    def _on_dock_call_done(self, name: str, fut) -> None:
        try:
            res = fut.result()
            ok, msg = bool(res.success), str(res.message)
        except Exception as exc:  # noqa: BLE001
            ok, msg = False, f"call_failed: {exc!r}"
        if not ok:
            self.get_logger().warn(f"/vica/dock/{name} 거절: {msg}")
        with self._dock_lock:
            self._dock_apply(self._dock.on_call_result(name, ok, msg))

    def _on_dock_state(self, msg: String) -> None:
        st = parse_node_state(msg.data)
        if st is None:
            return
        with self._dock_lock:
            self._dock_apply(self._dock.on_node_state(st))

    def _dock_staging(self):
        spot = read_dock_spot(dock_yaml_path(self._destinations_path))
        if spot is None or spot.map_id != self._map_id:
            return None
        return staging_destination(spot)

    def _dock_near(self, staging) -> tuple:
        """(도크 구역 안인가, 준비 위치 바로 옆이면 도크를 보려고 돌 각 rad·아니면 None). 위치를 모르면 (False, None)."""
        pose = getattr(self, "_robot_pose", None)
        if staging is None or pose is None:
            return False, None
        along, lateral, turn = staging_relation(
            pose.x, pose.y, pose.yaw_deg, staging.pose.x, staging.pose.y, staging.pose.yaw_deg)
        kind = classify_start(along, lateral)
        return kind == "zone", (turn if kind == "spin" else None)

    def _dock_far_check(self) -> None:
        """도크 근처·나오다 멈춤 표시가 낡았는가(1 s 마다). 도킹 노드는 일이 끝날 때만 이 표시를 고친다 — 그 뒤 앱 수동"""
        d = self._dock
        if d is None or d.busy() or d.mode or not (d.near_dock or d.undock_pending):
            return
        pose = getattr(self, "_robot_pose", None)
        if pose is None or self._pose_cov_xy > DOCK_FAR_COV:
            return
        spot = read_dock_spot(dock_yaml_path(self._destinations_path))
        if spot is None or spot.map_id != self._map_id:
            return
        dist = math.hypot(pose.x - spot.apex_x, pose.y - spot.apex_y)
        if dist <= DOCK_FAR_M:
            return
        with self._dock_lock:
            if d.busy() or d.mode:
                return
            self.get_logger().info(f"도크에서 {dist:.1f} m 떨어졌다 — 도크 근처·나오다 멈춤 표시를 내린다")
            self._dock_call(CALL_CANCEL)
            d.undock_pending = False
            d.near_dock = False
            self._dock_apply([])

    def _on_dock_start(self, request: MissionCommand.Request,
                       response: MissionCommand.Response) -> MissionCommand.Response:
        """앱 '도킹 시작'(목업 62). 도크 앞 1 m 까지 Nav2 로 간 뒤 들어간다(바로 옆이면 제자리 회전)."""
        with self._dock_lock:
            staging = self._dock_staging()
            in_zone, near_spin = self._dock_near(staging)
            ok, reason, cmds = self._dock.request_start(
                self.logic.state == State.IDLE, self._estop_active, self._nav2_ready(),
                staging, near_spin, in_zone)
            self._dock_apply(cmds)
        if ok:
            self._run_actions([Say(MSG_DOCK_GO, priority="narration")])
        response.accepted = ok
        response.message = self._DOCK_REPLY.get(reason, reason)
        self.get_logger().info(f"앱 도킹 시작: {reason}")
        return response

    def _on_dock_undock(self, request: MissionCommand.Request,
                        response: MissionCommand.Response) -> MissionCommand.Response:
        """앱 '도크에서 나오기'(목업 67). 후진 → 제자리 회전 → 주행 모드(그 자리)."""
        with self._dock_lock:
            ok, reason, cmds = self._dock.request_undock(
                self.logic.state == State.IDLE, self._estop_active)
            self._dock_apply(cmds)
        response.accepted = ok
        response.message = "도크에서 나옵니다." if ok else self._DOCK_REPLY.get(reason, reason)
        self.get_logger().info(f"앱 도크에서 나오기: {reason}")
        return response

    def _on_dock_cancel(self, request: MissionCommand.Request,
                        response: MissionCommand.Response) -> MissionCommand.Response:
        """앱 '도킹 중지'·'나오기 중지'(목업 65). 그 자리에 멈추고 다시 하지 않는다."""
        with self._dock_lock:
            ok, reason, cmds = self._dock.request_cancel()
            self._dock_apply(cmds)
        response.accepted = ok
        response.message = "멈춥니다." if ok else self._DOCK_REPLY.get(reason, reason)
        self.get_logger().info(f"앱 도킹 중지: {reason}")
        return response

    def _quiet_goal_events(self) -> bool:
        """지금 주행이 앱에 일반 주행 알림을 내지 않는 주행인가(대기 장소로 혼자 가는 길·순찰 구간)."""
        return self.logic.state in (
            State.MOVING_TO_WAIT_SPOT, State.MOVING_BACK_TO_DEST, State.PATROLLING)

    def _record_home_visit(self, visited_ok: bool) -> None:
        """복귀 결과를 home.yaml 에 남긴다. 실패해도 주행 판정을 막지 않는다."""
        reached = visited_ok or self._robot_near_home()
        try:
            home = self._home_storage.record_reach(self._map_id, reached)
        except (ValueError, OSError) as exc:
            self.get_logger().warn(f"홈 확인 상태를 기록하지 못했습니다: {exc}")
            return
        self._home_watch = (
            None if home is None or (home.visited_ok and not home.reach_failed) else (home.x, home.y)
        )

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
