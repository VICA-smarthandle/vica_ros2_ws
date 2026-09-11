#!/usr/bin/env python3
"""vica_mission_manager — VICA 음성→주행 통합의 유일한 신규 노드 (진행순서 ②)."""
from __future__ import annotations

import json
import math
import threading
from datetime import datetime
from pathlib import Path
from uuid import UUID

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.msg import SpeedLimit
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from vica_interfaces.msg import EmergencyEvent, RobotState, VicaIntent
from vica_interfaces.msg import PersonDetection
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
from .mission_logic import (
    HANDLE_SIDE_MIN_YAW_RAD,
    Haptic,
    MSG_APPROACH_ONBOARDING,
    MSG_APPROACH_QUESTION,
    MSG_DEST_RETRY,
    MSG_HANDLE_HINT,
    NEAR_CALL_MAX_M,
    NEAR_CALL_NO_SPIN_M,
    PERSON_APPROACH_SPEED_PERCENT,
    RETURN_RESUME_SEC,
    DEST_RETRY_RETURN_SEC,
    ApproachRequest,
    CancelNav,
    Destination,
    GateReason,
    IntentData,
    MissionLogic,
    Navigate,
    NavStatus,
    Say,
    StopSpeech,
    SetNavSpeedLimit,
    SpinInPlace,
    Pose2D,
    State,
    _REJECT_MESSAGES,
    check_gate,
    doa_to_spin_yaw,
    yaw_deg_to_quaternion,
)

HOME_DESTINATION_ID = "__home__"

try:
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
except ImportError:
    BasicNavigator = None
    TaskResult = None


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
        self.declare_parameter("approach_turn_yaw_deg", 180.0)
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

        dest_path = str(self.get_parameter("destinations_yaml").value)
        if not dest_path:
            raise RuntimeError("destinations_yaml parameter 가 비어 있습니다 (launch 에서 절대경로 지정)")
        self._destinations_path = str(Path(dest_path).expanduser())
        configured_map_id = str(self.get_parameter("map_id").value).strip()
        self._map_id = configured_map_id or Path(self._destinations_path).parent.name
        self.destinations = load_destinations(self._destinations_path)
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
        )
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
        self.navigator = BasicNavigator("vica_mission_navigator")
        self._nav_lock = threading.Lock()
        self._nav_lock_timeout_sec = 2.0
        self._nav_active = False
        self._nav_gen = 0

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

        self.pub_tts = self.create_publisher(String, "/vica/tts_request", 10)
        self.pub_listen_request = self.create_publisher(Bool, "/vica/listen_request", 10)
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
        if (self.logic.state == State.RETURNING
                and msg.intent in ("wait", "navigate")):
            self._run_actions(self.logic.on_return_brake(self._now(), quiet=True))
            self.get_logger().info(f"복귀 중 답 도착({msg.intent}) — 복귀 취소")

        if self.logic.is_awaiting_arrival_answer():
            if msg.intent == "navigate" and msg.need_confirm:
                self.logic.exit_arrival_dialog()
            else:
                self._on_arrival_answer(msg)
                return
        if msg.intent in ("cancel", "pause", "resume"):
            self._on_voice_mission_command(msg)
            return
        if msg.intent in ("affirm", "deny"):
            if self.logic.state == State.CONFIRMING:
                self._on_confirm_answer(msg.intent == "affirm")
            else:
                self._on_voice_answer(msg.intent == "affirm")
            return

        intent = IntentData(
            intent=msg.intent,
            matched_destination_id=msg.matched_destination_id,
            need_confirm=msg.need_confirm,
            safety_flag=msg.safety_flag,
            wait_minutes=int(getattr(msg, "wait_minutes", -1)),
        )
        dest = self.destinations.get(msg.matched_destination_id) or None
        actions = self.logic.on_intent(
            intent, dest, self.map_bounds, self._nav2_ready(), self._now()
        )
        self.get_logger().info(
            f"intent={msg.intent} dest={msg.matched_destination_id or '-'} "
            f"confirm={msg.need_confirm} -> state={self.logic.state.value}"
        )
        self._run_actions(actions)

    def _on_tts_done(self, msg: String) -> None:
        """TTS 가 끊기지 않고 끝까지 재생한 문장. 접근 질문일 때만 시계를 켠다."""
        if MSG_APPROACH_QUESTION in msg.data:
            self.logic.on_approach_question_spoken(self._now())
        if MSG_HANDLE_HINT in msg.data:
            self._run_actions(self.logic.on_handle_hint_spoken(self._now()))
        if MSG_APPROACH_ONBOARDING in msg.data or MSG_DEST_RETRY in msg.data:
            self._run_actions(self.logic.on_dest_prompt_spoken(self._now()))
        self.logic.on_arrival_question_spoken(self._now())

    def _on_confirm_answer(self, affirmative: bool) -> None:
        """확인 질문의 네/아니오. 확인 중 목적지를 되찾아 로직에 넘긴다."""
        dest_id = self.logic.confirming_dest_id or ""
        dest = self.destinations.get(dest_id) or None
        before = self.logic.state
        actions = self.logic.on_confirm_answer(
            affirmative, dest, self.map_bounds, self._nav2_ready(), self._now()
        )
        self.get_logger().info(
            f"확인 응답 {'긍정' if affirmative else '부정'}: dest={dest_id or '-'} "
            f"{before.value} -> {self.logic.state.value}"
        )
        self._run_actions(actions)

    def _on_voice_answer(self, affirmative: bool) -> None:
        before = self.logic.state
        actions = self.logic.on_approach_answer(affirmative, self._now())
        if not actions:
            self.get_logger().info(
                f"affirm/deny 무시: state={before.value} (접근 질문 대기 중이 아님)"
            )
            return
        self._run_actions(actions)
        self.get_logger().info(
            f"접근 응답 {'긍정' if affirmative else '부정'}: "
            f"{before.value} -> {self.logic.state.value}"
        )

    def _on_arrival_answer(self, msg: VicaIntent) -> None:
        """도착 후 대화 중의 답. navigate 답이면 다음 목적지를 게이트 없이"""
        intent = IntentData(
            intent=msg.intent,
            matched_destination_id=msg.matched_destination_id,
            need_confirm=msg.need_confirm,
            safety_flag=msg.safety_flag,
            wait_minutes=int(getattr(msg, "wait_minutes", -1)),
        )
        next_dest = None
        if msg.intent == "navigate":
            next_dest = self.destinations.get(msg.matched_destination_id) or None
        before = self.logic.state
        actions = self.logic.on_arrival_answer(intent, self._now(), next_dest=next_dest)
        self._run_actions(actions)
        self.get_logger().info(
            f"도착 후 답 intent={msg.intent}: {before.value} -> {self.logic.state.value}")

    def _on_wake(self, msg: String) -> None:
        """/vica/wake — WAITING 은 각성(다시 안내 질문), RETURNING 은 복귀"""
        if self.logic.state == State.RETURNING:
            actions = self.logic.on_return_brake(self._now())
            if actions:
                self._run_actions(actions)
                self.get_logger().info("복귀 중 '비카야' — 복귀 취소하고 응대")
            return
        before = self.logic.state
        actions = self.logic.on_wake(self._now())
        if actions or before != self.logic.state:
            self._run_actions(actions)
            self.get_logger().info(
                f"'비카야': {before.value} -> {self.logic.state.value}")

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
        before = self.logic.state
        actions = self.logic.on_person_detection(
            track_id=msg.track_id,
            distance_m=msg.distance_m,
            stable=msg.stable,
            approachable=msg.approachable,
            now=self._now(),
        )
        if before == self.logic.state:
            return
        self._run_actions(actions)
        self.get_logger().info(
            f"근접 호출: track={msg.track_id} dist={msg.distance_m:.2f}m "
            f"{before.value} -> {self.logic.state.value}"
        )

    def _on_voice_mission_command(self, msg: VicaIntent) -> None:
        """음성으로 온 취소·일시정지·재개를 처리한다."""
        now = self._now()
        before = self.logic.state

        if msg.intent == "cancel":
            if self.logic.cancel_confirm_pending:
                actions = self.logic.on_cancel_confirm_answer(True, now)
                self._run_actions(actions)
                self.get_logger().info(
                    f"음성 취소 확정: {before.value} -> {self.logic.state.value}"
                )
                return
            actions, reason = self.logic.on_cancel_confirm_request(now)
        elif msg.intent == "pause":
            actions, reason = self.logic.on_pause_request(now)
        else:
            actions, reason = self.logic.on_resume_request(self._nav2_ready(), now)

        if reason != GateReason.OK:
            message = _REJECT_MESSAGES.get(reason)
            if message:
                self._run_actions([Say(message, priority="response")])
            self.get_logger().warn(
                f"음성 {msg.intent} 거부: state={before.value} reason={reason.value}"
            )
            return

        self._run_actions(actions)
        self.get_logger().info(
            f"음성 {msg.intent} 처리: {before.value} -> {self.logic.state.value}"
        )

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
            return self.logic.on_app_cancel(now)
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
        try:
            request_id = str(UUID(request.request_id))
            destination_id = str(UUID(request.destination_id))
        except ValueError:
            response.accepted = False
            response.message = "request_id와 destination_id는 UUID여야 합니다."
            return response
        if request_id != request.request_id.lower():
            response.accepted = False
            response.message = "request_id가 canonical UUID 형식이 아닙니다."
            return response
        parsed_destination = UUID(destination_id)
        if (
            parsed_destination.version != 4
            or destination_id != request.destination_id.lower()
        ):
            response.accepted = False
            response.message = "destination_id는 canonical UUID v4여야 합니다."
            return response
        if request.map_id != self._map_id:
            response.accepted = False
            response.message = (
                f"현재 지도와 요청 지도가 다릅니다: "
                f"current={self._map_id}, requested={request.map_id}"
            )
            return response
        destination = self.destinations.get(destination_id)
        before = self.logic.state
        actions, reason = self.logic.on_app_destination(
            destination, self.map_bounds, self._nav2_ready(), self._now()
        )
        if reason != GateReason.OK:
            response.accepted = False
            response.message = f"목적지 요청 거부: {reason.value}"
            self.get_logger().warn(
                f"공개 목적지 요청 거부: map_id={request.map_id} "
                f"id={destination_id} reason={reason.value}"
            )
            return response
        if before != State.IDLE:
            self.get_logger().info(
                f"앱 선점 주행: {before.value} 를 취소하고 새 목적지로")
        self._run_actions(actions)
        response.accepted = self._nav_active
        response.message = (
            f"목적지 요청을 수락했습니다: {destination.name}"
            if response.accepted and destination is not None
            else "Nav2가 목적지 요청을 수락하지 않았습니다."
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
        actions = self.logic.on_tick(self._now(), status, distance)
        self._run_actions(actions)
        if State.SEEKING in (before, self.logic.state) and before != self.logic.state:
            self._publish_robot_state()

    def _publish_robot_state(self) -> None:
        msg = RobotState()
        msg.current_floor = int(self.get_parameter("current_floor").value)
        msg.current_building = str(self.get_parameter("current_building").value)
        msg.is_moving = self.logic.state in (State.NAVIGATING, State.SEEKING)
        msg.is_paused = self.logic.state == State.PAUSED
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
                    self.get_logger().info("질문 발화 — 재청취 요청 발행")
            elif isinstance(action, StopSpeech):
                out = String()
                out.data = "control:stop"
                self.pub_tts.publish(out)
                self.get_logger().info("발화 큐 청소 (취소·선점)")
            elif isinstance(action, CancelNav):
                self._cancel_nav(action.destination, action.event)
            elif isinstance(action, Navigate):
                self._start_nav(action)
            elif isinstance(action, SpinInPlace):
                self._start_spin(action)
            elif isinstance(action, SetNavSpeedLimit):
                self._publish_nav_speed_limit(action.percent)
            elif isinstance(action, Haptic):
                out = String()
                out.data = action.pattern
                self.pub_haptic.publish(out)
                self.get_logger().info(f"진동 요청: {action.pattern}")

    def _publish_nav_speed_limit(self, percent: float) -> None:
        msg = SpeedLimit()
        msg.percentage = True
        msg.speed_limit = float(percent)
        self.pub_speed_limit.publish(msg)
        if percent == 0.0:
            self.get_logger().info("Nav2 접근 속도 제한 해제")
        else:
            self.get_logger().info(f"Nav2 접근 속도 제한: 최대속도의 {percent:.1f}%")

    def _start_nav(self, action: Navigate) -> None:
        dest = action.destination
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
        try:
            accepted = self.navigator.goToPose(goal)
            self._nav_active = bool(accepted)
            if accepted:
                self._nav_gen += 1
        finally:
            self._nav_lock.release()
        if accepted:
            self._publish_goal_event("goal_sent", dest)
            self._publish_goal_event("goal_accepted", dest)
            self.get_logger().info(
                f"NavigateToPose 전송: {dest.id} ({dest.pose.x:.2f}, {dest.pose.y:.2f}, "
                f"{dest.pose.yaw_deg:.1f}deg)"
            )
        else:
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
            if not self.navigator.isTaskComplete():
                return NavStatus.RUNNING
            self._nav_active = False
            result = self.navigator.getResult()
        returning = self.logic.state == State.RETURNING
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

    def _publish_goal_event(self, event: str, destination, reason: str = "") -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "event": event,
                "map_id": self._map_id,
                "location_id": destination.id,
                "destination_id": destination.id,
                "name": destination.name,
                "x": destination.pose.x,
                "y": destination.pose.y,
                "yaw": destination.pose.yaw_deg,
                "reason": reason,
                "timestamp": datetime.now().isoformat(timespec="seconds"),
            },
            ensure_ascii=False,
        )
        self.pub_goal_event.publish(msg)


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
