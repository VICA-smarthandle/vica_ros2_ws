"""vica_mission_manager 게이트·상태 전이 순수 로직.

rclpy 에 의존하지 않는다 — 통합 계획(진행순서 ②)의 요구사항으로,
이 모듈의 모든 판단은 unit test 로 검증한다. ROS 배선은
mission_manager_node.py 가 담당하고, 여기서는 "무엇을 할지"만 결정해
Action 목록으로 돌려준다.

안전 원칙(불변): 이 로직이 허용해야만 Nav2 goal 이 나간다.
LLM(VicaIntent)은 어디까지나 제안이다.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Union

# 하드 긴급어: 즉시 goal 취소 + estopped 진입 (LLM 우회 경로).
# "천천히/느리게/잠깐" 등 감속·유보 계열은 v2 (TODOS.md #6) — 여기서는 무시한다.
HARD_EMERGENCY_KEYWORDS = frozenset({"멈춰", "정지", "스탑", "스톱", "안돼", "위험해"})


class State(str, Enum):
    IDLE = "idle"
    CONFIRMING = "confirming"
    NAVIGATING = "navigating"
    ARRIVED = "arrived"
    FAILED = "failed"
    ESTOPPED = "estopped"


class NavStatus(str, Enum):
    """노드가 nav2_simple_commander 에서 읽어 넘겨주는 주행 상태."""

    NONE = "none"          # 진행 중인 goal 없음
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    CANCELED = "canceled"
    FAILED = "failed"


class GateReason(str, Enum):
    OK = "ok"
    NOT_NAVIGATE = "not_navigate"
    NO_MATCHED_ID = "no_matched_id"
    NEED_CONFIRM = "need_confirm"
    SAFETY_FLAG = "safety_flag"
    ESTOP_ACTIVE = "estop_active"
    BUSY_NAVIGATING = "busy_navigating"
    UNKNOWN_DESTINATION = "unknown_destination"
    NOT_APPROACHABLE = "not_approachable"
    POSE_INVALID = "pose_invalid"
    NAV_NOT_READY = "nav_not_ready"


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw_deg: float  # destinations.yaml 은 도(deg) 단위 (함정 목록 2번)
    frame_id: str = "map"


@dataclass(frozen=True)
class Destination:
    id: str
    name: str
    pose: Pose2D
    is_approachable: bool = True
    unavailable_reason: str = ""
    # destinations.yaml 에 calibrated 필드가 없으면 None →
    # (0,0) 플레이스홀더 여부로 추정한다 (함정 목록 1번).
    # 캘리브레이션(진행순서 ①) 시 calibrated: true 를 명시하는 것이 정석.
    calibrated: Optional[bool] = None
    confirm_prompt: str = ""
    arrival_message: str = ""


@dataclass(frozen=True)
class MapBounds:
    min_x: float
    min_y: float
    max_x: float
    max_y: float

    def contains(self, x: float, y: float) -> bool:
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y


@dataclass(frozen=True)
class IntentData:
    """VicaIntent 중 게이트 판단에 쓰는 필드만."""

    intent: str
    matched_destination_id: str
    need_confirm: bool
    safety_flag: str


# ---- Actions: 노드가 실행할 일 ---------------------------------------------


@dataclass(frozen=True)
class Say:
    text: str


@dataclass(frozen=True)
class Navigate:
    destination: Destination


@dataclass(frozen=True)
class CancelNav:
    pass


Action = Union[Say, Navigate, CancelNav]


# ---- 멘트 (v1 임시 카피 — 시각장애인 관점 감수는 미결 사항 #4) ----------------

MSG_START = "{name}(으)로 안내를 시작합니다."
MSG_ARRIVED_FALLBACK = "{name}에 도착했습니다."
MSG_BUSY = "지금 이동 중입니다. 먼저 현재 안내를 멈춰 주세요."
MSG_UNKNOWN_DEST = "아직 안내할 수 없는 곳입니다."
MSG_NOT_APPROACHABLE = "죄송합니다. 지금은 안내할 수 없는 곳입니다."
MSG_POSE_INVALID = "아직 안내할 수 없는 곳입니다. 위치 등록이 필요합니다."
MSG_NAV_NOT_READY = "아직 준비 중입니다. 잠시 후 다시 말씀해 주세요."
MSG_ESTOP_REJECT = "지금은 비상 정지 상태입니다. 해제 후 다시 말씀해 주세요."
MSG_CONFIRM_TIMEOUT = "안내 요청이 취소되었습니다."
MSG_STALE_CONFIRM = "요청이 확인되지 않았습니다. 다시 말씀해 주세요."
MSG_NAV_FAILED = "죄송합니다. 이동에 실패했습니다. 다시 시도해 주세요."
MSG_ESTOPPED = "비상 정지합니다."
MSG_ESTOP_RELEASED = "비상 정지가 해제되었습니다. 새로운 목적지를 말씀해 주세요."

_REJECT_MESSAGES = {
    GateReason.BUSY_NAVIGATING: MSG_BUSY,
    GateReason.UNKNOWN_DESTINATION: MSG_UNKNOWN_DEST,
    GateReason.NOT_APPROACHABLE: MSG_NOT_APPROACHABLE,
    GateReason.POSE_INVALID: MSG_POSE_INVALID,
    GateReason.NAV_NOT_READY: MSG_NAV_NOT_READY,
    GateReason.ESTOP_ACTIVE: MSG_ESTOP_REJECT,
}


# ---- 순수 함수: pose 검증 / 게이트 ------------------------------------------

_ZERO_EPS = 1e-6


def pose_valid(dest: Destination, bounds: Optional[MapBounds]) -> bool:
    """게이트 ⑤: calibrated + (0,0) 아님 + frame_id=="map" + 지도 경계 내.

    이 검증이 없으면 미캘리브레이션 목적지 요청 시 로봇이 지도 원점(0,0)으로
    주행하는 사고가 난다 (함정 목록 1번).
    """
    if dest.calibrated is False:
        return False
    if dest.pose.frame_id != "map":
        return False
    if abs(dest.pose.x) < _ZERO_EPS and abs(dest.pose.y) < _ZERO_EPS:
        return False  # (0,0) 플레이스홀더
    if bounds is not None and not bounds.contains(dest.pose.x, dest.pose.y):
        return False
    return True


def check_gate(
    intent: IntentData,
    dest: Optional[Destination],
    bounds: Optional[MapBounds],
    estop_active: bool,
    nav_ready: bool,
) -> GateReason:
    """게이트 5조건 + 문맥 조건. 첫 번째 실패 사유를 돌려준다."""
    if intent.intent != "navigate":
        return GateReason.NOT_NAVIGATE
    if not intent.matched_destination_id:
        return GateReason.NO_MATCHED_ID
    if intent.need_confirm:
        return GateReason.NEED_CONFIRM
    if intent.safety_flag != "normal":
        return GateReason.SAFETY_FLAG
    if estop_active:
        return GateReason.ESTOP_ACTIVE
    if dest is None:
        return GateReason.UNKNOWN_DESTINATION
    if not dest.is_approachable:
        return GateReason.NOT_APPROACHABLE
    if not pose_valid(dest, bounds):
        return GateReason.POSE_INVALID
    if not nav_ready:
        return GateReason.NAV_NOT_READY
    return GateReason.OK


def yaw_deg_to_quaternion(yaw_deg: float) -> tuple:
    """도(deg) yaw → 쿼터니언 (x, y, z, w). 변환은 goal 생성 시에만 (함정 2번)."""
    import math

    half = math.radians(yaw_deg) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


# ---- 상태 머신 ---------------------------------------------------------------


class MissionLogic:
    """상태 6종(idle/confirming/navigating/arrived/failed/estopped) 상태 머신.

    시간은 전부 인자(now: float 초)로 받는다 — 테스트에서 시계를 주입하기 위함.
    """

    def __init__(
        self,
        confirm_timeout_sec: float = 30.0,
        dwell_sec: float = 2.0,
        estop_release_grace_sec: float = 2.0,
    ) -> None:
        self.confirm_timeout_sec = confirm_timeout_sec
        self.dwell_sec = dwell_sec
        self.estop_release_grace_sec = estop_release_grace_sec

        self.state: State = State.IDLE
        self.estop_active: bool = False
        self.active_destination: Optional[Destination] = None

        self._confirming_dest_id: Optional[str] = None
        self._confirm_deadline: Optional[float] = None
        self._dwell_until: Optional[float] = None
        self._estop_entered_at: Optional[float] = None
        self._estop_clear_since: Optional[float] = None

    # -- 입력 이벤트 -----------------------------------------------------------

    def on_intent(
        self,
        intent: IntentData,
        dest: Optional[Destination],
        bounds: Optional[MapBounds],
        nav_ready: bool,
        now: float,
    ) -> list:
        if intent.intent != "navigate":
            # 질문/잡담 등은 LLM(reply)과 ros_tts_node 몫 — 여기선 관여하지 않는다.
            return []

        if self.state == State.ESTOPPED:
            return [Say(MSG_ESTOP_REJECT)]

        if self.state == State.NAVIGATING:
            # v1 정책: 주행 중 새 목적지는 거부 (소프트 취소는 v2, TODOS.md #4)
            return [Say(MSG_BUSY)]

        if intent.need_confirm:
            # 확인 대기 시작/갱신. 확인 질문(confirm_prompt)은 LLM reply 로
            # ros_tts_node 가 이미 재생하므로 여기서 중복 발화하지 않는다.
            self.state = State.CONFIRMING
            self._confirming_dest_id = intent.matched_destination_id or None
            self._confirm_deadline = now + self.confirm_timeout_sec
            return []

        # need_confirm == false (확정 요청)
        if (
            self.state == State.CONFIRMING
            and self._confirming_dest_id
            and intent.matched_destination_id != self._confirming_dest_id
        ):
            # 오래된/엇갈린 confirm 방어 (request_id 없는 v1 의 임시 방어)
            self._to_idle()
            return [Say(MSG_STALE_CONFIRM)]

        reason = check_gate(intent, dest, bounds, self.estop_active, nav_ready)
        if reason != GateReason.OK:
            self._to_idle()
            msg = _REJECT_MESSAGES.get(reason)
            return [Say(msg)] if msg else []

        assert dest is not None  # check_gate 가 보장
        self.state = State.NAVIGATING
        self.active_destination = dest
        self._confirming_dest_id = None
        self._confirm_deadline = None
        return [Say(MSG_START.format(name=dest.name)), Navigate(dest)]

    def on_emergency(self, keyword: str, now: float) -> list:
        """/vica/emergency (긴급어). 하드 키워드만 처리 — LLM 을 거치지 않은 경로.

        모터 정지의 권위는 /emergency_stop 래치 체인(진행순서 ③에서 배선)이고,
        여기서의 goal 취소는 심층 방어 보조 경로다.
        """
        if keyword not in HARD_EMERGENCY_KEYWORDS:
            return []

        actions: list = []
        if self.state == State.NAVIGATING:
            actions.append(CancelNav())
        already_estopped = self.state == State.ESTOPPED
        self._enter_estopped(now)
        if not already_estopped:
            actions.append(Say(MSG_ESTOPPED))
        return actions

    def on_estop(self, active: bool, now: float) -> list:
        """/emergency_stop 래치 상태 (emergency_stop_node 가 20Hz 주기 발행)."""
        self.estop_active = active
        if active:
            self._estop_clear_since = None
            if self.state != State.ESTOPPED:
                actions: list = []
                if self.state == State.NAVIGATING:
                    actions.append(CancelNav())
                self._enter_estopped(now)
                actions.append(Say(MSG_ESTOPPED))
                return actions
        else:
            if self.state == State.ESTOPPED and self._estop_clear_since is None:
                self._estop_clear_since = now
        return []

    def on_tick(self, now: float, nav_status: NavStatus) -> list:
        actions: list = []

        if self.state == State.CONFIRMING:
            if self._confirm_deadline is not None and now >= self._confirm_deadline:
                self._to_idle()
                actions.append(Say(MSG_CONFIRM_TIMEOUT))

        elif self.state == State.NAVIGATING:
            if nav_status == NavStatus.SUCCEEDED:
                dest = self.active_destination
                text = (
                    dest.arrival_message
                    if dest and dest.arrival_message
                    else MSG_ARRIVED_FALLBACK.format(name=dest.name if dest else "목적지")
                )
                self.state = State.ARRIVED
                self._dwell_until = now + self.dwell_sec
                actions.append(Say(text))
            elif nav_status in (NavStatus.FAILED, NavStatus.CANCELED):
                # estop 경로의 취소는 이미 ESTOPPED 로 빠져나갔으므로,
                # 여기 도달한 취소/실패는 주행 실패로 취급한다.
                self.state = State.FAILED
                self._dwell_until = now + self.dwell_sec
                actions.append(Say(MSG_NAV_FAILED))

        elif self.state in (State.ARRIVED, State.FAILED):
            if self._dwell_until is None or now >= self._dwell_until:
                self._to_idle()

        elif self.state == State.ESTOPPED:
            if not self.estop_active:
                # 래치 해제 확인 후 grace 경과 시에만 idle 복귀.
                # 이전 goal 자동 재개는 금지 — 사용자가 다시 요청해야 한다.
                t0 = (
                    self._estop_clear_since
                    if self._estop_clear_since is not None
                    else self._estop_entered_at
                )
                if t0 is not None and now - t0 >= self.estop_release_grace_sec:
                    self._to_idle()
                    actions.append(Say(MSG_ESTOP_RELEASED))

        return actions

    # -- 내부 ------------------------------------------------------------------

    def _enter_estopped(self, now: float) -> None:
        self.state = State.ESTOPPED
        self.active_destination = None
        self._confirming_dest_id = None
        self._confirm_deadline = None
        self._estop_entered_at = now
        self._estop_clear_since = None

    def _to_idle(self) -> None:
        self.state = State.IDLE
        self.active_destination = None
        self._confirming_dest_id = None
        self._confirm_deadline = None
        self._dwell_until = None
        self._estop_entered_at = None
        self._estop_clear_since = None
