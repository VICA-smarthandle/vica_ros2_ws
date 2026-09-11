"""vica_mission_manager 게이트·상태 전이 순수 로직."""
from __future__ import annotations

import math

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Sequence, Union

from .approach_speed import ApproachSpeedLadder, NO_SPEED_LIMIT

HARD_EMERGENCY_KEYWORDS = frozenset({"멈춰", "정지", "스탑", "스톱", "안돼", "위험해"})


class State(str, Enum):
    IDLE = "idle"
    CONFIRMING = "confirming"
    NAVIGATING = "navigating"
    ARRIVED = "arrived"
    FAILED = "failed"
    ESTOPPED = "estopped"
    PAUSED = "paused"
    APPROACHING = "approaching"
    AWAITING_USER = "awaiting_user"
    TURNING = "turning"
    RETURNING = "returning"
    SEEKING = "seeking"
    ASKING_NEXT = "asking_next"
    ASKING_WAIT_TIME = "asking_wait_time"
    WAITING = "waiting"


class NavStatus(str, Enum):
    """노드가 nav2_simple_commander 에서 읽어 넘겨주는 주행 상태."""

    NONE = "none"
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
    PRIVATE_DESTINATION = "private_destination"
    NOT_APPROACHABLE = "not_approachable"
    POSE_INVALID = "pose_invalid"
    NAV_NOT_READY = "nav_not_ready"
    NOT_NAVIGATING = "not_navigating"
    NOT_PAUSED = "not_paused"
    NO_TRACK_ID = "no_track_id"
    TRACK_SUPPRESSED = "track_suppressed"
    BUSY_APPROACHING = "busy_approaching"
    NOT_APPROACHING = "not_approaching"
    NO_HOME = "no_home"
    ALREADY_HOME_BOUND = "already_home_bound"


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw_deg: float
    frame_id: str = "map"


@dataclass(frozen=True)
class Destination:
    id: str
    name: str
    pose: Pose2D
    authorization: str = "public"
    is_approachable: bool = True
    unavailable_reason: str = ""
    calibrated: Optional[bool] = None
    confirm_prompt: str = ""
    arrival_message: str = ""
    category: str = ""


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
    wait_minutes: int = -1


@dataclass(frozen=True)
class ApproachRequest:
    """RequestApproach.srv 한 건 중 게이트 판단에 쓰는 값만."""

    goal: Optional[Pose2D]
    track_id: int
    approachable: bool = True


@dataclass(frozen=True)
class Say:
    text: str
    priority: str = "narration"
    expects_reply: bool = False


@dataclass(frozen=True)
class Navigate:
    destination: Destination


@dataclass(frozen=True)
class CancelNav:
    destination: Optional[Destination] = None
    event: str = "goal_canceled"


@dataclass(frozen=True)
class StopSpeech:
    """하던 말을 끊고 대기 중인 비긴급 발화를 비운다 (노드가 /vica/tts_stop 발행)."""


@dataclass(frozen=True)
class SpinInPlace:
    """제자리 회전. 노드는 BasicNavigator.spin() 으로 실행한다."""

    yaw_rad: float
    reason: str = "회전"


@dataclass(frozen=True)
class SetNavSpeedLimit:
    """Nav2 controller 최대속도 제한율. 0.0은 제한 해제다."""

    percent: float


@dataclass(frozen=True)
class Haptic:
    """손잡이 진동 요청. 노드가 패턴 이름을 그대로 /vica/haptic_request 에"""

    pattern: str


Action = Union[
    Say, Navigate, CancelNav, SetNavSpeedLimit, Haptic, SpinInPlace, StopSpeech
]


def josa_euro(word: str) -> str:
    """단어 뒤에 붙는 조사 '으로 / 로' 를 받침에 맞게 돌려준다."""
    if not word:
        return "로"
    last = word[-1]
    if not ("가" <= last <= "힣"):
        return "로"
    jongseong = (ord(last) - 0xAC00) % 28
    return "로" if jongseong in (0, 8) else "으로"


def say_destination(template: str, name: str) -> str:
    """목적지 이름이 들어가는 멘트를 조사까지 맞춰 완성한다."""
    return template.format(name=name, josa=josa_euro(name))


MSG_START = "{name}{josa} 안내를 시작합니다."
MSG_ARRIVED_FALLBACK = "{name}에 도착했습니다."
MSG_BUSY = "지금 이동 중입니다. 먼저 현재 안내를 취소해 주세요."
MSG_UNKNOWN_DEST = "아직 안내할 수 없는 곳입니다."
MSG_PRIVATE_DEST = "비공개 목적지는 안내할 수 없습니다."
MSG_NOT_APPROACHABLE = "죄송합니다. 지금은 안내할 수 없는 곳입니다."
MSG_POSE_INVALID = "아직 안내할 수 없는 곳입니다. 위치 등록이 필요합니다."
MSG_NAV_NOT_READY = "아직 준비 중입니다. 잠시 후 다시 말씀해 주세요."
MSG_ESTOP_REJECT = "지금은 비상 멈춤 상태입니다. 해제 후 다시 말씀해 주세요."
MSG_CONFIRM_TIMEOUT = "안내 요청이 취소되었습니다."
MSG_NAV_FAILED = "죄송합니다. 이동에 실패했습니다. 다시 시도해 주세요."
MSG_ESTOPPED = "안전을 위해 멈추겠습니다. 관리자를 호출했습니다."
MSG_ESTOP_RELEASED = "비상멈춤이 해제되었습니다."

MSG_DISTANCE_REMAINING = "목적지까지 약 {meters}미터 남았습니다."
MSG_CANCELED = "안내를 취소했습니다."

MSG_ASK_RESTROOM = "다녀오시는 동안 여기서 기다릴까요?"
MSG_ASK_ENTRANCE = "여기까지 안내를 마칠까요?"
MSG_ASK_GENERIC = "여기서 대기할까요?"
MSG_ASK_WAIT_TIME = "몇 분쯤 걸리실까요?"
MSG_WAIT_CONFIRM = "{minutes}분 대기하겠습니다. 돌아오시면 '비카야'라고 말씀해 주세요."
MSG_WAIT_DEFAULT = "네, 최대 30분까지 여기서 기다리겠습니다. 돌아오시면 '비카야'라고 말씀해 주세요."
MSG_FINISH = "안내를 종료합니다."
MSG_ARRIVAL_RETRY = "잘 듣지 못했습니다. 계속 안내가 필요하시면 말씀해 주세요."
MSG_LEAVING_NOTICE = "응답이 없어 안내를 마치고 제자리로 돌아가겠습니다."

WAIT_MINUTES_CAP = 30
LEAVING_GRACE_SEC = 3.0
RETURN_RESUME_SEC = 15.0
EAR_GRACE_SEC = 6.0
EAR_HOLD_MAX_SEC = 20.0
ASKING_STUCK_FALLBACK_SEC = 30.0
MSG_PAUSED = "잠시 멈추겠습니다. 다시 출발하려면 말씀해 주세요."
MSG_RESUMED = "{name}{josa} 다시 출발합니다."
MSG_CANCEL_CONFIRM = "안내를 취소할까요?"
MSG_CANCEL_KEPT = "안내를 계속하겠습니다."
MSG_NOT_NAVIGATING = "지금은 안내 중이 아닙니다."
MSG_NOT_PAUSED = "다시 출발할 안내가 없습니다."
MSG_APPROACH_QUESTION = (
    "안녕하세요? 저는 시각장애인 안내로봇 비카입니다! "
    "저와 함께 목적지까지 동행해보시는건 어떠세요? 안내를 받으시겠어요?"
)
MSG_APPROACH_ACCEPTED = "네, 잠시만 기다려주세요. 로봇이 회전하니 주의하세요."
MSG_APPROACH_DECLINED = "알겠습니다. 이만 물러납니다."
MSG_APPROACH_ONBOARDING = (
    "안녕하세요? 반갑습니다! 저에게 말을 거실 때는 '비카야'라고 불러주세요. "
    "자, 이제 어디로 가고 싶으신가요?"
)
MSG_APPROACH_NO_ANSWER = "실례했습니다. 필요하시면 언제든 불러 주세요."
MSG_APPROACH_BUSY = "지금은 다른 응대 중입니다. 잠시 후 다시 말씀해 주세요."

MSG_DEST_RETRY = "잘 듣지 못했습니다. 어디로 가고 싶으신가요?"
DEST_ANSWER_WAIT_SEC = 15.0
DEST_PROMPT_FALLBACK_SEC = 40.0
DEST_RETRY_RETURN_SEC = 0.0

MSG_HANDLE_HINT = "손잡이는 지금 계신 쪽에 있습니다. 진동이 나는 곳을 잡아주세요."
HAPTIC_PATTERN_HANDLE_HINT = "long"

DISTANCE_MILESTONES_M = ()


APPROACH_RESPONSE_TIMEOUT_SEC = 8.0
APPROACH_QUESTION_STUCK_SEC = 30.0
APPROACH_TURN_TIMEOUT_SEC = 15.0
SEEK_LOOK_SEC = 6.0
SEEK_TURN_TIMEOUT_SEC = APPROACH_TURN_TIMEOUT_SEC
SEEK_MIN_YAW_RAD = math.radians(10.0)
HANDLE_SIDE_MIN_YAW_RAD = math.radians(135.0)
WAKE_CONSUMED_GUARD_SEC = 3.0
REAPPROACH_SUPPRESS_SEC = 60.0
USER_ATTACHED_SUPPRESS_SEC = 60.0
NEAR_CALL_MAX_M = 1.5
NEAR_CALL_NO_SPIN_M = 1.0
PERSON_APPROACH_SPEED_PERCENT = 100.0
APPROACH_GOAL_UPDATE_M = 0.5
TRACK_ID_NONE = 0
APPROACH_DESTINATION_PREFIX = "approach:"
APPROACH_DESTINATION_NAME = "접근 대상"

_APPROACH_STATES = (
    State.APPROACHING, State.AWAITING_USER, State.TURNING, State.RETURNING,
    State.SEEKING,
)
_GOAL_ACTIVE_STATES = (
    State.NAVIGATING, State.APPROACHING, State.TURNING, State.RETURNING,
    State.SEEKING,
)

_REJECT_MESSAGES = {
    GateReason.BUSY_NAVIGATING: MSG_BUSY,
    GateReason.UNKNOWN_DESTINATION: MSG_UNKNOWN_DEST,
    GateReason.PRIVATE_DESTINATION: MSG_PRIVATE_DEST,
    GateReason.NOT_APPROACHABLE: MSG_NOT_APPROACHABLE,
    GateReason.POSE_INVALID: MSG_POSE_INVALID,
    GateReason.NAV_NOT_READY: MSG_NAV_NOT_READY,
    GateReason.ESTOP_ACTIVE: MSG_ESTOP_REJECT,
    GateReason.NOT_NAVIGATING: MSG_NOT_NAVIGATING,
    GateReason.NOT_PAUSED: MSG_NOT_PAUSED,
}


_ZERO_EPS = 1e-6


def pose_valid(dest: Destination, bounds: Optional[MapBounds]) -> bool:
    """게이트 ⑤: calibrated + (0,0) 아님 + frame_id=="map" + 지도 경계 내."""
    if dest.calibrated is False:
        return False
    if dest.pose.frame_id != "map":
        return False
    if abs(dest.pose.x) < _ZERO_EPS and abs(dest.pose.y) < _ZERO_EPS:
        return False
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
    if dest.authorization != "public":
        return GateReason.PRIVATE_DESTINATION
    if not dest.is_approachable:
        return GateReason.NOT_APPROACHABLE
    if not pose_valid(dest, bounds):
        return GateReason.POSE_INVALID
    if not nav_ready:
        return GateReason.NAV_NOT_READY
    return GateReason.OK


def check_cancel_gate(state: State, estop_active: bool) -> GateReason:
    """취소 요청 게이트. 주행 중이거나 일시정지 상태일 때만 취소할 수 있다."""
    if estop_active or state == State.ESTOPPED:
        return GateReason.ESTOP_ACTIVE
    if state not in (State.NAVIGATING, State.PAUSED, State.WAITING):
        return GateReason.NOT_NAVIGATING
    return GateReason.OK


def check_pause_gate(state: State, estop_active: bool) -> GateReason:
    """일시정지 게이트. 실제로 주행 중일 때만 멈출 수 있다."""
    if estop_active or state == State.ESTOPPED:
        return GateReason.ESTOP_ACTIVE
    if state != State.NAVIGATING:
        return GateReason.NOT_NAVIGATING
    return GateReason.OK


def check_resume_gate(
    state: State,
    paused_destination: Optional[Destination],
    estop_active: bool,
    nav_ready: bool,
) -> GateReason:
    """재개 게이트. 일시정지로 보관한 목적지가 있어야 다시 출발할 수 있다."""
    if estop_active or state == State.ESTOPPED:
        return GateReason.ESTOP_ACTIVE
    if state != State.PAUSED or paused_destination is None:
        return GateReason.NOT_PAUSED
    if not nav_ready:
        return GateReason.NAV_NOT_READY
    return GateReason.OK


def approach_destination(request: ApproachRequest) -> Destination:
    """접근 goal 을 Navigate 가 받는 Destination 모양으로 감싼다."""
    assert request.goal is not None
    return Destination(
        id=f"{APPROACH_DESTINATION_PREFIX}{request.track_id}",
        name=APPROACH_DESTINATION_NAME,
        pose=request.goal,
        calibrated=True,
        arrival_message="",
    )


def check_approach_gate(
    request: ApproachRequest,
    state: State,
    active_track_id: Optional[int],
    bounds: Optional[MapBounds],
    estop_active: bool,
    nav_ready: bool,
    suppressed: bool,
) -> GateReason:
    """사람 접근 요청 게이트. 첫 번째 실패 사유를 돌려준다."""
    if not request.approachable:
        return GateReason.NOT_APPROACHABLE
    if request.track_id == TRACK_ID_NONE:
        return GateReason.NO_TRACK_ID
    if estop_active or state == State.ESTOPPED:
        return GateReason.ESTOP_ACTIVE
    if state == State.APPROACHING:
        if request.track_id != active_track_id:
            return GateReason.BUSY_APPROACHING
    elif state in (State.AWAITING_USER, State.RETURNING):
        return GateReason.BUSY_APPROACHING
    elif state != State.IDLE:
        return GateReason.BUSY_NAVIGATING
    if suppressed:
        return GateReason.TRACK_SUPPRESSED
    if request.goal is None:
        return GateReason.POSE_INVALID
    if not pose_valid(approach_destination(request), bounds):
        return GateReason.POSE_INVALID
    if not nav_ready:
        return GateReason.NAV_NOT_READY
    return GateReason.OK


def check_return_home_gate(
    state: State,
    home: Optional[Destination],
    estop_active: bool,
    nav_ready: bool,
) -> GateReason:
    """홈 복귀 게이트(`/vica/mission/return_home`). **관리자 전용 요청이다.**"""
    if estop_active or state == State.ESTOPPED:
        return GateReason.ESTOP_ACTIVE
    if home is None:
        return GateReason.NO_HOME
    if state == State.RETURNING:
        return GateReason.ALREADY_HOME_BOUND
    if state in (State.APPROACHING, State.AWAITING_USER, State.TURNING):
        return GateReason.BUSY_APPROACHING
    if state != State.IDLE:
        return GateReason.BUSY_NAVIGATING
    if not nav_ready:
        return GateReason.NAV_NOT_READY
    return GateReason.OK


def check_approach_cancel_gate(state: State, estop_active: bool) -> GateReason:
    """접근 취소 게이트(/vica/mission/cancel_approach)."""
    if estop_active or state == State.ESTOPPED:
        return GateReason.ESTOP_ACTIVE
    if state not in (State.APPROACHING, State.AWAITING_USER):
        return GateReason.NOT_APPROACHING
    return GateReason.OK


def wrap_to_pi(rad: float) -> float:
    """각도를 -π~π 로 접는다 — 언제나 짧은 쪽으로 돈다."""
    return math.atan2(math.sin(rad), math.cos(rad))


def doa_to_spin_yaw(doa_deg: float, sign: float = 1.0) -> float:
    """마이크 DOA(0~359°, 정면 0 / 핸들 180)를 제자리 회전량(rad)으로."""
    return wrap_to_pi(math.radians(float(doa_deg) * float(sign)))


def yaw_deg_to_quaternion(yaw_deg: float) -> tuple:
    """도(deg) yaw → 쿼터니언 (x, y, z, w). 변환은 goal 생성 시에만 (함정 2번)."""
    import math

    half = math.radians(yaw_deg) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


class MissionLogic:
    """상태 머신. 안내 7종 + 사람 접근 3종."""

    def __init__(
        self,
        confirm_timeout_sec: float = 30.0,
        dwell_sec: float = 2.0,
        estop_release_grace_sec: float = 1.0,
        approach_stages: Optional[Sequence] = None,
        nav_retry_limit: int = 2,
        nav_retry_delay_sec: float = 3.0,
        approach_response_timeout_sec: float = APPROACH_RESPONSE_TIMEOUT_SEC,
        reapproach_suppress_sec: float = REAPPROACH_SUPPRESS_SEC,
        person_approach_speed_percent: float = PERSON_APPROACH_SPEED_PERCENT,
        approach_goal_update_m: float = APPROACH_GOAL_UPDATE_M,
        return_destination: Optional[Destination] = None,
        auto_return_home: bool = False,
        approach_turn_yaw_rad: float = math.pi,
        arrival_dialog: bool = False,
        wake_doa_sign: float = 1.0,
        seek_look_sec: float = SEEK_LOOK_SEC,
        near_call_max_m: float = NEAR_CALL_MAX_M,
        near_call_no_spin_m: float = NEAR_CALL_NO_SPIN_M,
        return_resume_sec: float = RETURN_RESUME_SEC,
        handle_side_min_yaw_rad: float = HANDLE_SIDE_MIN_YAW_RAD,
        dest_retry_return_sec: float = DEST_RETRY_RETURN_SEC,
    ) -> None:
        self.confirm_timeout_sec = confirm_timeout_sec
        self.dwell_sec = dwell_sec
        self.estop_release_grace_sec = estop_release_grace_sec
        self.nav_retry_limit = nav_retry_limit
        self.nav_retry_delay_sec = nav_retry_delay_sec
        self._approach = ApproachSpeedLadder(approach_stages)

        self.approach_response_timeout_sec = approach_response_timeout_sec
        self.reapproach_suppress_sec = reapproach_suppress_sec
        self.person_approach_speed_percent = person_approach_speed_percent
        self.approach_goal_update_m = approach_goal_update_m
        self.return_destination = return_destination
        self.auto_return_home = auto_return_home
        self._returning_home: bool = False
        self.approach_turn_yaw_rad = approach_turn_yaw_rad
        self.wake_doa_sign = wake_doa_sign
        self.seek_look_sec = seek_look_sec
        self.near_call_max_m = near_call_max_m
        self.near_call_no_spin_m = near_call_no_spin_m
        self.return_resume_sec = return_resume_sec
        self.handle_side_min_yaw_rad = handle_side_min_yaw_rad
        self.dest_retry_return_sec = dest_retry_return_sec
        self._wake_consumed_at: Optional[float] = None
        self._user_attached_until: Optional[float] = None
        self._seek_return_yaw: Optional[float] = None
        self._seek_deadline: Optional[float] = None
        self._turn_deadline: Optional[float] = None
        self._return_interrupted: bool = False
        self._return_resume_deadline: Optional[float] = None
        self._return_notice_given: bool = False
        self._dest_prompt_stage: Optional[str] = None
        self._dest_prompt_deadline: Optional[float] = None
        self._near_call_no_spin: bool = False
        self._never_approached: bool = False
        self._pending_handle_hint: bool = False
        self._estop_announced = False

        self.arrival_dialog = arrival_dialog
        self._asking_is_finish = False
        self._asking_time_after_yes = False
        self._nav_from_app = False
        self._asking_entered_at: Optional[float] = None
        self._arrival_retried = False
        self._leaving_deadline: Optional[float] = None
        self._wait_until: Optional[float] = None
        self._ear_busy = False
        self._ear_busy_since: Optional[float] = None
        self._ear_grace_until: Optional[float] = None
        self._ear_speaking = False
        self._ear_speech_since: Optional[float] = None

        self.state: State = State.IDLE
        self.estop_active: bool = False
        self.active_destination: Optional[Destination] = None
        self.paused_destination: Optional[Destination] = None
        self.cancel_confirm_pending: bool = False

        self._cancel_confirm_deadline: Optional[float] = None
        self._confirming_dest_id: Optional[str] = None
        self._confirm_deadline: Optional[float] = None
        self._dwell_until: Optional[float] = None
        self._estop_entered_at: Optional[float] = None
        self._estop_clear_since: Optional[float] = None
        self._nav_retry_count: int = 0
        self._retry_destination: Optional[Destination] = None
        self._retry_at: Optional[float] = None
        self._announced_milestones: set = set()
        self._distance_baseline: Optional[float] = None
        self.approach_track_id: Optional[int] = None
        self.approach_goal_pose: Optional[Pose2D] = None
        self._response_deadline: Optional[float] = None
        self._suppressed_tracks: dict = {}

    @property
    def approach_stages(self) -> tuple:
        """검증·정렬을 마친 접근 감속 단계 목록 (먼 거리부터)."""
        return self._approach.stages

    @property
    def approach_speed_limit_percent(self) -> float:
        """지금 걸려 있는 접근 제한율. 제한 전이면 0.0(해제)."""
        return self._approach.percent

    def on_intent(
        self,
        intent: IntentData,
        dest: Optional[Destination],
        bounds: Optional[MapBounds],
        nav_ready: bool,
        now: float,
    ) -> list:
        self._forget_dest_prompt()
        if intent.intent != "navigate":
            return []

        if self.state == State.ESTOPPED:
            return [Say(MSG_ESTOP_REJECT, priority="response")]

        if self.state == State.NAVIGATING:
            return [Say(MSG_BUSY, priority="response")]

        if self.state in _APPROACH_STATES:
            return [Say(MSG_APPROACH_BUSY, priority="response")]

        if intent.need_confirm:
            if (self.state == State.CONFIRMING
                    and self._confirming_dest_id
                    and intent.matched_destination_id == self._confirming_dest_id):
                return self.on_confirm_answer(True, dest, bounds, nav_ready, now)
            else:
                self.state = State.CONFIRMING
                self._confirming_dest_id = intent.matched_destination_id or None
                self._confirm_deadline = now + self.confirm_timeout_sec
                return []

        if (
            self.state == State.CONFIRMING
            and self._confirming_dest_id
            and intent.matched_destination_id != self._confirming_dest_id
        ):
            self._to_idle()
            return []

        reason = check_gate(intent, dest, bounds, self.estop_active, nav_ready)
        if reason != GateReason.OK:
            self._to_idle()
            msg = _REJECT_MESSAGES.get(reason)
            return [Say(msg, priority="response")] if msg else []

        assert dest is not None
        self.state = State.NAVIGATING
        self.active_destination = dest
        self._confirming_dest_id = None
        self._confirm_deadline = None
        self._announced_milestones = set()
        self._distance_baseline = None
        self._approach.reset()
        self._seek_deadline = None
        self._seek_return_yaw = None
        self._forget_interrupted_return()
        return [
            SetNavSpeedLimit(NO_SPEED_LIMIT),
            Say(say_destination(MSG_START, dest.name)),
            Navigate(dest),
        ]

    @property
    def confirming_dest_id(self) -> Optional[str]:
        """확인 중인 목적지 id. CONFIRMING 밖에서는 None."""
        if self.state != State.CONFIRMING:
            return None
        return self._confirming_dest_id

    @property
    def return_interrupted(self) -> bool:
        """복귀 재개 사다리가 도는 중인가. on_wake_doa 와 노드의 진단 로그가"""
        return self._return_interrupted

    def on_confirm_answer(
        self,
        affirmative: bool,
        dest: Optional[Destination],
        bounds: Optional[MapBounds],
        nav_ready: bool,
        now: float,
    ) -> list:
        """확인 질문("…로 안내해 드릴까요?")에 대한 네/아니오의 정식 통로."""
        if self.state != State.CONFIRMING:
            return []
        if not affirmative:
            self._to_idle()
            return [Say(MSG_CONFIRM_TIMEOUT, priority="response")]
        if dest is None or dest.id != (self._confirming_dest_id or ""):
            return []
        confirmed = IntentData(
            intent="navigate",
            matched_destination_id=dest.id,
            need_confirm=False,
            safety_flag="normal",
        )
        return self.on_intent(confirmed, dest, bounds, nav_ready, now)

    def _force_clear_all(self, now: float) -> list:
        """진행 중인 모든 활동을 강제 정리한다 (앱 선점·전체 취소 전용)."""
        actions: list = [StopSpeech(), SetNavSpeedLimit(NO_SPEED_LIMIT)]
        if self.state in _GOAL_ACTIVE_STATES:
            actions.append(CancelNav(self.active_destination))
        self._reset_arrival_dialog()
        self._to_idle()
        self._forget_interrupted_return()
        return actions

    def on_app_destination(self, dest: Optional[Destination],
                           bounds: Optional[MapBounds], nav_ready: bool,
                           now: float) -> tuple:
        """앱(관리자) 새 목적지 — ESTOPPED 만 빼고 어느 상태든 선점한다."""
        if self.estop_active or self.state == State.ESTOPPED:
            return [], GateReason.ESTOP_ACTIVE
        if dest is None:
            return [], GateReason.UNKNOWN_DESTINATION
        if dest.authorization != "public":
            return [], GateReason.PRIVATE_DESTINATION
        if not dest.is_approachable:
            return [], GateReason.NOT_APPROACHABLE
        if not pose_valid(dest, bounds):
            return [], GateReason.POSE_INVALID
        if not nav_ready:
            return [], GateReason.NAV_NOT_READY
        actions = self._force_clear_all(now)
        self.state = State.NAVIGATING
        self.active_destination = dest
        self._nav_from_app = True
        actions.append(Say(say_destination(MSG_START, dest.name)))
        actions.append(Navigate(dest))
        return actions, GateReason.OK

    def on_cancel_request(self, now: float) -> tuple:
        """음성 취소. 게이트(주행·일시정지·대기)를 지킨다 — 앱 취소는"""
        reason = check_cancel_gate(self.state, self.estop_active)
        if reason != GateReason.OK:
            return [], reason
        actions = self._force_clear_all(now)
        actions.append(Say(MSG_CANCELED, priority="response"))
        return actions, GateReason.OK

    def on_app_cancel(self, now: float) -> tuple:
        """앱(관리자) 취소 — ESTOPPED 만 빼고 어느 상태든 전부 정리하고"""
        if self.estop_active or self.state == State.ESTOPPED:
            return [], GateReason.ESTOP_ACTIVE
        if self.state == State.IDLE:
            self._forget_interrupted_return()
            return [], GateReason.OK
        actions = self._force_clear_all(now)
        actions.append(Say(MSG_CANCELED, priority="response"))
        return actions, GateReason.OK

    def on_pause_request(self, now: float) -> tuple:
        """일시정지. goal 을 취소하되 목적지는 보관해 재개할 수 있게 둔다."""
        reason = check_pause_gate(self.state, self.estop_active)
        if reason != GateReason.OK:
            return [], reason

        destination = self.active_destination
        actions: list = [
            SetNavSpeedLimit(NO_SPEED_LIMIT),
            CancelNav(destination, event="goal_paused"),
        ]
        self.state = State.PAUSED
        self.paused_destination = destination
        self.active_destination = None
        self.cancel_confirm_pending = False
        self._cancel_confirm_deadline = None
        self._announced_milestones = set()
        self._distance_baseline = None
        self._approach.reset()
        actions.append(Say(MSG_PAUSED, priority="response"))
        return actions, GateReason.OK

    def on_resume_request(self, nav_ready: bool, now: float) -> tuple:
        """다시 출발. 보관한 목적지로 새 goal 을 만든다."""
        reason = check_resume_gate(
            self.state, self.paused_destination, self.estop_active, nav_ready
        )
        if reason != GateReason.OK:
            return [], reason

        destination = self.paused_destination
        assert destination is not None
        self.state = State.NAVIGATING
        self.active_destination = destination
        self.paused_destination = None
        self._announced_milestones = set()
        self._distance_baseline = None
        self._approach.reset()
        return (
            [
                SetNavSpeedLimit(NO_SPEED_LIMIT),
                Say(say_destination(MSG_RESUMED, destination.name)),
                Navigate(destination),
            ],
            GateReason.OK,
        )

    def on_cancel_confirm_request(self, now: float) -> tuple:
        """음성 취소 요청. 바로 취소하지 않고 사용자에게 되묻는다."""
        reason = check_cancel_gate(self.state, self.estop_active)
        if reason != GateReason.OK:
            return [], reason
        self.cancel_confirm_pending = True
        self._cancel_confirm_deadline = now + self.confirm_timeout_sec
        return [
            Say(MSG_CANCEL_CONFIRM, priority="response", expects_reply=True)
        ], GateReason.OK

    def on_cancel_confirm_answer(self, affirmative: bool, now: float) -> list:
        """취소 재확인에 대한 응답. 긍정이면 실제로 취소한다."""
        if not self.cancel_confirm_pending:
            return []
        self.cancel_confirm_pending = False
        self._cancel_confirm_deadline = None
        if not affirmative:
            return [Say(MSG_CANCEL_KEPT, priority="response")]
        actions, _ = self.on_cancel_request(now)
        return actions

    def on_approach_request(
        self,
        request: ApproachRequest,
        bounds: Optional[MapBounds],
        nav_ready: bool,
        now: float,
    ) -> tuple:
        """사람 접근 요청. (actions, GateReason) 을 돌려준다."""
        self._prune_suppressed(now)
        reason = check_approach_gate(
            request,
            self.state,
            self.approach_track_id,
            bounds,
            self.estop_active,
            nav_ready,
            self._is_suppressed(request.track_id, now),
        )
        if reason != GateReason.OK:
            return [], reason

        destination = approach_destination(request)

        if self.state == State.APPROACHING:
            if not self._approach_goal_moved(request.goal):
                return [], GateReason.OK
            self.approach_goal_pose = request.goal
            self.active_destination = destination
            return [Navigate(destination)], GateReason.OK

        self.state = State.APPROACHING
        self.active_destination = destination
        self.approach_track_id = request.track_id
        self.approach_goal_pose = request.goal
        self._response_deadline = None
        self._announced_milestones = set()
        self._distance_baseline = None
        self._approach.reset()
        return (
            [
                SetNavSpeedLimit(self.person_approach_speed_percent),
                Navigate(destination),
            ],
            GateReason.OK,
        )

    def on_approach_cancel_request(self, now: float) -> tuple:
        """접근 포기·대상 이탈 통보(/vica/mission/cancel_approach)."""
        reason = check_approach_cancel_gate(self.state, self.estop_active)
        if reason != GateReason.OK:
            return [], reason

        actions: list = []
        if self.state == State.APPROACHING:
            actions.append(CancelNav(self.active_destination))
        actions.extend(self._enter_returning(now))
        return actions, GateReason.OK

    def on_approach_question_spoken(self, now: float) -> list:
        """질문 재생이 끝났다. 응답 대기 8초는 여기서부터 센다(설계 6.2절)."""
        if self.state != State.AWAITING_USER:
            return []
        self._response_deadline = now + self.approach_response_timeout_sec
        return []

    def on_handle_hint_spoken(self, now: float) -> list:
        """손잡이 위치 안내(MSG_HANDLE_HINT) 재생이 끝났다 — 이 순간 진동을"""
        if not self._pending_handle_hint:
            return []
        self._pending_handle_hint = False
        return [Haptic(HAPTIC_PATTERN_HANDLE_HINT)]

    def _enter_awaiting_user(self, now: float) -> list:
        """질문을 던지고 AWAITING_USER 로 들어간다."""
        self.state = State.AWAITING_USER
        self._response_deadline = now + APPROACH_QUESTION_STUCK_SEC
        self._approach.reset()
        return [
            SetNavSpeedLimit(NO_SPEED_LIMIT),
            Say(MSG_APPROACH_QUESTION, priority="response", expects_reply=True),
        ]

    def on_person_detection(
        self,
        track_id: int,
        distance_m: float,
        stable: bool,
        approachable: bool,
        now: float,
    ) -> list:
        """/vica/person_detection 원본 결과 (근접 호출, 2026-09-10 확장)."""
        if self.state != State.IDLE or self._seek_deadline is None:
            return []
        if approachable or not stable:
            return []
        if track_id == TRACK_ID_NONE:
            return []
        if math.isnan(distance_m) or distance_m >= self.near_call_max_m:
            return []
        if self.estop_active:
            return []
        self._prune_suppressed(now)
        if self._is_suppressed(track_id, now):
            return []

        self._seek_deadline = None
        self._seek_return_yaw = None
        self.approach_track_id = track_id
        self.active_destination = None
        self._near_call_no_spin = distance_m < self.near_call_no_spin_m
        return self._enter_awaiting_user(now)

    def on_approach_answer(self, affirmative: bool, now: float) -> list:
        """접근 질문에 대한 사람의 답. 여기서는 갈래만 만든다."""
        if self.state != State.AWAITING_USER:
            return []

        if affirmative:
            track_id = self.approach_track_id
            self._suppress_track(track_id, now)
            if self.approach_turn_yaw_rad == 0.0 or self._near_call_no_spin:
                self._to_idle()
                self._arm_dest_prompt(now)
                self._user_attached_until = now + USER_ATTACHED_SUPPRESS_SEC
                self._pending_handle_hint = True
                return [
                    Say(MSG_HANDLE_HINT, priority="response"),
                    Say(MSG_APPROACH_ONBOARDING, priority="response",
                        expects_reply=True),
                ]
            self.state = State.TURNING
            self.active_destination = None
            self._response_deadline = None
            self._turn_deadline = now + APPROACH_TURN_TIMEOUT_SEC
            return [
                Say(MSG_APPROACH_ACCEPTED, priority="response"),
                SpinInPlace(self.approach_turn_yaw_rad, reason="수락 — 핸들을 사람 쪽으로"),
            ]

        actions: list = [Say(MSG_APPROACH_DECLINED, priority="response")]
        if self._never_approached:
            self._to_idle()
        else:
            actions.extend(self._enter_returning(now))
        return actions

    def is_awaiting_arrival_answer(self) -> bool:
        """도착 후 질문의 답을 기다리는 중인가. 노드가 라우팅에 쓴다 —"""
        return self.state in (State.ASKING_NEXT, State.ASKING_WAIT_TIME)

    def is_waiting_in_place(self) -> bool:
        """WAITING(제자리 대기) 중인가. 이때 "비카야"는 on_wake 로 간다."""
        return self.state == State.WAITING

    def on_listen_state(self, state: str, now: float) -> list:
        """/vica/listen_state (open/speech/closed/empty[:이유]). 발화 없음."""
        if state in ("open", "speech"):
            if not self._ear_busy:
                self._ear_busy_since = now
            self._ear_busy = True
            if state == "speech":
                if not self._ear_speaking:
                    self._ear_speech_since = now
                self._ear_speaking = True
            else:
                self._ear_speaking = False
        elif state == "closed":
            self._ear_busy = False
            self._ear_speaking = False
            self._ear_grace_until = now + EAR_GRACE_SEC
        else:
            self._ear_busy = False
            self._ear_speaking = False
            self._ear_grace_until = None
        if state.startswith("empty") and self._dest_prompt_stage in (
            "asked", "retried",
        ):
            return self._advance_dest_prompt(now)
        return []

    def _dest_prompt_holds(self, now: float) -> bool:
        """온보딩 되묻기 사다리의 시계를 잡아둘 이유가 있는가."""
        if (self._ear_speaking and self._ear_speech_since is not None
                and now - self._ear_speech_since <= EAR_HOLD_MAX_SEC):
            return True
        return (self._ear_grace_until is not None
                and now < self._ear_grace_until)

    def on_dest_prompt_spoken(self, now: float) -> list:
        """온보딩(MSG_APPROACH_ONBOARDING)·되묻기(MSG_DEST_RETRY) 재생이"""
        if self._dest_prompt_stage in ("asked", "retried"):
            self._dest_prompt_deadline = now + DEST_ANSWER_WAIT_SEC
        return []

    def _ear_holds(self, now: float) -> bool:
        """무응답 시계를 잡아둘 이유가 있는가. 상한(EAR_HOLD_MAX_SEC)은"""
        if (self._ear_busy and self._ear_busy_since is not None
                and now - self._ear_busy_since <= EAR_HOLD_MAX_SEC):
            return True
        return (self._ear_grace_until is not None
                and now < self._ear_grace_until)

    def exit_arrival_dialog(self) -> None:
        """도착 후 대화를 조용히 닫는다 — 새 목적지 '제안'(need_confirm=True)이"""
        if self.state in (State.ASKING_NEXT, State.ASKING_WAIT_TIME):
            self._reset_arrival_dialog()
            self.state = State.IDLE
            self._seek_deadline = None
            self._seek_return_yaw = None

    def _ask_arrival(self, dest: Optional[Destination], now: float,
                     arrival_text: str = "") -> list:
        """유형별 질문을 던지고 ASKING_NEXT 로 들어간다. "네"의 뜻(_asking_is_finish)"""
        category = (dest.category if dest else "") or ""
        if category == "restroom":
            question, is_finish, ask_time = MSG_ASK_RESTROOM, False, False
        elif category == "entrance":
            question, is_finish, ask_time = MSG_ASK_ENTRANCE, True, False
        else:
            question, is_finish, ask_time = MSG_ASK_GENERIC, False, True
        self.state = State.ASKING_NEXT
        self.active_destination = None
        self._asking_is_finish = is_finish
        self._asking_time_after_yes = ask_time
        self._asking_entered_at = now
        self._arrival_retried = False
        self._leaving_deadline = None
        self._response_deadline = None
        text = f"{arrival_text} {question}".strip() if arrival_text else question
        return [Say(text, priority="response", expects_reply=True)]

    def on_arrival_question_spoken(self, now: float) -> list:
        """도착 후 질문 재생이 끝났다. 응답 대기 8초는 여기서부터 센다"""
        if self.state in (State.ASKING_NEXT, State.ASKING_WAIT_TIME):
            self._response_deadline = now + self.approach_response_timeout_sec
        return []

    def on_wake(self, now: float) -> list:
        """"비카야" — 새 대화의 시작 신호."""
        self._forget_dest_prompt()
        if self.state == State.WAITING:
            self._wait_until = None
            self._reset_arrival_dialog()
            self._to_idle()
            self._wake_consumed_at = now
            return []
        if self.state == State.CONFIRMING:
            self._to_idle()
            self._wake_consumed_at = now
            return []
        if self.state in (State.ASKING_NEXT, State.ASKING_WAIT_TIME):
            self._reset_arrival_dialog()
            self._to_idle()
            self._wake_consumed_at = now
            return []
        if self.state == State.AWAITING_USER:
            if self.wake_guard_active(now):
                return []
            self._suppress_track(self.approach_track_id, now)
            self._approach.reset()
            self._to_idle()
            self._wake_consumed_at = now
            return []
        if self.user_attached_guard_active(now):
            self._user_attached_until = now + USER_ATTACHED_SUPPRESS_SEC
        return []

    def wake_guard_active(self, now: float) -> bool:
        """`_wake_consumed_at` 직후 가드가 지금 유효한가."""
        return (self._wake_consumed_at is not None
                and now - self._wake_consumed_at < WAKE_CONSUMED_GUARD_SEC)

    def user_attached_guard_active(self, now: float) -> bool:
        """접근 온보딩 직후 억제(`_user_attached_until`)가 지금 유효한가."""
        return (self._user_attached_until is not None
                and now < self._user_attached_until)

    def on_wake_doa(self, doa_deg: float, nav_ready: bool, now: float) -> list:
        """"비카야"가 온 방향으로 고개를 돌린다 (호출 접근 설계 §4)."""
        if self.state != State.IDLE or self.estop_active or not nav_ready:
            return []
        if self._return_interrupted:
            return []
        if self.wake_guard_active(now):
            return []
        if self.user_attached_guard_active(now):
            return []
        yaw = doa_to_spin_yaw(doa_deg, self.wake_doa_sign)
        back = wrap_to_pi((self._seek_return_yaw or 0.0) - yaw)
        if abs(yaw) < SEEK_MIN_YAW_RAD:
            self._seek_return_yaw = back
            self._seek_deadline = now + self.seek_look_sec
            return []
        if abs(yaw) > self.handle_side_min_yaw_rad:
            self._seek_return_yaw = None
            self._seek_deadline = None
            self.approach_track_id = None
            self.active_destination = None
            self._near_call_no_spin = True
            self._never_approached = True
            self._wake_consumed_at = now
            return self._enter_awaiting_user(now)
        self.state = State.SEEKING
        self._seek_return_yaw = back
        self._seek_deadline = None
        self._turn_deadline = now + SEEK_TURN_TIMEOUT_SEC
        return [SpinInPlace(yaw, reason="호출 방향으로")]

    def on_arrival_answer(self, intent: "IntentData", now: float,
                          next_dest: Optional[Destination] = None) -> list:
        """도착 후 질문에 대한 답. 정리된 intent 만 받아 갈래를 만든다."""
        if self.state not in (State.ASKING_NEXT, State.ASKING_WAIT_TIME):
            return []
        kind = intent.intent

        if kind == "navigate" and next_dest is not None:
            self.state = State.NAVIGATING
            self.active_destination = next_dest
            self._reset_arrival_dialog()
            return [SetNavSpeedLimit(NO_SPEED_LIMIT), Navigate(next_dest)]

        if (kind in ("finish", "cancel")
                or (kind == "affirm" and self._asking_is_finish)):
            self._reset_arrival_dialog()
            return [Say(MSG_FINISH, priority="response"), *self._go_home(now)]

        if self.state == State.ASKING_WAIT_TIME and kind in ("affirm", "deny"):
            return self._arrival_no_answer(now)

        if kind == "wait" or (kind == "affirm" and not self._asking_is_finish):
            minutes = intent.wait_minutes if kind == "wait" else -1
            if minutes is None or minutes < 0:
                if (self.state == State.ASKING_NEXT
                        and not self._asking_time_after_yes):
                    return self._enter_waiting(WAIT_MINUTES_CAP, now,
                                               default_msg=True)
                self.state = State.ASKING_WAIT_TIME
                self._asking_entered_at = now
                self._response_deadline = None
                return [Say(MSG_ASK_WAIT_TIME, priority="response",
                            expects_reply=True)]
            return self._enter_waiting(min(minutes, WAIT_MINUTES_CAP), now)

        if kind == "deny":
            if self._asking_is_finish:
                return self._enter_waiting(WAIT_MINUTES_CAP, now,
                                           default_msg=True)
            self._reset_arrival_dialog()
            return [Say(MSG_FINISH, priority="response"), *self._go_home(now)]

        return self._arrival_no_answer(now)

    def _enter_waiting(self, minutes: int, now: float,
                       default_msg: bool = False) -> list:
        """WAITING 진입 + 대기 확정 멘트. 사람접근은 WAITING 상태값으로 자연히 꺼진다."""
        self.state = State.WAITING
        self._wait_until = now + minutes * 60.0
        self._response_deadline = None
        self._leaving_deadline = None
        msg = (MSG_WAIT_DEFAULT if default_msg
               else MSG_WAIT_CONFIRM.format(minutes=minutes))
        return [Say(msg, priority="response")]

    def _arrival_no_answer(self, now: float) -> list:
        """무응답 사다리: 못 알아들으면 1회 재질문, 그 뒤엔 떠나기 예고."""
        if not self._arrival_retried:
            self._arrival_retried = True
            self._response_deadline = None
            self._asking_entered_at = now
            return [Say(MSG_ARRIVAL_RETRY, priority="response", expects_reply=True)]
        return self._leaving_notice(now)

    def _leaving_notice(self, now: float) -> list:
        """떠나기 예고 + 유예. 유예 안에 답이 오면 산다(on_arrival_answer)."""
        self._leaving_deadline = now + LEAVING_GRACE_SEC
        self._response_deadline = None
        return [Say(MSG_LEAVING_NOTICE, priority="response")]

    def _go_home(self, now: float) -> list:
        """안내 종료의 홈 복귀(도착 후 대화의 종료 답·무응답 떠남·대기 만료)."""
        return self._enter_returning(now, dialog_finish=True)

    def _arm_dest_prompt(self, now: float) -> None:
        """온보딩 질문(MSG_APPROACH_ONBOARDING)을 던진 직후 되묻기 사다리를"""
        self._dest_prompt_stage = "asked"
        self._dest_prompt_deadline = now + DEST_PROMPT_FALLBACK_SEC
        self._forget_interrupted_return()

    def _forget_dest_prompt(self) -> None:
        """온보딩 되묻기 사다리를 청산한다 — 대화가 다른 경로로 넘어가거나"""
        self._dest_prompt_stage = None
        self._dest_prompt_deadline = None

    def _advance_dest_prompt(self, now: float) -> list:
        """되묻기 사다리를 한 단 전진한다."""
        if self._dest_prompt_stage == "asked":
            self._dest_prompt_stage = "retried"
            self._dest_prompt_deadline = now + DEST_PROMPT_FALLBACK_SEC
            return [Say(MSG_DEST_RETRY, priority="response", expects_reply=True)]
        if self._dest_prompt_stage == "retried":
            self._dest_prompt_stage = "leaving"
            self._dest_prompt_deadline = now + self.dest_retry_return_sec
            return []
        if self._dest_prompt_stage == "leaving":
            self._dest_prompt_stage = "notice"
            self._dest_prompt_deadline = now + LEAVING_GRACE_SEC
            return [Say(MSG_LEAVING_NOTICE, priority="response")]
        if self._dest_prompt_stage == "notice":
            self._forget_dest_prompt()
            return self._go_home(now)
        return []

    def on_return_brake(self, now: float, quiet: bool = False) -> list:
        """홈 복귀 중 "비카야"/늦은 답 (작업 E). 복귀를 취소한다."""
        if self.state != State.RETURNING:
            return []
        self._wake_consumed_at = now
        cancel_dest = self.active_destination
        self.active_destination = None
        actions: list = [SetNavSpeedLimit(NO_SPEED_LIMIT)]
        if cancel_dest is not None:
            actions.append(CancelNav(cancel_dest))
        if quiet:
            self.state = State.ASKING_NEXT
            self._asking_is_finish = False
            self._arrival_retried = False
            self._response_deadline = None
            self._asking_entered_at = now
        else:
            self._reset_arrival_dialog()
            self._to_idle()
            self._return_interrupted = True
            self._return_resume_deadline = now + self.return_resume_sec
            self._return_notice_given = False
        return actions

    def _reset_arrival_dialog(self) -> None:
        self._asking_is_finish = False
        self._asking_time_after_yes = False
        self._asking_entered_at = None
        self._arrival_retried = False
        self._leaving_deadline = None
        self._wait_until = None
        self._response_deadline = None

    def _forget_interrupted_return(self) -> None:
        """복귀 재개 사다리를 청산한다 — "복귀가 끊겨 있다"는 사실과 그"""
        self._return_interrupted = False
        self._return_resume_deadline = None
        self._return_notice_given = False

    def on_emergency(self, keyword: str, now: float) -> list:
        """/vica/emergency (긴급어). 하드 키워드만 처리 — LLM 을 거치지 않은 경로."""
        if keyword not in HARD_EMERGENCY_KEYWORDS:
            return []

        actions: list = []
        if self.state in _GOAL_ACTIVE_STATES:
            actions.append(SetNavSpeedLimit(NO_SPEED_LIMIT))
            actions.append(CancelNav(self.active_destination))
        already_estopped = self.state == State.ESTOPPED
        moving = self.state in _GOAL_ACTIVE_STATES
        self._enter_estopped(now)
        if not already_estopped:
            self._estop_announced = moving
            if moving:
                actions.append(Say(MSG_ESTOPPED, priority="emergency"))
        return actions

    def on_estop(self, active: bool, now: float) -> list:
        """/emergency_stop 래치 상태 (emergency_stop_node 가 20Hz 주기 발행)."""
        self.estop_active = active
        if active:
            self._estop_clear_since = None
            if self.state != State.ESTOPPED:
                actions: list = []
                if self.state in _GOAL_ACTIVE_STATES:
                    actions.append(SetNavSpeedLimit(NO_SPEED_LIMIT))
                    actions.append(CancelNav(self.active_destination))
                moving = self.state in _GOAL_ACTIVE_STATES
                self._enter_estopped(now)
                self._estop_announced = moving
                if moving:
                    actions.append(Say(MSG_ESTOPPED, priority="emergency"))
                return actions
        else:
            if self.state == State.ESTOPPED and self._estop_clear_since is None:
                self._estop_clear_since = now
        return []

    def on_tick(
        self,
        now: float,
        nav_status: NavStatus,
        distance_remaining: Optional[float] = None,
    ) -> list:
        """주기 처리. distance_remaining 은 Nav2 feedback 의 남은 거리(m)다."""
        actions: list = []
        self._prune_suppressed(now)

        if self.state == State.CONFIRMING:
            if self._confirm_deadline is not None and now >= self._confirm_deadline:
                self._to_idle()
                actions.append(Say(MSG_CONFIRM_TIMEOUT))

        elif self.state == State.NAVIGATING:
            if (
                self.cancel_confirm_pending
                and self._cancel_confirm_deadline is not None
                and now >= self._cancel_confirm_deadline
            ):
                self.cancel_confirm_pending = False
                self._cancel_confirm_deadline = None

            if nav_status == NavStatus.SUCCEEDED:
                dest = self.active_destination
                text = (
                    dest.arrival_message
                    if dest and dest.arrival_message
                    else MSG_ARRIVED_FALLBACK.format(name=dest.name if dest else "목적지")
                )
                self._approach.reset()
                actions.append(SetNavSpeedLimit(NO_SPEED_LIMIT))
                if self.arrival_dialog and not self._nav_from_app:
                    actions.extend(self._ask_arrival(dest, now, arrival_text=text))
                else:
                    self.state = State.ARRIVED
                    self._dwell_until = now + self.dwell_sec
                    actions.append(Say(text))
            elif nav_status == NavStatus.RUNNING:
                approach_percent = self._approach.update(distance_remaining)
                if approach_percent is not None:
                    actions.append(SetNavSpeedLimit(approach_percent))
                milestone = self._crossed_milestone(distance_remaining)
                if milestone is not None:
                    actions.append(
                        Say(MSG_DISTANCE_REMAINING.format(meters=int(milestone)))
                    )
            elif nav_status in (NavStatus.FAILED, NavStatus.CANCELED):
                failed_dest = self.active_destination
                self.state = State.FAILED
                self._dwell_until = now + self.dwell_sec
                self._approach.reset()
                actions.append(SetNavSpeedLimit(NO_SPEED_LIMIT))

                retryable = (
                    nav_status == NavStatus.FAILED
                    and failed_dest is not None
                    and self._nav_retry_count < self.nav_retry_limit
                )
                if retryable:
                    self._nav_retry_count += 1
                    self._retry_destination = failed_dest
                    self._retry_at = now + self.nav_retry_delay_sec
                else:
                    self._retry_destination = None
                    self._retry_at = None
                    actions.append(Say(MSG_NAV_FAILED, priority="response"))

        elif self.state == State.APPROACHING:
            if nav_status == NavStatus.SUCCEEDED:
                self._near_call_no_spin = False
                actions.extend(self._enter_awaiting_user(now))
            elif nav_status in (NavStatus.FAILED, NavStatus.CANCELED):
                actions.extend(self._enter_returning(now))

        elif self.state == State.SEEKING:
            if nav_status in (NavStatus.SUCCEEDED, NavStatus.FAILED,
                              NavStatus.CANCELED):
                back = self._seek_return_yaw
                self._to_idle()
                if back is not None:
                    self._seek_return_yaw = back
                    self._seek_deadline = now + self.seek_look_sec
            elif (self._turn_deadline is not None
                  and now >= self._turn_deadline):
                self._to_idle()

        elif self.state == State.IDLE:
            if self._seek_deadline is not None and now >= self._seek_deadline:
                back = self._seek_return_yaw
                self._seek_deadline = None
                self._seek_return_yaw = None
                if back is not None and abs(back) >= SEEK_MIN_YAW_RAD:
                    self.state = State.SEEKING
                    self._turn_deadline = now + SEEK_TURN_TIMEOUT_SEC
                    actions.append(SpinInPlace(back, reason="못 찾아 원위치로"))

            if self.state == State.IDLE and self._return_interrupted:
                if self._return_resume_deadline is None:
                    self._return_resume_deadline = now + self.return_resume_sec
                elif now >= self._return_resume_deadline:
                    if not self._return_notice_given:
                        self._return_notice_given = True
                        self._return_resume_deadline = now + LEAVING_GRACE_SEC
                        actions.append(Say(MSG_LEAVING_NOTICE, priority="response"))
                    else:
                        self._forget_interrupted_return()
                        actions.extend(self._go_home(now))

            if (self.state == State.IDLE and self._dest_prompt_stage is not None
                    and self._dest_prompt_deadline is not None
                    and now >= self._dest_prompt_deadline
                    and not self._dest_prompt_holds(now)):
                actions.extend(self._advance_dest_prompt(now))

        elif self.state == State.TURNING:
            if nav_status == NavStatus.SUCCEEDED:
                self._to_idle()
                self._arm_dest_prompt(now)
                self._user_attached_until = now + USER_ATTACHED_SUPPRESS_SEC
                self._pending_handle_hint = True
                actions.append(Say(MSG_HANDLE_HINT, priority="response"))
                actions.append(Say(MSG_APPROACH_ONBOARDING, priority="response",
                                   expects_reply=True))
            elif nav_status in (NavStatus.FAILED, NavStatus.CANCELED):
                self._to_idle()
                self._arm_dest_prompt(now)
                self._user_attached_until = now + USER_ATTACHED_SUPPRESS_SEC
                self._pending_handle_hint = True
                actions.append(Say(MSG_HANDLE_HINT, priority="response"))
                actions.append(Say(MSG_APPROACH_ONBOARDING, priority="response",
                                   expects_reply=True))
            elif (self._turn_deadline is not None
                  and now >= self._turn_deadline):
                self._to_idle()

        elif self.state == State.AWAITING_USER:
            if self._response_deadline is not None and now >= self._response_deadline:
                actions.append(Say(MSG_APPROACH_NO_ANSWER, priority="response"))
                if self._never_approached:
                    self._to_idle()
                else:
                    actions.extend(self._enter_returning(now))

        elif self.state in (State.ASKING_NEXT, State.ASKING_WAIT_TIME):
            if (self._leaving_deadline is not None
                    and now >= self._leaving_deadline
                    and not self._ear_holds(now)):
                self._reset_arrival_dialog()
                actions.extend(self._go_home(now))
            elif (self._leaving_deadline is None
                  and self._response_deadline is not None
                  and now >= self._response_deadline
                  and not self._ear_holds(now)):
                actions.extend(self._leaving_notice(now))
            elif (self._leaving_deadline is None
                  and self._response_deadline is None
                  and self._asking_entered_at is not None
                  and now - self._asking_entered_at >= ASKING_STUCK_FALLBACK_SEC
                  and not self._ear_holds(now)):
                actions.extend(self._leaving_notice(now))

        elif self.state == State.WAITING:
            if self._wait_until is not None and now >= self._wait_until:
                self._reset_arrival_dialog()
                actions.extend(self._go_home(now))

        elif self.state == State.RETURNING:
            if self.active_destination is None or nav_status in (
                NavStatus.SUCCEEDED,
                NavStatus.FAILED,
                NavStatus.CANCELED,
            ):
                self._finish_returning(now)

        elif self.state in (State.ARRIVED, State.FAILED):
            pending_retry = (
                self.state == State.FAILED
                and self._retry_at is not None
                and self._retry_destination is not None
            )
            if pending_retry and self.estop_active:
                self._retry_at = None
                self._retry_destination = None
                pending_retry = False
            if pending_retry:
                if now >= self._retry_at:
                    dest = self._retry_destination
                    self._retry_at = None
                    self._retry_destination = None
                    self.state = State.NAVIGATING
                    self.active_destination = dest
                    self._dwell_until = None
                    self._announced_milestones = set()
                    self._distance_baseline = None
                    self._approach.reset()
                    actions.append(SetNavSpeedLimit(NO_SPEED_LIMIT))
                    actions.append(Navigate(dest))
            elif self._dwell_until is None or now >= self._dwell_until:
                self._to_idle()

        elif self.state == State.ESTOPPED:
            if not self.estop_active:
                t0 = (
                    self._estop_clear_since
                    if self._estop_clear_since is not None
                    else self._estop_entered_at
                )
                if t0 is not None and now - t0 >= self.estop_release_grace_sec:
                    announced = self._estop_announced
                    self._estop_announced = False
                    self._to_idle()
                    if announced:
                        actions.append(Say(MSG_ESTOP_RELEASED, priority="emergency"))

        return actions

    def _crossed_milestone(self, distance_remaining: Optional[float]) -> Optional[float]:
        """이번 tick 에 새로 지난 안내 지점을 돌려준다 (없으면 None)."""
        if distance_remaining is None or distance_remaining <= 0.0:
            return None

        if self._distance_baseline is None:
            self._distance_baseline = distance_remaining
            self._announced_milestones.update(
                m for m in DISTANCE_MILESTONES_M if m >= distance_remaining
            )

        crossed = [
            m
            for m in DISTANCE_MILESTONES_M
            if distance_remaining <= m and m not in self._announced_milestones
        ]
        if not crossed:
            return None

        self._announced_milestones.update(crossed)
        return min(crossed)

    def _approach_goal_moved(self, goal: Optional[Pose2D]) -> bool:
        """접근 goal 을 다시 보낼 만큼 사람이 움직였는가."""
        import math

        if goal is None or self.approach_goal_pose is None:
            return True
        moved = math.hypot(
            goal.x - self.approach_goal_pose.x, goal.y - self.approach_goal_pose.y
        )
        return moved >= self.approach_goal_update_m

    def on_return_home_request(self, nav_ready: bool, now: float) -> tuple:
        """관리자가 앱에서 홈 복귀를 요청했다(`/vica/mission/return_home`)."""
        reason = check_return_home_gate(
            self.state, self.return_destination, self.estop_active, nav_ready
        )
        if reason is not GateReason.OK:
            return False, reason, []
        self._forget_interrupted_return()
        return True, reason, self._enter_returning(now, is_home=True)

    def _enter_returning(
        self, now: float, is_home: bool = False, dialog_finish: bool = False
    ) -> list:
        """대기 위치(= 홈)로 돌아간다."""
        destination = (
            self.return_destination
            if is_home or dialog_finish or self.auto_return_home
            else None
        )
        self.state = State.RETURNING
        self._returning_home = is_home
        self.active_destination = destination
        self.approach_goal_pose = None
        self._response_deadline = None
        self._announced_milestones = set()
        self._distance_baseline = None
        self._approach.reset()
        self._forget_dest_prompt()
        actions: list = [SetNavSpeedLimit(NO_SPEED_LIMIT)]
        if destination is not None:
            actions.append(Navigate(destination))
        return actions

    def _finish_returning(self, now: float) -> None:
        """복귀 완료."""
        track_id = self.approach_track_id
        was_home = self._returning_home
        self._to_idle()
        if not was_home:
            self._suppress_track(track_id, now)

    def _suppress_track(self, track_id: Optional[int], now: float) -> None:
        """이 사람에게 당분간 다시 다가가지 않는다."""
        if track_id is None or track_id == TRACK_ID_NONE:
            return
        self._suppressed_tracks[track_id] = now + self.reapproach_suppress_sec

    def _is_suppressed(self, track_id: int, now: float) -> bool:
        until = self._suppressed_tracks.get(track_id)
        return until is not None and now < until

    def _prune_suppressed(self, now: float) -> None:
        """지난 억제를 버린다. 하루 종일 서 있으면 track_id 가 계속 쌓인다."""
        expired = [t for t, until in self._suppressed_tracks.items() if now >= until]
        for track_id in expired:
            del self._suppressed_tracks[track_id]

    def _enter_estopped(self, now: float) -> None:
        self.state = State.ESTOPPED
        self.active_destination = None
        self.paused_destination = None
        self.cancel_confirm_pending = False
        self._cancel_confirm_deadline = None
        self._confirming_dest_id = None
        self._confirm_deadline = None
        self._estop_entered_at = now
        self._estop_clear_since = None
        self._announced_milestones = set()
        self._distance_baseline = None
        self._approach.reset()
        self._suppress_track(self.approach_track_id, now)
        self.approach_track_id = None
        self.approach_goal_pose = None
        self._response_deadline = None
        self._nav_from_app = False

    def _to_idle(self) -> None:
        self.state = State.IDLE
        self.active_destination = None
        self.paused_destination = None
        self.cancel_confirm_pending = False
        self._cancel_confirm_deadline = None
        self._confirming_dest_id = None
        self._confirm_deadline = None
        self._dwell_until = None
        self._estop_entered_at = None
        self._estop_clear_since = None
        self._turn_deadline = None
        self._seek_return_yaw = None
        self._seek_deadline = None
        self._return_resume_deadline = None
        self._return_notice_given = False
        self._dest_prompt_stage = None
        self._dest_prompt_deadline = None
        self._announced_milestones = set()
        self._distance_baseline = None
        self._approach.reset()
        self._nav_retry_count = 0
        self._retry_destination = None
        self._retry_at = None
        self.approach_track_id = None
        self._returning_home = False
        self.approach_goal_pose = None
        self._response_deadline = None
        self._nav_from_app = False
        self._near_call_no_spin = False
        self._never_approached = False
