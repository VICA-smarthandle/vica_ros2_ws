"""vica_mission_manager 게이트·상태 전이 순수 로직."""
from __future__ import annotations

import math
import uuid

from dataclasses import dataclass, replace
from enum import Enum
from typing import Callable, Optional, Sequence, Union

from .approach_recheck import (
    RECHECK_RESEND_M,
    SAME_PERSON_RADIUS_M,
    SUPPRESS_RADIUS_M,
    MidcourseRecheck,
    same_person,
)
from .approach_speed import ApproachSpeedLadder, NO_SPEED_LIMIT
from .grip_meter import GripMeter

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
    WAITING_RELEASE = "waiting_release"
    MOVING_TO_WAIT_SPOT = "moving_to_wait_spot"
    MOVING_BACK_TO_DEST = "moving_back_to_dest"


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
    NO_WAIT_SPOT = "no_wait_spot"


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw_deg: float
    frame_id: str = "map"


@dataclass(frozen=True)
class WaitSpot:
    """목적지에 딸린 대기 장소 (2026-10-07). destinations.yaml 의 wait_spot 칸."""

    x: float
    y: float
    yaw_deg: float
    side: str


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
    door_yaw_deg: Optional[float] = None
    wait_spot: Optional[WaitSpot] = None


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
    reply: str = ""


@dataclass(frozen=True)
class ApproachRequest:
    """RequestApproach.srv 한 건 중 게이트 판단에 쓰는 값만."""

    goal: Optional[Pose2D]
    track_id: int
    approachable: bool = True
    person: Optional[Pose2D] = None


@dataclass(frozen=True)
class Say:
    text: str
    priority: str = "narration"
    expects_reply: bool = False


@dataclass(frozen=True)
class Navigate:
    destination: Destination
    tree: str = ""


@dataclass(frozen=True)
class GoalEvent:
    """앱에 알릴 사건 하나. 노드가 /vica_goal_event JSON 으로 낸다."""

    event: str
    destination: Optional[Destination] = None
    reason: str = ""
    wait_place: str = ""
    wait_minutes: int = -1


@dataclass(frozen=True)
class HelpEvent:
    """도움 요청 사건 하나 — 노드가 /vica_goal_event JSON 에 help_id 를 붙여 앱에 낸다."""

    event: str
    help_id: str


@dataclass(frozen=True)
class CancelNav:
    destination: Optional[Destination] = None
    event: str = "goal_canceled"


@dataclass(frozen=True)
class StopSpeech:
    """하던 말을 끊고 대기 중인 비긴급 발화를 비운다 (노드가 /vica/tts_stop 발행)."""


@dataclass(frozen=True)
class WakeReply:
    """"비카야"에 대한 미션의 판정 — 노드가 /vica/wake_reply 로 "listen"/"ignore" 를 낸다."""

    listen: bool


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
    Say, Navigate, CancelNav, SetNavSpeedLimit, Haptic, SpinInPlace, StopSpeech,
    GoalEvent, WakeReply,
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


_DIGIT_HAS_BATCHIM = {
    "0": True, "1": True, "2": False, "3": True, "4": False,
    "5": False, "6": True, "7": True, "8": True, "9": False,
}


def josa_eun_neun(word: str) -> str:
    """단어 뒤에 붙는 조사 '은 / 는' 을 받침에 맞게 돌려준다 (M1, 2026-10-07)."""
    word = (word or "").rstrip()
    if not word:
        return "는"
    last = word[-1]
    if "가" <= last <= "힣":
        return "은" if (ord(last) - 0xAC00) % 28 else "는"
    if last in _DIGIT_HAS_BATCHIM:
        return "은" if _DIGIT_HAS_BATCHIM[last] else "는"
    return "는"


def say_destination(template: str, name: str) -> str:
    """목적지 이름이 들어가는 멘트를 조사까지 맞춰 완성한다."""
    return template.format(name=name, josa=josa_euro(name))


MSG_START = "{name}{josa} 안내를 시작합니다."
MSG_START_DELIVERY = "{name}{josa} 배달을 시작합니다."
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
MSG_ESTOPPED_BY_APP = "안전을 위해 멈추겠습니다."
MSG_TIPPED = "비카가 넘어져 관리자를 호출했습니다. 지나가실 때 주의하세요."
TIP_REPEAT_SEC = 10.0
TIP_SELF_WAKE_GUARD_SEC = 7.0

MSG_WAKE_GREETING = "네?"
MSG_ESTOP_WAKE = "지금은 비상 멈춤 상태입니다."
MSG_CONFIRM_PROMPT_FALLBACK = "{name}{josa} 안내해드릴까요?"
MSG_CONFIRM_SWITCH = "네, {prompt}"

MSG_DISTANCE_REMAINING = "목적지까지 약 {meters}미터 남았습니다."
MSG_CANCELED = "안내를 취소했습니다."

MSG_ASK_RESTROOM = "다녀오시는 동안 여기서 기다릴까요?"
MSG_ASK_ENTRANCE = "여기까지 안내를 마칠까요?"
MSG_ASK_GENERIC = "여기서 대기할까요?"
MSG_ASK_WAIT_TIME = "몇 분쯤 걸리실까요?"
MSG_WAIT_CONFIRM = "{minutes}분 대기하겠습니다. 돌아오시면 '비카야'라고 불러 주세요."
MSG_WAIT_DEFAULT = "네, 최대 30분까지 여기서 기다리겠습니다. 돌아오시면 '비카야'라고 불러 주세요."
MSG_DOOR_SIDE = "{name}{eun} {side}에 있습니다."
MSG_WAIT_SPOT_CONFIRM = (
    "{minutes}분 동안 {place}에서 기다리겠습니다. 돌아오시면 '비카야'라고 불러 주세요.")
MSG_WAIT_SPOT_DEFAULT = (
    "최대 30분 동안 {place}에서 기다리겠습니다. 돌아오시면 '비카야'라고 불러 주세요.")
MSG_WAIT_BEACON = "비카가 대기 중입니다."
MSG_WAIT_SPOT_BLOCKED = "대기 자리가 막혀 입구 앞에서 기다리겠습니다."
MSG_WAIT_EXPIRED = "대기 시간이 종료되어 제자리로 돌아갑니다."
MSG_OBSTACLE_AVOID = "앞에 장애물이 있어 비켜갈게요."
MSG_OBSTACLE_SLOW = "앞에 장애물이 있어 감속할게요."
MSG_OBSTACLE_STOP = "앞에 장애물이 있어 정지합니다."
MIC_OFF_MESSAGES = frozenset({MSG_OBSTACLE_STOP})
OBSTACLE_STALE_SEC = 2.5
MSG_WAIT_FINISH_ASK = "네, 어디로 모실까요?"
MSG_WAIT_NEED_ASK = "안내가 필요 없으신가요?"
QUESTION_REASK_SEC = 15.0
WAIT_PLACE_PHRASES = {"right": "입구 오른쪽", "left": "입구 왼쪽", "across": "입구 맞은편"}
WAIT_PLACE_AT_DESTINATION = "입구 앞"
MSG_FINISH = "안내를 종료합니다."
MSG_ARRIVAL_RETRY = "잘 듣지 못했습니다. 계속 안내가 필요하시면 말씀해 주세요."
MSG_LEAVING_NOTICE = "응답이 없어 안내를 마치고 제자리로 돌아가겠습니다."

WAIT_MINUTES_CAP = 30
WAIT_BEACON_INTERVAL_SEC = 10.0
HOME_BEACON_INTERVAL_SEC = 60.0
HOME_BEACON_RADIUS_M = 0.5
WAIT_RELEASE_SEC = 5.0
WAIT_RELEASE_SPEECH_FALLBACK_SEC = 30.0
WAIT_FINISH_REPEAT_SEC = 30.0
DOOR_FRONT_BACK_DEG = 30.0
WAIT_SPOT_DESTINATION_PREFIX = "wait_spot:"
WAIT_BACK_DESTINATION_PREFIX = "wait_back:"
LEAVING_GRACE_SEC = 3.0
RETURN_RESUME_SEC = 15.0
EAR_GRACE_SEC = 8.0
EAR_HOLD_MAX_SEC = 20.0
STALE_ANSWER_KINDS = ("affirm", "deny")
SAME_QUESTION_SEC = 60.0
LLM_THINKING_TAIL_SEC = 1.0
ASKING_STUCK_FALLBACK_SEC = 30.0
MSG_PAUSED = "잠시 멈추겠습니다. 다시 출발하려면 말씀해 주세요."
MSG_RESUMED = "{name}{josa} 다시 출발합니다."
MSG_CANCEL_CONFIRM = "안내를 취소할까요?"
MSG_CANCEL_KEPT = "안내를 계속하겠습니다."
MSG_NOT_NAVIGATING = "지금은 안내 중이 아닙니다."
MSG_NOT_PAUSED = "다시 출발할 안내가 없습니다."
MSG_ALREADY_GOING = "지금 {name}{josa} 가는 중이에요."
MSG_HELP_CONFIRM = "직원의 도움이 필요하신가요?"
MSG_HELP_SENT = "지금 도움 요청을 보냈습니다"
MSG_HELP_COMING = "관리자가 오고있습니다. 잠시만 기다려주세요"
MSG_HELP_CANCEL_CONFIRM = "도움 요청을 취소할까요?"
MSG_HELP_CANCELED = "도움 요청을 취소했습니다."
MSG_HELP_UNAVAILABLE = "지금은 도움 요청을 보낼 수 없습니다"
HELP_RESEND_SEC = 10.0
MSG_APPROACH_COMING = "동행로봇 비카가 다가가고 있어요."
MSG_APPROACH_QUESTION = "안녕하세요? 시각장애인 안내로봇 비카입니다. 안내를 받으시겠어요?"
MSG_APPROACH_ACCEPTED = "네, 잠시만 기다려주세요. 로봇이 회전하니 주의하세요."
MSG_APPROACH_DECLINED = "알겠습니다. 이만 물러납니다."
MSG_APPROACH_ONBOARDING = "저에게 말을 거실 때는 '비카야'라고 불러주세요. 어디로 가고 싶으신가요?"
MSG_APPROACH_NO_ANSWER = "실례했습니다. 필요하시면 언제든 불러 주세요."
MSG_APPROACH_REASK = "안내를 받으시겠어요?"
MSG_APPROACH_UNANSWERED = "필요하시면 '비카야'라고 불러 주세요."
MSG_APPROACH_BUSY = "지금은 다른 응대 중입니다. 잠시 후 다시 말씀해 주세요."

MSG_DEST_RETRY = "잘 듣지 못했습니다. 어디로 가고 싶으신가요?"
DEST_ANSWER_WAIT_SEC = 15.0
DEST_PROMPT_FALLBACK_SEC = 40.0
DEST_RETRY_RETURN_SEC = 0.0

MSG_HANDLE_HINT = "손잡이가 진동하고 있습니다. 잡아주세요."
MSG_HANDLE_LOST = "안전을 위해 손잡이를 다시 잡아주세요"
MSG_HANDLE_UNAVAILABLE = "손잡이 연결에 문제가 있어 일반 안내로 진행합니다."
HAPTIC_PATTERN_HANDLE_HINT = "long"
HAPTIC_PATTERN_GRIP_ACK = "tick"
HAPTIC_PATTERN_WAKE_LOCATE = "locate"
GRIP_ENTER_WINDOW_SEC = 2.0
GRIP_RATIO = 0.8
GRIP_WAIT_TIMEOUT_SEC = 15.0
GRIP_HINT_PULSE_SEC = 2.0
GRIP_RELEASE_GRACE_SEC = 0.5
GRIP_RESUME_WINDOW_SEC = 1.0
GRIP_DEPART_WINDOW_SEC = 0.5
HANDLE_LOST_REPEAT_SEC = 15.0
HANDLE_LOST_GIVE_UP_SEC = 180.0
HANDLE_ARRIVAL_FREE_M = 1.0
HANDLE_ARRIVAL_FREE_PATH_M = 1.5
HANDLE_STATE_STALE_SEC = 1.0
DIALOG_GRIP_WAIT = "grip_wait"
DIALOG_PAUSED_HANDLE = "paused_handle"

DISTANCE_MILESTONES_M = ()


APPROACH_RESPONSE_TIMEOUT_SEC = 8.0
APPROACH_REASK_MAX = 3
APPROACH_QUESTION_STUCK_SEC = 30.0
APPROACH_TURN_TIMEOUT_SEC = 15.0
APPROACH_TURN_YAW_DEG = 175.0
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
APPROACH_BT_FILE = "vica_navigate_to_pose_approach.xml"


def nav_behavior_tree(destination_id: str, approach_bt: str) -> str:
    """이 goal 에 쓸 Nav2 행동 트리 파일. 빈 문자열은 bt_navigator 의 기본 트리(레일)다."""
    if approach_bt and destination_id.startswith(APPROACH_DESTINATION_PREFIX):
        return approach_bt
    return ""


NAV_TREE_DEFAULT = ""
NAV_TREE_GUIDED = "guided"
NAV_TREE_WAIT = "wait"
GUIDED_BT_FILE = "vica_navigate_to_pose_guided.xml"
GUIDED_NO_RAIL_BT_FILE = "vica_navigate_to_pose_guided_no_rail.xml"
WAIT_SPOT_BT_FILE = "vica_navigate_to_pose_wait_spot.xml"


def door_side_word(door_yaw_deg: float, robot_yaw_deg: float) -> str:
    """입구가 로봇(= 뒤에서 손잡이를 잡은 사용자) 기준 어느 쪽인가 (M1)."""
    rel = math.degrees(wrap_to_pi(math.radians(door_yaw_deg - robot_yaw_deg)))
    if abs(rel) <= DOOR_FRONT_BACK_DEG:
        return "앞"
    if abs(rel) >= 180.0 - DOOR_FRONT_BACK_DEG:
        return "뒤"
    return "왼쪽" if rel > 0 else "오른쪽"


def wait_spot_destination(dest: Destination) -> Destination:
    """목적지의 대기 장소를 goal 로 감싼다. 나가는 방향까지 맞춰 선다."""
    spot = dest.wait_spot
    assert spot is not None
    return Destination(
        id=WAIT_SPOT_DESTINATION_PREFIX + dest.id,
        name=f"{dest.name}-대기",
        pose=Pose2D(spot.x, spot.y, spot.yaw_deg, dest.pose.frame_id),
        calibrated=True,
    )


def wait_back_destination(dest: Destination) -> Destination:
    """대기 장소가 막혀 목적지로 돌아가는 goal. 혼자 가므로 입구 쪽을 보고 선다."""
    yaw = dest.door_yaw_deg if dest.door_yaw_deg is not None else dest.pose.yaw_deg
    return Destination(
        id=WAIT_BACK_DESTINATION_PREFIX + dest.id,
        name=dest.name,
        pose=Pose2D(dest.pose.x, dest.pose.y, yaw, dest.pose.frame_id),
        calibrated=True,
    )


def is_wait_destination_id(destination_id: str) -> bool:
    """대기 장소·목적지 복귀 goal 의 합성 id 인가 — 대장·앱 알림에서 거른다."""
    return destination_id.startswith(
        (WAIT_SPOT_DESTINATION_PREFIX, WAIT_BACK_DESTINATION_PREFIX))


def with_door_yaw(dest: Destination) -> Destination:
    """배송 도착 방향 — 입구 방향이 있으면 그쪽을 바라보고 선다(10-07 결정)."""
    if dest.door_yaw_deg is None:
        return dest
    return replace(dest, pose=replace(dest.pose, yaw_deg=dest.door_yaw_deg))

_APPROACH_STATES = (
    State.APPROACHING, State.AWAITING_USER, State.TURNING, State.RETURNING,
    State.SEEKING,
)
_GOAL_ACTIVE_STATES = (
    State.NAVIGATING, State.APPROACHING, State.TURNING, State.RETURNING,
    State.SEEKING, State.MOVING_TO_WAIT_SPOT, State.MOVING_BACK_TO_DEST,
)
_WAIT_STATES = (
    State.WAITING_RELEASE, State.MOVING_TO_WAIT_SPOT, State.MOVING_BACK_TO_DEST,
    State.WAITING,
)
_WAIT_MOVING_STATES = (State.MOVING_TO_WAIT_SPOT, State.MOVING_BACK_TO_DEST)
_WAKE_IGNORE_STATES = (
    State.APPROACHING, State.TURNING, State.FAILED, *_WAIT_MOVING_STATES,
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


def check_pause_gate(
    state: State, estop_active: bool, returning_home: bool = False
) -> GateReason:
    """일시정지 게이트. 실제로 주행 중일 때만 멈출 수 있다."""
    if estop_active or state == State.ESTOPPED:
        return GateReason.ESTOP_ACTIVE
    if state == State.NAVIGATING:
        return GateReason.OK
    if state == State.RETURNING and returning_home:
        return GateReason.OK
    return GateReason.NOT_NAVIGATING


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
    same_person: bool = False,
) -> GateReason:
    """사람 접근 요청 게이트. 첫 번째 실패 사유를 돌려준다."""
    if not request.approachable:
        return GateReason.NOT_APPROACHABLE
    if request.track_id == TRACK_ID_NONE:
        return GateReason.NO_TRACK_ID
    if estop_active or state == State.ESTOPPED:
        return GateReason.ESTOP_ACTIVE
    if state == State.APPROACHING:
        if request.track_id != active_track_id and not same_person:
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
        approach_turn_yaw_rad: float = math.radians(APPROACH_TURN_YAW_DEG),
        arrival_dialog: bool = False,
        wake_doa_sign: float = 1.0,
        seek_look_sec: float = SEEK_LOOK_SEC,
        near_call_max_m: float = NEAR_CALL_MAX_M,
        near_call_no_spin_m: float = NEAR_CALL_NO_SPIN_M,
        return_resume_sec: float = RETURN_RESUME_SEC,
        handle_side_min_yaw_rad: float = HANDLE_SIDE_MIN_YAW_RAD,
        dest_retry_return_sec: float = DEST_RETRY_RETURN_SEC,
        grip_enter_window_sec: float = GRIP_ENTER_WINDOW_SEC,
        grip_ratio: float = GRIP_RATIO,
        grip_wait_timeout_sec: float = GRIP_WAIT_TIMEOUT_SEC,
        grip_hint_pulse_sec: float = GRIP_HINT_PULSE_SEC,
        grip_release_grace_sec: float = GRIP_RELEASE_GRACE_SEC,
        grip_resume_window_sec: float = GRIP_RESUME_WINDOW_SEC,
        handle_lost_repeat_sec: float = HANDLE_LOST_REPEAT_SEC,
        handle_lost_give_up_sec: float = HANDLE_LOST_GIVE_UP_SEC,
        handle_state_stale_sec: float = HANDLE_STATE_STALE_SEC,
        grip_assume_held: bool = False,
        home_beacon_interval_sec: float = HOME_BEACON_INTERVAL_SEC,
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
        self.home_beacon_interval_sec = home_beacon_interval_sec
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

        self.grip_enter_window_sec = grip_enter_window_sec
        self.grip_ratio = grip_ratio
        self.grip_wait_timeout_sec = grip_wait_timeout_sec
        self.grip_hint_pulse_sec = grip_hint_pulse_sec
        self.grip_release_grace_sec = grip_release_grace_sec
        self.grip_resume_window_sec = grip_resume_window_sec
        self.handle_lost_repeat_sec = handle_lost_repeat_sec
        self.handle_lost_give_up_sec = handle_lost_give_up_sec
        self.grip_assume_held = grip_assume_held
        self._grip = GripMeter(stale_sec=handle_state_stale_sec)
        self.handle_active: bool = False
        self._handle_engaged: bool = False
        self._grip_wait_since: Optional[float] = None
        self._grip_pulse_at: Optional[float] = None
        self._handle_pause: bool = False
        self._handle_lost_since: Optional[float] = None
        self._handle_lost_notice_at: Optional[float] = None
        self._estop_announced = False
        self.tipped = False
        self._tip_notice_at: Optional[float] = None
        self.app_estop_active = False
        self.help_phase = "none"
        self.help_id: Optional[str] = None
        self._help_last_sent_at: Optional[float] = None
        self._help_confirm_reask_at: Optional[float] = None
        self._help_confirm_deadline: Optional[float] = None
        self._help_confirm_reasked = False
        self.help_cancel_pending = False
        self._help_cancel_reask_at: Optional[float] = None
        self._help_cancel_deadline: Optional[float] = None
        self._help_cancel_reasked = False

        self.arrival_dialog = arrival_dialog
        self._asking_is_finish = False
        self._asking_time_after_yes = False
        self._nav_from_app = False
        self._asking_entered_at: Optional[float] = None
        self._arrival_retried = False
        self._asking_question = ""
        self._deny_reconfirmed = False
        self._asking_where = False
        self._leaving_deadline: Optional[float] = None
        self._wait_until: Optional[float] = None
        self._wait_minutes_requested = -1
        self.robot_yaw_deg: Optional[float] = None
        self.door_side: str = ""
        self._arrived_destination: Optional[Destination] = None
        self._last_guided: Optional[Destination] = None
        self._late_answer_finish: Optional[bool] = None
        self._wait_place: str = ""
        self._beacon_next_at: Optional[float] = None
        self.robot_pose: Optional[Pose2D] = None
        self._home_beacon_next_at: Optional[float] = None
        self._release_text: str = ""
        self._release_spoken_at: Optional[float] = None
        self._release_entered_at: Optional[float] = None
        self._wait_finish_asked_at: Optional[float] = None
        self._wait_need_asked_at: Optional[float] = None
        self._wait_finish_reasked = False
        self._wait_need_reasked = False
        self._wait_hold: Optional[State] = None
        self._change_from: Optional[Destination] = None
        self._approach_dest: Optional[Destination] = None
        self._confirm_from_approach = False
        self._confirm_prompt = ""
        self._confirm_reask_at: Optional[float] = None
        self._confirm_reasked = False
        self._cancel_reask_at: Optional[float] = None
        self._cancel_reasked = False
        self._nav_tree: str = NAV_TREE_DEFAULT
        self._ear_busy = False
        self._ear_busy_since: Optional[float] = None
        self._ear_grace_until: Optional[float] = None
        self._ear_spoke_at: Optional[float] = None
        self._question_at: Optional[float] = None
        self._question_text = ""
        self.voice_note = ""
        self._llm_thinking = False
        self._llm_thinking_since: Optional[float] = None
        self._llm_thinking_tail_until: Optional[float] = None
        self._ear_speaking = False
        self._ear_speech_since: Optional[float] = None

        self.state: State = State.IDLE
        self.estop_active: bool = False
        self.active_destination: Optional[Destination] = None
        self.paused_destination: Optional[Destination] = None
        self._paused_returning_home: bool = False
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
        self.approach_person: Optional[Pose2D] = None
        self._recheck = MidcourseRecheck()
        self._response_deadline: Optional[float] = None
        self._approach_reasks = 0
        self._approach_after_reply: Optional[tuple] = None
        self._suppressed_tracks: dict = {}
        self._suppressed_places: list = []

    @property
    def approach_stages(self) -> tuple:
        """검증·정렬을 마친 접근 감속 단계 목록 (먼 거리부터)."""
        return self._approach.stages

    @property
    def approach_speed_limit_percent(self) -> float:
        """지금 걸려 있는 접근 제한율. 제한 전이면 0.0(해제)."""
        return self._approach.percent

    def on_voice_intent(
        self,
        intent: IntentData,
        now: float,
        lookup: Callable[[str], Optional[Destination]],
        bounds: Optional[MapBounds] = None,
        nav_ready: bool = True,
    ) -> list:
        """음성 요청 하나에 대한 반응. 노드는 나온 동작을 실행하고 기록만 한다."""
        self.voice_note = ""
        if self._answer_predates_question(intent):
            self.voice_note = "질문 전에 시작한 말이라 대답으로 안 씀"
            return []
        reacted = self._react_by_table(intent, now, lookup, bounds, nav_ready)
        if reacted is not None:
            return reacted
        return self._route_voice_intent(intent, now, lookup, bounds, nav_ready)

    def note_question_asked(self, text: str, now: float) -> None:
        """노드가 대답을 기다리는 말(Say.expects_reply)을 내보낼 때 부른다. 같은 질문을 SAME_QUESTION_SEC 안에"""
        if (self._question_at is None or text != self._question_text
                or now - self._question_at > SAME_QUESTION_SEC):
            self._question_at, self._question_text = now, text

    def _answer_predates_question(self, intent: IntentData) -> bool:
        """이 예/아니요의 말이 마지막 질문보다 먼저 시작됐나 — 그러면 그 질문의 대답이 아니다. 말 시작을 못"""
        return (intent.intent in STALE_ANSWER_KINDS and self._question_at is not None
                and self._ear_spoke_at is not None and self._ear_spoke_at < self._question_at)

    def _react_by_table(
        self,
        intent: IntentData,
        now: float,
        lookup: Callable[[str], Optional[Destination]],
        bounds: Optional[MapBounds],
        nav_ready: bool,
    ) -> Optional[list]:
        """반응표로 새로 정한 칸(2026-10-08). 맡지 않는 칸은 None — 옛 갈래가 처리한다."""
        helped = self._react_help(intent, now, lookup, bounds, nav_ready)
        if helped is not None:
            return helped
        if self.cancel_confirm_pending and intent.intent in ("affirm", "deny"):
            return self.on_cancel_confirm_answer(intent.intent == "affirm", now)
        if self.cancel_confirm_pending and intent.intent == "unknown":
            if not self._cancel_reasked:
                self._cancel_reasked = True
                return [self._ask(MSG_CANCEL_CONFIRM)]
            return []
        handler = {
            State.IDLE: self._react_idle,
            State.CONFIRMING: self._react_confirming,
            State.NAVIGATING: self._react_navigating,
            State.PAUSED: self._react_paused,
            State.ASKING_NEXT: self._react_asking,
            State.ASKING_WAIT_TIME: self._react_asking,
            State.WAITING_RELEASE: self._react_waiting,
            State.WAITING: self._react_waiting,
            State.AWAITING_USER: self._react_awaiting_user,
            State.TURNING: self._react_turning,
            State.RETURNING: self._react_returning,
        }.get(self.state)
        if handler is None:
            return None
        return handler(intent, now, lookup, bounds, nav_ready)

    def _wait_at_last_guided(self, minutes: int, now: float) -> list:
        """홈 가는 중의 "기다려" — 직전 목적지 대기 장소로 가서 기다린다(2026-10-08 결정 1)."""
        last = self._last_guided
        self._reset_arrival_dialog()
        self._late_answer_finish = None
        self._arrived_destination = last
        if minutes is not None and minutes > 0:
            return self._enter_waiting(min(minutes, WAIT_MINUTES_CAP), now, away=True)
        return self._enter_waiting(WAIT_MINUTES_CAP, now, default_msg=True, away=True)

    @staticmethod
    def _ask(text: str) -> Say:
        """대답을 기다리는 말 — 노드가 듣기 창을 연다(expects_reply)."""
        return Say(text, priority="response", expects_reply=True)

    def _react_idle(self, intent, now, lookup, bounds, nav_ready) -> Optional[list]:
        """안내 없음. 홈 가다 "비카야"로 세운 뒤(복귀 재개 사다리)면 기다려 = 직전 목적지 대기"""
        kind = intent.intent
        if self._grip_wait_since is not None or self._dest_prompt_stage is not None:
            return None
        if self._return_interrupted:
            if kind not in ("wait", "finish", "resume"):
                return None
            self._forget_interrupted_return()
            if kind == "wait":
                return self._wait_at_last_guided(intent.wait_minutes, now)
            return [Say(MSG_FINISH, priority="response"), *self._go_home(now)]
        if kind in ("wait", "finish"):
            return [Say(MSG_NOT_NAVIGATING, priority="response")]
        return None

    def _react_confirming(self, intent, now, lookup, bounds, nav_ready) -> Optional[list]:
        """확인 질문. "네"만 출발이고 기다려·다 됐어·취소는 "아니요"와 같다 — 대기·주행 중"""
        kind = intent.intent
        change = self._change_from is not None
        if kind == "finish" and change:
            return self._voice_mission_command("cancel", now, nav_ready)
        if kind == "wait" and change:
            self._hold_change_as_pause()
            return [Say(MSG_PAUSED, priority="response")]
        if kind in ("finish", "cancel"):
            self._confirm_from_approach = False
        if kind in ("wait", "finish") or (kind == "cancel" and not change):
            dest = lookup(self.confirming_dest_id or "")
            return self.on_confirm_answer(False, dest, bounds, nav_ready, now)
        if kind == "pause" and not change:
            self._arm_confirm(now)
            return [self._ask(MSG_WAKE_GREETING)]
        if kind == "resume" and not change and self._confirm_prompt:
            self._arm_confirm(now)
            return [self._ask(self._confirm_prompt)]
        if kind == "unknown":
            if not self._confirm_reasked and self._confirm_prompt:
                self._confirm_reasked = True
                return [self._ask(self._confirm_prompt)]
            return []
        return None

    def _arm_confirm(self, now: float) -> None:
        """확인 질문 시계 — 15초 조용하면 같은 질문을 한 번 더, 30초면 접는다(2026-10-08)."""
        self._confirm_deadline = now + self.confirm_timeout_sec
        self._confirm_reask_at = now + QUESTION_REASK_SEC
        self._confirm_reasked = False

    @staticmethod
    def _confirm_prompt_for(dest: Optional[Destination]) -> str:
        """목적지 확인 질문 문장. 목적지에 문장이 없으면 음성과 같은 기본 문장이다."""
        if dest is None:
            return ""
        return dest.confirm_prompt or say_destination(MSG_CONFIRM_PROMPT_FALLBACK, dest.name)

    def _react_navigating(self, intent, now, lookup, bounds, nav_ready) -> Optional[list]:
        """안내 주행. 기다려 = 잠깐과 같이 멈춘다, 다 됐어 = 취소처럼 되묻는다, 다시 가자 ="""
        kind = intent.intent
        if kind == "wait":
            return self._voice_mission_command("pause", now, nav_ready)
        if kind == "finish":
            return self._voice_mission_command("cancel", now, nav_ready)
        if kind == "resume" and self.active_destination is not None and not self._nav_from_app:
            return [Say(say_destination(MSG_ALREADY_GOING, self.active_destination.name),
                        priority="response")]
        return None

    def _react_paused(self, intent, now, lookup, bounds, nav_ready) -> Optional[list]:
        """일시정지. 잠깐·기다려 = 선 채로 "잠시 멈추겠습니다…"를 다시(규칙 1, 옛 말은 "안내"""
        if intent.intent == "finish":
            return self._voice_mission_command("cancel", now, nav_ready)
        if intent.intent in ("pause", "wait"):
            actions, reason = self.on_pause_request(now)
            if reason == GateReason.OK:
                return actions
            return [Say(MSG_PAUSED, priority="response")]
        return None

    def _react_asking(self, intent, now, lookup, bounds, nav_ready) -> Optional[list]:
        """도착 질문·시간 질문. 잠깐 = "네?" 하고 질문 유지 — 다시 묻기 기회를 쓰지 않는다."""
        kind = intent.intent
        if kind == "pause":
            self._response_deadline = None
            self._asking_entered_at = now
            return [self._ask(MSG_WAKE_GREETING)]
        if kind == "resume":
            self.state = State.ASKING_NEXT
            self._asking_where = True
            self._asking_is_finish = False
            self._asking_time_after_yes = False
            self._asking_question = MSG_WAIT_FINISH_ASK
            self._response_deadline = None
            self._asking_entered_at = now
            return [self._ask(MSG_WAIT_FINISH_ASK)]
        if self._asking_where and kind in ("affirm", "deny"):
            if kind == "deny":
                self._reset_arrival_dialog()
                return [Say(MSG_FINISH, priority="response"), *self._go_home(now)]
            return self._arrival_no_answer(now)
        if self.state == State.ASKING_WAIT_TIME and kind == "affirm":
            if not self._arrival_retried:
                self._arrival_retried = True
                self._response_deadline = None
                self._asking_entered_at = now
                return [self._ask(MSG_ASK_WAIT_TIME)]
            return self._enter_waiting(WAIT_MINUTES_CAP, now, default_msg=True)
        if self.state == State.ASKING_WAIT_TIME and kind == "deny":
            return self._ask_end_confirm(now)
        return None

    def _ask_end_confirm(self, now: float) -> list:
        """끝낼지 한 번 확인한다 — "여기까지 안내를 마칠까요?"(종료형). 네 = 종료·홈, 아니요 ="""
        self.state = State.ASKING_NEXT
        self._deny_reconfirmed = True
        self._asking_is_finish = True
        self._asking_time_after_yes = False
        self._asking_where = False
        self._asking_question = MSG_ASK_ENTRANCE
        self._arrival_retried = False
        self._leaving_deadline = None
        self._response_deadline = None
        self._asking_entered_at = now
        return [self._ask(MSG_ASK_ENTRANCE)]

    def _react_waiting(self, intent, now, lookup, bounds, nav_ready) -> Optional[list]:
        """손 놓기 기다림·대기 중. 기다려 = 시간을 지금부터 다시 세고 대기 안내를 다시 말한다."""
        kind = intent.intent
        release = self.state == State.WAITING_RELEASE
        if kind == "wait":
            return self._rewait(intent.wait_minutes, now)
        if kind == "resume":
            return self._ask_where(now)
        if release:
            if kind == "pause":
                return [self._ask(MSG_WAKE_GREETING)]
            if kind in ("cancel", "deny"):
                return self._ask_end_confirm(now)
            return None
        if self._wait_need_asked_at is not None and kind in ("affirm", "deny", "cancel", "finish"):
            self._wait_need_asked_at = None
            if kind == "deny":
                return [Say(MSG_CANCEL_KEPT, priority="response")]
            self._reset_arrival_dialog()
            return [Say(MSG_FINISH, priority="response"), *self._go_home(now)]
        if self._wait_need_asked_at is not None and kind == "unknown":
            if not self._wait_need_reasked:
                self._wait_need_reasked = True
                self._wait_need_asked_at = now
                return [self._ask(MSG_WAIT_NEED_ASK)]
            self._wait_need_asked_at = None
            return [Say(MSG_CANCEL_KEPT, priority="response")]
        if kind == "cancel":
            self._wait_need_asked_at = now
            self._wait_need_reasked = False
            self._wait_finish_asked_at = None
            return [self._ask(MSG_WAIT_NEED_ASK)]
        asked = self._wait_finish_asked_at
        within = asked is not None and now - asked <= WAIT_FINISH_REPEAT_SEC
        if kind == "deny" and within:
            self._reset_arrival_dialog()
            return [Say(MSG_FINISH, priority="response"), *self._go_home(now)]
        if within and kind in ("unknown", "affirm"):
            if not self._wait_finish_reasked:
                self._wait_finish_reasked = True
                self._wait_finish_asked_at = now
                return [self._ask(MSG_WAIT_FINISH_ASK)]
            return []
        return None

    def _wait_question_tick(self, now: float) -> list:
        """대기 중 미션 질문의 다시 묻기(2026-10-08). 15초 조용하면 한 번 더 묻는다. "안내가"""
        if self._ear_holds(now):
            return []
        asked = self._wait_finish_asked_at
        if (asked is not None and not self._wait_finish_reasked
                and now - asked >= QUESTION_REASK_SEC):
            self._wait_finish_reasked = True
            self._wait_finish_asked_at = now
            return [self._ask(MSG_WAIT_FINISH_ASK)]
        need = self._wait_need_asked_at
        if need is not None and now - need >= QUESTION_REASK_SEC:
            if not self._wait_need_reasked:
                self._wait_need_reasked = True
                self._wait_need_asked_at = now
                return [self._ask(MSG_WAIT_NEED_ASK)]
            self._wait_need_asked_at = None
            return [Say(MSG_CANCEL_KEPT, priority="response")]
        return []

    def _ask_where(self, now: float) -> list:
        """대기 중 "다시 가자" — 어디로 갈지 묻는다. "다 됐어"의 첫 질문과 같은 문장이지만"""
        self._wait_finish_asked_at = now
        self._wait_finish_reasked = False
        self._wait_need_asked_at = None
        return [self._ask(MSG_WAIT_FINISH_ASK)]

    def _rewait(self, minutes: int, now: float) -> list:
        """대기 중 "기다려"·"20분 기다려" — 시간을 지금부터 다시 세고 대기 안내를 다시 말한다."""
        if minutes is None or minutes <= 0:
            minutes = (self._wait_minutes_requested if self._wait_minutes_requested > 0
                       else WAIT_MINUTES_CAP)
        minutes = min(int(minutes), WAIT_MINUTES_CAP)
        self._wait_until = now + minutes * 60.0
        self._wait_minutes_requested = minutes
        self._wait_finish_asked_at = None
        self._wait_need_asked_at = None
        place = self.wait_place
        msg = (MSG_WAIT_SPOT_CONFIRM.format(minutes=minutes, place=place) if place
               else MSG_WAIT_CONFIRM.format(minutes=minutes))
        if self.state == State.WAITING_RELEASE:
            self._release_text = msg
            self._release_spoken_at = None
            self._release_entered_at = now
        return [Say(msg, priority="response")]

    def _react_awaiting_user(self, intent, now, lookup, bounds, nav_ready) -> Optional[list]:
        """접근 질문. 목적지로 답하면("응, 화장실 가고 싶어") 수락으로 받고 그 목적지를 기억한다"""
        if intent.intent == "navigate":
            self._approach_dest = self._approach_destination(intent, lookup, bounds, nav_ready)
            return self.on_approach_answer(True, now)
        if intent.intent == "cancel":
            return self.on_approach_answer(False, now)
        if intent.intent in ("affirm", "deny"):
            return None
        return self._approach_not_answered(getattr(intent, "reply", ""), now)

    def _react_turning(self, intent, now, lookup, bounds, nav_ready) -> Optional[list]:
        """돌아서는 중에 말한 목적지 — 기억했다가 손잡이를 내준 뒤 확인 질문으로 묻는다."""
        if intent.intent != "navigate":
            return None
        dest = self._approach_destination(intent, lookup, bounds, nav_ready)
        if dest is not None:
            self._approach_dest = dest
        return []

    def _approach_destination(self, intent, lookup, bounds, nav_ready) -> Optional[Destination]:
        """접근 대화 중 받은 목적지 — 갈 수 있는 곳만 기억한다. 못 가는 곳이면 None 이고,"""
        dest = lookup(intent.matched_destination_id)
        confirmed = replace(intent, need_confirm=False)
        if check_gate(confirmed, dest, bounds, self.estop_active, nav_ready) != GateReason.OK:
            return None
        return dest

    def _react_returning(self, intent, now, lookup, bounds, nav_ready) -> Optional[list]:
        """홈 복귀. 기다려 = 세우고 직전 목적지 대기 장소로(결정 1). 다 됐어 = 그대로 홈 + 한마디."""
        kind = intent.intent
        if kind == "wait":
            return (self.on_return_brake(now, quiet=True)
                    + self._wait_at_last_guided(intent.wait_minutes, now))
        if kind == "finish":
            self._late_answer_finish = None
            return [Say(MSG_FINISH, priority="response")]
        if kind in ("affirm", "deny") and self._late_answer_finish is not None:
            wants_wait = (kind == "affirm") != self._late_answer_finish
            if wants_wait:
                return (self.on_return_brake(now, quiet=True)
                        + self._wait_at_last_guided(-1, now))
            self._late_answer_finish = None
            return [Say(MSG_FINISH, priority="response")]
        if kind == "pause" and not self._returning_home:
            return self.on_return_brake(now) + [self._ask(MSG_WAKE_GREETING)]
        return None

    def _route_voice_intent(
        self,
        intent: IntentData,
        now: float,
        lookup: Callable[[str], Optional[Destination]],
        bounds: Optional[MapBounds],
        nav_ready: bool,
    ) -> list:
        """옛 노드 _on_intent 의 갈래 그대로(2026-10-08 이전 동작)."""
        kind = intent.intent
        actions: list = []
        if self.state == State.RETURNING and kind in ("wait", "navigate"):
            actions.extend(self.on_return_brake(now, quiet=True))
        if self.is_awaiting_arrival_answer():
            if kind == "navigate" and intent.need_confirm:
                self.exit_arrival_dialog()
            else:
                next_dest = (lookup(intent.matched_destination_id)
                             if kind == "navigate" else None)
                return actions + self.on_arrival_answer(
                    intent, now, next_dest=next_dest, bounds=bounds, nav_ready=nav_ready)
        if kind in ("cancel", "pause", "resume"):
            return actions + self._voice_mission_command(kind, now, nav_ready)
        if kind in ("affirm", "deny"):
            if self.state == State.CONFIRMING:
                dest = lookup(self.confirming_dest_id or "")
                return actions + self.on_confirm_answer(
                    kind == "affirm", dest, bounds, nav_ready, now)
            return actions + self.on_approach_answer(kind == "affirm", now)
        dest = lookup(intent.matched_destination_id)
        return actions + self.on_intent(intent, dest, bounds, nav_ready, now)

    def _voice_mission_command(self, kind: str, now: float, nav_ready: bool) -> list:
        """음성 취소·일시정지·재개(옛 노드 _on_voice_mission_command). 취소는 곧바로 하지"""
        if kind == "cancel":
            if self.cancel_confirm_pending:
                return self.on_cancel_confirm_answer(True, now)
            actions, reason = self.on_cancel_confirm_request(now)
        elif kind == "pause":
            actions, reason = self.on_pause_request(now)
        else:
            actions, reason = self.on_resume_request(nav_ready, now)
        if reason != GateReason.OK:
            if reason == GateReason.ESTOP_ACTIVE:
                self._note_tip_notice(now)
            msg = _REJECT_MESSAGES.get(reason)
            return [Say(msg, priority="response")] if msg else []
        return actions

    def on_intent(
        self,
        intent: IntentData,
        dest: Optional[Destination],
        bounds: Optional[MapBounds],
        nav_ready: bool,
        now: float,
    ) -> list:
        self._forget_dest_prompt()
        if self._return_interrupted and self.state == State.IDLE:
            self._return_resume_deadline = now + self.return_resume_sec
            self._return_notice_given = False
        if (intent.intent == "finish"
                and self.state in (State.WAITING, State.WAITING_RELEASE)):
            return self._wait_finish(now)
        if intent.intent != "navigate":
            return []
        if self._grip_wait_since is not None:
            self._grip_wait_since = None
            self._grip_pulse_at = None

        if self.state == State.ESTOPPED:
            self._note_tip_notice(now)
            return [Say(MSG_ESTOP_REJECT, priority="response")]

        if self.state in _WAIT_MOVING_STATES:
            return []

        if self.state == State.NAVIGATING:
            return self._navigating_destination_request(
                intent, dest, bounds, nav_ready, now)

        if self.state in _APPROACH_STATES:
            if intent.need_confirm:
                return []
            return [Say(MSG_APPROACH_BUSY, priority="response")]

        if intent.need_confirm:
            if (self.state == State.CONFIRMING
                    and self._confirming_dest_id
                    and intent.matched_destination_id == self._confirming_dest_id):
                return self.on_confirm_answer(True, dest, bounds, nav_ready, now)
            else:
                if self.state in _WAIT_STATES and self._wait_hold is None:
                    self._wait_hold = self.state
                self.state = State.CONFIRMING
                self._confirm_from_approach = False
                self._confirming_dest_id = intent.matched_destination_id or None
                self._arm_confirm(now)
                self._confirm_prompt = self._confirm_prompt_for(dest)
                return []

        if (
            self.state == State.CONFIRMING
            and self._confirming_dest_id
            and intent.matched_destination_id != self._confirming_dest_id
        ):
            reason = check_gate(intent, dest, bounds, self.estop_active, nav_ready)
            if reason != GateReason.OK:
                msg = _REJECT_MESSAGES.get(reason)
                return [Say(msg, priority="response")] if msg else []
            assert dest is not None
            self._confirming_dest_id = dest.id
            self._arm_confirm(now)
            self._confirm_prompt = self._confirm_prompt_for(dest)
            return [self._ask(MSG_CONFIRM_SWITCH.format(prompt=self._confirm_prompt))]

        reason = check_gate(intent, dest, bounds, self.estop_active, nav_ready)
        if reason != GateReason.OK:
            folded: list = []
            if (reason == GateReason.NAV_NOT_READY and self._changing_destination()):
                self._hold_change_as_pause()
            elif self.state == State.CONFIRMING:
                folded = self._fold_confirming(now)
            elif self.state not in _WAIT_STATES:
                self._to_idle()
            msg = _REJECT_MESSAGES.get(reason)
            return ([Say(msg, priority="response")] if msg else []) + folded

        assert dest is not None
        if self.state in _WAIT_STATES or self._wait_hold is not None:
            self._reset_arrival_dialog()
        self._change_from = None
        self._nav_retry_count = 0
        self._retry_at = None
        self._nav_from_app = False
        self.state = State.NAVIGATING
        self.active_destination = dest
        self._nav_tree = NAV_TREE_GUIDED
        self.door_side = ""
        self.handle_active = self._decide_handle_mode(now)
        self._confirming_dest_id = None
        self._confirm_deadline = None
        self._announced_milestones = set()
        self._distance_baseline = None
        self._approach.reset()
        self._seek_deadline = None
        self._seek_return_yaw = None
        self._forget_interrupted_return()
        self._last_guided = None
        self._late_answer_finish = None
        return [
            SetNavSpeedLimit(NO_SPEED_LIMIT),
            Say(say_destination(MSG_START, dest.name)),
            Navigate(dest, tree=NAV_TREE_GUIDED),
        ]

    def _navigating_destination_request(
        self,
        intent: IntentData,
        dest: Optional[Destination],
        bounds: Optional[MapBounds],
        nav_ready: bool,
        now: float,
    ) -> list:
        """안내 주행 중 들어온 목적지 요청 — 멈추고 바꿀지 묻는다(2026-10-07 사용자 결정)."""
        current = self.active_destination
        if self._nav_from_app or current is None:
            return [] if intent.need_confirm else [Say(MSG_BUSY, priority="response")]
        new_id = intent.matched_destination_id or ""
        if new_id and new_id == current.id:
            return []
        ask: list = []
        if not intent.need_confirm:
            reason = check_gate(intent, dest, bounds, self.estop_active, nav_ready)
            if reason != GateReason.OK:
                msg = _REJECT_MESSAGES.get(reason)
                return [Say(msg, priority="response")] if msg else []
            assert dest is not None
            prompt = (dest.confirm_prompt
                      or say_destination(MSG_CONFIRM_PROMPT_FALLBACK, dest.name))
            ask = [Say(prompt, priority="response", expects_reply=True)]
        elif dest is None or not new_id:
            return []
        actions: list = [
            SetNavSpeedLimit(NO_SPEED_LIMIT),
            CancelNav(current, event="goal_paused"),
        ]
        self._change_from = current
        self.active_destination = None
        self.state = State.CONFIRMING
        self._confirm_from_approach = False
        self._confirming_dest_id = new_id
        self._arm_confirm(now)
        self._confirm_prompt = self._confirm_prompt_for(dest)
        self.cancel_confirm_pending = False
        self._cancel_confirm_deadline = None
        self._announced_milestones = set()
        self._distance_baseline = None
        self._approach.reset()
        return actions + ask

    def _wait_finish(self, now: float) -> list:
        """대기 중 "다 됐어"(finish) — 다음 목적지를 묻는다(2026-10-07 사용자 결정)."""
        asked = self._wait_finish_asked_at
        if asked is not None and now - asked <= WAIT_FINISH_REPEAT_SEC:
            self._reset_arrival_dialog()
            return [Say(MSG_FINISH, priority="response"), *self._go_home(now)]
        self._wait_finish_asked_at = now
        self._wait_finish_reasked = False
        self._wait_need_asked_at = None
        return [Say(MSG_WAIT_FINISH_ASK, priority="response", expects_reply=True)]

    def _changing_destination(self) -> bool:
        """주행 중 바꾸기 질문(CONFIRMING + _change_from)으로 멈춰 있는가."""
        return self.state == State.CONFIRMING and self._change_from is not None

    def _hold_change_as_pause(self) -> None:
        """바꾸기 질문을 접고 일시정지로 둔다 — 원래 목적지를 보관해 "다시 가자"로 잇는다."""
        self.paused_destination = self._change_from
        self._change_from = None
        self._confirming_dest_id = None
        self._confirm_deadline = None
        self._paused_returning_home = False
        self.cancel_confirm_pending = False
        self._cancel_confirm_deadline = None
        self.state = State.PAUSED

    @property
    def confirming_dest_id(self) -> Optional[str]:
        """확인 중인 목적지 id. CONFIRMING 밖에서는 None."""
        if self.state != State.CONFIRMING:
            return None
        return self._confirming_dest_id

    def _in_wait(self) -> bool:
        """대기 중인가 — 대기 장소로 가는 길·확인 질문으로 잠깐 들어간 동안도 포함."""
        return (self.state in _WAIT_STATES
                or (self.state == State.CONFIRMING and self._wait_hold is not None))

    def wait_minutes_requested(self) -> int:
        """대장(P1): 대기 요청 분. 대기 중이 아니면 -1."""
        return self._wait_minutes_requested if self._in_wait() else -1

    def wait_left_sec(self, now: float) -> int:
        """대장(P1): 대기 남은 초(0 이상). 대기 중이 아니면 -1."""
        if not self._in_wait() or self._wait_until is None:
            return -1
        return max(0, int(self._wait_until - now))

    @property
    def wait_place(self) -> str:
        """상황판 RobotState.wait_place — 지금 기다리는 곳의 말. 대기 장소 없는 대기는 ""."""
        if not self._in_wait():
            return ""
        if self._wait_place == "spot":
            dest = self._arrived_destination
            if dest is not None and dest.wait_spot is not None:
                return WAIT_PLACE_PHRASES.get(dest.wait_spot.side, "")
            return ""
        if self._wait_place == "destination":
            return WAIT_PLACE_AT_DESTINATION
        return ""

    def _fold_confirming(self, now: float, announce: bool = True) -> list:
        """확인 질문(CONFIRMING)을 새 출발 없이 접는다. 질문 전에 하던 일로 돌아간다."""
        change_from = self._change_from
        if change_from is not None:
            self._change_from = None
            self._confirming_dest_id = None
            self._confirm_deadline = None
            self.state = State.NAVIGATING
            self.active_destination = change_from
            self._announced_milestones = set()
            self._distance_baseline = None
            self._approach.reset()
            actions: list = [SetNavSpeedLimit(NO_SPEED_LIMIT)]
            if announce:
                actions.append(Say(say_destination(MSG_RESUMED, change_from.name),
                                   priority="response"))
            actions.append(Navigate(change_from, tree=self._nav_tree))
            return actions
        held = self._wait_hold
        if held is None:
            self._to_idle()
            return []
        self._wait_hold = None
        self.state = held
        self._confirming_dest_id = None
        self._confirm_deadline = None
        return []

    def _fold_approach_confirm(self, now: float) -> list:
        """접근에서 온 확인 질문의 아니요·무응답 — 안내를 끝내지 않고 온보딩 질문으로 어디 갈지"""
        self._confirm_from_approach = False
        self._fold_confirming(now)
        self._arm_dest_prompt(now)
        return [Say(MSG_APPROACH_ONBOARDING, priority="response", expects_reply=True)]

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
            if self._change_from is not None:
                return self._fold_confirming(now)
            if self._confirm_from_approach:
                return self._fold_approach_confirm(now)
            self._fold_confirming(now)
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
        else:
            held = (self._change_from if self._changing_destination()
                    else self.paused_destination if self.state == State.PAUSED else None)
            if held is not None:
                actions.append(GoalEvent("goal_canceled", held))
        self._reset_arrival_dialog()
        self._to_idle()
        self._handle_engaged = False
        self._forget_interrupted_return()
        return actions

    def on_app_destination(self, dest: Optional[Destination],
                           bounds: Optional[MapBounds], nav_ready: bool,
                           now: float, allow_private: bool = False,
                           is_delivery: bool = False) -> tuple:
        """앱(관리자) 새 목적지 — ESTOPPED 만 빼고 어느 상태든 선점한다."""
        if self.estop_active or self.state == State.ESTOPPED:
            return [], GateReason.ESTOP_ACTIVE
        if dest is None:
            return [], GateReason.UNKNOWN_DESTINATION
        if dest.authorization != "public" and not allow_private:
            return [], GateReason.PRIVATE_DESTINATION
        if not dest.is_approachable:
            return [], GateReason.NOT_APPROACHABLE
        if not pose_valid(dest, bounds):
            return [], GateReason.POSE_INVALID
        if not nav_ready:
            return [], GateReason.NAV_NOT_READY
        actions = [*self._end_help(), *self._force_clear_all(now)]
        self.state = State.NAVIGATING
        self.active_destination = with_door_yaw(dest) if is_delivery else dest
        self._nav_from_app = True
        self._nav_tree = NAV_TREE_DEFAULT
        self.door_side = ""
        template = MSG_START_DELIVERY if is_delivery else MSG_START
        actions.append(Say(say_destination(template, dest.name)))
        actions.append(Navigate(self.active_destination))
        return actions, GateReason.OK

    def on_app_wait_spot(self, dest: Optional[Destination],
                         bounds: Optional[MapBounds], nav_ready: bool,
                         now: float) -> tuple:
        """관리자 '대기 장소로 가보기'(`/vica/mission/request_wait_spot`, 2026-10-07)."""
        if self.estop_active or self.state == State.ESTOPPED:
            return [], GateReason.ESTOP_ACTIVE
        if self.state != State.IDLE:
            return [], GateReason.BUSY_NAVIGATING
        if dest is None:
            return [], GateReason.UNKNOWN_DESTINATION
        if dest.wait_spot is None:
            return [], GateReason.NO_WAIT_SPOT
        wait_dest = wait_spot_destination(dest)
        if not pose_valid(wait_dest, bounds):
            return [], GateReason.POSE_INVALID
        if not nav_ready:
            return [], GateReason.NAV_NOT_READY
        actions = [*self._end_help(), *self._force_clear_all(now)]
        self.state = State.NAVIGATING
        self.active_destination = wait_dest
        self._nav_from_app = True
        self._nav_tree = NAV_TREE_WAIT
        self.door_side = ""
        actions.append(Navigate(wait_dest, tree=NAV_TREE_WAIT))
        return actions, GateReason.OK

    def _voice_cancel_gate(self) -> GateReason:
        """음성 취소 게이트. 주행 중 바꾸기 질문으로 멈춘 동안(CONFIRMING)도 아직"""
        if self._changing_destination() and not self.estop_active:
            return GateReason.OK
        return check_cancel_gate(self.state, self.estop_active)

    def on_cancel_request(self, now: float) -> tuple:
        """음성 취소. 게이트(주행·일시정지·대기)를 지킨다 — 앱 취소는"""
        reason = self._voice_cancel_gate()
        if reason != GateReason.OK:
            return [], reason
        actions = self._force_clear_all(now)
        actions.append(Say(MSG_CANCELED, priority="response"))
        return actions, GateReason.OK

    def on_app_cancel(self, now: float) -> tuple:
        """앱(관리자) 취소 — ESTOPPED 만 빼고 어느 상태든 전부 정리하고"""
        ended = self._end_help()
        if self.estop_active or self.state == State.ESTOPPED:
            return ended, GateReason.ESTOP_ACTIVE
        if self.state == State.IDLE:
            self._forget_interrupted_return()
            return ended, GateReason.OK
        actions = [*ended, *self._force_clear_all(now)]
        actions.append(Say(MSG_CANCELED, priority="response"))
        return actions, GateReason.OK

    def on_pause_request(self, now: float) -> tuple:
        """일시정지("잠깐"·앱). goal 을 취소하되 목적지는 보관해 재개할 수 있게 둔다."""
        if self.state == State.PAUSED and self._handle_pause:
            self._clear_handle_pause()
            return [Say(MSG_PAUSED, priority="response")], GateReason.OK
        if self._changing_destination() and not self.estop_active:
            self._hold_change_as_pause()
            return [Say(MSG_PAUSED, priority="response")], GateReason.OK
        actions, reason = self._enter_paused(now)
        if reason != GateReason.OK:
            return [], reason
        self._clear_handle_pause()
        actions.append(Say(MSG_PAUSED, priority="response"))
        return actions, GateReason.OK

    def _enter_paused(self, now: float) -> tuple:
        """goal 을 취소하고 목적지를 보관한다. 멘트는 부르는 쪽이 붙인다."""
        reason = check_pause_gate(
            self.state, self.estop_active, returning_home=self._returning_home
        )
        if reason != GateReason.OK:
            return [], reason

        destination = self.active_destination
        actions: list = [
            SetNavSpeedLimit(NO_SPEED_LIMIT),
            CancelNav(destination, event="goal_paused"),
        ]
        self._paused_returning_home = (
            self.state == State.RETURNING and self._returning_home
        )
        self._returning_home = False
        self.state = State.PAUSED
        self.paused_destination = destination
        self.active_destination = None
        self.cancel_confirm_pending = False
        self._cancel_confirm_deadline = None
        self._announced_milestones = set()
        self._distance_baseline = None
        self._approach.reset()
        return actions, GateReason.OK

    def on_resume_request(self, nav_ready: bool, now: float) -> tuple:
        """다시 출발. 성공하면 걸린 도움 요청을 끝낸다(설계 5.6절) — 음성 "다시 가자"는 관리자 확인"""
        actions, reason = self._resume_request(nav_ready, now)
        if reason == GateReason.OK and self.help_phase != "none":
            actions = [*actions, *self._end_help()]
        return actions, reason

    def _resume_request(self, nav_ready: bool, now: float) -> tuple:
        """다시 출발. 보관한 목적지로 새 goal 을 만든다."""
        if self._changing_destination() and not self.estop_active:
            if not nav_ready:
                return [], GateReason.NAV_NOT_READY
            return self._fold_confirming(now), GateReason.OK
        reason = check_resume_gate(
            self.state, self.paused_destination, self.estop_active, nav_ready
        )
        if reason != GateReason.OK:
            return [], reason
        self.cancel_confirm_pending = False
        self._cancel_confirm_deadline = None
        if self._handle_pause:
            if not self._holding(now, self.grip_resume_window_sec,
                                 since=self._handle_lost_since):
                self.handle_active = False
            self._clear_handle_pause()

        destination = self.paused_destination
        assert destination is not None
        if self._paused_returning_home:
            self._paused_returning_home = False
            self.paused_destination = None
            return self._enter_returning(now, is_home=True), GateReason.OK
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
                Navigate(destination, tree=self._nav_tree),
            ],
            GateReason.OK,
        )

    def on_cancel_confirm_request(self, now: float) -> tuple:
        """음성 취소 요청. 바로 취소하지 않고 사용자에게 되묻는다."""
        reason = self._voice_cancel_gate()
        if reason != GateReason.OK:
            return [], reason
        self.cancel_confirm_pending = True
        self._cancel_confirm_deadline = now + self.confirm_timeout_sec
        self._cancel_reask_at = now + QUESTION_REASK_SEC
        self._cancel_reasked = False
        return [
            Say(MSG_CANCEL_CONFIRM, priority="response", expects_reply=True)
        ], GateReason.OK

    def _cancel_confirm_tick(self, now: float) -> list:
        """"안내를 취소할까요?" 시계. 15초 조용하면 한 번 다시 묻고(2026-10-08 다시 묻기), 시간이"""
        if (not self._cancel_reasked and self._cancel_reask_at is not None
                and now >= self._cancel_reask_at and not self._ear_holds(now)):
            self._cancel_reasked = True
            self._cancel_confirm_deadline = max(self._cancel_confirm_deadline or now,
                                                now + QUESTION_REASK_SEC)
            return [self._ask(MSG_CANCEL_CONFIRM)]
        if self._cancel_confirm_deadline is not None and now >= self._cancel_confirm_deadline:
            self.cancel_confirm_pending = False
            self._cancel_confirm_deadline = None
        return []

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

    @property
    def help_active(self) -> bool:
        """도움 요청을 보낸 뒤인가(관리자 확인 전·후). 확인 질문 중(asking)은 아니다."""
        return self.help_phase in ("requested", "acknowledged")

    def _react_help(self, intent, now, lookup, bounds, nav_ready) -> Optional[list]:
        """도움 요청이 걸린 음성 요청(설계 5.2절 순서 1~7). 맡지 않으면 None — 반응표·옛 갈래가 처리한다."""
        kind = intent.intent
        if self.help_phase == "asking" and self.state != State.NAVIGATING:
            self._drop_help_question()
        if self.help_cancel_pending:
            if kind in ("affirm", "cancel", "finish"):
                return [*self._end_help(notify_after_ack=True),
                        Say(MSG_HELP_CANCELED, priority="response")]
            if kind == "deny":
                self._close_help_cancel()
                return []
            if kind == "unknown":
                if not self._help_cancel_reasked:
                    self._help_cancel_reasked = True
                    return [self._ask(MSG_HELP_CANCEL_CONFIRM)]
                return []
        if self.help_phase == "asking":
            if kind == "affirm":
                return self._send_help(now)
            if kind in ("deny", "cancel"):
                self._drop_help_question()
                return []
            if kind == "finish":
                self._drop_help_question()
                return None
            if kind == "unknown":
                if not self._help_confirm_reasked:
                    self._help_confirm_reasked = True
                    return [self._ask(MSG_HELP_CONFIRM)]
                return []
        if kind == "help":
            return self._on_help_intent(now)
        if not self.help_active:
            return None
        if kind in ("cancel", "finish"):
            self.help_cancel_pending = True
            self._help_cancel_deadline = now + self.confirm_timeout_sec
            self._help_cancel_reask_at = now + QUESTION_REASK_SEC
            self._help_cancel_reasked = False
            return [self._ask(MSG_HELP_CANCEL_CONFIRM)]
        if kind == "navigate" and intent.need_confirm:
            return []
        if kind in ("resume", "navigate") and self.help_phase == "acknowledged":
            return [Say(MSG_HELP_COMING, priority="response")]
        if kind == "navigate":
            dest = lookup(intent.matched_destination_id)
            reason = check_gate(intent, dest, bounds, self.estop_active, nav_ready)
            if reason != GateReason.OK:
                msg = _REJECT_MESSAGES.get(reason)
                return [Say(msg, priority="response")] if msg else []
            actions = self.on_intent(intent, dest, bounds, nav_ready, now)
            if self.state == State.NAVIGATING:
                actions.extend(self._end_help())
            return actions
        return None

    def _on_help_intent(self, now: float) -> list:
        """help 의도(5.3절). 안내 주행 중에만 묻고, 이미 보냈으면 새 요청을 만들지 않는다(D1)."""
        if self.help_phase == "asking":
            return []
        if self.help_phase == "requested":
            return [Say(MSG_HELP_SENT, priority="response")]
        if self.help_phase == "acknowledged":
            return [Say(MSG_HELP_COMING, priority="response")]
        if self.state != State.NAVIGATING:
            return [Say(MSG_HELP_UNAVAILABLE, priority="response")]
        self.cancel_confirm_pending = False
        self._cancel_confirm_deadline = None
        self.help_phase = "asking"
        self._help_confirm_deadline = now + self.confirm_timeout_sec
        self._help_confirm_reask_at = now + QUESTION_REASK_SEC
        self._help_confirm_reasked = False
        return [self._ask(MSG_HELP_CONFIRM)]

    def _send_help(self, now: float) -> list:
        """확인 질문에 "네"(R4) — 일시정지(목적지 보관, goal_paused) → "보냈습니다" → 첫 알림."""
        self._drop_help_question()
        actions, reason = self._enter_paused(now)
        if reason != GateReason.OK:
            return []
        self._clear_handle_pause()
        self.help_phase = "requested"
        self.help_id = str(uuid.uuid4())
        self._help_last_sent_at = now
        actions.append(Say(MSG_HELP_SENT, priority="response"))
        actions.append(HelpEvent("help_requested", self.help_id))
        return actions

    def on_help_ack(self, request_id: str, now: float) -> tuple:
        """관리자가 앱에서 확인(/vica/mission/help_ack, 5.5절). (actions, 받았는가)를 돌려준다."""
        if self.help_id is None or request_id != self.help_id:
            return [], False
        if self.help_phase == "acknowledged":
            return [], True
        if self.help_phase != "requested":
            return [], False
        self.help_phase = "acknowledged"
        self._help_last_sent_at = None
        return [Say(MSG_HELP_COMING, priority="response"),
                HelpEvent("help_acknowledged", self.help_id)], True

    def _help_tick(self, now: float) -> list:
        """도움 요청 시계(5.4절) — 미션 상태와 상관없이 본다. 비상 정지 중에도 재전송한다(D5)."""
        actions: list = []
        if self.help_phase == "asking":
            if self.state != State.NAVIGATING:
                self._drop_help_question()
            elif (not self._help_confirm_reasked and self._help_confirm_reask_at is not None
                    and now >= self._help_confirm_reask_at and not self._ear_holds(now)):
                self._help_confirm_reasked = True
                self._help_confirm_deadline = max(self._help_confirm_deadline or now,
                                                  now + QUESTION_REASK_SEC)
                actions.append(self._ask(MSG_HELP_CONFIRM))
            elif self._help_confirm_deadline is not None and now >= self._help_confirm_deadline:
                self._drop_help_question()
        elif self.help_phase == "requested" and self.help_id is not None:
            if self._help_last_sent_at is None or now - self._help_last_sent_at >= HELP_RESEND_SEC:
                self._help_last_sent_at = now
                actions.append(HelpEvent("help_requested", self.help_id))
        if self.help_cancel_pending:
            if (not self._help_cancel_reasked and self._help_cancel_reask_at is not None
                    and now >= self._help_cancel_reask_at and not self._ear_holds(now)):
                self._help_cancel_reasked = True
                self._help_cancel_deadline = max(self._help_cancel_deadline or now,
                                                 now + QUESTION_REASK_SEC)
                actions.append(self._ask(MSG_HELP_CANCEL_CONFIRM))
            elif self._help_cancel_deadline is not None and now >= self._help_cancel_deadline:
                self._close_help_cancel()
        return actions

    def _drop_help_question(self) -> None:
        """확인 질문(asking)을 말없이 접는다."""
        self.help_phase = "none"
        self._help_confirm_reask_at = None
        self._help_confirm_deadline = None
        self._help_confirm_reasked = False

    def _close_help_cancel(self) -> None:
        self.help_cancel_pending = False
        self._help_cancel_reask_at = None
        self._help_cancel_deadline = None
        self._help_cancel_reasked = False

    def _end_help(self, notify_after_ack: bool = False) -> list:
        """도움 요청을 끝낸다(5.6절). 관리자 확인 전이면 앱 팝업이 닫히게 help_canceled 를 낸다."""
        notify = self.help_id is not None and (
            self.help_phase == "requested"
            or (notify_after_ack and self.help_phase == "acknowledged"))
        actions = [HelpEvent("help_canceled", self.help_id)] if notify else []
        self._drop_help_question()
        self._close_help_cancel()
        self.help_id = None
        self._help_last_sent_at = None
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
        same = (
            self.state == State.APPROACHING
            and same_person(request.person, self.approach_person, SAME_PERSON_RADIUS_M)
        )
        reason = check_approach_gate(
            request,
            self.state,
            self.approach_track_id,
            bounds,
            self.estop_active,
            nav_ready,
            self._is_suppressed(request.track_id, now)
            or self._place_suppressed(request.person, now),
            same_person=same,
        )
        if reason != GateReason.OK:
            return [], reason

        destination = approach_destination(request)

        if self.state == State.APPROACHING:
            if request.track_id != self.approach_track_id:
                self.approach_track_id = request.track_id
            self._recheck.update_robot(self.robot_pose)
            if self._recheck.finished:
                return [], GateReason.OK
            if not self._approach_goal_moved(request.goal):
                return [], GateReason.OK
            self.approach_goal_pose = request.goal
            if request.person is not None:
                self.approach_person = request.person
                self._recheck.move_person(request.person)
            self.active_destination = destination
            return [Navigate(destination)], GateReason.OK

        self.state = State.APPROACHING
        self._approach_dest = None
        self.active_destination = destination
        self.approach_track_id = request.track_id
        self.approach_goal_pose = request.goal
        self.approach_person = request.person
        self._recheck.start(request.person, self.robot_pose)
        self._response_deadline = None
        self._announced_milestones = set()
        self._distance_baseline = None
        self._approach.reset()
        return (
            [
                Say(MSG_APPROACH_COMING, priority="response"),
                SetNavSpeedLimit(self.person_approach_speed_percent),
                Navigate(destination),
            ],
            GateReason.OK,
        )

    @property
    def approach_recheck(self) -> MidcourseRecheck:
        """중간 재측정 상태(단계·시점·표본 수) — 노드 로그용, 읽기만 한다."""
        return self._recheck

    def on_approach_detection(
        self, person: Pose2D, confidence: float, now: float
    ) -> Optional[Pose2D]:
        """접근 중 /vica/person_detection 한 건 — 중간 재측정(2026-10-10)."""
        if self.state != State.APPROACHING:
            return None
        return self._recheck.observe(self.robot_pose, person, confidence)

    def on_approach_recheck_goal(
        self,
        person: Pose2D,
        goal: Optional[Pose2D],
        bounds: Optional[MapBounds],
        now: float,
    ) -> tuple:
        """중간 재측정 자리로 다시 계산한 목표. (actions, 목표가 옮겨질 거리 m)."""
        if self.state != State.APPROACHING or goal is None or self.approach_goal_pose is None:
            return [], 0.0
        moved = math.hypot(goal.x - self.approach_goal_pose.x, goal.y - self.approach_goal_pose.y)
        self.approach_person = person
        if moved < RECHECK_RESEND_M:
            return [], moved
        destination = approach_destination(ApproachRequest(
            goal=goal, track_id=self.approach_track_id or TRACK_ID_NONE, person=person))
        if not pose_valid(destination, bounds):
            return [], moved
        self.approach_goal_pose = goal
        self.active_destination = destination
        return [Navigate(destination)], moved

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

    def on_handle_state(self, contact: bool, fresh: bool, now: float) -> None:
        """/vica/smart_handle_state 한 건. 사실만 적고 판정은 on_tick 이 한다."""
        self._grip.update(now, contact, fresh)

    @property
    def dialog_state(self) -> str:
        """LLM 상황판에 내는 대화 단계(RobotState.dialog_state)."""
        if self.state == State.IDLE and self._grip_wait_since is not None:
            return DIALOG_GRIP_WAIT
        if self.state == State.PAUSED and self._handle_pause:
            return DIALOG_PAUSED_HANDLE
        return self.state.value

    def _holding(self, now: float, window: float, since: Optional[float] = None) -> bool:
        return (self._grip.fresh(now)
                and self._grip.ratio(now, window, since=since) >= self.grip_ratio)

    def _decide_handle_mode(self, now: float) -> bool:
        """음성으로 시작한 안내의 출발 순간 — 활성 모드인가(설계 4.3 (나))."""
        if self.grip_assume_held:
            return False
        if not self._grip.fresh(now):
            return False
        if self._handle_engaged and self.user_attached_guard_active(now):
            return True
        return self._holding(now, GRIP_DEPART_WINDOW_SEC)

    def _start_grip_wait(self, now: float) -> list:
        """수락 뒤 손잡이를 내준 순간(회전 완료·실패·생략). 힌트 + 진동."""
        self._handle_engaged = False
        actions: list = [
            Haptic(HAPTIC_PATTERN_HANDLE_HINT),
            Say(MSG_HANDLE_HINT, priority="response"),
        ]
        if not self._grip.fresh(now):
            actions.extend(self._finish_grip_wait(now, engaged=False))
            return actions
        self._grip_wait_since = now
        self._grip_pulse_at = now
        return actions

    def _finish_grip_wait(self, now: float, engaged: bool) -> list:
        self._grip_wait_since = None
        self._grip_pulse_at = None
        self._handle_engaged = engaged
        actions: list = []
        if engaged:
            actions.append(Haptic(HAPTIC_PATTERN_GRIP_ACK))
        approach_dest = self._approach_dest
        self._approach_dest = None
        if approach_dest is not None:
            self.state = State.CONFIRMING
            self._confirm_from_approach = True
            self._confirming_dest_id = approach_dest.id
            self._arm_confirm(now)
            self._confirm_prompt = self._confirm_prompt_for(approach_dest)
            actions.append(self._ask(self._confirm_prompt))
            return actions
        self._arm_dest_prompt(now)
        actions.append(Say(MSG_APPROACH_ONBOARDING, priority="response",
                           expects_reply=True))
        return actions

    def _grip_wait_tick(self, now: float) -> list:
        since = self._grip_wait_since
        assert since is not None
        if not self._grip.fresh(now):
            return self._finish_grip_wait(now, engaged=False)
        if self._holding(now, self.grip_enter_window_sec, since=since):
            return self._finish_grip_wait(now, engaged=True)
        if now - since >= self.grip_wait_timeout_sec:
            return self._finish_grip_wait(now, engaged=self.grip_assume_held)
        if (self._grip_pulse_at is not None
                and now - self._grip_pulse_at >= self.grip_hint_pulse_sec):
            self._grip_pulse_at = now
            return [Haptic(HAPTIC_PATTERN_HANDLE_HINT)]
        return []

    def _handle_nav_tick(self, now: float, distance_remaining: Optional[float] = None) -> list:
        """활성 주행 중 손 놓침·상향 두절을 본다. NAVIGATING·주행 중에만 부른다."""
        if not self._grip.fresh(now):
            self.handle_active = False
            return [Say(MSG_HANDLE_UNAVAILABLE, priority="response")]
        if self._grip.released_for(now) < self.grip_release_grace_sec:
            return []
        if self._release_ok_near_goal(distance_remaining):
            return []
        actions, reason = self._enter_paused(now)
        if reason != GateReason.OK:
            return []
        self._handle_pause = True
        self._handle_lost_since = now
        self._handle_lost_notice_at = now
        actions.append(Haptic(HAPTIC_PATTERN_HANDLE_HINT))
        actions.append(Say(MSG_HANDLE_LOST, priority="response"))
        return actions

    def _release_ok_near_goal(self, distance_remaining: Optional[float]) -> bool:
        """목적지 HANDLE_ARRIVAL_FREE_M 안인가 — 그러면 손 놓침으로 세우지 않는다."""
        dest, pose = self.active_destination, self.robot_pose
        if dest is None or pose is None:
            return False
        if math.hypot(pose.x - dest.pose.x, pose.y - dest.pose.y) > HANDLE_ARRIVAL_FREE_M:
            return False
        return (distance_remaining is None or distance_remaining <= 0.0
                or distance_remaining <= HANDLE_ARRIVAL_FREE_PATH_M)

    def _handle_pause_tick(self, now: float, nav_ready: bool) -> list:
        """손 놓침으로 선 PAUSED. 다시 잡으면 출발, 오래 안 잡으면 끝낸다."""
        lost_since = self._handle_lost_since
        assert lost_since is not None
        if not self._grip.fresh(now):
            self.handle_active = False
            self._clear_handle_pause()
            return [Say(MSG_PAUSED, priority="response")]
        if self._holding(now, self.grip_resume_window_sec, since=lost_since):
            actions, reason = self.on_resume_request(nav_ready, now)
            if reason != GateReason.OK:
                return []
            return [Haptic(HAPTIC_PATTERN_GRIP_ACK)] + actions
        if now - lost_since >= self.handle_lost_give_up_sec:
            actions, _ = self.on_cancel_request(now)
            return actions
        if (self._handle_lost_notice_at is not None
                and now - self._handle_lost_notice_at >= self.handle_lost_repeat_sec):
            self._handle_lost_notice_at = now
            return [Haptic(HAPTIC_PATTERN_HANDLE_HINT),
                    Say(MSG_HANDLE_LOST, priority="response")]
        return []

    def _clear_handle_pause(self) -> None:
        self._handle_pause = False
        self._handle_lost_since = None
        self._handle_lost_notice_at = None

    def _enter_awaiting_user(self, now: float) -> list:
        """질문을 던지고 AWAITING_USER 로 들어간다."""
        self.state = State.AWAITING_USER
        self._response_deadline = now + APPROACH_QUESTION_STUCK_SEC
        self._approach.reset()
        self._approach_reasks = 0
        self._approach_after_reply = None
        return [
            SetNavSpeedLimit(NO_SPEED_LIMIT),
            Say(MSG_APPROACH_QUESTION, priority="response", expects_reply=True),
        ]

    def _approach_not_answered(self, reply: str, now: float) -> list:
        """접근 질문에 예·아니요가 아닌 말(질문·못 알아들음·잠깐 등, 2026-10-09 사용자 결정)."""
        reply = (reply or "").strip()
        if reply:
            self._approach_after_reply = (reply, now + self.approach_response_timeout_sec)
            self._response_deadline = None
            return []
        return self._approach_follow_up(now)

    def _approach_follow_up(self, now: float) -> list:
        """다시 묻기(이번 접근에서 APPROACH_REASK_MAX 번까지) 또는 물러나기(다 썼으면 — 홈으로)."""
        self._approach_after_reply = None
        if self._approach_reasks < APPROACH_REASK_MAX:
            self._approach_reasks += 1
            self._response_deadline = now + APPROACH_QUESTION_STUCK_SEC
            return [self._ask(MSG_APPROACH_REASK)]
        return self._approach_leave(MSG_APPROACH_UNANSWERED, now)

    def on_approach_reply_spoken(self, text: str, now: float) -> list:
        """접근 질문 중 질문에 LLM 이 답한 말이 재생을 마쳤다(/vica/tts_done) — 이제 다시 묻거나 물러난다."""
        if self.state != State.AWAITING_USER or self._approach_after_reply is None:
            return []
        if (text or "").strip() != self._approach_after_reply[0]:
            return []
        return self._approach_follow_up(now)

    def on_person_detection(
        self,
        track_id: int,
        distance_m: float,
        stable: bool,
        approachable: bool,
        now: float,
        person: Optional[Pose2D] = None,
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
        if self._is_suppressed(track_id, now) or self._place_suppressed(person, now):
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
            self._suppress_track(track_id, now, self.approach_person)
            if self.approach_turn_yaw_rad == 0.0 or self._near_call_no_spin:
                self._to_idle()
                self._user_attached_until = now + USER_ATTACHED_SUPPRESS_SEC
                return self._start_grip_wait(now)
            self.state = State.TURNING
            self.active_destination = None
            self._response_deadline = None
            self._turn_deadline = now + APPROACH_TURN_TIMEOUT_SEC
            return [
                Say(MSG_APPROACH_ACCEPTED, priority="response"),
                SpinInPlace(self.approach_turn_yaw_rad, reason="수락 — 핸들을 사람 쪽으로"),
            ]

        return self._approach_leave(MSG_APPROACH_DECLINED, now)

    def _approach_leave(self, message: str, now: float) -> list:
        """접근 질문을 접고 물러난다 — 아니요(MSG_APPROACH_DECLINED)든 대답 없음(MSG_APPROACH_UNANSWERED)이든"""
        actions: list = [Say(message, priority="response")]
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
                    self._ear_spoke_at = now
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
        if (state.startswith("empty") and self.state == State.IDLE
                and self._dest_prompt_stage in ("asked", "retried")):
            return self._advance_dest_prompt(now)
        return []

    def on_llm_thinking(self, thinking: bool, now: float) -> list:
        """/vica/thinking — LLM 노드가 사용자 말을 판단하는 중인가(2026-10-10 사용자 결정). 발화 없음."""
        if thinking:
            if not self._llm_thinking:
                self._llm_thinking_since = now
            self._llm_thinking = True
            self._llm_thinking_tail_until = None
        else:
            if self._llm_thinking:
                self._llm_thinking_tail_until = now + LLM_THINKING_TAIL_SEC
            self._llm_thinking = False
        return []

    def _llm_thinking_holds(self, now: float) -> bool:
        if (self._llm_thinking and self._llm_thinking_since is not None
                and now - self._llm_thinking_since <= EAR_HOLD_MAX_SEC):
            return True
        return (self._llm_thinking_tail_until is not None
                and now < self._llm_thinking_tail_until)

    def _dest_prompt_holds(self, now: float) -> bool:
        """온보딩 되묻기 사다리의 시계를 잡아둘 이유가 있는가."""
        if (self._ear_speaking and self._ear_speech_since is not None
                and now - self._ear_speech_since <= EAR_HOLD_MAX_SEC):
            return True
        if self._llm_thinking_holds(now):
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
        if self._llm_thinking_holds(now):
            return True
        return (self._ear_grace_until is not None
                and now < self._ear_grace_until)

    def _obstacle_ear_holds(self, now: float) -> bool:
        """장애물 안내용 _ear_holds — 이 말의 LLM 답이 이미 왔으면 닫힘 뒤 유예(EAR_GRACE_SEC)는 보지 않는다"""
        if (self._ear_busy and self._ear_busy_since is not None
                and now - self._ear_busy_since <= EAR_HOLD_MAX_SEC):
            return True
        if self._llm_thinking_holds(now):
            return True
        if self._ear_grace_until is None or now >= self._ear_grace_until:
            return False
        answered = (not self._llm_thinking and self._llm_thinking_since is not None
                    and self._ear_busy_since is not None
                    and self._llm_thinking_since >= self._ear_busy_since)
        return not answered

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
        self.cancel_confirm_pending = False
        self._cancel_confirm_deadline = None
        if dest is not None:
            self._arrived_destination = dest
            self._last_guided = dest
        self.active_destination = None
        self._asking_is_finish = is_finish
        self._asking_time_after_yes = ask_time
        self._asking_entered_at = now
        self._arrival_retried = False
        self._asking_question = question
        self._deny_reconfirmed = False
        self._asking_where = False
        self._leaving_deadline = None
        self._response_deadline = None
        text = f"{arrival_text} {question}".strip() if arrival_text else question
        return [Say(text, priority="response", expects_reply=True)]

    def on_arrival_question_spoken(self, now: float) -> list:
        """도착 후 질문 재생이 끝났다. 응답 대기 8초는 여기서부터 센다"""
        if self.state in (State.ASKING_NEXT, State.ASKING_WAIT_TIME):
            self._response_deadline = now + self.approach_response_timeout_sec
        return []

    def on_wake_call(self, now: float) -> list:
        """"비카야" 한 번 — 호출 반응표대로 대답할지 정한다(2026-10-07 사용자 결정)."""
        if self.estop_active or self.state == State.ESTOPPED:
            notice_at = self._tip_notice_at
            if (self.tipped and notice_at is not None
                    and now - notice_at < TIP_SELF_WAKE_GUARD_SEC):
                return [WakeReply(listen=False)]
            self._note_tip_notice(now)
            return [Say(MSG_ESTOP_WAKE, priority="response"), WakeReply(listen=False)]
        if self.state in _WAKE_IGNORE_STATES:
            return [WakeReply(listen=False)]
        if self.state == State.AWAITING_USER:
            return [WakeReply(listen=True)]
        actions: list = [StopSpeech(), Say(MSG_WAKE_GREETING, priority="response"),
                         WakeReply(listen=True)]
        actions.extend(self.on_wake(now))
        return actions

    def on_wake(self, now: float) -> list:
        """"비카야" — 새 대화의 시작 신호. 상태 정리만 한다("네?"·판정은 on_wake_call)."""
        self._forget_dest_prompt()
        if self.state in (State.WAITING, State.WAITING_RELEASE):
            self._wake_consumed_at = now
            if self.state == State.WAITING:
                return [Haptic(HAPTIC_PATTERN_WAKE_LOCATE)]
            return []
        if self.state in _WAIT_MOVING_STATES:
            return []
        if self.state == State.CONFIRMING:
            actions = self._fold_confirming(now, announce=False)
            self._wake_consumed_at = now
            return actions
        if self.state in (State.ASKING_NEXT, State.ASKING_WAIT_TIME):
            self._leaving_deadline = None
            self._arrival_retried = False
            self._response_deadline = None
            self._asking_entered_at = now
            self._wake_consumed_at = now
            return []
        if self.state == State.AWAITING_USER:
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
        if self._user_attached_until is None:
            return False
        return now < self._user_attached_until or self._grip.contact(now)

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
                          next_dest: Optional[Destination] = None,
                          bounds: Optional[MapBounds] = None,
                          nav_ready: bool = True) -> list:
        """도착 후 질문에 대한 답. 정리된 intent 만 받아 갈래를 만든다."""
        if self.state not in (State.ASKING_NEXT, State.ASKING_WAIT_TIME):
            return []
        kind = intent.intent

        if kind == "navigate" and next_dest is not None:
            reason = check_gate(intent, next_dest, bounds, self.estop_active, nav_ready)
            if reason != GateReason.OK:
                msg = _REJECT_MESSAGES.get(reason)
                return [Say(msg, priority="response")] if msg else []
            self.state = State.NAVIGATING
            self.active_destination = next_dest
            self.handle_active = self._decide_handle_mode(now)
            self._reset_arrival_dialog()
            self._nav_tree = NAV_TREE_GUIDED
            self.door_side = ""
            return [SetNavSpeedLimit(NO_SPEED_LIMIT),
                    Say(say_destination(MSG_START, next_dest.name)),
                    Navigate(next_dest, tree=NAV_TREE_GUIDED)]

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
                self._asking_question = MSG_ASK_WAIT_TIME
                self._response_deadline = None
                return [Say(MSG_ASK_WAIT_TIME, priority="response",
                            expects_reply=True)]
            return self._enter_waiting(min(minutes, WAIT_MINUTES_CAP), now)

        if kind == "deny":
            if self._asking_is_finish:
                if self._wait_minutes_requested > 0:
                    return self._enter_waiting(self._wait_minutes_requested, now)
                return self._enter_waiting(WAIT_MINUTES_CAP, now,
                                           default_msg=True)
            if not self._deny_reconfirmed:
                self._deny_reconfirmed = True
                self._asking_is_finish = True
                self._asking_time_after_yes = False
                self._asking_question = MSG_ASK_ENTRANCE
                self._asking_entered_at = now
                self._response_deadline = None
                return [Say(MSG_ASK_ENTRANCE, priority="response", expects_reply=True)]
            self._reset_arrival_dialog()
            return [Say(MSG_FINISH, priority="response"), *self._go_home(now)]

        if kind in ("question", "clarify"):
            self._response_deadline = None
            self._asking_entered_at = now
            return []

        return self._arrival_no_answer(now)

    def _enter_waiting(self, minutes: int, now: float,
                       default_msg: bool = False, away: bool = False) -> list:
        """대기 확정 + 멘트. 사람접근은 대기 상태값으로 자연히 꺼진다."""
        self._wait_until = now + minutes * 60.0
        self._wait_minutes_requested = int(minutes)
        self._response_deadline = None
        self._leaving_deadline = None
        dest = self._arrived_destination
        spot = dest.wait_spot if dest is not None else None
        place = WAIT_PLACE_PHRASES.get(spot.side, "") if spot is not None else ""
        if not place and away and dest is not None:
            msg = (MSG_WAIT_SPOT_DEFAULT.format(place=WAIT_PLACE_AT_DESTINATION) if default_msg
                   else MSG_WAIT_SPOT_CONFIRM.format(minutes=minutes,
                                                     place=WAIT_PLACE_AT_DESTINATION))
            back = wait_back_destination(dest)
            self.state = State.MOVING_BACK_TO_DEST
            self.active_destination = back
            self.handle_active = False
            self._wait_place = "destination"
            self._beacon_next_at = now + WAIT_BEACON_INTERVAL_SEC
            return [Say(msg, priority="response"), SetNavSpeedLimit(NO_SPEED_LIMIT),
                    Navigate(back, tree=NAV_TREE_WAIT)]
        if not place:
            self.state = State.WAITING
            self._wait_place = ""
            msg = (MSG_WAIT_DEFAULT if default_msg
                   else MSG_WAIT_CONFIRM.format(minutes=minutes))
            return [Say(msg, priority="response")]
        msg = (MSG_WAIT_SPOT_DEFAULT.format(place=place) if default_msg
               else MSG_WAIT_SPOT_CONFIRM.format(minutes=minutes, place=place))
        self.state = State.WAITING_RELEASE
        self._wait_place = "spot"
        self._release_text = msg
        self._release_spoken_at = None
        self._release_entered_at = now
        return [Say(msg, priority="response")]

    def _door_side_sentence(self, dest: Optional[Destination]) -> str:
        """도착 순간의 M1 문장("화장실은 오른쪽에 있습니다."). 못 정하면 ""."""
        self.door_side = ""
        if (dest is None or dest.door_yaw_deg is None
                or self.robot_yaw_deg is None):
            return ""
        side = door_side_word(dest.door_yaw_deg, self.robot_yaw_deg)
        self.door_side = side
        return MSG_DOOR_SIDE.format(
            name=dest.name, eun=josa_eun_neun(dest.name), side=side)

    def on_wait_speech_spoken(self, text: str, now: float) -> list:
        """M2 재생이 끝났다(노드가 /vica/tts_done 문구로 알려 준다). 손 놓기 판정은"""
        if (self.state == State.WAITING_RELEASE and self._release_text
                and self._release_text in (text or "")):
            self._release_spoken_at = now
        return []

    def _wait_release_tick(self, now: float) -> list:
        """WAITING_RELEASE: M2 가 끝나고 손을 놓았으면 대기 장소로 떠난다."""
        spoken = self._release_spoken_at is not None or (
            self._release_entered_at is not None
            and now - self._release_entered_at >= WAIT_RELEASE_SPEECH_FALLBACK_SEC)
        if not spoken:
            return []
        sensor = (not self.grip_assume_held) and self._grip.fresh(now)
        if sensor and self._grip.released_for(now) < WAIT_RELEASE_SEC:
            return []
        if self._ear_holds(now):
            return []
        return self._start_moving_to_wait_spot(now)

    def _start_moving_to_wait_spot(self, now: float) -> list:
        dest = self._arrived_destination
        if dest is None or dest.wait_spot is None:
            self.state = State.WAITING
            self._wait_place = ""
            return []
        wait_dest = wait_spot_destination(dest)
        self.state = State.MOVING_TO_WAIT_SPOT
        self.active_destination = wait_dest
        self.handle_active = False
        self._release_text = ""
        self.door_side = ""
        self._beacon_next_at = now + WAIT_BEACON_INTERVAL_SEC
        return [SetNavSpeedLimit(NO_SPEED_LIMIT), Navigate(wait_dest, tree=NAV_TREE_WAIT)]

    def _wait_spot_blocked(self, now: float) -> list:
        """대기 장소에 못 들어갔다(Nav2 실패 신호). 다시 출발시키지 않는다 —"""
        dest = self._arrived_destination
        wait_name = f"{dest.name}-대기" if dest is not None else ""
        actions: list = [
            Say(MSG_WAIT_SPOT_BLOCKED, priority="response"),
            GoalEvent("wait_spot_blocked", dest,
                      f"주행(Nav2)이 '{wait_name}'에 들어가지 못했습니다.",
                      wait_place="spot", wait_minutes=self._wait_minutes_requested),
        ]
        self._wait_place = "destination"
        if dest is None:
            self.state = State.WAITING
            self.active_destination = None
            return actions
        back = wait_back_destination(dest)
        self.state = State.MOVING_BACK_TO_DEST
        self.active_destination = back
        actions.append(Navigate(back, tree=NAV_TREE_WAIT))
        return actions

    def _beacon_tick(self, now: float) -> list:
        """M3 — 대기 장소가 있는 대기에서 10초마다. 대화 중이면 건너뛴다(박자는 유지)."""
        if not self._wait_place or self._beacon_next_at is None:
            return []
        if now < self._beacon_next_at:
            return []
        while self._beacon_next_at <= now:
            self._beacon_next_at += WAIT_BEACON_INTERVAL_SEC
        if self._ear_holds(now):
            return []
        return [Say(MSG_WAIT_BEACON, priority="ambient")]

    def _at_home(self) -> bool:
        """IDLE 이고 실제 위치가 홈 HOME_BEACON_RADIUS_M 안이다. 위치나 홈이 없으면 False."""
        home = self.return_destination
        pose = self.robot_pose
        if self.state != State.IDLE or home is None or pose is None:
            return False
        return math.hypot(pose.x - home.pose.x,
                          pose.y - home.pose.y) <= HOME_BEACON_RADIUS_M

    def obstacle_cue(self, phrase: str, onset: float, now: float) -> tuple[list, str]:
        """장애물 안내 후보를 말할지 정한다 → (actions, 뺀 이유). 말하면 이유는 ""."""
        text = {"avoid": MSG_OBSTACLE_AVOID, "slow": MSG_OBSTACLE_SLOW, "stop": MSG_OBSTACLE_STOP}.get(phrase)
        if text is None:
            return [], "unknown_phrase"
        if self.dialog_state != State.NAVIGATING.value:
            return [], "not_navigating"
        if self.cancel_confirm_pending:
            return [], "question_pending"
        if self._obstacle_ear_holds(now):
            return [], "ear_busy"
        if now - onset > OBSTACLE_STALE_SEC:
            return [], "stale"
        return [Say(text, priority="ambient")], ""

    def _home_beacon_tick(self, now: float) -> list:
        """홈 알림 — 홈에서 쉬는 동안 1분마다 M3. 첫 마디는 홈에 선 지 1분 뒤."""
        if self.home_beacon_interval_sec <= 0 or not self._at_home():
            self._home_beacon_next_at = None
            return []
        if self._home_beacon_next_at is None:
            self._home_beacon_next_at = now + self.home_beacon_interval_sec
            return []
        if now < self._home_beacon_next_at:
            return []
        while self._home_beacon_next_at <= now:
            self._home_beacon_next_at += self.home_beacon_interval_sec
        if self._ear_holds(now):
            return []
        return [Say(MSG_WAIT_BEACON, priority="ambient")]

    def _wait_expired(self, now: float) -> list:
        """M7 + 앱 알림 + 홈. 대기 장소가 있든 없든 모든 대기의 만료다."""
        dest = self._arrived_destination
        minutes = self._wait_minutes_requested
        place = self.wait_place or "목적지"
        actions: list = [
            Say(MSG_WAIT_EXPIRED, priority="response"),
            GoalEvent("wait_expired", dest,
                      f"{place}에서 {minutes}분 기다렸습니다." if minutes > 0 else "",
                      wait_place=self._wait_place or "destination",
                      wait_minutes=minutes),
        ]
        self._reset_arrival_dialog()
        actions.extend(self._go_home(now))
        return actions

    def _arrival_no_answer(self, now: float) -> list:
        """무응답 사다리: 못 알아들으면 같은 질문을 1회 다시 묻고, 그 뒤엔 떠나기 예고."""
        if not self._arrival_retried:
            self._arrival_retried = True
            self._response_deadline = None
            self._asking_entered_at = now
            return [self._ask(self._asking_question or MSG_ARRIVAL_RETRY)]
        return self._leaving_notice(now)

    def _arrival_silence(self, now: float) -> list:
        """침묵 사다리 (2026-09-20 사용자 결정): 8초 침묵이면 같은 질문을 한 번 더"""
        if not self._arrival_retried and self._asking_question:
            self._arrival_retried = True
            self._response_deadline = None
            self._asking_entered_at = now
            return [Say(self._asking_question, priority="response", expects_reply=True)]
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
        self._late_answer_finish = None
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
        self._asking_question = ""
        self._deny_reconfirmed = False
        self._asking_where = False
        self._leaving_deadline = None
        self._wait_until = None
        self._wait_minutes_requested = -1
        self._response_deadline = None
        self._arrived_destination = None
        self._wait_place = ""
        self._beacon_next_at = None
        self._release_text = ""
        self._release_spoken_at = None
        self._release_entered_at = None
        self._wait_finish_asked_at = None
        self._wait_need_asked_at = None
        self._wait_finish_reasked = False
        self._wait_need_reasked = False
        self._wait_hold = None

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
        if moving and not already_estopped:
            self._estop_announced = True
            actions.append(Say(MSG_ESTOPPED, priority="emergency"))
        elif not self._estop_announced:
            self._estop_announced = True
            actions.append(Say(MSG_ESTOP_WAKE, priority="emergency"))
        return actions

    def on_app_estop(self, active: bool) -> None:
        """/app_estop_state 의 app_active (1 Hz, 앱 버튼을 누른 순간에도 한 번 더 온다)."""
        self.app_estop_active = active

    def on_tipped(self, active: bool, now: float) -> list:
        """/vica/tipped (IMU 어댑터 C++, 바뀔 때만·transient_local). 2026-10-15."""
        if not active:
            self.tipped = False
            self._tip_notice_at = None
            return []
        if self.tipped:
            return []
        actions: list = []
        if self.state in _GOAL_ACTIVE_STATES:
            actions.append(SetNavSpeedLimit(NO_SPEED_LIMIT))
            actions.append(CancelNav(self.active_destination))
        if self.state != State.ESTOPPED:
            self._enter_estopped(now)
        self._tip_notice_at = now
        self.tipped = True
        self._estop_announced = True
        actions.append(Say(MSG_TIPPED, priority="emergency"))
        return actions

    def _note_tip_notice(self, now: float) -> None:
        """넘어져 있는 동안 비상정지를 말로 알렸다 — 다음 반복을 이때부터 센다."""
        if self.tipped:
            self._tip_notice_at = now

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
                moving = self.state in _GOAL_ACTIVE_STATES and not self.tipped
                self._enter_estopped(now)
                self._estop_announced = moving or self.tipped
                if moving:
                    text = MSG_ESTOPPED_BY_APP if self.app_estop_active else MSG_ESTOPPED
                    actions.append(Say(text, priority="emergency"))
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
        nav_ready: bool = True,
    ) -> list:
        """주기 처리. distance_remaining 은 Nav2 feedback 의 남은 거리(m)다."""
        actions: list = []
        self._prune_suppressed(now)

        notice_at = self._tip_notice_at
        if (self.tipped and notice_at is not None
                and now - notice_at >= TIP_REPEAT_SEC):
            self._tip_notice_at = now
            actions.append(Say(MSG_TIPPED, priority="emergency"))

        actions.extend(self._help_tick(now))

        if (self.state == State.NAVIGATING and self.handle_active
                and nav_status not in (NavStatus.SUCCEEDED, NavStatus.FAILED,
                                       NavStatus.CANCELED)):
            handle_actions = self._handle_nav_tick(now, distance_remaining)
            if self.state == State.PAUSED:
                return actions + handle_actions
            actions.extend(handle_actions)

        if self.cancel_confirm_pending:
            actions.extend(self._cancel_confirm_tick(now))

        if self.state == State.CONFIRMING:
            if (not self._confirm_reasked and self._confirm_prompt
                    and self._confirm_reask_at is not None and now >= self._confirm_reask_at
                    and not self._ear_holds(now)):
                self._confirm_reasked = True
                self._confirm_deadline = max(self._confirm_deadline or now,
                                             now + QUESTION_REASK_SEC)
                actions.append(self._ask(self._confirm_prompt))
            elif self._confirm_deadline is not None and now >= self._confirm_deadline:
                if self._change_from is not None:
                    actions.extend(self._fold_confirming(now))
                elif self._confirm_from_approach:
                    actions.extend(self._fold_approach_confirm(now))
                else:
                    self._fold_confirming(now)
                    actions.append(Say(MSG_CONFIRM_TIMEOUT))

        elif self.state == State.NAVIGATING:
            if nav_status == NavStatus.SUCCEEDED:
                dest = self.active_destination
                text = (
                    dest.arrival_message
                    if dest and dest.arrival_message
                    else MSG_ARRIVED_FALLBACK.format(name=dest.name if dest else "목적지")
                )
                self._approach.reset()
                self.handle_active = False
                actions.append(SetNavSpeedLimit(NO_SPEED_LIMIT))
                if dest is not None and is_wait_destination_id(dest.id):
                    self.state = State.ARRIVED
                    self._dwell_until = now + self.dwell_sec
                elif self.arrival_dialog and not self._nav_from_app:
                    door = self._door_side_sentence(dest)
                    arrival = f"{text} {door}".strip() if door else text
                    actions.extend(self._ask_arrival(dest, now, arrival_text=arrival))
                else:
                    self.state = State.ARRIVED
                    self._dwell_until = now + self.dwell_sec
                    door = "" if self._nav_from_app else self._door_side_sentence(dest)
                    actions.append(Say(f"{text} {door}".strip() if door else text))
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

                wait_check = (failed_dest is not None
                              and is_wait_destination_id(failed_dest.id))
                retryable = (
                    nav_status == NavStatus.FAILED
                    and failed_dest is not None
                    and not wait_check
                    and self._nav_retry_count < self.nav_retry_limit
                )
                if retryable:
                    self._nav_retry_count += 1
                    self._retry_destination = failed_dest
                    self._retry_at = now + self.nav_retry_delay_sec
                else:
                    self._retry_destination = None
                    self._retry_at = None
                    if not wait_check:
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

        elif self.state == State.PAUSED:
            if self._handle_pause:
                actions.extend(self._handle_pause_tick(now, nav_ready))

        elif self.state == State.IDLE:
            if self._grip_wait_since is not None:
                actions.extend(self._grip_wait_tick(now))
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
                elif (now >= self._return_resume_deadline
                      and not self._ear_holds(now)):
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
                self._user_attached_until = now + USER_ATTACHED_SUPPRESS_SEC
                actions.extend(self._start_grip_wait(now))
            elif nav_status in (NavStatus.FAILED, NavStatus.CANCELED):
                self._to_idle()
                self._user_attached_until = now + USER_ATTACHED_SUPPRESS_SEC
                actions.extend(self._start_grip_wait(now))
            elif (self._turn_deadline is not None
                  and now >= self._turn_deadline):
                self._approach_dest = None
                self._to_idle()

        elif self.state == State.AWAITING_USER:
            if self._approach_after_reply is not None:
                if now >= self._approach_after_reply[1]:
                    actions.extend(self._approach_follow_up(now))
            elif (self._response_deadline is not None and now >= self._response_deadline
                    and not self._ear_holds(now)):
                actions.extend(self._approach_follow_up(now))

        elif self.state in (State.ASKING_NEXT, State.ASKING_WAIT_TIME):
            if (self._leaving_deadline is not None
                    and now >= self._leaving_deadline
                    and not self._ear_holds(now)):
                self._late_answer_finish = (
                    self._asking_is_finish if self._asking_question else None)
                self._reset_arrival_dialog()
                actions.extend(self._go_home(now))
            elif (self._leaving_deadline is None
                  and self._response_deadline is not None
                  and now >= self._response_deadline
                  and not self._ear_holds(now)):
                actions.extend(self._arrival_silence(now))
            elif (self._leaving_deadline is None
                  and self._response_deadline is None
                  and self._asking_entered_at is not None
                  and now - self._asking_entered_at >= ASKING_STUCK_FALLBACK_SEC
                  and not self._ear_holds(now)):
                actions.extend(self._leaving_notice(now))

        elif self.state == State.WAITING_RELEASE:
            if (self._wait_until is not None and now >= self._wait_until
                    and not self._ear_holds(now)):
                actions.extend(self._wait_expired(now))
            else:
                actions.extend(self._wait_release_tick(now))

        elif self.state == State.MOVING_TO_WAIT_SPOT:
            if nav_status == NavStatus.SUCCEEDED:
                self.state = State.WAITING
                self.active_destination = None
            elif nav_status in (NavStatus.FAILED, NavStatus.CANCELED):
                actions.extend(self._wait_spot_blocked(now))
            actions.extend(self._beacon_tick(now))

        elif self.state == State.MOVING_BACK_TO_DEST:
            if nav_status in (NavStatus.SUCCEEDED, NavStatus.FAILED,
                              NavStatus.CANCELED):
                self.state = State.WAITING
                self.active_destination = None
            actions.extend(self._beacon_tick(now))

        elif self.state == State.WAITING:
            if (self._wait_until is not None and now >= self._wait_until
                    and not self._ear_holds(now)):
                actions.extend(self._wait_expired(now))
            else:
                asked = self._wait_question_tick(now)
                actions.extend(asked)
                if not asked:
                    actions.extend(self._beacon_tick(now))

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
                    actions.append(Navigate(dest, tree=self._nav_tree))
            elif self._dwell_until is None or now >= self._dwell_until:
                self._to_idle()

        elif self.state == State.ESTOPPED:
            if not self.estop_active and not self.tipped:
                t0 = (
                    self._estop_clear_since
                    if self._estop_clear_since is not None
                    else self._estop_entered_at
                )
                if t0 is not None and now - t0 >= self.estop_release_grace_sec:
                    announced = self._estop_announced
                    self._estop_announced = False
                    self._to_idle()
                    actions.extend(self._end_help())
                    if announced:
                        actions.append(Say(MSG_ESTOP_RELEASED, priority="emergency"))

        actions.extend(self._home_beacon_tick(now))
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
        return True, reason, [*self._end_help(), *self._enter_returning(now, is_home=True)]

    def _enter_returning(
        self, now: float, is_home: bool = False, dialog_finish: bool = False
    ) -> list:
        """대기 위치(= 홈)로 돌아간다."""
        self._arrived_destination = None
        if not dialog_finish:
            self._last_guided = None
            self._late_answer_finish = None
        self.door_side = ""
        destination = (
            self.return_destination
            if is_home or dialog_finish or self.auto_return_home
            else None
        )
        self.state = State.RETURNING
        self._returning_home = is_home
        self.active_destination = destination
        self._handle_engaged = False
        self.handle_active = False
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
        person = self.approach_person
        was_home = self._returning_home
        self._last_guided = None
        self._late_answer_finish = None
        self._to_idle()
        if not was_home:
            self._suppress_track(track_id, now, person)

    def _suppress_track(
        self, track_id: Optional[int], now: float, person: Optional[Pose2D] = None
    ) -> None:
        """이 사람에게 당분간 다시 다가가지 않는다."""
        until = now + self.reapproach_suppress_sec
        if person is not None:
            self._suppressed_places.append((person, until))
        if track_id is None or track_id == TRACK_ID_NONE:
            return
        self._suppressed_tracks[track_id] = until

    def _is_suppressed(self, track_id: int, now: float) -> bool:
        until = self._suppressed_tracks.get(track_id)
        return until is not None and now < until

    def _place_suppressed(self, person: Optional[Pose2D], now: float) -> bool:
        """이 자리(반경 SUPPRESS_RADIUS_M)의 사람을 최근 억제했나. 자리를 모르면 아니다."""
        return any(
            now < until and same_person(person, place, SUPPRESS_RADIUS_M)
            for place, until in self._suppressed_places
        )

    def _prune_suppressed(self, now: float) -> None:
        """지난 억제를 버린다. 하루 종일 서 있으면 track_id 가 계속 쌓인다."""
        expired = [t for t, until in self._suppressed_tracks.items() if now >= until]
        for track_id in expired:
            del self._suppressed_tracks[track_id]
        self._suppressed_places = [
            (place, until) for place, until in self._suppressed_places if now < until
        ]

    def _enter_estopped(self, now: float) -> None:
        self.state = State.ESTOPPED
        self.active_destination = None
        self.paused_destination = None
        self._paused_returning_home = False
        self.cancel_confirm_pending = False
        self._cancel_confirm_deadline = None
        self._confirming_dest_id = None
        self._confirm_deadline = None
        self._last_guided = None
        self._late_answer_finish = None
        self._approach_dest = None
        self._estop_entered_at = now
        self._estop_clear_since = None
        self._announced_milestones = set()
        self._distance_baseline = None
        self._approach.reset()
        self._suppress_track(self.approach_track_id, now, self.approach_person)
        self.approach_track_id = None
        self.approach_goal_pose = None
        self.approach_person = None
        self._recheck.reset()
        self._response_deadline = None
        self._nav_from_app = False
        self._handle_engaged = False
        self.handle_active = False
        self._grip_wait_since = None
        self._grip_pulse_at = None
        self._clear_handle_pause()
        self._arrived_destination = None
        self.door_side = ""
        self._reset_arrival_dialog()
        self._change_from = None

    def _to_idle(self) -> None:
        self.state = State.IDLE
        self.active_destination = None
        self._arrived_destination = None
        self.paused_destination = None
        self._paused_returning_home = False
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
        self.approach_person = None
        self._recheck.reset()
        self._returning_home = False
        self.approach_goal_pose = None
        self._response_deadline = None
        self._nav_from_app = False
        self._nav_tree = NAV_TREE_DEFAULT
        self._wait_hold = None
        self._change_from = None
        self._wait_place = ""
        self._beacon_next_at = None
        self._release_text = ""
        self._release_spoken_at = None
        self._release_entered_at = None
        self._wait_finish_asked_at = None
        self._near_call_no_spin = False
        self._never_approached = False
        self.handle_active = False
        self._grip_wait_since = None
        self._grip_pulse_at = None
        self._clear_handle_pause()
