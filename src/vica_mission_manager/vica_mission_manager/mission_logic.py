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
from typing import Optional, Sequence, Union

from .approach_speed import ApproachSpeedLadder, NO_SPEED_LIMIT

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
    # 목적지를 기억한 채 멈춘 상태. 사용자가 다시 출발을 요청하면 그 목적지로
    # 새 goal 을 만든다. E-stop 과 달리 래치도 reset 도 없다.
    PAUSED = "paused"


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
    PRIVATE_DESTINATION = "private_destination"
    NOT_APPROACHABLE = "not_approachable"
    POSE_INVALID = "pose_invalid"
    NAV_NOT_READY = "nav_not_ready"
    # 취소·일시정지·재개 요청 전용 사유.
    NOT_NAVIGATING = "not_navigating"
    NOT_PAUSED = "not_paused"


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
    authorization: str = "public"
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
    # ros_tts_node 큐 우선순위 (긴급 > 내레이션 > 응답).
    # 노드가 "{priority}:{text}" 접두어로 /vica/tts_request 에 발행한다.
    priority: str = "narration"  # emergency / narration / response


@dataclass(frozen=True)
class Navigate:
    destination: Destination


@dataclass(frozen=True)
class CancelNav:
    destination: Optional[Destination] = None
    # 노드가 /vica_goal_event 로 알릴 이벤트 이름. 일시정지도 Nav2 goal 을 취소하지만
    # 목적지를 기억하므로 취소와 구분해서 알려야 앱이 "주행 끝"으로 오해하지 않는다.
    event: str = "goal_canceled"


@dataclass(frozen=True)
class SetNavSpeedLimit:
    """Nav2 controller 최대속도 제한율. 0.0은 제한 해제다."""

    percent: float


Action = Union[Say, Navigate, CancelNav, SetNavSpeedLimit]


# ---- 멘트 (v1 임시 카피 — 시각장애인 관점 감수는 미결 사항 #4) ----------------
#
# 주의: 이 멘트에 HARD_EMERGENCY_KEYWORDS 가 들어가면 안 된다. 상시 긴급어 감시가
# 스피커로 나간 로봇 자기 목소리를 다시 긴급어로 인식해 E-stop 을 거는 자가 트리거가
# 생긴다 (/vica/emergency → emergency_estop_bridge → /voice_emergency_stop).
# test/test_spoken_text.py 가 이를 강제한다.


def josa_euro(word: str) -> str:
    """단어 뒤에 붙는 조사 '으로 / 로' 를 받침에 맞게 돌려준다.

    받침 없음 또는 ㄹ 받침이면 '로', 그 외 받침이면 '으로'.
    예) 화장실 -> 로, 안내센터 -> 로, 식당 -> 으로

    "(으)로" 를 그대로 두면 TTS 가 "화장실으로" 처럼 읽는다 (2026-08-04 실기 확인).
    vica-voice-llm 의 destination_loader._josa_euro 와 같은 로직이며, 저장소 간
    의존을 만들지 않으려고 사본을 둔다 (freshness.py 사본과 같은 이유).
    """
    if not word:
        return "로"
    last = word[-1]
    if not ("가" <= last <= "힣"):  # 한글이 아니면 '로'로 둔다
        return "로"
    jongseong = (ord(last) - 0xAC00) % 28  # 0=받침없음, 8=ㄹ
    return "로" if jongseong in (0, 8) else "으로"


def say_destination(template: str, name: str) -> str:
    """목적지 이름이 들어가는 멘트를 조사까지 맞춰 완성한다.

    호출부가 josa 를 빠뜨리면 KeyError 로 바로 드러나지만, 매번 두 인자를 넘기는
    대신 여기 한 곳을 거치게 해 빠뜨릴 자리를 없앤다.
    """
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
MSG_STALE_CONFIRM = "요청이 확인되지 않았습니다. 다시 말씀해 주세요."
MSG_NAV_FAILED = "죄송합니다. 이동에 실패했습니다. 다시 시도해 주세요."
MSG_ESTOPPED = "안전을 위해 멈추겠습니다."
MSG_ESTOP_RELEASED = "비상 멈춤이 해제되었습니다. 새로운 목적지를 말씀해 주세요."
MSG_DISTANCE_REMAINING = "목적지까지 약 {meters}미터 남았습니다."
MSG_CANCELED = "안내를 취소했습니다."
MSG_PAUSED = "잠시 멈추겠습니다. 다시 출발하려면 말씀해 주세요."
MSG_RESUMED = "{name}{josa} 다시 출발합니다."
MSG_CANCEL_CONFIRM = "안내를 취소할까요?"
MSG_CANCEL_KEPT = "안내를 계속하겠습니다."
MSG_NOT_NAVIGATING = "지금은 안내 중이 아닙니다."
MSG_NOT_PAUSED = "다시 출발할 안내가 없습니다."

# 남은 거리를 알리는 지점(미터). 눈으로 확인할 수 없는 사용자가 도착을 미리
# 준비할 수 있게 하려는 것이므로, 자주 말하기보다 접근 시점만 짚는다.
# 각 지점은 목적지 하나당 한 번만 안내한다.
#
# 감속 단계(approach_speed.DEFAULT_APPROACH_STAGES)와는 별개다. 안내는 3 m 에서
# 하고 감속은 1.5 m 부터 시작한다 — 사용자가 먼저 듣고, 그 다음 몸으로 느낀다.
DISTANCE_MILESTONES_M = (10.0, 3.0)

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
    """취소 요청 게이트. 주행 중이거나 일시정지 상태일 때만 취소할 수 있다.

    E-stop 중에는 거부한다. E-stop 이 상위 상태이고, 취소로 그 상태를 흔들면
    안 되기 때문이다 (해제는 관리자 reset 경로만 담당한다).
    """
    if estop_active or state == State.ESTOPPED:
        return GateReason.ESTOP_ACTIVE
    if state not in (State.NAVIGATING, State.PAUSED):
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
    """재개 게이트. 일시정지로 보관한 목적지가 있어야 다시 출발할 수 있다.

    E-stop 이 걸리면 보관분을 폐기하므로 여기서 목적지가 없다면 재개할 수 없다.
    "E-stop 해제 후 이전 Goal 을 자동 재개하지 않는다"는 원칙과 같은 방향이다.
    """
    if estop_active or state == State.ESTOPPED:
        return GateReason.ESTOP_ACTIVE
    if state != State.PAUSED or paused_destination is None:
        return GateReason.NOT_PAUSED
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
        approach_stages: Optional[Sequence] = None,
    ) -> None:
        self.confirm_timeout_sec = confirm_timeout_sec
        self.dwell_sec = dwell_sec
        self.estop_release_grace_sec = estop_release_grace_sec
        # 접근 감속 사다리. 단계 검증(거리 양수·비율 범위·단조 감소)은
        # ApproachSpeedLadder 가 하고 잘못된 값이면 여기서 ValueError 로 죽는다.
        self._approach = ApproachSpeedLadder(approach_stages)

        self.state: State = State.IDLE
        self.estop_active: bool = False
        self.active_destination: Optional[Destination] = None
        # 일시정지로 보관한 목적지. active_destination 은 _to_idle/_enter_estopped 에서
        # 비워지므로 재개할 목적지는 따로 들고 있어야 한다.
        self.paused_destination: Optional[Destination] = None
        # 음성 취소 재확인 대기 여부. 확인하는 동안에도 로봇은 계속 주행한다.
        self.cancel_confirm_pending: bool = False

        self._cancel_confirm_deadline: Optional[float] = None
        self._confirming_dest_id: Optional[str] = None
        self._confirm_deadline: Optional[float] = None
        self._dwell_until: Optional[float] = None
        self._estop_entered_at: Optional[float] = None
        self._estop_clear_since: Optional[float] = None
        self._announced_milestones: set = set()  # 이번 목적지에서 안내한 거리 지점

    # -- 접근 감속 조회 ---------------------------------------------------------

    @property
    def approach_stages(self) -> tuple:
        """검증·정렬을 마친 접근 감속 단계 목록 (먼 거리부터)."""
        return self._approach.stages

    @property
    def approach_speed_limit_percent(self) -> float:
        """지금 걸려 있는 접근 제한율. 제한 전이면 0.0(해제)."""
        return self._approach.percent

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
            return [Say(MSG_ESTOP_REJECT, priority="response")]

        if self.state == State.NAVIGATING:
            # v1 정책: 주행 중 새 목적지는 거부 (소프트 취소는 v2, TODOS.md #4)
            return [Say(MSG_BUSY, priority="response")]

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
            return [Say(MSG_STALE_CONFIRM, priority="response")]

        reason = check_gate(intent, dest, bounds, self.estop_active, nav_ready)
        if reason != GateReason.OK:
            self._to_idle()
            msg = _REJECT_MESSAGES.get(reason)
            return [Say(msg, priority="response")] if msg else []

        assert dest is not None  # check_gate 가 보장
        self.state = State.NAVIGATING
        self.active_destination = dest
        self._confirming_dest_id = None
        self._confirm_deadline = None
        self._announced_milestones = set()
        self._approach.reset()
        return [
            SetNavSpeedLimit(NO_SPEED_LIMIT),
            Say(say_destination(MSG_START, dest.name)),
            Navigate(dest),
        ]

    # -- 취소 / 일시정지 / 재개 --------------------------------------------------
    #
    # 세 요청 모두 안전 사건이 아니라 목표 조작이다. E-stop 과 달리 래치도 reset 도
    # 없고, 처리 뒤 바로 새 요청을 받을 수 있다. 판정은 여기(Mission Manager)가 하고
    # 앱·LLM·CLI 는 요청만 보낸다.

    def on_cancel_request(self, now: float) -> tuple:
        """목적지 취소. (actions, GateReason) 을 돌려준다."""
        reason = check_cancel_gate(self.state, self.estop_active)
        if reason != GateReason.OK:
            return [], reason

        actions: list = [SetNavSpeedLimit(NO_SPEED_LIMIT)]
        if self.state == State.NAVIGATING:
            actions.append(CancelNav(self.active_destination))
        self._to_idle()
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
        # 재개하면 새 goal 이므로 거리 안내도 처음부터 다시 한다.
        self._announced_milestones = set()
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
        assert destination is not None  # check_resume_gate 가 보장
        self.state = State.NAVIGATING
        self.active_destination = destination
        self.paused_destination = None
        self._announced_milestones = set()
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
        """음성 취소 요청. 바로 취소하지 않고 사용자에게 되묻는다.

        확인을 기다리는 동안에도 주행은 계속된다. 확인 전에 goal 을 멈추면
        사용자가 "아니오"라고 답했을 때 되돌릴 수 없기 때문이다.
        """
        reason = check_cancel_gate(self.state, self.estop_active)
        if reason != GateReason.OK:
            return [], reason
        self.cancel_confirm_pending = True
        self._cancel_confirm_deadline = now + self.confirm_timeout_sec
        return [Say(MSG_CANCEL_CONFIRM, priority="response")], GateReason.OK

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

    def on_emergency(self, keyword: str, now: float) -> list:
        """/vica/emergency (긴급어). 하드 키워드만 처리 — LLM 을 거치지 않은 경로.

        모터 정지의 권위는 /emergency_stop 래치 체인(진행순서 ③에서 배선)이고,
        여기서의 goal 취소는 심층 방어 보조 경로다.
        """
        if keyword not in HARD_EMERGENCY_KEYWORDS:
            return []

        actions: list = []
        if self.state == State.NAVIGATING:
            actions.append(SetNavSpeedLimit(NO_SPEED_LIMIT))
            actions.append(CancelNav(self.active_destination))
        already_estopped = self.state == State.ESTOPPED
        self._enter_estopped(now)
        if not already_estopped:
            actions.append(Say(MSG_ESTOPPED, priority="emergency"))
        return actions

    def on_estop(self, active: bool, now: float) -> list:
        """/emergency_stop 래치 상태 (emergency_stop_node 가 20Hz 주기 발행)."""
        self.estop_active = active
        if active:
            self._estop_clear_since = None
            if self.state != State.ESTOPPED:
                actions: list = []
                if self.state == State.NAVIGATING:
                    actions.append(SetNavSpeedLimit(NO_SPEED_LIMIT))
                    actions.append(CancelNav(self.active_destination))
                self._enter_estopped(now)
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

        if self.state == State.CONFIRMING:
            if self._confirm_deadline is not None and now >= self._confirm_deadline:
                self._to_idle()
                actions.append(Say(MSG_CONFIRM_TIMEOUT))

        elif self.state == State.NAVIGATING:
            # 취소 재확인에 답이 없으면 주행을 그대로 이어간다(취소하지 않는다).
            if (
                self.cancel_confirm_pending
                and self._cancel_confirm_deadline is not None
                and now >= self._cancel_confirm_deadline
            ):
                self.cancel_confirm_pending = False
                self._cancel_confirm_deadline = None
                actions.append(Say(MSG_CANCEL_KEPT, priority="response"))

            if nav_status == NavStatus.SUCCEEDED:
                dest = self.active_destination
                text = (
                    dest.arrival_message
                    if dest and dest.arrival_message
                    else MSG_ARRIVED_FALLBACK.format(name=dest.name if dest else "목적지")
                )
                self.state = State.ARRIVED
                self._dwell_until = now + self.dwell_sec
                self._approach.reset()
                actions.append(SetNavSpeedLimit(NO_SPEED_LIMIT))
                actions.append(Say(text))
            elif nav_status == NavStatus.RUNNING:
                # 접근 감속: 목적지에 가까워질수록 최대속도 상한을 한 단계씩 내려
                # 도착 순간의 속도 낙차(Δv)를 줄인다. 근거와 단계 값의 뜻은
                # approach_speed.py 모듈 docstring 에 있다.
                #
                # 사다리는 한 방향으로만 내려가므로, 재계획으로 잔여거리가 다시
                # 늘어도 제한은 풀리지 않는다. 새 단계에 진입한 tick 에서만
                # 값이 나오고 그 외에는 None 이라 중복 발행도 없다.
                approach_percent = self._approach.update(distance_remaining)
                if approach_percent is not None:
                    actions.append(SetNavSpeedLimit(approach_percent))
                milestone = self._crossed_milestone(distance_remaining)
                if milestone is not None:
                    actions.append(
                        Say(MSG_DISTANCE_REMAINING.format(meters=int(milestone)))
                    )
            elif nav_status in (NavStatus.FAILED, NavStatus.CANCELED):
                # estop 경로의 취소는 이미 ESTOPPED 로 빠져나갔으므로,
                # 여기 도달한 취소/실패는 주행 실패로 취급한다.
                self.state = State.FAILED
                self._dwell_until = now + self.dwell_sec
                self._approach.reset()
                actions.append(SetNavSpeedLimit(NO_SPEED_LIMIT))
                # narration 은 큐 정원 초과 시 가장 먼저 버려진다(tts_queue._trim).
                # 주행 실패는 사용자가 왜 멈췄는지 알 유일한 단서라 버려지면 안 된다.
                actions.append(Say(MSG_NAV_FAILED, priority="response"))

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
                    # 위와 같은 이유. 해제를 못 들으면 사용자는 계속 멈춰 있는 줄 안다.
                    actions.append(Say(MSG_ESTOP_RELEASED, priority="response"))

        return actions

    # -- 내부 ------------------------------------------------------------------

    def _crossed_milestone(self, distance_remaining: Optional[float]) -> Optional[float]:
        """이번 tick 에 새로 지난 안내 지점을 돌려준다 (없으면 None).

        Nav2 는 초기에 0.0 이나 None 을 주기도 해서 양수만 신뢰한다.
        여러 지점을 한꺼번에 지났으면(예: 2m 앞에서 출발) 실제 거리에 가장 가까운
        지점만 말하고 나머지는 지난 것으로 처리한다 — 2m 남았는데 "10미터 남았다"고
        하면 안 되기 때문이다.
        """
        if distance_remaining is None or distance_remaining <= 0.0:
            return None

        crossed = [
            m
            for m in DISTANCE_MILESTONES_M
            if distance_remaining <= m and m not in self._announced_milestones
        ]
        if not crossed:
            return None

        self._announced_milestones.update(crossed)
        return min(crossed)

    def _enter_estopped(self, now: float) -> None:
        self.state = State.ESTOPPED
        self.active_destination = None
        # 보관한 목적지도 함께 버린다. 남겨두면 E-stop 뒤에 "다시 출발"이 통해
        # 이전 Goal 자동 재개 금지 원칙이 깨진다.
        self.paused_destination = None
        self.cancel_confirm_pending = False
        self._cancel_confirm_deadline = None
        self._confirming_dest_id = None
        self._confirm_deadline = None
        self._estop_entered_at = now
        self._estop_clear_since = None
        self._announced_milestones = set()
        self._approach.reset()

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
        self._announced_milestones = set()
        self._approach.reset()
