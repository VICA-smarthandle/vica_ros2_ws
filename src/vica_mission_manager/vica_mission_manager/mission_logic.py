"""vica_mission_manager 게이트·상태 전이 순수 로직.

rclpy 에 의존하지 않는다 — 통합 계획(진행순서 ②)의 요구사항으로,
이 모듈의 모든 판단은 unit test 로 검증한다. ROS 배선은
mission_manager_node.py 가 담당하고, 여기서는 "무엇을 할지"만 결정해
Action 목록으로 돌려준다.

안전 원칙(불변): 이 로직이 허용해야만 Nav2 goal 이 나간다.
LLM(VicaIntent)은 어디까지나 제안이다.
"""
from __future__ import annotations

import math

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
    # ---- 사람 접근 (devlog/2026-08-23-사람접근-구현설계.md 4절) ----------------
    #
    # 아래 셋은 모두 "안내를 받는 사용자가 아직 없는" 구간이다. 시각장애인을
    # 탐지해 1.1 m 앞까지 다가가(APPROACHING) 안내가 필요한지 묻고
    # (AWAITING_USER), 끝나면 대기 위치로 돌아온다(RETURNING).
    APPROACHING = "approaching"
    # 질문을 던지고 답을 기다린다. 주도권은 음성 쪽에 있고 Mission 은 타임아웃만
    # 센다 — 여기서 말을 알아듣는 일은 이 모듈의 몫이 아니다.
    AWAITING_USER = "awaiting_user"
    # 수락 후 180도 제자리 회전 - 핸들(로봇 뒤)을 사람 쪽으로 낸다.
    # 정지 거리 1.1 m 가 이 회전의 반경 기준으로 설계돼 있다(설계 6.3절).
    TURNING = "turning"
    RETURNING = "returning"
    # ---- 목적지 도착 후 대화 (2026-08-30, arrival-dialog-flow) ------------------
    #
    # 도착하면 유형별로 묻고(ASKING_NEXT), 대기를 고르면 시간을 묻고
    # (ASKING_WAIT_TIME), 그 자리에 서서 기다린다(WAITING). 답이 없으면 결국
    # 홈으로 복귀한다. AWAITING_USER 와 같은 모양이다 — 질문을 던지고, 재생이
    # 끝난 시점부터 시한을 세고, 답이 오면 갈래를 만든다.
    ASKING_NEXT = "asking_next"
    # "몇 분쯤?" 답 대기 (restroom·entrance 가 아닌 곳에서 대기를 골랐을 때만).
    ASKING_WAIT_TIME = "asking_wait_time"
    # 제자리 대기. 사람접근 OFF(기다리라 해놓고 행인을 쫓지 않게). "비카야"로
    # 깨어나거나 시간이 초과되면 나간다.
    WAITING = "waiting"


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
    # 사람 접근 요청 전용 사유.
    NO_TRACK_ID = "no_track_id"
    TRACK_SUPPRESSED = "track_suppressed"
    BUSY_APPROACHING = "busy_approaching"
    NOT_APPROACHING = "not_approaching"


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
    # destinations.yaml 의 category2. 도착 후 질문을 유형별로 고른다
    # (restroom=대기 제안, entrance=종료 제안, 그 외=대기 여부). 없으면 "".
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
    # 도착 후 대화의 wait 요청 시간(분). 없거나 무관하면 -1 (2026-08-30).
    wait_minutes: int = -1


@dataclass(frozen=True)
class ApproachRequest:
    """RequestApproach.srv 한 건 중 게이트 판단에 쓰는 값만.

    `goal` 은 **이미 계산이 끝난 접근 goal** 이다 — 사람 위치가 아니다. 사람
    위치에서 로봇 쪽으로 1.1 m 물러난 지점을 구하는 계산은
    `approach_geometry.approach_goal(person, robot)` 이 하고, 노드가 그 결과를
    여기에 넣는다. 이 모듈이 rclpy 를 물지 않는 것과 같은 이유로 기하 계산도
    물지 않는다 — 판단과 계산을 따로 두어야 각각을 따로 시험할 수 있다.

    사람과 로봇이 겹쳐 방향을 정할 수 없으면 그 계산이 None 을 돌려주므로
    `goal` 도 Optional 이며, None 이면 이 요청은 거부된다.
    """

    goal: Optional[Pose2D]
    track_id: int
    # 시계열 판정(신뢰도 1초 연속 + 3초 정지)은 person_detector_node 몫이다.
    # Mission 은 탐지 이력을 쌓지 않지만, 실려 온 판정 결과가 false 면 그대로
    # 거부한다 — 요청자를 믿기만 하지는 않는다는 뜻이다.
    approachable: bool = True


# ---- Actions: 노드가 실행할 일 ---------------------------------------------


@dataclass(frozen=True)
class Say:
    text: str
    # ros_tts_node 큐 우선순위 (긴급 > 내레이션 > 응답).
    # 노드가 "{priority}:{text}" 접두어로 /vica/tts_request 에 발행한다.
    priority: str = "narration"  # emergency / narration / response
    # 이 말이 '질문'이라 사용자 답을 기다리는가. true 면 노드가
    # /vica/listen_request 를 함께 발행하고, 웨이크워드 노드가 질문 TTS 종료
    # 직후 재청취 창을 연다 — "비카야" 재호출 없이 "네/아니요"로 답하게 한다.
    expects_reply: bool = False


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
class SpinInPlace:
    """제자리 회전. 노드는 BasicNavigator.spin() 으로 실행한다.

    Navigate 가 아니다 - goal 을 만들지 않고 behavior server 의 Spin 을 탄다.
    Spin 은 회전 중 costmap 충돌을 스스로 검사한다. 양수 = 반시계.
    """

    yaw_rad: float


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
# 자동 재시도 중임을 알린다. 로봇이 멈춰 있는 이유를 이용자가 알 수 있어야
# 하고, 곧 다시 움직인다는 것도 알아야 놀라지 않는다.
MSG_NAV_RETRY = "길이 막혀 잠시 기다렸다가 다시 가겠습니다."
# E-stop 안내 3종 (2026-08-29 사용자 확정 문구). 통신 순단(정지 중)은 자동
# 복구되므로(AutoRecoveryPolicy) "관리자를 호출했습니다"가 거짓이 된다 —
# 그 경우만 자동 복구 예고로 갈린다. 원인 판별 정본은
# vica_safety/auto_recovery.py 의 COMM_SOURCES (갈라지면 판별이 어긋난다).
MSG_ESTOPPED = "안전을 위해 멈추겠습니다. 관리자를 호출했습니다."
MSG_ESTOP_COMM = "연결 문제로 잠시 섰습니다. 곧 자동으로 복구됩니다."
MSG_ESTOP_WAIT_ADMIN = (
    "안전 확인이 필요해서 관리자를 기다리고 있습니다. 잠시만 기다려 주세요."
)
MSG_ESTOP_RELEASED = "비상 멈춤이 해제되었습니다. 새로운 목적지를 말씀해 주세요."

_COMM_ESTOP_SOURCES = frozenset({
    "motor_can",
    "physical_stale",
    "motor_can_stale",
    "physical_waiting",
    "motor_can_waiting",
})
MSG_DISTANCE_REMAINING = "목적지까지 약 {meters}미터 남았습니다."
MSG_CANCELED = "안내를 취소했습니다."

# ---- 도착 후 대화 (2026-08-30, arrival-dialog-flow) --------------------------
# 유형별 첫 질문. category(destinations.yaml category2)로 고른다. "네"의 뜻이
# 유형마다 뒤집히므로 질문을 던질 때 종료형인지(_asking_is_finish)를 함께
# 기억한다. 문구 정본은 이 상수들 — 캐시가 구운 판을 재생한다(글자 일치).
MSG_ASK_RESTROOM = "다녀오시는 동안 여기서 기다릴까요?"          # 대기형
MSG_ASK_ENTRANCE = "여기까지 안내를 마칠까요?"                  # 종료형
MSG_ASK_GENERIC = "이제 괜찮으신가요? 아니면 여기서 대기할까요?"  # 종료형
MSG_ASK_WAIT_TIME = "몇 분쯤 걸리실까요?"
# 대기 확정. {minutes} 는 코드가 채운다 — 캐시엔 넣지 않는다(가변).
MSG_WAIT_CONFIRM = "{minutes}분 대기하겠습니다. 돌아오시면 '비카야'라고 말씀해 주세요."
MSG_WAIT_DEFAULT = "네, 최대 30분까지 여기서 기다리겠습니다. 돌아오시면 '비카야'라고 말씀해 주세요."
MSG_WAIT_RESUME_ASK = "다시 안내를 시작할까요?"
MSG_FINISH = "안내를 종료합니다."
# 무응답 사다리 (3절): 못 알아들으면 재질문 1회, 그 뒤/침묵이면 떠나기 예고.
MSG_ARRIVAL_RETRY = "잘 듣지 못했습니다. 계속 안내가 필요하시면 말씀해 주세요."
MSG_LEAVING_NOTICE = "응답이 없어 안내를 마치고 제자리로 돌아가겠습니다."
MSG_GOING_HOME = "홈으로 복귀중입니다."

WAIT_MINUTES_CAP = 30
LEAVING_GRACE_SEC = 3.0        # 떠나기 예고 후 마지막 끼어들기 유예
MSG_PAUSED = "잠시 멈추겠습니다. 다시 출발하려면 말씀해 주세요."
MSG_RESUMED = "{name}{josa} 다시 출발합니다."
MSG_CANCEL_CONFIRM = "안내를 취소할까요?"
MSG_CANCEL_KEPT = "안내를 계속하겠습니다."
MSG_NOT_NAVIGATING = "지금은 안내 중이 아닙니다."
MSG_NOT_PAUSED = "다시 출발할 안내가 없습니다."
# 사람 접근. 질문은 되묻기와 같은 이유로 expects_reply 를 달아 내보낸다.
# ⚠️ 문구 정본은 voice replies.py·ment_cache (approach-voice-flow.md 확정 흐름).
# 글자까지 일치해야 사전 녹음이 재생된다 — 바꾸려면 양쪽을 함께 고치고
# 재녹음한다(voice scripts/make_cue_wavs.py). 수락 멘트가 회전 예고인 이유:
# 시각장애인에게 예고 없는 움직임 금지(2026-08-25 결정).
MSG_APPROACH_QUESTION = (
    "안녕하세요? 저는 시각장애인 안내로봇 비카입니다! "
    "저와 함께 목적지까지 동행해보시는건 어떠세요? 안내를 받으시겠어요?"
)
MSG_APPROACH_ACCEPTED = "네, 잠시만 기다려주세요. 로봇이 회전하니 주의하세요."
MSG_APPROACH_DECLINED = "알겠습니다. 이만 물러납니다."
MSG_APPROACH_TURN_DONE = "회전이 완료되었습니다."
MSG_APPROACH_ONBOARDING = (
    "안녕하세요? 반갑습니다! 저에게 말을 거실 때는 '비카야'라고 불러주세요. "
    "자, 이제 어디로 가고 싶으신가요?"
)
MSG_APPROACH_NO_ANSWER = "실례했습니다. 필요하시면 언제든 불러 주세요."
MSG_APPROACH_BUSY = "지금은 다른 응대 중입니다. 잠시 후 다시 말씀해 주세요."

# 남은 거리를 알리는 지점(미터). 눈으로 확인할 수 없는 사용자가 도착을 미리
# 준비할 수 있게 하려는 것이므로, 자주 말하기보다 접근 시점만 짚는다.
# 각 지점은 목적지 하나당 한 번만 안내한다.
#
# 빈 튜플 = 거리 안내 전면 제거 (2026-08-26 사용자 결정). 실사용에서 거리
# 멘트가 회전·도착 멘트와 겹쳐 큐를 밀리게 했고, 정보 가치보다 소음이 컸다.
# (연혁: 10 m 는 8/20 오보 문제로, 3 m 는 8/26 사용성 문제로 제거)
# 판정 배선(_crossed_milestone)은 남겨 둔다 — 되살릴 땐 지점만 넣으면 된다.
DISTANCE_MILESTONES_M = ()

# ---- 사람 접근 값 (설계 6.2절) -----------------------------------------------

# 질문 뒤 답을 기다리는 시간. STT 검증 1.84초를 포함한 값이며, 재생이 끝난
# 시점부터 센다(on_approach_question_spoken).
APPROACH_RESPONSE_TIMEOUT_SEC = 8.0
# 수락 후 회전이 이 시간 안에 끝나지 않으면 포기하고 IDLE 로 내린다.
# 180도 / 회전 상한 0.4 rad/s = 7.9 s 에 수락·기동 지연 여유를 더한 값.
APPROACH_TURN_TIMEOUT_SEC = 15.0
# 접근을 마친 뒤 같은 track_id 에 다시 다가가지 않는 시간. 거절한 사람을 로봇이
# 계속 쫓아다니는 것이 이 기능의 가장 나쁜 실패 방식이라 값을 넉넉히 둔다.
REAPPROACH_SUPPRESS_SEC = 60.0
# 사람에게 다가가는 구간의 최대속도 상한. 주행 상한 0.5 m/s 의 60 % = 0.3 m/s 다.
# 마지막 1.1 m 는 collision_monitor 의 PolygonSlow 가 0.12 m/s 로 한 번 더
# 줄인다(설계 6.4절).
#
# [이름 주의] 이 파일에서 "접근"은 두 가지를 뜻한다. approach_speed.py 의
# ApproachSpeedLadder 와 MissionLogic.approach_speed_limit_percent 는 **등록
# 목적지에 가까워질 때의 감속**이고, 여기 PERSON_* 은 **사람에게 다가가는 구간의
# 고정 상한**이다. 사람 접근에는 사다리를 쓰지 않는다 — 처음부터 끝까지 느리다.
PERSON_APPROACH_SPEED_PERCENT = 60.0
# goal 재전송 임계(m). NavigateToPose 는 preempt 되지만 그때마다 BT 가 처음부터
# 다시 시작한다. 사람이 조금 흔들릴 때마다 goal 을 바꾸면 재계획만 반복하고 한
# 발도 못 뗀다(설계 5절).
APPROACH_GOAL_UPDATE_M = 0.5
# PersonDetection.TRACK_ID_NONE. 추적 id 가 없으면 재접근 억제를 걸 수 없어
# 같은 사람에게 무한히 다가갈 수 있으므로 요청 자체를 받지 않는다.
TRACK_ID_NONE = 0
# 접근 goal 을 감싼 Destination 의 이름·id 접두어. 로그에서 등록 목적지와
# 구분하고 어느 사람을 향한 goal 이었는지 남기려는 것이다.
APPROACH_DESTINATION_PREFIX = "approach:"
APPROACH_DESTINATION_NAME = "접근 대상"

# 접근 세 상태를 한 묶음으로 본다 — 새 목적지 요청을 거부하는 구간이 이 셋이다.
_APPROACH_STATES = (
    State.APPROACHING, State.AWAITING_USER, State.TURNING, State.RETURNING
)
# Nav2 goal 이 살아 있는 상태. E-stop·긴급어가 goal 을 취소해야 하는 구간이다.
_GOAL_ACTIVE_STATES = (
    State.NAVIGATING, State.APPROACHING, State.TURNING, State.RETURNING
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


def approach_destination(request: ApproachRequest) -> Destination:
    """접근 goal 을 Navigate 가 받는 Destination 모양으로 감싼다.

    Nav2 에게 이 goal 은 등록된 목적지로 갈 때와 완전히 같은 NavigateToPose 다 —
    **Nav2 는 그것이 사람인지 모른다**(설계 5절). 그래서 접근 전용 Action 을 새로
    만들지 않고 기존 Navigate 를 그대로 쓴다.

    `calibrated=True` 는 "실측으로 등록한 좌표"라는 뜻이 아니라 `pose_valid` 의
    미캘리브레이션 검사 대상이 아니라는 뜻이다. 이 좌표는 destinations.yaml 이
    아니라 방금 센서에서 계산된 값이라 캘리브레이션 개념 자체가 없다. (0,0)
    검사와 지도 경계·frame 검사는 그대로 받는다.
    """
    assert request.goal is not None  # 호출부가 None 을 먼저 걸러야 한다
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
    """사람 접근 요청 게이트. 첫 번째 실패 사유를 돌려준다.

    check_gate 와 같은 순서로 읽는다 — 요청 자체의 흠 → 안전 → 문맥 → 좌표 →
    Nav2 준비. 다른 점 하나는 이 게이트의 거절이 **말이 되어 나가지 않는다**는
    것이다. 요청자는 사람이 아니라 person_detector_node 이고 사유는 서비스
    응답으로만 돌아간다. 아직 아무 관계도 없는 사람 앞에서 로봇이 거절 사유를
    혼잣말하면 그것이 더 이상한 동작이다.

    탐지 결과는 요청이지 goal 이 아니다. 승인하는 곳은 여기 하나뿐이다.
    """
    if not request.approachable:
        return GateReason.NOT_APPROACHABLE
    if request.track_id == TRACK_ID_NONE:
        return GateReason.NO_TRACK_ID
    if estop_active or state == State.ESTOPPED:
        return GateReason.ESTOP_ACTIVE
    if state == State.APPROACHING:
        # 같은 사람이면 goal 갱신이고 다른 사람이면 거절이다. 접근 중에 대상을
        # 갈아타면 두 사람 모두에게 이상한 동작이 된다.
        if request.track_id != active_track_id:
            return GateReason.BUSY_APPROACHING
    elif state in (State.AWAITING_USER, State.RETURNING):
        return GateReason.BUSY_APPROACHING
    elif state != State.IDLE:
        # 안내를 받고 있는 사용자가 우선이다. 접근은 IDLE 에서만 시작한다.
        return GateReason.BUSY_NAVIGATING
    if suppressed:
        return GateReason.TRACK_SUPPRESSED
    if request.goal is None:
        # approach_geometry 가 방향을 정하지 못한 경우(사람과 로봇이 겹침).
        return GateReason.POSE_INVALID
    if not pose_valid(approach_destination(request), bounds):
        return GateReason.POSE_INVALID
    if not nav_ready:
        return GateReason.NAV_NOT_READY
    return GateReason.OK


def check_approach_cancel_gate(state: State, estop_active: bool) -> GateReason:
    """접근 취소 게이트(/vica/mission/cancel_approach).

    이탈·포기 판정(최초 탐지 위치에서 2.0 m·3초)은 시계열을 보는 판정이라
    person_detector_node 가 하고, 여기서는 그 통보를 받아 상태만 옮긴다.

    복귀 중(RETURNING)에는 취소할 접근이 이미 없으므로 거부한다. E-stop 중
    거부는 check_cancel_gate 와 같은 이유다 — E-stop 이 상위 상태다.
    """
    if estop_active or state == State.ESTOPPED:
        return GateReason.ESTOP_ACTIVE
    if state not in (State.APPROACHING, State.AWAITING_USER):
        return GateReason.NOT_APPROACHING
    return GateReason.OK


def yaw_deg_to_quaternion(yaw_deg: float) -> tuple:
    """도(deg) yaw → 쿼터니언 (x, y, z, w). 변환은 goal 생성 시에만 (함정 2번)."""
    import math

    half = math.radians(yaw_deg) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


# ---- 상태 머신 ---------------------------------------------------------------


class MissionLogic:
    """상태 머신. 안내 7종 + 사람 접근 3종.

        안내:      idle / confirming / navigating / arrived / failed /
                   estopped / paused
        사람 접근: approaching / awaiting_user / returning

    시간은 전부 인자(now: float 초)로 받는다 — 테스트에서 시계를 주입하기 위함.
    """

    def __init__(
        self,
        confirm_timeout_sec: float = 30.0,
        dwell_sec: float = 2.0,
        estop_release_grace_sec: float = 2.0,
        approach_stages: Optional[Sequence] = None,
        nav_retry_limit: int = 2,
        nav_retry_delay_sec: float = 3.0,
        approach_response_timeout_sec: float = APPROACH_RESPONSE_TIMEOUT_SEC,
        reapproach_suppress_sec: float = REAPPROACH_SUPPRESS_SEC,
        person_approach_speed_percent: float = PERSON_APPROACH_SPEED_PERCENT,
        approach_goal_update_m: float = APPROACH_GOAL_UPDATE_M,
        return_destination: Optional[Destination] = None,
        approach_turn_yaw_rad: float = math.pi,
        arrival_dialog: bool = False,
    ) -> None:
        self.confirm_timeout_sec = confirm_timeout_sec
        self.dwell_sec = dwell_sec
        self.estop_release_grace_sec = estop_release_grace_sec
        # 주행 실패 뒤 같은 목적지로 스스로 다시 시도하는 횟수와 간격.
        #
        # 왜 필요한가 - 2026-08-15 실기에서 정체의 절반이 "사람이 앱을 다시
        # 누르기까지 걸린 시간" 이었다. run9 #6 구간 46초의 내역:
        #   198.3~222.4 s (24 s)  Nav2 가 복구를 시도하다 Goal failed
        #   222.4~242.6 s (20 s)  아무도 아무것도 안 함. 로봇은 실패 상태로 대기
        #   242.6 s               사람이 앱에서 목적지를 다시 보냄
        # 뒤의 20초는 Nav2 도 nvblox 도 아니다. 재시도가 없어서 생긴 공백이다.
        #
        # 무한 재시도는 하지 않는다. 통과 불가능한 자리(통로 1.0 m 에 사람이 서면
        # 남는 0.55 m 로는 어떤 설정으로도 못 지나간다)에서 영원히 시도하면
        # 이용자가 상황을 알 수 없다. 한도를 넘으면 안내하고 멈춘다.
        self.nav_retry_limit = nav_retry_limit
        self.nav_retry_delay_sec = nav_retry_delay_sec
        # 접근 감속 사다리. 단계 검증(거리 양수·비율 범위·단조 감소)은
        # ApproachSpeedLadder 가 하고 잘못된 값이면 여기서 ValueError 로 죽는다.
        self._approach = ApproachSpeedLadder(approach_stages)

        # 사람 접근 값. 근거는 위 상수 정의에 있다.
        self.approach_response_timeout_sec = approach_response_timeout_sec
        self.reapproach_suppress_sec = reapproach_suppress_sec
        self.person_approach_speed_percent = person_approach_speed_percent
        self.approach_goal_update_m = approach_goal_update_m
        # 접근을 마치고 돌아갈 대기 위치. 지도상 좌표는 아직 [미정] 이라(설계
        # 12절) None 을 허용하며, 없으면 제자리에서 접근만 끝낸다.
        self.return_destination = return_destination
        # 수락 후 제자리 회전량. 0.0 이면 회전 없이 예전처럼 바로 끝낸다.
        self.approach_turn_yaw_rad = approach_turn_yaw_rad
        self._turn_deadline: Optional[float] = None
        # E-stop 안내용: 마지막으로 본 원인 목록과 "관리자 개입이 필요한
        # 래치인가" 판정. 판정은 걸리는 순간 확정한다 — 원인은 곧 해제돼
        # 사라지므로 나중엔 알 수 없다.
        self._estop_sources: frozenset = frozenset()
        self._estop_admin_needed = True
        self._estop_wait_announced = False

        # 도착 후 대화 (arrival-dialog-flow). 꺼져 있으면 도착 후 기존 dwell→
        # idle 로 간다 — 실기 검증 전까지 기본 off, 노드가 파라미터로 켠다.
        self.arrival_dialog = arrival_dialog
        self._asking_is_finish = False   # 방금 던진 질문이 종료형인가("네"=끝)
        self._arrival_retried = False    # 무응답 재질문을 이미 한 번 했나
        self._leaving_deadline: Optional[float] = None   # 떠나기 예고 유예
        self._wait_until: Optional[float] = None         # WAITING 만료 시각

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
        # 재시도 상태. 목적지가 바뀌거나 IDLE 로 돌아가면 초기화한다.
        self._nav_retry_count: int = 0
        self._retry_destination: Optional[Destination] = None
        self._retry_at: Optional[float] = None
        self._announced_milestones: set = set()  # 이번 목적지에서 안내한 거리 지점
        self._distance_baseline: Optional[float] = None  # 이번 목적지의 출발 거리
        # 접근 대상. 안내 사용자가 없는 구간이라 "어디로" 뿐 아니라 "누구에게"
        # 가는 중인지를 따로 들고 있어야 재접근 억제를 걸 수 있다.
        self.approach_track_id: Optional[int] = None
        self.approach_goal_pose: Optional[Pose2D] = None
        self._response_deadline: Optional[float] = None
        # track_id -> 재접근을 다시 허용할 시각. 사람마다 따로 센다.
        self._suppressed_tracks: dict = {}

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

        if self.state in _APPROACH_STATES:
            # 접근·질문·복귀 중에는 새 목적지를 받지 않는다. 특히
            # AWAITING_USER 는 방금 던진 질문의 답을 기다리는 구간이라, 그 자리에
            # 다른 목적지를 끼워 넣으면 누구의 요청인지 알 수 없게 된다(설계 4절).
            return [Say(MSG_APPROACH_BUSY, priority="response")]

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
            return [Say(MSG_STALE_CONFIRM, priority="response", expects_reply=True)]

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
        self._distance_baseline = None
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
        assert destination is not None  # check_resume_gate 가 보장
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
        """음성 취소 요청. 바로 취소하지 않고 사용자에게 되묻는다.

        확인을 기다리는 동안에도 주행은 계속된다. 확인 전에 goal 을 멈추면
        사용자가 "아니오"라고 답했을 때 되돌릴 수 없기 때문이다.
        """
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

    # -- 사람 접근 --------------------------------------------------------------
    #
    # 탐지 → 접근 → 질문 → 응답분기까지가 이번 범위다. 회전·핸들 접촉·목적지
    # 안내는 다음 사이클이며, "네"를 받으면 여기서 손을 뗀다.

    def on_approach_request(
        self,
        request: ApproachRequest,
        bounds: Optional[MapBounds],
        nav_ready: bool,
        now: float,
    ) -> tuple:
        """사람 접근 요청. (actions, GateReason) 을 돌려준다.

        같은 track_id 로 접근 중에 다시 오면 goal 갱신으로 받는다 — 사람이
        움직이면 goal 도 따라가야 하기 때문이다. 다만 갱신 임계(0.5 m)보다 덜
        움직였으면 아무것도 하지 않고 승인만 돌려준다.
        """
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
                # 재계획 폭주 억제. 승인은 하되 goal 은 그대로 둔다.
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
                # 접근 구간 전체를 0.3 m/s 로 묶는다. 목적지 접근 감속 사다리와
                # 달리 거리에 따라 내려가지 않는다 — 사람에게 다가가는 동안은
                # 처음부터 끝까지 느려야 한다.
                SetNavSpeedLimit(self.person_approach_speed_percent),
                Navigate(destination),
            ],
            GateReason.OK,
        )

    def on_approach_cancel_request(self, now: float) -> tuple:
        """접근 포기·대상 이탈 통보(/vica/mission/cancel_approach).

        안내 주행용 on_cancel_request 와 경로를 나눈 이유는 대상이 다르기
        때문이다. 접근 취소가 안내를 끊어서는 안 되고, 안내 취소가 접근을
        끊어서도 안 된다.
        """
        reason = check_approach_cancel_gate(self.state, self.estop_active)
        if reason != GateReason.OK:
            return [], reason

        actions: list = []
        if self.state == State.APPROACHING:
            actions.append(CancelNav(self.active_destination))
        actions.extend(self._enter_returning(now))
        return actions, GateReason.OK

    def on_approach_question_spoken(self, now: float) -> list:
        """질문 재생이 끝났다. 응답 대기 8초는 여기서부터 센다(설계 6.2절).

        재생에 몇 초가 걸리는지는 TTS 만 알 수 있어 노드가 알려준다. 알려주지
        않아도 동작은 한다 — 그때는 질문을 만든 시각부터 세므로, 어긋나더라도
        사람에게 불리한 쪽(더 짧게 기다리는 쪽)으로만 어긋난다.
        """
        if self.state != State.AWAITING_USER:
            return []
        self._response_deadline = now + self.approach_response_timeout_sec
        return []

    def on_approach_answer(self, affirmative: bool, now: float) -> list:
        """접근 질문에 대한 사람의 답. 여기서는 갈래만 만든다.

        말을 알아듣는 일은 STT·LLM 몫이고, 이 모듈은 "네/아니오"로 정리된
        결과만 받는다. 판정 권한은 그대로 Mission 에 있다 — LLM 이 goal 을
        만들지 않는다.
        """
        if self.state != State.AWAITING_USER:
            return []

        if affirmative:
            # 수락 -> 180도 돌아 핸들(로봇 뒤)을 사람 쪽으로 낸다(2026-08-24 확장).
            # 핸들 접촉·목적지 안내는 여전히 다음 사이클이다. 방금 수락한 사람에게
            # 로봇이 곧바로 다시 다가가면 안 되므로 재접근 억제는 회전 전에 건다.
            track_id = self.approach_track_id
            self._suppress_track(track_id, now)
            if self.approach_turn_yaw_rad == 0.0:
                # 회전이 없으면 회전 예고는 거짓말 — 온보딩으로 바로 간다.
                # 온보딩 끝은 질문이라 expects_reply 로 재청취 창이 열린다.
                self._to_idle()
                return [Say(MSG_APPROACH_ONBOARDING, priority="response",
                            expects_reply=True)]
            self.state = State.TURNING
            self.active_destination = None
            self._response_deadline = None
            self._turn_deadline = now + APPROACH_TURN_TIMEOUT_SEC
            return [
                Say(MSG_APPROACH_ACCEPTED, priority="response"),
                SpinInPlace(self.approach_turn_yaw_rad),
            ]

        actions: list = [Say(MSG_APPROACH_DECLINED, priority="response")]
        actions.extend(self._enter_returning(now))
        return actions

    def on_estop_sources(self, sources: list, now: float) -> list:
        """/estop_sources (원인 목록, 쉼표 분리 전 상태). reset 대기를 알린다.

        원인이 전부 해제됐는데 래치가 남아 있으면 관리자 reset 만 남은
        구간이다 — 시각장애인에게는 "말없이 안 움직이는 시간"이라 1회
        알린다. 자동 복구가 올 상황(통신 원인·정지 중)이면 침묵한다.
        """
        new = frozenset(s for s in sources if s)
        actions: list = []
        if (self.estop_active
                and self._estop_sources and not new
                and self._estop_admin_needed
                and not self._estop_wait_announced):
            self._estop_wait_announced = True
            actions.append(Say(MSG_ESTOP_WAIT_ADMIN, priority="response"))
        if new:
            # 원인이 다시 켜졌다 — 다음에 비면 다시 1회 알릴 수 있게 한다.
            self._estop_wait_announced = False
        self._estop_sources = new
        return actions

    # -- 도착 후 대화 (arrival-dialog-flow) ------------------------------------
    def is_awaiting_arrival_answer(self) -> bool:
        """도착 후 질문의 답을 기다리는 중인가. 노드가 라우팅에 쓴다 —
        이때 들어온 wait/finish/cancel/affirm/deny/navigate 는 전부
        on_arrival_answer 로 보낸다(그 외 상태의 뜻과 다르므로)."""
        return self.state in (State.ASKING_NEXT, State.ASKING_WAIT_TIME)

    def is_waiting_in_place(self) -> bool:
        """WAITING(제자리 대기) 중인가. 이때 "비카야"는 on_wake 로 간다."""
        return self.state == State.WAITING

    def exit_arrival_dialog(self) -> None:
        """도착 후 대화를 조용히 닫는다 — 새 목적지 '제안'(need_confirm=True)이
        왔을 때 노드가 부른다. navigate 는 2단계(제안→확정)라 제안에서 바로
        출발하면 확인 질문 전에 달린다(2026-08-30 실기). IDLE 로 내려가
        on_intent 게이트가 평소처럼 CONFIRMING 부터 밟게 한다."""
        if self.state in (State.ASKING_NEXT, State.ASKING_WAIT_TIME):
            self._reset_arrival_dialog()
            self.state = State.IDLE

    def _ask_arrival(self, dest: Optional[Destination], now: float,
                     arrival_text: str = "") -> list:
        """유형별 질문을 던지고 ASKING_NEXT 로 들어간다. "네"의 뜻(_asking_is_finish)
        을 함께 기억한다 — restroom 은 대기형(네=대기), 나머지는 종료형(네=끝).

        arrival_text 가 있으면 도착 멘트와 질문을 한 발화로 합쳐 낸다(순서 역전
        방지). 재질문(on_wake·복귀 브레이크)에서는 도착 멘트 없이 질문만 낸다.
        """
        category = (dest.category if dest else "") or ""
        if category == "restroom":
            question, is_finish = MSG_ASK_RESTROOM, False
        elif category == "entrance":
            question, is_finish = MSG_ASK_ENTRANCE, True
        else:
            question, is_finish = MSG_ASK_GENERIC, True
        self.state = State.ASKING_NEXT
        self.active_destination = None
        self._asking_is_finish = is_finish
        self._arrival_retried = False
        self._leaving_deadline = None
        self._response_deadline = None   # 재생완료(on_arrival_question_spoken)에서 시작
        text = f"{arrival_text} {question}".strip() if arrival_text else question
        return [Say(text, priority="response", expects_reply=True)]

    def on_arrival_question_spoken(self, now: float) -> list:
        """도착 후 질문 재생이 끝났다. 응답 대기 8초는 여기서부터 센다
        (AWAITING_USER 와 같은 방식). 재생시간을 TTS 만 아므로 노드가 알려준다."""
        if self.state in (State.ASKING_NEXT, State.ASKING_WAIT_TIME):
            self._response_deadline = now + self.approach_response_timeout_sec
        return []

    def on_wake(self, now: float) -> list:
        """WAITING 중 "비카야" — 다시 안내를 시작할지 묻는다 (제자리 대기 각성)."""
        if self.state != State.WAITING:
            return []
        self._wait_until = None
        self._asking_is_finish = False   # "네" = 계속 (대기형처럼 다룬다)
        self.state = State.ASKING_NEXT
        self._arrival_retried = False
        self._response_deadline = None
        return [Say(MSG_WAIT_RESUME_ASK, priority="response", expects_reply=True)]

    def on_arrival_answer(self, intent: "IntentData", now: float,
                          next_dest: Optional[Destination] = None) -> list:
        """도착 후 질문에 대한 답. 정리된 intent 만 받아 갈래를 만든다.

        next_dest 는 navigate 답일 때 노드가 게이트 통과시킨 다음 목적지다.
        """
        if self.state not in (State.ASKING_NEXT, State.ASKING_WAIT_TIME):
            return []
        kind = intent.intent

        # 다음 목적지(확정만) — 제안(need_confirm=True)은 여기 오기 전에
        # 노드가 exit_arrival_dialog 로 일반 확인 흐름에 합류시킨다.
        if kind == "navigate" and next_dest is not None:
            self.state = State.NAVIGATING
            self.active_destination = next_dest
            self._reset_arrival_dialog()
            return [SetNavSpeedLimit(NO_SPEED_LIMIT), Navigate(next_dest)]

        # 종료: finish, 도착 후 cancel(=finish, 2026-08-30), 종료형 질문의 affirm.
        if (kind in ("finish", "cancel")
                or (kind == "affirm" and self._asking_is_finish)):
            self._reset_arrival_dialog()
            return [Say(MSG_FINISH, priority="response"), *self._go_home(now)]

        # 대기: wait, 또는 대기형(restroom) 질문의 affirm.
        if kind == "wait" or (kind == "affirm" and not self._asking_is_finish):
            minutes = intent.wait_minutes if kind == "wait" else -1
            # restroom·entrance(질문 태그로 왔거나 시간이 실림): 시간 안 묻고 대기.
            # 그 외에서 시간 없이 대기만 원하면 "몇 분쯤?" 후속 질문.
            if minutes is None or minutes < 0:
                if self.state == State.ASKING_NEXT and not self._asking_is_finish:
                    return self._enter_waiting(WAIT_MINUTES_CAP, now,
                                               default_msg=True)
                self.state = State.ASKING_WAIT_TIME
                self._response_deadline = None
                return [Say(MSG_ASK_WAIT_TIME, priority="response",
                            expects_reply=True)]
            return self._enter_waiting(min(minutes, WAIT_MINUTES_CAP), now)

        # deny: 종료형이면 "안 끝났다"=대기, 대기형이면 "대기 싫다"=종료.
        if kind == "deny":
            if self._asking_is_finish:
                return self._enter_waiting(WAIT_MINUTES_CAP, now,
                                           default_msg=True)
            self._reset_arrival_dialog()
            return [Say(MSG_FINISH, priority="response"), *self._go_home(now)]

        # 못 알아들음(unknown 등): 무응답 사다리 ①.
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
            return [Say(MSG_ARRIVAL_RETRY, priority="response", expects_reply=True)]
        return self._leaving_notice(now)

    def _leaving_notice(self, now: float) -> list:
        """떠나기 예고 + 유예. 유예 안에 답이 오면 산다(on_arrival_answer)."""
        self._leaving_deadline = now + LEAVING_GRACE_SEC
        self._response_deadline = None
        return [Say(MSG_LEAVING_NOTICE, priority="response")]

    def _go_home(self, now: float) -> list:
        """홈 복귀 진입. 홈이 없으면 _enter_returning 이 제자리로 처리한다."""
        return self._enter_returning(now)

    def on_return_brake(self, now: float) -> list:
        """홈 복귀 중 "비카야" (작업 E). 복귀를 취소하고 다시 안내를 묻는다 —
        부른 사람은 대개 로봇이 가버려 부른 사용자다. 긴급어("멈춰")는 별도
        경로로 어느 상태든 항상 통한다."""
        if self.state != State.RETURNING:
            return []
        cancel_dest = self.active_destination
        self.state = State.ASKING_NEXT
        self.active_destination = None
        self._asking_is_finish = False   # "네" = 계속
        self._arrival_retried = False
        self._response_deadline = None
        actions: list = [SetNavSpeedLimit(NO_SPEED_LIMIT)]
        if cancel_dest is not None:
            actions.append(CancelNav(cancel_dest))
        actions.append(Say(MSG_WAIT_RESUME_ASK, priority="response",
                           expects_reply=True))
        return actions

    def _reset_arrival_dialog(self) -> None:
        self._asking_is_finish = False
        self._arrival_retried = False
        self._leaving_deadline = None
        self._wait_until = None
        self._response_deadline = None

    def on_emergency(self, keyword: str, now: float) -> list:
        """/vica/emergency (긴급어). 하드 키워드만 처리 — LLM 을 거치지 않은 경로.

        모터 정지의 권위는 /emergency_stop 래치 체인(진행순서 ③에서 배선)이고,
        여기서의 goal 취소는 심층 방어 보조 경로다.
        """
        if keyword not in HARD_EMERGENCY_KEYWORDS:
            return []

        actions: list = []
        # 접근·복귀 중에도 goal 이 살아 있다. 로봇이 사람을 향해 움직이는 구간이
        # 있으므로 여기서 취소가 빠지면 긴급어 경로가 죽는다(설계 7절).
        if self.state in _GOAL_ACTIVE_STATES:
            actions.append(SetNavSpeedLimit(NO_SPEED_LIMIT))
            actions.append(CancelNav(self.active_destination))
        already_estopped = self.state == State.ESTOPPED
        self._enter_estopped(now)
        if not already_estopped:
            self._estop_admin_needed = True      # 긴급어 = 사람 개입 원인
            self._estop_wait_announced = False
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
                # 통신 원인뿐이고 정지 중이면 자동 복구가 온다(1.4초 실측) —
                # "관리자 호출"은 거짓이 된다. 주행 중 끊김은 관리자 몫.
                comm_only = (bool(self._estop_sources)
                             and self._estop_sources <= _COMM_ESTOP_SOURCES)
                auto_expected = comm_only and self.state != State.NAVIGATING
                self._estop_admin_needed = not auto_expected
                self._estop_wait_announced = False
                self._enter_estopped(now)
                actions.append(Say(
                    MSG_ESTOP_COMM if auto_expected else MSG_ESTOPPED,
                    priority="emergency"))
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
                self._approach.reset()
                actions.append(SetNavSpeedLimit(NO_SPEED_LIMIT))
                if self.arrival_dialog:
                    # 도착 멘트와 유형별 질문을 한 발화로 합쳐 낸다 — 따로 내면
                    # 우선순위(narration vs response)로 순서가 뒤집힌다(실기 확인
                    # 2026-08-30). 한 문장이라 재생 완료 시점이 명확해 8초 응답
                    # 창 시작도 어긋나지 않는다.
                    actions.extend(self._ask_arrival(dest, now, arrival_text=text))
                else:
                    self.state = State.ARRIVED
                    self._dwell_until = now + self.dwell_sec
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
                failed_dest = self.active_destination
                self.state = State.FAILED
                self._dwell_until = now + self.dwell_sec
                self._approach.reset()
                actions.append(SetNavSpeedLimit(NO_SPEED_LIMIT))

                # 사용자 취소(NavStatus.CANCELED)는 재시도하지 않는다. 목표를
                # 거둔 것이 사용자의 뜻이므로 로봇이 되살리면 안 된다.
                retryable = (
                    nav_status == NavStatus.FAILED
                    and failed_dest is not None
                    and self._nav_retry_count < self.nav_retry_limit
                )
                if retryable:
                    self._nav_retry_count += 1
                    self._retry_destination = failed_dest
                    self._retry_at = now + self.nav_retry_delay_sec
                    actions.append(Say(MSG_NAV_RETRY, priority="response"))
                else:
                    self._retry_destination = None
                    self._retry_at = None
                    # narration 은 큐 정원 초과 시 가장 먼저 버려진다
                    # (tts_queue._trim). 주행 실패는 사용자가 왜 멈췄는지 알
                    # 유일한 단서라 버려지면 안 된다.
                    actions.append(Say(MSG_NAV_FAILED, priority="response"))

        elif self.state == State.APPROACHING:
            if nav_status == NavStatus.SUCCEEDED:
                # 사람 앞 1.1 m 에 섰다. 여기서부터 주도권은 음성 쪽으로 넘어가고
                # Mission 은 타임아웃만 센다(설계 4절).
                self.state = State.AWAITING_USER
                self._response_deadline = now + self.approach_response_timeout_sec
                self._approach.reset()
                actions.append(SetNavSpeedLimit(NO_SPEED_LIMIT))
                actions.append(
                    Say(
                        MSG_APPROACH_QUESTION,
                        priority="response",
                        expects_reply=True,
                    )
                )
            elif nav_status in (NavStatus.FAILED, NavStatus.CANCELED):
                # 접근은 재시도하지 않는다. 등록 목적지는 제자리에 있지만 사람은
                # 3초 뒤 그 자리에 없다. 실패하면 돌아가서 다시 탐지하는 편이
                # 빠르고, 같은 자리로 되풀이 진입하면 통행에 방해가 된다.
                actions.extend(self._enter_returning(now))

        elif self.state == State.TURNING:
            if nav_status == NavStatus.SUCCEEDED:
                # 회전 완료 훅 (approach-voice-flow.md 확정 흐름): 완료를 알리고
                # 온보딩을 말한다. 온보딩 끝은 질문 -> expects_reply 로 재청취
                # 창이 열리고, 회전으로 사용자가 핸들 방향에 정렬됐으므로
                # DOA 방향 관문도 자연히 유효해진다.
                self._to_idle()
                actions.append(Say(MSG_APPROACH_TURN_DONE, priority="response"))
                actions.append(Say(MSG_APPROACH_ONBOARDING, priority="response",
                                   expects_reply=True))
            elif nav_status in (NavStatus.FAILED, NavStatus.CANCELED):
                # 회전 실패에 "완료되었습니다"는 거짓말 - 생략한다. 다만 방금
                # 수락한 사람을 침묵 속에 버려두지 않도록 온보딩은 한다.
                # (핸들 방향은 어긋났을 수 있다 - 안내 실패는 아니다.)
                self._to_idle()
                actions.append(Say(MSG_APPROACH_ONBOARDING, priority="response",
                                   expects_reply=True))
            elif (self._turn_deadline is not None
                  and now >= self._turn_deadline):
                # spin 이 시작조차 안 됐다(노드 결함 등). 시계로 탈출한다.
                self._to_idle()

        elif self.state == State.AWAITING_USER:
            # 여기서 하는 일은 시계를 보는 것뿐이다. 말을 알아듣는 쪽은 음성이다.
            if self._response_deadline is not None and now >= self._response_deadline:
                # 오탐이라 답할 이유가 없었을 수도, 답할 수 없는 상황일 수도 있다.
                # 어느 쪽이든 계속 서서 기다리면 사람 앞을 막는 셈이 된다.
                actions.append(Say(MSG_APPROACH_NO_ANSWER, priority="response"))
                actions.extend(self._enter_returning(now))

        elif self.state in (State.ASKING_NEXT, State.ASKING_WAIT_TIME):
            # 무응답 사다리 (arrival-dialog 3절). 떠나기 예고 후면 유예를 세고,
            # 아니면 8초 침묵을 센다. 완전 침묵은 재질문 없이 바로 예고로 간다
            # (자리를 뜬 신호). 말을 알아듣는 쪽은 음성이다.
            if self._leaving_deadline is not None and now >= self._leaving_deadline:
                self._reset_arrival_dialog()
                actions.append(Say(MSG_GOING_HOME, priority="response"))
                actions.extend(self._go_home(now))
            elif (self._leaving_deadline is None
                  and self._response_deadline is not None
                  and now >= self._response_deadline):
                actions.extend(self._leaving_notice(now))

        elif self.state == State.WAITING:
            # 제자리 대기. 사람접근은 이 상태값으로 자연히 꺼진다(_GOAL_ACTIVE
            # 아님·IDLE 아님). 시간이 다 되면 예고 없이 홈으로 — 이미 대기
            # 안내에서 "돌아오면 비카야"를 말했고, 30분을 채운 자리라 예고보다
            # 복귀가 자연스럽다.
            if self._wait_until is not None and now >= self._wait_until:
                self._reset_arrival_dialog()
                actions.append(Say(MSG_GOING_HOME, priority="response"))
                actions.extend(self._go_home(now))

        elif self.state == State.RETURNING:
            # 복귀 실패도 완료로 친다. 대기 위치에 못 갔다고 접근 상태에 갇히면
            # 다음 사람을 아예 못 본다 — 복귀는 안전 사건이 아니다.
            if self.return_destination is None or nav_status in (
                NavStatus.SUCCEEDED,
                NavStatus.FAILED,
                NavStatus.CANCELED,
            ):
                self._finish_returning(now)

        elif self.state in (State.ARRIVED, State.FAILED):
            # 재시도 예약이 있으면 그것이 dwell 보다 우선한다.
            #
            # [함정] dwell_sec(2.0)이 nav_retry_delay_sec(3.0)보다 짧다. 아래를
            # elif 로 두면 재시도 시각이 오기 전에 dwell 이 먼저 끝나 IDLE 로
            # 내려가고, _to_idle 이 예약을 지워 재시도가 영영 실행되지 않는다.
            # 그래서 '예약이 있으면 기다린다'를 먼저 판정한다.
            pending_retry = (
                self.state == State.FAILED
                and self._retry_at is not None
                and self._retry_destination is not None
            )
            if pending_retry and self.estop_active:
                # E-stop 중에는 되살리지 않는다. 예약을 버리고 평소 경로로 보낸다 —
                # 이전 goal 자동 재개 금지 원칙(ESTOPPED 분기 주석)과 같은 이유다.
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
                # 아직 시각이 안 됐으면 FAILED 로 머물며 기다린다.
            elif self._dwell_until is None or now >= self._dwell_until:
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

        if self._distance_baseline is None:
            # 첫 양수 거리 = 출발 거리. 출발점보다 먼 지점은 지난 것으로 접어
            # "4 m 남았는데 10미터"류 오보를 막는다. 출발이 지점 이하면 그
            # 지점은 침묵한다 — 곧 도착 멘트가 나올 참이라 겹치면 소음이다.
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
        """접근 goal 을 다시 보낼 만큼 사람이 움직였는가.

        NavigateToPose 는 preempt 되므로 취소·재전송이 필요 없지만, 새 goal 마다
        BT 가 처음부터 다시 시작한다. 임계를 두지 않으면 5 Hz 로 들어오는 탐지가
        그대로 재계획 요청이 되어 한 발도 못 뗀다(설계 5절).
        """
        import math

        if goal is None or self.approach_goal_pose is None:
            return True
        moved = math.hypot(
            goal.x - self.approach_goal_pose.x, goal.y - self.approach_goal_pose.y
        )
        return moved >= self.approach_goal_update_m

    def _enter_returning(self, now: float) -> list:
        """접근을 끝내고 대기 위치로 돌아간다.

        복귀는 접근이 아니므로 0.3 m/s 제한을 여기서 푼다. 대기 위치 좌표는 아직
        [미정] 이라(설계 12절) 없을 수 있고, 없으면 제자리에서 접근만 끝낸다 —
        상태만 지나가고 다음 tick 에 IDLE 로 내려간다.

        track_id 는 아직 지우지 않는다. 재접근 억제는 복귀가 끝난 시점부터
        세야 하므로 _finish_returning 까지 들고 간다(설계 4절).
        """
        self.state = State.RETURNING
        self.active_destination = self.return_destination
        self.approach_goal_pose = None
        self._response_deadline = None
        self._announced_milestones = set()
        self._distance_baseline = None
        self._approach.reset()
        actions: list = [SetNavSpeedLimit(NO_SPEED_LIMIT)]
        if self.return_destination is not None:
            actions.append(Navigate(self.return_destination))
        return actions

    def _finish_returning(self, now: float) -> None:
        """복귀 완료. 이 시점부터 같은 사람에 대한 재접근 억제를 센다."""
        track_id = self.approach_track_id
        self._to_idle()
        self._suppress_track(track_id, now)

    def _suppress_track(self, track_id: Optional[int], now: float) -> None:
        """이 사람에게 당분간 다시 다가가지 않는다.

        거절했거나 답하지 않은 사람을 로봇이 계속 쫓아다니는 것이 이 기능의 가장
        나쁜 실패 방식이다. 억제는 IDLE 로 내려가도 남아 있어야 하므로
        _to_idle 에서 지우지 않는다.
        """
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
        self._distance_baseline = None
        self._approach.reset()
        # 접근도 함께 버린다. 목적지를 보관하지 않으므로 해제 뒤 자동 재개는
        # 없고, 사람에게 다시 가려면 탐지부터 다시 해야 한다(설계 4절).
        #
        # 억제까지 거는 것은 설계에 없는 판단이다. 방금 비상 정지가 걸린 그
        # 사람에게 해제 직후 로봇이 다시 다가가는 것이 더 나쁘다고 봤다.
        self._suppress_track(self.approach_track_id, now)
        self.approach_track_id = None
        self.approach_goal_pose = None
        self._response_deadline = None

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
        self._announced_milestones = set()
        self._distance_baseline = None
        self._approach.reset()
        # 재시도 예산은 목적지 하나당이다. IDLE 로 내려오면 이번 시도가 끝난
        # 것이므로 비운다. 이것을 빠뜨리면 한 번 실패한 뒤로 영영 재시도가
        # 안 되거나, 반대로 취소된 목적지가 되살아난다.
        self._nav_retry_count = 0
        self._retry_destination = None
        self._retry_at = None
        # 접근 대상도 비운다. _suppressed_tracks 는 남긴다 — 재접근 억제는
        # IDLE 로 돌아온 뒤에 효력을 내야 하는 값이라 여기서 지우면 무의미해진다.
        self.approach_track_id = None
        self.approach_goal_pose = None
        self._response_deadline = None
