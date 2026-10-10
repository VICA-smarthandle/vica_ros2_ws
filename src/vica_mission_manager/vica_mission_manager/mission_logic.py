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

from dataclasses import dataclass, replace
from enum import Enum
from typing import Callable, Optional, Sequence, Union

from .approach_speed import ApproachSpeedLadder, NO_SPEED_LIMIT
from .grip_meter import GripMeter

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
    # "비카야"를 듣고 그 방향으로 고개를 돌리는 중(또는 못 찾고 되돌아 도는
    # 중). 안내 받는 사용자가 아직 없는 구간이다 (호출 접근 설계 §4).
    SEEKING = "seeking"
    # ---- 목적지 도착 후 대화 (2026-08-30, arrival-dialog-flow) ------------------
    #
    # 도착하면 유형별로 묻고(ASKING_NEXT), 대기를 고르면 시간을 묻고
    # (ASKING_WAIT_TIME), 그 자리에 서서 기다린다(WAITING). 답이 없으면 결국
    # 홈으로 복귀한다. AWAITING_USER 와 같은 모양이다 — 질문을 던지고, 재생이
    # 끝난 시점부터 시한을 세고, 답이 오면 갈래를 만든다.
    ASKING_NEXT = "asking_next"
    # "몇 분쯤?" 답 대기 (restroom·entrance 가 아닌 곳에서 대기를 골랐을 때만).
    ASKING_WAIT_TIME = "asking_wait_time"
    # 대기. 사람접근 OFF(기다리라 해놓고 행인을 쫓지 않게). 목적지가 정해지거나
    # 시간이 초과되면 나간다. "비카야"로는 끝나지 않는다(2026-10-07).
    WAITING = "waiting"
    # ---- 대기 장소 (2026-10-07, 작업 계획 탭) -----------------------------------
    #
    # 목적지에 대기 장소가 있으면 M2 를 말한 뒤 손을 놓기를 기다리고
    # (WAITING_RELEASE), 혼자 대기 장소로 가서(MOVING_TO_WAIT_SPOT) WAITING 이 된다.
    # 대기 장소에 못 들어가면 M6 을 말하고 목적지로 돌아와(MOVING_BACK_TO_DEST)
    # 거기서 WAITING 이 된다. 셋 다 '대기의 일부'라 대기 시간은 M2 순간부터 흐른다.
    WAITING_RELEASE = "waiting_release"
    MOVING_TO_WAIT_SPOT = "moving_to_wait_spot"
    MOVING_BACK_TO_DEST = "moving_back_to_dest"


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
    # 홈 복귀 요청 전용 사유.
    NO_HOME = "no_home"
    ALREADY_HOME_BOUND = "already_home_bound"
    # 대기 장소로 가보기(관리자) 전용 사유.
    NO_WAIT_SPOT = "no_wait_spot"


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw_deg: float  # destinations.yaml 은 도(deg) 단위 (함정 목록 2번)
    frame_id: str = "map"


@dataclass(frozen=True)
class WaitSpot:
    """목적지에 딸린 대기 장소 (2026-10-07). destinations.yaml 의 wait_spot 칸.

    side 는 앱이 저장할 때 계산한 '입구 기준 방향'(right/left/across)이다. 로봇은
    다시 계산하지 않고 멘트에 그대로 쓴다 — 관리자가 화면에서 본 글자와 같아야 한다.
    """

    x: float
    y: float
    yaw_deg: float   # 나가는 방향(도). 혼자 가는 곳이라 방향까지 맞춰 선다.
    side: str        # right / left / across


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
    # 입구 방향(도, 지도 기준, 2026-10-07). 도착 멘트 M1("…은/는 오른쪽에 있습니다")
    # 계산과 배송 도착 방향에 쓴다. 없으면 None — M1 을 말하지 않고, 배송은 pose.yaw.
    door_yaw_deg: Optional[float] = None
    # 대기 장소. 없으면 None — 지금처럼 목적지에서 그대로 기다린다.
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
    # 도착 후 대화의 wait 요청 시간(분). 없거나 무관하면 -1 (2026-08-30).
    wait_minutes: int = -1
    # LLM 이 이 요청에 소리 내어 답하는 말(없으면 ""). 접근 질문 중의 질문은 그 답이 끝난 뒤
    # 다시 묻는다(2026-10-09) — 미션이 먼저 말하면 순서가 뒤바뀐다.
    reply: str = ""


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
    # ros_tts_node 큐 우선순위 (긴급 > 응답 > 내레이션 > 배경).
    # 노드가 "{priority}:{text}" 접두어로 /vica/tts_request 에 발행한다.
    # ambient(2026-10-07)는 배경 알림 — 대기 중·홈 알림(M3)과 주행 중 장애물 안내(2026-10-09). 다른 말이 나가거나 줄 서
    # 있으면 TTS 가 바로 버리고, 재생 중 다른 말이 오면 비킨다. 옛 TTS 는 이 이름을
    # 몰라 글자로 읽으므로 음성 저장소와 같은 날 올린다.
    priority: str = "narration"  # emergency / response / narration / ambient
    # 이 말이 '질문'이라 사용자 답을 기다리는가. true 면 노드가
    # /vica/listen_request 를 함께 발행하고, 웨이크워드 노드가 질문 TTS 종료
    # 직후 재청취 창을 연다 — "비카야" 재호출 없이 "네/아니요"로 답하게 한다.
    expects_reply: bool = False


@dataclass(frozen=True)
class Navigate:
    destination: Destination
    # 어느 Nav2 행동 트리로 갈지(NAV_TREE_*). 노드가 파일 경로로 바꾼다. 사람 접근
    # goal 은 이 값과 무관하게 목적지 id 접두어로 접근 트리를 쓴다(nav_behavior_tree).
    tree: str = ""


@dataclass(frozen=True)
class GoalEvent:
    """앱에 알릴 사건 하나. 노드가 /vica_goal_event JSON 으로 낸다.

    주행 결과(goal_succeeded 등)는 노드가 Nav2 결과를 보고 스스로 내지만, 대기
    장소 막힘·대기 시간 만료는 주행 결과가 아니라 미션의 판단이라 여기서 낸다
    (2026-10-07, 앱 알림 2종)."""

    event: str
    destination: Optional[Destination] = None
    reason: str = ""
    # 앱 팝업의 아래 칸(목업 10·11번). "spot"=대기 장소, "destination"=목적지 앞.
    wait_place: str = ""
    wait_minutes: int = -1


@dataclass(frozen=True)
class CancelNav:
    destination: Optional[Destination] = None
    # 노드가 /vica_goal_event 로 알릴 이벤트 이름. 일시정지도 Nav2 goal 을 취소하지만
    # 목적지를 기억하므로 취소와 구분해서 알려야 앱이 "주행 끝"으로 오해하지 않는다.
    event: str = "goal_canceled"


@dataclass(frozen=True)
class StopSpeech:
    """하던 말을 끊고 대기 중인 비긴급 발화를 비운다 (노드가 /vica/tts_stop 발행).

    취소·앱 선점의 큐 청소 (2026-09-01 사용자 결정): 상태는 즉시 바뀌는데
    입에 물린 낡은 멘트("방2 앞에 도착했습니다…")가 이어 나오면 헛소리가
    된다. tts_stop 은 긴급 발화를 건드리지 않는다 (tts_queue 계약)."""


@dataclass(frozen=True)
class WakeReply:
    """"비카야"에 대한 미션의 판정 — 노드가 /vica/wake_reply 로 "listen"/"ignore" 를 낸다.

    호출 반응표(2026-10-07, 작업 계획 탭): 대답할지는 상태를 아는 미션이 정한다. 음성
    쪽은 호출 즉시 듣기 창을 열어 두고("비카야 화장실 가자" 한 호흡 보존), 들은 말을
    이 판정이 올 때까지 쥐고 있다가 listen 이면 LLM 으로 넘기고 ignore 면 버린다.
    미션 노드가 멈추면 판정이 오지 않아 음성 쪽이 시간을 넘겨 버린다(대답 없음).
    """

    listen: bool


@dataclass(frozen=True)
class SpinInPlace:
    """제자리 회전. 노드는 BasicNavigator.spin() 으로 실행한다.

    Navigate 가 아니다 - goal 을 만들지 않고 behavior server 의 Spin 을 탄다.
    Spin 은 회전 중 costmap 충돌을 스스로 검사한다. 양수 = 반시계.
    """

    yaw_rad: float
    # 이 회전이 무엇인지. 로그에만 쓴다.
    #
    # 지금 제자리 회전은 세 종류다(수락 뒤 핸들 내주기·호출 방향 보기·못 찾아
    # 원위치). 셋이 같은 문구로 찍히면 실기에서 "왜 돌았는지"를 사후에 가릴 수
    # 없다 — 특히 호출 회전과 그 8초 뒤의 원위치 회전은 밖에서 보면 "부르지도
    # 않았는데 또 돌았다"로 보인다 (2026-09-10 실기 관찰).
    reason: str = "회전"


@dataclass(frozen=True)
class SetNavSpeedLimit:
    """Nav2 controller 최대속도 제한율. 0.0은 제한 해제다."""

    percent: float


@dataclass(frozen=True)
class Haptic:
    """손잡이 진동 요청. 노드가 패턴 이름을 그대로 /vica/haptic_request 에
    발행한다 — 이 모듈은 그 토픽도, 값을 해석하는 펌웨어도 모른다.

    쓸 수 있는 패턴은 드라이버가 아는 넷이다("short"/"long"/"tick"/"locate",
    user_guidance_driver_node.HAPTIC_PATTERNS). 미션은 "long"(손잡이 찾기)·
    "tick"(잡음 확인)·"locate"(대기 중 호출 위치 알림, 2026-10-09)만 낸다 — 도착·비상
    진동은 펌웨어가 상태 진입 때 스스로 낸다.
    """

    pattern: str


Action = Union[
    Say, Navigate, CancelNav, SetNavSpeedLimit, Haptic, SpinInPlace, StopSpeech,
    GoalEvent, WakeReply,
]


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


# 숫자로 끝나는 이름을 읽을 때 마지막 숫자의 받침(한국어 수 읽기: 영·일·삼·육·칠·팔 받침
# 있음, 이·사·오·구 없음). "B1"→"비일은", "2"→"이는".
_DIGIT_HAS_BATCHIM = {
    "0": True, "1": True, "2": False, "3": True, "4": False,
    "5": False, "6": True, "7": True, "8": True, "9": False,
}


def josa_eun_neun(word: str) -> str:
    """단어 뒤에 붙는 조사 '은 / 는' 을 받침에 맞게 돌려준다 (M1, 2026-10-07).

    한글로 끝나면 받침 있음 → '은', 없음 → '는'. 숫자로 끝나면 그 숫자를 읽은
    소리의 받침을 본다(_DIGIT_HAS_BATCHIM). 그 밖의 글자(영문 등)는 읽는 소리를
    알 수 없어 '는'으로 둔다.
    예) 화장실 → 은, 안내센터 → 는, 407호 → 는, 회의실B1 → 은
    """
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
    """목적지 이름이 들어가는 멘트를 조사까지 맞춰 완성한다.

    호출부가 josa 를 빠뜨리면 KeyError 로 바로 드러나지만, 매번 두 인자를 넘기는
    대신 여기 한 곳을 거치게 해 빠뜨릴 자리를 없앤다.
    """
    return template.format(name=name, josa=josa_euro(name))


MSG_START = "{name}{josa} 안내를 시작합니다."
# 물류 배송 전용 출발 멘트 (2026-09-03 사용자 승인). 배송은 사람을 데려가는
# 일이 아니라 물건을 옮기는 일이라 "안내"가 어색하다. 문장 틀만 다르고 조사
# 계산(say_destination)은 그대로다 — 407호든 식당이든 받침에 맞게 붙는다.
# 배송 경로(request_delivery)에서만 쓴다. 음성·원격 주행은 MSG_START 그대로.
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
# E-stop 안내 — 움직이는 중에 걸릴 때만 말한다 (2026-08-31 최종 감량).
# 정지 중 걸림은 침묵: 로봇이 어차피 안 움직여 사용자에게 달라지는 게
# 없고(통신 순단이면 1.4초 자동복구), 안 알린 걸림은 해제도 침묵한다.
# 그래서 통신/사람 원인 판별(/estop_sources 스냅샷)도 함께 없앴다 —
# 자동복구 예고 멘트가 그 판별의 유일한 소비자였다.
MSG_ESTOPPED = "안전을 위해 멈추겠습니다. 관리자를 호출했습니다."
MSG_ESTOP_RELEASED = "비상멈춤이 해제되었습니다."   # 2026-08-31 사용자 확정(짧게)

# ---- "비카야" 호출 반응 (2026-10-07, 작업 계획 탭 '"비카야" 반응표') ----------
# 호출 인사. 예전엔 호출어 노드가 상태와 상관없이 말했고, 이제 미션이 반응표대로
# 말한다. 음성 replies.WAKE_GREETING 과 글자가 같아야 구운 판이 0초에 난다.
MSG_WAKE_GREETING = "네?"
# 비상 정지 중 호출 — "네?" 없이 이 한 마디만 하고 듣지 않는다(사용자 결정).
# MSG_ESTOP_REJECT 의 앞 문장이다: 무엇을 말해도 받지 못하므로 "다시 말씀해 주세요"는 뺀다.
MSG_ESTOP_WAKE = "지금은 비상 멈춤 상태입니다."
# 확인 질문 문구가 빈 목적지를 미션이 직접 물을 때(주행 중 바로 온 확정 요청). 음성
# destination_loader._fill_defaults 의 기본 확인 문구와 같은 글자 — 그쪽이 미리 합성해 둔다.
MSG_CONFIRM_PROMPT_FALLBACK = "{name}{josa} 안내해드릴까요?"
# 확인 질문 중 다른 목적지를 확정하면 새 목적지로 다시 묻는다(2026-10-08 사용자 결정 4, 사용자
# 문구). {prompt} 는 새 목적지의 확인 질문이다 — 음성 쪽이 목적지마다 미리 합성한다.
MSG_CONFIRM_SWITCH = "네, {prompt}"

MSG_DISTANCE_REMAINING = "목적지까지 약 {meters}미터 남았습니다."
MSG_CANCELED = "안내를 취소했습니다."

# ---- 도착 후 대화 (2026-08-30, arrival-dialog-flow) --------------------------
# 유형별 첫 질문. category(destinations.yaml category2)로 고른다. "네"의 뜻이
# 유형마다 뒤집히므로 질문을 던질 때 종료형인지(_asking_is_finish)를 함께
# 기억한다. 문구 정본은 이 상수들 — 캐시가 구운 판을 재생한다(글자 일치).
MSG_ASK_RESTROOM = "다녀오시는 동안 여기서 기다릴까요?"          # 대기형
MSG_ASK_ENTRANCE = "여기까지 안내를 마칠까요?"                  # 종료형
MSG_ASK_GENERIC = "여기서 대기할까요?"                          # 대기형·시간 질문
MSG_ASK_WAIT_TIME = "몇 분쯤 걸리실까요?"
# 대기 확정. {minutes} 는 코드가 채운다 — 캐시엔 넣지 않는다(가변).
# 끝말 "불러 주세요"는 대기 장소 멘트(M2)와 맞춘 것이다(2026-10-07 사용자 결정, 옛 "말씀해 주세요").
MSG_WAIT_CONFIRM = "{minutes}분 대기하겠습니다. 돌아오시면 '비카야'라고 불러 주세요."
MSG_WAIT_DEFAULT = "네, 최대 30분까지 여기서 기다리겠습니다. 돌아오시면 '비카야'라고 불러 주세요."
# ---- 대기 장소 (2026-10-07, 작업 계획 탭 '멘트') -------------------------------
# 문구 정본은 이 상수들이다. 음성 쪽이 미리 합성·굽는 문장과 글자가 같아야 한다.
# M1 — 도착 멘트 바로 뒤. 입구 방향이 있는 목적지만. {name}{eun} 은 코드가 채운다.
MSG_DOOR_SIDE = "{name}{eun} {side}에 있습니다."
# M2 / M2′ — 대기 장소가 있는 목적지에서 대기 확정. {place} = WAIT_PLACE_PHRASES.
MSG_WAIT_SPOT_CONFIRM = (
    "{minutes}분 동안 {place}에서 기다리겠습니다. 돌아오시면 '비카야'라고 불러 주세요.")
MSG_WAIT_SPOT_DEFAULT = (
    "최대 30분 동안 {place}에서 기다리겠습니다. 돌아오시면 '비카야'라고 불러 주세요.")
# M3 — 대기 중 10초마다(대기 장소가 있는 목적지만). 사용자가 소리를 따라 로봇을 찾는다.
# 홈에서 쉴 때도 1분마다 같은 말을 한다(HOME_BEACON_*, 2026-10-08).
MSG_WAIT_BEACON = "비카가 대기 중입니다."
# M6 — 대기 장소로 가다 실패(Nav2 실패 신호). 말한 뒤 목적지로 돌아간다.
MSG_WAIT_SPOT_BLOCKED = "대기 자리가 막혀 입구 앞에서 기다리겠습니다."
# M7 — 대기 시간 만료(대기 장소가 있든 없든). 말한 뒤 홈으로 간다.
MSG_WAIT_EXPIRED = "대기 시간이 종료되어 제자리로 돌아갑니다."
# 주행 중 장애물 안내(2026-10-08 사용자 선택 A1·S1, 10-09 미션에 넣음). 안내 주행 중 앞에 지도에 없는 물체가
# 있어 크게 비키거나(avoid) 줄이거나 설 때(slow) 한 번. 판정은 obstacle_judge.py(설계서 3절), 말할지는
# MissionLogic.obstacle_cue(설계서 5절 2단계). 음성 mission_phrases 에 같은 글자의 녹음이 있다.
MSG_OBSTACLE_AVOID = "앞에 장애물이 있어 피해 갈게요."
MSG_OBSTACLE_SLOW = "앞에 장애물이 있어 천천히 갈게요."
# 행동이 시작되고 이만큼 지난 후보는 버린다 — 늦은 장애물 안내는 지나간 물체 이야기다(1단계 도구와 같은 값).
OBSTACLE_STALE_SEC = 2.5
# 대기 중 "다 됐어"(finish) — 다음 목적지를 묻는다(2026-10-07 사용자 결정). 대기는 목적지가
# 정해져 출발할 때 끝나므로(호출 반응표) 묻기만 한다. 다음 목적지를 미리 아는 기능이
# 생기면 "OO으로 갈까요?"로 바꾼다(미구현).
MSG_WAIT_FINISH_ASK = "네, 어디로 모실까요?"
# 대기 중 "취소" — 안내를 끝낼지 묻는다(2026-10-08 사용자 결정 3, 사용자 문구). 부정 질문이라
# "네"(필요 없다) = 종료·홈, "아니요"(필요하다) = 계속 대기. 음성 쪽이 미리 굽는 글자와 같아야 한다.
MSG_WAIT_NEED_ASK = "안내가 필요 없으신가요?"
# 질문을 다시 묻기까지 기다리는 시간(2026-10-08 사용자 제안 "대답이 없거나 이상하면 다시
# 묻기"). 다시 묻기는 질문마다 한 번이다 — 옆사람 말이 섞여도 끝없이 되풀이하지 않게.
QUESTION_REASK_SEC = 15.0
# 대기 장소 입구 기준 방향 → 멘트 속 장소 말. 상황판(RobotState.wait_place)에도 같은 말을 쓴다.
WAIT_PLACE_PHRASES = {"right": "입구 오른쪽", "left": "입구 왼쪽", "across": "입구 맞은편"}
# 대기 장소가 막혀 목적지로 돌아와 기다릴 때의 장소 말.
WAIT_PLACE_AT_DESTINATION = "입구 앞"
MSG_FINISH = "안내를 종료합니다."
# 무응답 사다리 (3절): 못 알아들으면 재질문 1회, 그 뒤/침묵이면 떠나기 예고.
MSG_ARRIVAL_RETRY = "잘 듣지 못했습니다. 계속 안내가 필요하시면 말씀해 주세요."
MSG_LEAVING_NOTICE = "응답이 없어 안내를 마치고 제자리로 돌아가겠습니다."

WAIT_MINUTES_CAP = 30
# M3 간격. 대기 장소로 출발한 순간부터 대기가 끝날 때까지, 상태가 바뀌어도 박자를 잇는다.
WAIT_BEACON_INTERVAL_SEC = 10.0
# 홈 알림(2026-10-08 사용자 요청) — 홈에서 쉬는(IDLE) 동안 1분마다 M3 를 말해 "여기
# 있다"를 알린다. 홈인지는 실제 위치(/amcl_pose)로 본다 — 복귀가 실패해도 RETURNING 은
# 끝나므로 상태만으로는 홈에 있는지 모른다. 노드 파라미터 home_beacon_interval_sec 로
# 바꾸고 0 이면 끈다.
HOME_BEACON_INTERVAL_SEC = 60.0
HOME_BEACON_RADIUS_M = 0.5
# 손잡이를 이만큼 계속 놓고 있으면 대기 장소로 떠난다(터치 센서가 살아 있을 때).
# 센서가 없거나 끊겼거나 시연 스위치(grip_assume_held)면 M2 가 끝나자마자 떠난다.
WAIT_RELEASE_SEC = 5.0
# M2 재생 완료(tts_done)를 끝내 못 받았을 때의 보험 — 영영 서 있지 않게 한다.
WAIT_RELEASE_SPEECH_FALLBACK_SEC = 30.0
# 대기 중 "다 됐어"에 "어디로 모실까요?"를 물은 뒤 이 시간 안에 또 끝말(finish)이 오면
# 갈 곳이 없다는 뜻이다 — 같은 질문을 되풀이하지 않고 안내를 끝낸다(도착 질문의 finish 와
# 같은 길). 질문 답 창(30초)과 맞춘다.
WAIT_FINISH_REPEAT_SEC = 30.0
# M1 에서 '앞'·'뒤'라고 말하는 폭(도). 그 밖은 모두 오른쪽·왼쪽이다 — 입구는 대개
# 옆에 있어 90° 입구가 경계에서 멀리 떨어진 '오른쪽' 한가운데에 들어간다.
DOOR_FRONT_BACK_DEG = 30.0
# 대기 장소·목적지 복귀 goal 을 감싼 Destination 의 id 접두어. 앱 알림·대장에서
# 등록 목적지와 구분한다(사람 접근의 APPROACH_DESTINATION_PREFIX 와 같은 방식).
WAIT_SPOT_DESTINATION_PREFIX = "wait_spot:"
WAIT_BACK_DESTINATION_PREFIX = "wait_back:"
LEAVING_GRACE_SEC = 3.0        # 떠나기 예고 후 마지막 끼어들기 유예
# 홈 복귀 재개 (2026-09-10 사용자 결정). 복귀 중 호출("비카야")로 브레이크가
# 걸리면(on_return_brake) 그 순간부터 이 시간만큼 침묵하면 떠나기 예고를
# 낸다(멘트는 MSG_LEAVING_NOTICE 재사용) — 청취 창(음성 쪽, 약 8초)의 길이는
# 이 모듈이 모른다. 기준은 항상 "호출이 브레이크를 건 시각" 또는(회전이
# 끼어들었으면) "그 회전을 마치고 IDLE 로 돌아온 시각" 이며, 절대 두 구간을
# 이어 붙여 세지 않는다.
RETURN_RESUME_SEC = 15.0
# 귀 홀드 (2026-08-30): 무응답 시계는 귀가 바쁜 동안 멈춘다 — 답이 STT·LLM
# 을 통과하는 동안 8초가 먼저 울려 떠나던 결함. closed(전사 성공) 후 LLM
# 처리 유예, open 이 닫힘 신호를 잃어도 상한 뒤엔 떠난다(무한 대기 방지).
# 6 -> 8초(2026-10-10 사용자 결정): 10-10 12:13 "그래."가 STT 를 지난 뒤 Realtime 이 9초 만에 실패해
# 글자 경로의 affirm 이 closed 7.4초 뒤 왔다 — 6초 유예라 미션이 먼저 "안내를 받으시겠어요?"를 다시 묻고
# 1.3초 뒤 "네, 잠시만…"으로 수락해, 묻고 바로 스스로 답한 꼴이었다.
EAR_GRACE_SEC = 8.0
EAR_HOLD_MAX_SEC = 20.0
# 최후 안전망: ASKING 진입 후 이 시간 안에 응답 시계가 못 열리면 강제로
# 연다. 근본 수리는 TTS 쪽 — 끊긴 발화도 tts_done 을 발행한다(2026-08-31)
# — 이라 정상 계통에선 절대 안 울리고, 신호 유실·TTS 사망 같은 계통 밖
# 사고에서만 마지막으로 "결국 홈으로" 원칙을 지킨다(watchdog).
ASKING_STUCK_FALLBACK_SEC = 30.0
MSG_PAUSED = "잠시 멈추겠습니다. 다시 출발하려면 말씀해 주세요."
MSG_RESUMED = "{name}{josa} 다시 출발합니다."
MSG_CANCEL_CONFIRM = "안내를 취소할까요?"
MSG_CANCEL_KEPT = "안내를 계속하겠습니다."
MSG_NOT_NAVIGATING = "지금은 안내 중이 아닙니다."
MSG_NOT_PAUSED = "다시 출발할 안내가 없습니다."
# 안내 주행 중 "다시 가자" — 이미 가는 중이다(미션 요청 반응표 2026-10-08). 음성
# replies.ALREADY_GOING 과 같은 글자다 — 음성이 같은 말을 이미 합성해 쓴다.
MSG_ALREADY_GOING = "지금 {name}{josa} 가는 중이에요."
# 사람 접근. 질문은 되묻기와 같은 이유로 expects_reply 를 달아 내보낸다.
# ⚠️ 문구 정본은 voice replies.py·ment_cache (approach-voice-flow.md 확정 흐름).
# 글자까지 일치해야 사전 녹음이 재생된다 — 바꾸려면 양쪽을 함께 고치고
# 재녹음한다(voice scripts/make_cue_wavs.py). 수락 멘트가 회전 예고인 이유:
# 시각장애인에게 예고 없는 움직임 금지(2026-08-25 결정).
# 접근 질문·온보딩은 2026-10-05 짧은 원고로 통일했다(사용자 결정). 옛 긴 원고
# ("저와 함께 목적지까지 동행해보시는건 어떠세요?" 등)는 길어서 대답이 늦고, 같은
# 날 잠깐 둔 설정값 short_approach_ments 로 LLM 종류별로 가르던 것도 걷어냈다 —
# 이 멘트는 LLM 이 아니라 미션이 말하므로 OpenAI·로컬 모두 같은 녹음이 나간다.
# 다가가기 시작할 때 한 번 (2026-10-07 사용자 결정). 다가가는 동안의 위치 알림은
# 음성 쪽 차임(dialog_state=approaching, 2초마다 종 두 음) 몫이다. 후진음은 '피하라'로
# 들려 쓰지 않는다. 구운 판(assets/baked)과 글자가 같아야 한다.
MSG_APPROACH_COMING = "동행로봇 비카가 다가가고 있어요."
MSG_APPROACH_QUESTION = "안녕하세요? 시각장애인 안내로봇 비카입니다. 안내를 받으시겠어요?"
MSG_APPROACH_ACCEPTED = "네, 잠시만 기다려주세요. 로봇이 회전하니 주의하세요."
MSG_APPROACH_DECLINED = "알겠습니다. 이만 물러납니다."
MSG_APPROACH_ONBOARDING = "저에게 말을 거실 때는 '비카야'라고 불러주세요. 어디로 가고 싶으신가요?"
MSG_APPROACH_NO_ANSWER = "실례했습니다. 필요하시면 언제든 불러 주세요."
# 접근 질문을 다시 묻는 말(2026-10-09 사용자 결정·문구). 예·아니요가 아닌 말이나 8초 침묵에
# APPROACH_REASK_MAX 번까지. 첫 질문의 끝과 같은 글자라 노드가 재생 끝(tts_done)에서 이 글자로 8초를 다시 센다.
MSG_APPROACH_REASK = "안내를 받으시겠어요?"
# 다시 묻기를 다 쓰고도 예·아니요를 못 들어 물러날 때의 말(2026-10-09 18:12 실기 뒤 사용자 결정 '나').
# 대답을 못 들었는데 "알겠습니다. 이만 물러납니다."는 맞지 않는다(18:39 — LLM 이 답한 직후 그 말로 떠났다).
# 분명한 아니요는 MSG_APPROACH_DECLINED 그대로. 구운 판(assets/baked)과 글자가 같아야 한다.
MSG_APPROACH_UNANSWERED = "필요하시면 '비카야'라고 불러 주세요."
MSG_APPROACH_BUSY = "지금은 다른 응대 중입니다. 잠시 후 다시 말씀해 주세요."

# ---- 온보딩 뒤 빈손 되묻기 사다리 (실기 2026-09-11, 사용자 결정) --------------
# 온보딩(MSG_APPROACH_ONBOARDING) 뒤 mission_logic 이 걸던 시계가 아예
# 없었다 — 음성이 15초 창을 열었는데 STT 가 너무 짧거나 조용해 기각하면
# (/vica/listen_state 가 "empty:ghost"/"empty:short-reject" 등만 내고) 로봇도
# 사용자도 조용히 서 있기만 했다. 도착 후 대화의 무응답 사다리
# (MSG_ARRIVAL_RETRY/_arrival_retried, 위 3절)를 그대로 본떠, 한 번 되묻고
# 그래도 빈손이면 떠남을 예고한 뒤 홈으로 돌아간다. LLM 이 못 알아들은
# 경우(재청취를 음성 노드가 스스로 여는 "안내와 관련된 요청이 아니에요")는
# 범위 밖이다 — on_intent 가 그 도착만으로 이 사다리를 청산한다.
MSG_DEST_RETRY = "잘 듣지 못했습니다. 어디로 가고 싶으신가요?"
# 답을 기다리는 시간 — 온보딩·되묻기 **재생이 끝난 시점**(노드가 tts_done 으로
# 알려주는 on_dest_prompt_spoken)부터 센다. 음성의 질문 뒤 청취 창은 30초
# (확인 질문용)라 그 창의 빈손 신호를 기다리면 한 단이 30초가 되고, 사다리
# 한 판이 80초를 넘겼다(2026-09-11 실기). 접근 질문의 8초처럼 미션이 직접
# 잰다. 사용자가 말을 시작하면(listen_state "speech") 시계를 잡는다 —
# 창이 열렸다는 신호("open")만으로는 잡지 않는다.
DEST_ANSWER_WAIT_SEC = 15.0
# tts_done 이 끝내 안 왔을 때의 보험. 힌트+온보딩 재생(≈14초) + 답 대기를
# 덮어야 하므로 이만큼 크다 — 정상 경로는 on_dest_prompt_spoken 이 먼저 와
# 이 시계를 DEST_ANSWER_WAIT_SEC 로 갈아 끼운다.
DEST_PROMPT_FALLBACK_SEC = 40.0
# 되묻기 뒤 빈손에서 예고까지 더 기다리는 시간. 사용자 결정(2026-09-11 실기):
# "15초 → 되묻기 → 15초 → 예고 → 3초" — 이미 두 번 기다렸으므로 곧장
# 예고한다(0). launch `dest_retry_return_sec` 로 늘릴 수 있다.
DEST_RETRY_RETURN_SEC = 0.0

# ── 손잡이 터치 × 진동 (2026-09-30) ─────────────────────────────────────────
# 정본: docs/superpowers/specs/2026-09-28-touch-haptic-integration-final.md.
# 손잡이는 "쥐고 있는 동안만 로봇이 걷는 줄"이다. 수락 뒤 잡기 대기에서 잡으면
# 활성 모드(손을 놓으면 선다), 못 잡으면 비활성 모드(놓아도 간다). 두 모드의
# 주행 방식은 같다 — 다른 것은 "놓으면 서는가" 하나다(설계 D9).
#
# 손잡이 위치 안내 (사용자 문구 2026-09-28, D7). 회전 여부와 무관하게 수락 직후
# 나간다 — 회전해서 손잡이를 내준 경우도 시각장애인은 그 사실을 알 방법이 없다.
# 진동은 이 멘트와 **같은 순간** 시작해 잡을 때까지 GRIP_HINT_PULSE_SEC 마다
# 이어진다. 그래서 "진동하고 있습니다"가 거짓말이 되지 않는다 — 09-11 의 "힌트
# 재생이 끝난 뒤 진동"(I-2) 장치는 문구가 바뀌며 필요가 없어져 걷어냈다.
# 음성 저장소에 이 글자 그대로 구운 음성(mission_msg_handle_hint.wav, 10-05)이 있다 —
# 글자를 바꾸면 합성으로 떨어져 늦게 나오니 다시 굽는다.
MSG_HANDLE_HINT = "손잡이가 진동하고 있습니다. 잡아주세요."
# 주행 중 손을 놓쳐 섰을 때(09-03 사용자 문구). 음성 저장소에 이 글자 그대로
# 구운 음성(handle_grip_lost.wav)이 있다 — 구운 판은 문장 전체 대조라 **마침표를
# 붙이면 안 쓰인다.** '정지'·'멈춰'가 없어 긴급어 자가 트리거도 없다.
MSG_HANDLE_LOST = "안전을 위해 손잡이를 다시 잡아주세요"
# 활성 주행 중 핸들 상향이 끊겼을 때. 음성 replies.HANDLE_UNAVAILABLE 과 같은 글자
# (구운 음성 reply_handle_unavailable.wav 있음). 끊기면 다시 잡아도 알 길이 없어
# 세우지 않고 비활성으로 계속 간다(설계 4.3 (라)).
MSG_HANDLE_UNAVAILABLE = "손잡이 연결에 문제가 있어 일반 안내로 진행합니다."
# 손잡이를 찾는 신호. 짧은 진동은 지나치기 쉬워 긴 진동을 쓴다.
HAPTIC_PATTERN_HANDLE_HINT = "long"
# "잡은 걸 알아챘다"(D5). 도착(짧게 ×3)과 횟수로 구별된다. 진행 중인 긴 진동을
# 덮어써 곧바로 끊는 역할도 한다(펌웨어 hapticStart 는 새 명령이 이긴다).
HAPTIC_PATTERN_GRIP_ACK = "tick"
# 대기(WAITING) 중 "비카야"를 들으면 손잡이를 1초씩 두 번(사이 1초) 떤다(2026-10-09 사용자).
# 볼일을 마친 사용자가 대기 장소의 비카를 소리(M3)에 더해 손으로도 찾게 한다 — 접근
# 시나리오에서 돌아선 뒤 손잡이를 떨어 위치를 알리는 것과 같은 뜻이다.
HAPTIC_PATTERN_WAKE_LOCATE = "locate"
# 잡음 판정: 최근 2초 중 80 % 이상 접촉(D1). 비율은 시간으로 잰다(grip_meter).
GRIP_ENTER_WINDOW_SEC = 2.0
GRIP_RATIO = 0.8
# 잡기 대기 상한. 넘으면 비활성으로 온보딩한다(D2). 청취 창 15초와 맞췄다.
GRIP_WAIT_TIMEOUT_SEC = 15.0
# 대기 중 긴 진동(1.2 s) 반복 간격 — 1.2 켜짐 + 0.8 쉼으로 "계속 떨리는" 느낌.
GRIP_HINT_PULSE_SEC = 2.0
# 이만큼 계속 놓으면 선다. 0.408 m/s 에서 약 0.2 m. **고쳐 잡기 공백 bag 실측으로
# 확정할 값이다**(설계 7절 4번) — 실측 전 활성 주행 금지.
GRIP_RELEASE_GRACE_SEC = 0.5
# 선 뒤 다시 잡았다고 볼 창. 진입(2초)보다 짧게 — 이미 한 번 잡았던 사람이다.
GRIP_RESUME_WINDOW_SEC = 1.0
# 출발 순간 "쥐고 있나"를 볼 창. 잡기 대기를 거치지 않은 출발에 쓴다(4.3 (나)).
GRIP_DEPART_WINDOW_SEC = 0.5
# 놓침 안내 반복 간격과 포기 시한(실기 조정값).
HANDLE_LOST_REPEAT_SEC = 15.0
HANDLE_LOST_GIVE_UP_SEC = 180.0
# 미션 쪽 신선도 시한. SmartHandleState 주기 2 Hz 의 두 배.
HANDLE_STATE_STALE_SEC = 1.0
# LLM 상황판(RobotState.dialog_state)에 내는 값(D10, 설계 4.7). 손 놓침으로 선
# 것과 "잠깐"으로 선 것이 둘 다 paused 로 보이면 LLM 이 "다시 가자라고
# 말하세요"라고 엉뚱하게 답한다. 음성 저장소 ledger_view.DIALOG_KO 가 번역한다.
DIALOG_GRIP_WAIT = "grip_wait"
DIALOG_PAUSED_HANDLE = "paused_handle"

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
# 접근 질문을 다시 묻는 횟수 상한(2026-10-09 18:12 실기 뒤 사용자 결정 — 처음엔 한 번이었다).
# 한 번이면 '비카야'(꺼 둔 호출어가 STT 로 들어와 못 알아들은 말이 된다) 하나에 기회를 다 썼다.
APPROACH_REASK_MAX = 3
# 질문 재생완료(tts_done)가 영영 안 올 때(TTS 사망 등)의 탈출용 안전망.
# 예전엔 질문을 큐에 넣는 시각부터 8초를 세는 폴백이었는데, 질문 음성이
# 정확히 8.0초라 재생이 끝나는 순간 시계도 끝나 답할 창이 0초였다
# (2026-08-31 실기 로그 2건 — 질문 후 8.2초 만에 "실례했습니다" 이탈).
APPROACH_QUESTION_STUCK_SEC = 30.0
# 수락 후 회전이 이 시간 안에 끝나지 않으면 포기하고 IDLE 로 내린다.
# 180도 / 회전 상한 0.4 rad/s = 7.9 s 에 수락·기동 지연 여유를 더한 값.
APPROACH_TURN_TIMEOUT_SEC = 15.0
# 호출 접근(설계 2026-09-10). "비카야"를 듣고 그쪽으로 고개를 돌린 뒤,
# 카메라가 사람을 찾을 때까지 기다리는 시간.
#
# 8.0 인 이유(2026-09-10 재검토 — 처음 잡은 6.0 은 여유가 1.5 s 뿐이었다):
# 바닥값은 stable 1.0 s + still window 3.0 s(detection_gate, 5 Hz) 만이 아니다.
#   - /vica/robot_state 는 1 Hz 발행이라 회전 종료(is_moving=false) 갱신이
#     최대 1.0 s 늦는다 — 그동안 YOLO 는 여전히 꺼져 있다(SEEKING 진입·이탈
#     즉시 발행으로 이 지연은 회수했지만, 값 자체는 그 지연 없이도 여유가
#     있도록 넉넉히 잡는다).
#   - 회전 중 추론이 끊겨 있었으므로 stable 3.0 s 는 창이 열린 뒤 새로
#     쌓인다.
# 바닥값 ≈ 1.0 + 0.2 + 3.0 ≈ 4.2~4.5 s. 젯슨 CPU 경합으로 프레임 간격이
# detection_gap 0.6 s 를 한 번만 넘겨도 연속이 깨져 처음부터 다시 세므로,
# 처음엔 8.0 으로 여유를 뒀다.
# 6.0 으로 내린 이유(2026-09-11 실기, 사용자 결정): 회전이 끝나고 사람이
# 확인되기까지 실측 1.5~4.0 s(11회, 최대 4.0). 8 초는 사람이 없을 때(오탐·
# 미검출)만 체감되는 시간이라 짧을수록 좋고, 그동안 "비카야"가 쌓인다.
# 6.0 은 실측 최댓값에 2 초 여유 — 프레임 간격 리셋 한 번을 견딘다.
# launch `seek_look_sec` 로 조정한다.
SEEK_LOOK_SEC = 6.0
# 이 값과 RETURN_RESUME_SEC(복귀 재개 사다리) 은 서로 대소를 지킬 필요가
# 없다 — on_wake_doa 가 _return_interrupted 동안 호출 자체를 거절해
# (2026-09-10 사용자 결정) 탐색 창이 그 사다리와 아예 같은 IDLE 위에 놓이지
# 않기 때문이다. 이 값을 올려도 사다리와의 우연한 여유(예전엔 8.0 < 15.0
# 에만 기대고 있었다)를 다시 계산할 필요가 없다.
# 회전이 시작조차 안 됐을 때(노드 결함 등) 상태에서 빠져나오는 시계.
# 접근 수락 회전과 같은 값을 쓴다 — 같은 Spin 액션이다.
SEEK_TURN_TIMEOUT_SEC = APPROACH_TURN_TIMEOUT_SEC
# 이보다 작은 회전은 하지 않는다. DOA 퍼짐이 ±4~16° 라 10° 미만은 잡음이고,
# 0 에 가까운 spin 은 behavior server 가 거부하거나 즉시 끝나 무의미하다.
SEEK_MIN_YAW_RAD = math.radians(10.0)
# 위 SEEK_MIN_YAW_RAD(정면 사각지대)의 거울쌍 — 핸들 쪽(로봇 뒤) 사각지대다.
# 이보다 큰 회전량(180°에 가까움)이면 소리가 핸들 부채꼴에서 왔다는 뜻이고,
# 그 자체가 "이미 핸들 옆에 서 있다"는 증거라 카메라 확인 없이 곧바로 접근
# 질문을 낸다 — 돌면 오히려 핸들을 사람에게서 빼앗는다(2026-09-10 사용자
# 결정). 부채꼴 180°±45°, 즉 135°가 경계다.
#
# 폭의 근거(2026-09-10 실기): 뒤에서 부른 호출의 DOA 가 163°·181°·185°·
# 160°·172° 로 전부 180±20° 안에 들어왔다. ±45° 면 넉넉한 여유다.
HANDLE_SIDE_MIN_YAW_RAD = math.radians(135.0)
# 같은 호출의 /vica/wake 와 /vica/wake_doa 는 몇 ms 간격으로 온다(2026-09-10
# 실기 재현). 콜백이 같은 MutuallyExclusive 그룹이라 wake 가 먼저 상태를
# IDLE 로 내린 뒤에야 wake_doa 가 처리될 수 있는데, 그때 IDLE 만 보고 통과
# 시키면 "옛 대화를 접었을 뿐"인 wake 를 새 호출로 오인해 SEEKING 이 열린다
# — 사람이 핸들을 잡고 로봇 뒤에 서 있을 때 "비카야"로 최대 180도 제자리
# 회전이 터지는 사고.
#
# 이 가드가 실제로 재는 것은 두 토픽의 **수신 시각 차**가 아니라 **콜백이
# 실행된 시각 차**다 — 둘 사이에 긴 콜백이 하나라도 끼면 그만큼 간격이
# 벌어진다. 같은 MutuallyExclusive 그룹에서 알려진 최악값이
# mission_manager_node._nav_lock_timeout_sec(2.0초, cancelTask 응답을
# 기다리는 상한)이다 — 젯슨 CPU 경합에서 이 콜백이 wake 와 wake_doa 사이에
# 끼어 꽉 채워 걸리면 옛 2.0초 가드와 정확히 같아져, 사람이 핸들을 잡은 채
# 최대 180도 제자리 회전이 도는 사고가 되살아난다.
# 3.0초는 그 최악값보다 크게 잡아 여유를 둔 값이다. 같은 호출의 두 토픽
# 간격을 넉넉히 덮으면서 진짜 새 호출(수 초 뒤)까지 막기에는 여전히 짧다.
WAKE_CONSUMED_GUARD_SEC = 3.0
# 접근을 마친 뒤 같은 track_id 에 다시 다가가지 않는 시간. 거절한 사람을 로봇이
# 계속 쫓아다니는 것이 이 기능의 가장 나쁜 실패 방식이라 값을 넉넉히 둔다.
REAPPROACH_SUPPRESS_SEC = 60.0
# 접근 회전이 끝나 사용자가 손잡이를 받아든 뒤 이만큼은 wake_doa 를 거절한다
# (2026-09-10 사용자 결정). 이 전이는 wake 가 아니라 회전 완료가 일으킨
# 것이라 WAKE_CONSUMED_GUARD_SEC 도장(_wake_consumed_at)이 안 찍힌다 —
# 재청취 창이 만료된 뒤 "비카야, 화장실"처럼 부르면 DOA≈180(핸들 쪽)이
# 그대로 SEEKING 을 열어, 손잡이를 잡고 로봇 옆에 선 사용자 앞에서 같은
# 사고가 재현된다.
#
# 60초인 이유는 둘이다. 하나, 같은 뜻(방금 상대한 사람을 다시 사고 대상으로
# 만들지 않는다)의 REAPPROACH_SUPPRESS_SEC 이 이미 60초라 — 임시방편에
# 숫자를 하나 더 만들지 않고 맞춘다. 둘, 이 규칙 전체가 터치센서가 붙기
# 전까지의 임시방편이라 — 정식 판정(터치)이 들어올 때 시나리오를 다시
# 정리하기로 했고, 그 전에 이 숫자만 정교하게 다듬는 것은 값어치가 없다.
#
# on_wake 는 이 억제가 이미 살아 있을 때만 now + USER_ATTACHED_SUPPRESS_SEC
# 로 되감는다(on_wake 참고) — 사용자가 그 사이 다시 말을 걸면 대화가 이어지는
# 한 계속 막힌다.
#
# [수정됨, 2026-09-11] 온보딩 질문("어디로 가고 싶으신가요?") 뒤에 걸리는
# 시계는 이제 아래 "온보딩 뒤 빈손 되묻기 사다리"(_dest_prompt_stage /
# _dest_prompt_deadline, _arm_dest_prompt 가 세 온보딩 자리 전부에서 건다)다
# — 한때 여기 적혀 있던 "시간 제한이 아예 없다"는 더는 사실이 아니다.
# 그래도 사용자가 **아무 말 없이 60초를 넘긴 뒤** "비카야"라고 부르면(즉
# 되묻기 사다리도 이미 만료돼 청산된 뒤) 이 억제는 풀려 있어 회전이 그대로
# 열린다 — 되감기는 그 사이 사용자가 말을 걸었을 때만 돕는다는 뜻은
# 그대로다.
#
# [임시방편] 이 판정의 참뜻은 "사용자가 지금 손잡이를 잡고 있는가"이고,
# 정답은 터치센서(SmartHandleState.user_contact)다. 2026-09-30 부터 그 조건을
# **덧댔다**(user_attached_guard_active) — 60초가 지나도 센서가 접촉을 보는
# 동안은 억제를 풀지 않는다. 시계는 지우지 않는다: 센서 고장은 위험한 쪽으로
# 난다(신호가 안 오면 규약상 false = "아무도 안 잡았다").
USER_ATTACHED_SUPPRESS_SEC = 60.0
# 근접 호출(2026-09-10 확장). 탐색 창(IDLE + _seek_deadline) 중 detection_gate 가
# 거리 하나만으로 TOO_NEAR 거절한 결과(stable=true·approachable=false·
# distance_m)가 person_detection 원본 토픽으로 들어오면, Mission 이 그 값을 직접
# 보고 판단한다 — 접근 goal(1.1 m)이 이미 지나간 자리인 사람에게 걸어가는 대신
# 그 자리에서 바로 질문한다. 부른 사람이 코앞에 있는데 8초 동안 쳐다보기만 하다
# 말없이 돌아가는 동작(2026-09-10 실기 관찰)을 없앤다.
#
# vica_perception.detection_gate.DEFAULT_MIN_DISTANCE_M 과 값은 같지만(1.5) 별개
# 상수다 — Mission 은 그 감지기 상수를 알 수 없다(패키지 경계, vica_perception 은
# 이 저장소에서도 건드리지 않는다). 두 값이 우연히 같을 뿐 하나가 다른 하나를
# 참조하지 않는다 — detection_gate 쪽을 조정해도 이 값은 저절로 안 따라간다.
NEAR_CALL_MAX_M = 1.5
# 이보다 가까우면 수락해도 회전하지 않는다(2026-09-10 사용자 결정, 안전).
# 손잡이가 뒤로 길게 나와 있어 제자리 회전의 실제 스윕이 차체보다 크다 — 이
# 거리에서 180도를 돌면 손잡이가 사람을 칠 수 있다. Nav2 의 Spin 은 회전 중
# costmap 충돌을 스스로 검사하지만, 코앞 사람은 costmap 에 잘 안 잡힌다는 실측
# 기록이 있어(잔상 15.8초, close-person-leaks-into-static) 그 검사에 기댈 수
# 없다. 이 거리면 사용자가 로봇에 손이 닿으므로 더듬어 손잡이를 찾을 수 있고,
# 나중에 햅틱이 붙으면 그 단계가 쉬워진다.
NEAR_CALL_NO_SPIN_M = 1.0
# 사람에게 다가가는 구간의 최대속도 상한. 주행 상한 0.5 m/s 의 100 % = 0.5 m/s 다.
# 마지막 1.1 m 는 collision_monitor 의 PolygonSlow 가 0.2 m/s 로 한 번 더
# 줄인다(설계 6.4절) — 그 구간은 이 값과 무관하다.
#
# [60 -> 100, 2026-09-09 실기] 60 % 일 때 7.77 m 접근에 19.6 초가 걸렸다. 속도가
# 깎이는 곳을 전 구간에서 확인하니 요청·승인·실제 바퀴가 0.300/0.300/0.40 으로
# 일치했다 — 어디서 막힌 것이 아니라 처음부터 이 값만큼만 허락된 것이었다.
# 100 % 로 풀고 다시 재니 승인(6.39 m)부터 도착까지 **7.9 초**로, 접근 거리를
# 4 m 에서 8 m 로 늘렸는데도 종전보다 빨라졌다. 사용자가 그 자리에 서서 다가오는
# 속도를 직접 확인하고 판정했다.
#
# [이름 주의] 이 파일에서 "접근"은 두 가지를 뜻한다. approach_speed.py 의
# ApproachSpeedLadder 와 MissionLogic.approach_speed_limit_percent 는 **등록
# 목적지에 가까워질 때의 감속**이고, 여기 PERSON_* 은 **사람에게 다가가는 구간의
# 고정 상한**이다. 사람 접근에는 사다리를 쓰지 않는다 — 처음부터 끝까지 느리다.
PERSON_APPROACH_SPEED_PERCENT = 100.0
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
# 사람 접근 전용 Nav2 행동 트리 파일 이름(vica_nav2/behavior_trees). 2026-10-02 run60.
APPROACH_BT_FILE = "vica_navigate_to_pose_approach.xml"


def nav_behavior_tree(destination_id: str, approach_bt: str) -> str:
    """이 goal 에 쓸 Nav2 행동 트리 파일. 빈 문자열은 bt_navigator 의 기본 트리(레일)다.

    사람 접근 goal 만 approach_bt 를 쓴다(2026-10-02, run60). 사람은 레일 위가 아니라
    방 가운데 서 있어, 레일 트리로 보내면 바로 앞 사람에게도 레일을 빙 돌아간다(사람이
    정남쪽 2.6 m 인데 8.5 m 우회 + 87 s 맴돌기). 목적지·홈 복귀는 그대로 레일 트리다.
    approach_bt 가 비어 있으면 종전처럼 모든 goal 이 기본 트리를 쓴다.
    """
    if approach_bt and destination_id.startswith(APPROACH_DESTINATION_PREFIX):
        return approach_bt
    return ""


# ---- 행동 트리 종류 (Navigate.tree, 2026-10-07) ---------------------------------
# 도착 판정을 목적에 따라 가른다. 노드가 종류를 파일 경로로 바꾸고, 파일을 못 찾으면
# 경고를 남기고 기본 트리로 간다(사람 접근 트리와 같은 방식).
NAV_TREE_DEFAULT = ""        # bt_navigator 기본(레일, 위치+방향): 홈·배송·원격 주행
NAV_TREE_GUIDED = "guided"   # 레일 + 위치만 판정: 사용자 안내 도착(방향 정렬 없음)
NAV_TREE_WAIT = "wait"       # 레일 없는 짧은 트리 + 위치·방향: 대기 장소·목적지 복귀·가보기
GUIDED_BT_FILE = "vica_navigate_to_pose_guided.xml"
GUIDED_NO_RAIL_BT_FILE = "vica_navigate_to_pose_guided_no_rail.xml"  # 레일 없는 지도용
WAIT_SPOT_BT_FILE = "vica_navigate_to_pose_wait_spot.xml"


def door_side_word(door_yaw_deg: float, robot_yaw_deg: float) -> str:
    """입구가 로봇(= 뒤에서 손잡이를 잡은 사용자) 기준 어느 쪽인가 (M1).

    사용자는 로봇 뒤에서 같은 쪽을 보고 서 있으므로 로봇의 오른쪽이 곧 사용자의
    오른쪽이다. 앞·뒤는 ±DOOR_FRONT_BACK_DEG 안일 때만 말한다.
    """
    rel = math.degrees(wrap_to_pi(math.radians(door_yaw_deg - robot_yaw_deg)))
    if abs(rel) <= DOOR_FRONT_BACK_DEG:
        return "앞"
    if abs(rel) >= 180.0 - DOOR_FRONT_BACK_DEG:
        return "뒤"
    # ROS yaw 는 반시계(왼쪽)가 양수다.
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
    """배송 도착 방향 — 입구 방향이 있으면 그쪽을 바라보고 선다(10-07 결정).

    입구 화살표 하나를 안내 멘트와 배송 정렬에 같이 쓴다. 없으면 옛 pose.yaw 그대로.
    """
    if dest.door_yaw_deg is None:
        return dest
    return replace(dest, pose=replace(dest.pose, yaw_deg=dest.door_yaw_deg))

# 접근 상태를 한 묶음으로 본다 — 새 목적지 요청을 거부하는 구간이다. SEEKING
# 이 빠지면 회전 중 음성 목적지 요청이 그대로 통과해 Navigate 가 나가고,
# 진행 중인 SpinInPlace 를 취소하지 않은 채 두 goal 이 동시에 나가게 된다
# (TURNING 과 같은 처리 — 설계 4절, 2026-09-10).
_APPROACH_STATES = (
    State.APPROACHING, State.AWAITING_USER, State.TURNING, State.RETURNING,
    State.SEEKING,
)
# Nav2 goal 이 살아 있는 상태. E-stop·긴급어가 goal 을 취소해야 하는 구간이다.
# 대기 장소로 혼자 가는 두 상태도 바퀴가 돈다(2026-10-07).
_GOAL_ACTIVE_STATES = (
    State.NAVIGATING, State.APPROACHING, State.TURNING, State.RETURNING,
    State.SEEKING, State.MOVING_TO_WAIT_SPOT, State.MOVING_BACK_TO_DEST,
)
# 대기의 일부인 상태들(2026-10-07). 대기 시간·상황판 대기 칸이 이 구간 내내 살아 있다.
_WAIT_STATES = (
    State.WAITING_RELEASE, State.MOVING_TO_WAIT_SPOT, State.MOVING_BACK_TO_DEST,
    State.WAITING,
)
# 혼자 움직이는 대기 구간. 사용자 말에 대답하지 않는다(호출 반응표, 대기 상태가 될 때까지).
_WAIT_MOVING_STATES = (State.MOVING_TO_WAIT_SPOT, State.MOVING_BACK_TO_DEST)
# "비카야"에 대답하지 않는 상태(호출 반응표, 2026-10-07 사용자 결정). "네?"도 하지 않고
# 음성 쪽에 들은 말을 버리라고 알린다. 사람에게 다가가거나 도는 중에는 응대할 수 없고,
# 주행 실패는 관리자 몫이며, 대기 장소로 가는 길은 짧아 대기 상태가 된 뒤 받는다.
# 비상 정지는 따로 한 마디(MSG_ESTOP_WAKE)를 하고 역시 듣지 않는다.
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
    # 음성 취소의 게이트다 — 주행·일시정지·대기(WAITING, 8/31 회수 수리)만.
    # 접근 응대·도착 질문을 지나가는 "취소" 한마디가 끊으면 안 된다.
    # **앱(관리자) 취소는 이 게이트를 안 탄다** — on_app_cancel 이 전면
    # 개방(ESTOPPED 예외만)으로 따로 처리한다 (2026-08-31 사용자 결정).
    if state not in (State.NAVIGATING, State.PAUSED, State.WAITING):
        return GateReason.NOT_NAVIGATING
    return GateReason.OK


def check_pause_gate(
    state: State, estop_active: bool, returning_home: bool = False
) -> GateReason:
    """일시정지 게이트. 실제로 주행 중일 때만 멈출 수 있다.

    ``returning_home`` 은 관리자가 앱에서 부른 홈 복귀 중이라는 뜻이다
    (2026-09-03). 그 복귀도 앱 버튼으로 멈췄다 다시 보낼 수 있어야 한다 —
    배송을 마치고 홈으로 돌아가는 로봇을 관리자가 잠깐 세울 길이 없었다.
    접근 뒤 자동 복귀는 여전히 막는다. 그 복귀는 사람이 부른 주행이 아니라
    "다시 출발"을 누를 주체가 없다.
    """
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


def check_return_home_gate(
    state: State,
    home: Optional[Destination],
    estop_active: bool,
    nav_ready: bool,
) -> GateReason:
    """홈 복귀 게이트(`/vica/mission/return_home`). **관리자 전용 요청이다.**

    사용자(음성)에게는 이 문이 아예 없다. 홈은 `destinations.yaml` 에 없어서
    UUID 가 없고, 음성 경로는 목적지를 UUID 로 지목하므로 **지목할 대상 자체가
    존재하지 않는다.** 전화번호부에 없는 번호로는 전화를 걸 수 없는 것과 같다.

    안내 주행 중에는 거부한다. 사용자가 핸들을 잡고 따라 걷는 중에 로봇이 홈으로
    방향을 틀면 **사용자는 자기가 어디로 끌려가는지 모른다** — 눈으로 확인할 수
    없고 로봇이 목적지 변경을 말해 주지도 않는다. 관리자는 먼저 주행을 취소해
    사용자에게 안내 종료를 알린 뒤 보내면 된다. 막는 것이 아니라 **알린 뒤 하게**
    만드는 것이다.

    일시정지 중에도 거부한다. 사용자가 돌아와 재개할 목적지가 살아 있기 때문이다.
    """
    if estop_active or state == State.ESTOPPED:
        return GateReason.ESTOP_ACTIVE
    if home is None:
        return GateReason.NO_HOME
    if state == State.RETURNING:
        return GateReason.ALREADY_HOME_BOUND
    if state in (State.APPROACHING, State.AWAITING_USER, State.TURNING):
        return GateReason.BUSY_APPROACHING
    if state != State.IDLE:
        # NAVIGATING·PAUSED·CONFIRMING·ARRIVED·FAILED 가 여기 걸린다.
        return GateReason.BUSY_NAVIGATING
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


def wrap_to_pi(rad: float) -> float:
    """각도를 -π~π 로 접는다 — 언제나 짧은 쪽으로 돈다."""
    return math.atan2(math.sin(rad), math.cos(rad))


def doa_to_spin_yaw(doa_deg: float, sign: float = 1.0) -> float:
    """마이크 DOA(0~359°, 정면 0 / 핸들 180)를 제자리 회전량(rad)으로.

    SpinInPlace 는 양수 = 반시계다. sign 은 마이크 각도가 반시계로
    커지면 +1, 시계로 커지면 -1 이며 장비마다 실측으로 정한다 — 틀리면
    로봇이 정확히 반대로 돈다.
    """
    return wrap_to_pi(math.radians(float(doa_deg) * float(sign)))


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
        estop_release_grace_sec: float = 1.0,  # 2.0→1.0 (8/31): 해제 소식을 더 빨리
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
        # 접근을 마치고 돌아갈 대기 위치이자 안내를 끝내고 돌아갈 홈이다.
        # 노드가 home.yaml 을 읽어 넣어 준다(2026-08-26). 지정되지 않았으면
        # None 이며, 그때는 제자리에서 접근만 끝낸다 — 홈이 없어도 안내는
        # 정상 동작하고 자동 복귀만 꺼진다.
        self.return_destination = return_destination
        # 접근을 마쳤을 때 홈까지 **스스로** 돌아갈 것인가.
        #
        # 기본값 False 는 사람이 부르지 않았는데 로봇이 혼자 달리는 일을 막는다.
        # 홈 복귀는 2026-08-27 현재 실기로 한 번도 확인되지 않았고, 확인되기 전에
        # 자동 주행부터 켜면 **아무도 안 보는 사이에 처음 달려 보게 된다.**
        # 관리자가 앱에서 부르는 복귀는 이 값과 무관하게 늘 동작하므로, 실기
        # 확인은 그쪽으로 먼저 한다.
        #
        # False 이면 접근을 마친 자리에 그대로 선다 — 홈 좌표를 넣기 전의 원래
        # 동작이다. 실기 확인이 끝나면 True 로 바꿔 '제자리 = 홈'으로 만든다.
        self.auto_return_home = auto_return_home
        # 홈 알림 간격(초). 0 이하면 끈다. 뜻은 HOME_BEACON_INTERVAL_SEC 주석.
        self.home_beacon_interval_sec = home_beacon_interval_sec
        # 지금 복귀가 '접근 뒤 복귀'인가 '관리자가 부른 홈 복귀'인가.
        #
        # 두 복귀는 가는 곳이 같아서 State.RETURNING 을 함께 쓰지만 **끝낼 때
        # 할 일이 다르다.** 접근 뒤 복귀는 방금 거절한 사람에게 다시 다가가지
        # 않도록 재접근 억제를 걸어야 하는데, 관리자 홈 복귀에는 억제할 사람이
        # 없다. 이 플래그가 없으면 엉뚱한 track_id 를 억제해 **다음에 만나는
        # 사람을 이유 없이 무시하게 된다.**
        self._returning_home: bool = False
        # 수락 후 제자리 회전량. 0.0 이면 회전 없이 예전처럼 바로 끝낸다.
        self.approach_turn_yaw_rad = approach_turn_yaw_rad
        # 호출 접근. 마이크 각도 증가 방향(+1 반시계 / -1 시계)은 장비 실측값
        # 이고 노드가 파라미터로 넣어 준다.
        self.wake_doa_sign = wake_doa_sign
        self.seek_look_sec = seek_look_sec
        # 근접 호출 임계값. 근거는 위 상수 정의에 있다.
        self.near_call_max_m = near_call_max_m
        self.near_call_no_spin_m = near_call_no_spin_m
        # 홈 복귀 재개. 근거는 RETURN_RESUME_SEC 주석에 있다.
        self.return_resume_sec = return_resume_sec
        # 핸들 쪽 호출 사각지대(위 SEEK_MIN_YAW_RAD 의 거울쌍). 근거는
        # HANDLE_SIDE_MIN_YAW_RAD 주석에 있다. 실기에서 폭을 조정한다.
        self.handle_side_min_yaw_rad = handle_side_min_yaw_rad
        # 온보딩 뒤 빈손 되묻기 사다리의 "leaving" 단 대기시간. 근거는
        # DEST_RETRY_RETURN_SEC 주석에 있다.
        self.dest_retry_return_sec = dest_retry_return_sec
        # on_wake/on_return_brake 가 실제로 상태를 바꾼 시각. on_wake_doa 가
        # 이 시각으로부터 WAKE_CONSUMED_GUARD_SEC 이내면 거절한다 — 두 토픽의
        # 도착 순서와 무관하게 결과가 같아지게 하려는 것이다.
        self._wake_consumed_at: Optional[float] = None
        # 접근 회전이 끝나 사용자가 손잡이를 받아든 것으로 보는 만료 시각
        # (USER_ATTACHED_SUPPRESS_SEC). on_wake_doa 가 이 값이 살아 있는 동안
        # 거절한다. _to_idle() 은 이 값을 비우지 않는다 — 회전 완료가 곧장
        # _to_idle() 을 부르므로 거기서 지우면 억제가 걸리기도 전에 사라진다.
        # on_wake 는 이 값이 이미 살아 있을 때만 now + USER_ATTACHED_SUPPRESS_SEC
        # 로 되감는다 — 사용자가 계속 말을 거는 동안은 대화가 이어지는 한
        # 막힌 채로 있고, 말이 없으면 이 값을 지나는 순간 자연히 풀린다.
        # 만료는 시간 비교(now 와의 대소)만으로 판정하므로 별도로 None 처리할
        # 지점이 필요 없다 — 지울 곳을 하나라도 놓치면 억제가 예상보다 오래
        # 남거나 일찍 사라지는 실수가 생기는데, 그 실수 자체를 없앤 것이다.
        self._user_attached_until: Optional[float] = None
        # 원래 자세로 돌아가기 위해 돌아야 할 누적 각도. 탐색 창 중에 다시
        # 부르면 또 돌므로 덮어쓰지 않고 더한다. None 이면 지금이 복귀 회전이다.
        self._seek_return_yaw: Optional[float] = None
        # 회전을 마치고 사람을 찾는 창의 만료 시각. 이 값이 살아 있는 동안
        # 상태는 IDLE 이다 — 접근 관문을 건드리지 않으려는 설계다.
        self._seek_deadline: Optional[float] = None
        self._turn_deadline: Optional[float] = None
        # 홈 복귀 재개(2026-09-10). "복귀가 끊겨 있다"는 사실과 "언제 재개할지"
        # 를 따로 든다 — 탐색 회전(SEEKING)이 끼어들어도 사실은 살아남아야
        # 하고, 시각은 회전이 끝나 IDLE 로 돌아온 시점 기준으로 다시 잡아야
        # 한다. _return_interrupted 는 _to_idle() 이 지우지 않는다(그게
        # 이 기능의 요점이다) — 지우는 자리는 on_intent·on_app_destination
        # (새 목적지로 주행 시작)과 on_return_home_request(관리자 직접 복귀
        # 명령), 그리고 이 사다리 자신이 실제로 복귀를 재개하는 순간뿐이다.
        self._return_interrupted: bool = False
        self._return_resume_deadline: Optional[float] = None
        self._return_notice_given: bool = False
        # 온보딩 뒤 빈손 되묻기 사다리(2026-09-11). stage 는 "asked" →
        # "retried" → "leaving" → "notice" 순으로 전진하며, None 이면 사다리가
        # 안 걸려 있다는 뜻이다. _arm_dest_prompt/_advance_dest_prompt/
        # _forget_dest_prompt 참고.
        self._dest_prompt_stage: Optional[str] = None
        self._dest_prompt_deadline: Optional[float] = None
        # 근접 호출로 들어온 AWAITING_USER 에서 수락해도 회전을 생략할지.
        # on_person_detection 이 거리로 정하고 on_approach_answer 가 소비한다.
        # AWAITING_USER 로 새로 들어올 때마다(정상 접근·근접 호출 두 진입점
        # 모두) 다시 명시적으로 정해지므로 묵은 값이 남을 자리가 없다.
        self._near_call_no_spin: bool = False
        # 이 AWAITING_USER 에 "걸어서 다가간 적이 없는가"(Ruling 10, I-1).
        # 핸들 쪽 호출(on_wake_doa)만 세운다 — 정상 접근·근접 호출은 걸어서건
        # 코앞이건 실제로 그 사람 쪽으로 움직였으므로 물러날 여지가 있다.
        # 핸들 쪽은 사람이 이미 손잡이 자리(로봇 뒤)에 서 있어 물러날 곳이
        # 없다: 거절·무응답에 복귀 주행을 걸면 출발 회전이 손잡이로 그
        # 사람을 훑는다. _near_call_no_spin(회전 생략 여부)과는 뜻이 달라
        # 겹쳐 쓰지 않는다.
        self._never_approached: bool = False

        # ── 손잡이 터치 × 진동 (2026-09-30, 상수 절 주석 참고) ─────────────
        self.grip_enter_window_sec = grip_enter_window_sec
        self.grip_ratio = grip_ratio
        self.grip_wait_timeout_sec = grip_wait_timeout_sec
        self.grip_hint_pulse_sec = grip_hint_pulse_sec
        self.grip_release_grace_sec = grip_release_grace_sec
        self.grip_resume_window_sec = grip_resume_window_sec
        self.handle_lost_repeat_sec = handle_lost_repeat_sec
        self.handle_lost_give_up_sec = handle_lost_give_up_sec
        # [시연 스위치 2026-10-05] 터치 모듈이 고장 나(손을 안 대도 "잡음"을 낸다) OUT 선을
        # 빼고 시연한다. 켜면 ① 잡기 대기가 시간을 다 채우면 "잡았다"로 처리해
        # 확인 진동(tick)과 온보딩으로 넘어가고, ② 출발 때 활성 모드(손 놓침 정지)를
        # 쓰지 않는다 — 선이 빠져 늘 "놓음"이라 켜 두면 출발 0.5 s 만에 선다.
        # 터치 모듈을 고치면 launch 의 grip_assume_held 를 false 로 되돌린다.
        self.grip_assume_held = grip_assume_held
        # 접촉 사실은 노드가 /vica/smart_handle_state 로 넣어 준다(on_handle_state).
        self._grip = GripMeter(stale_sec=handle_state_stale_sec)
        # 지금 주행이 활성 모드(손을 놓으면 섬)인가. 출발 순간에 정한다.
        self.handle_active: bool = False
        # 이 사용자가 잡기 대기를 통과했는가. 온보딩 → 목적지 확인 → 출발까지
        # 이어지는 한 사람의 안내 동안 유효하다(_decide_handle_mode). _to_idle 은
        # 지우지 않는다 — 확인 시간초과처럼 IDLE 을 거쳐 다시 목적지를 말하는
        # 사람도 같은 사람이다. 대신 60초 자물쇠(user_attached_guard_active)가
        # 살아 있을 때만 믿는다 — 떠난 뒤 다른 사람이 부른 안내에 번지지 않게.
        self._handle_engaged: bool = False
        # 잡기 대기(IDLE 의 하위 단계). 시작 시각이 None 이 아니면 대기 중이다.
        self._grip_wait_since: Optional[float] = None
        self._grip_pulse_at: Optional[float] = None
        # 지금의 PAUSED 가 손 놓침 때문인가. "잠깐"으로 선 PAUSED 는 쥐고 있어도
        # 자동 출발하지 않는다 — 자동 재출발은 이 값이 True 일 때만이다(설계 4.3 (다)).
        self._handle_pause: bool = False
        self._handle_lost_since: Optional[float] = None
        self._handle_lost_notice_at: Optional[float] = None
        # 걸림을 말로 알렸는가 — 침묵 걸림(정지 중)은 해제도 침묵한다.
        self._estop_announced = False

        # 도착 후 대화 (arrival-dialog-flow). 꺼져 있으면 도착 후 기존 dwell→
        # idle 로 간다 — 실기 검증 전까지 기본 off, 노드가 파라미터로 켠다.
        self.arrival_dialog = arrival_dialog
        self._asking_is_finish = False   # 방금 던진 질문이 종료형인가("네"=끝)
        self._asking_time_after_yes = False  # 대기 수락 시 시간을 물을 유형인가
        # 이번 주행의 출처가 앱(관리자)인가. 앱 주행 도착엔 대화·홈 복귀를
        # 붙이지 않는다 (2026-08-31 사용자 결정 — 관리 중 소음 제거).
        self._nav_from_app = False
        self._asking_entered_at: Optional[float] = None  # 시계 유실 폴백 기준
        self._arrival_retried = False    # 무응답 재질문을 이미 한 번 했나
        self._asking_question = ""       # 지금 던져 둔 도착 질문(침묵 시 같은 질문을 다시 묻는다)
        self._deny_reconfirmed = False   # 대기형 질문의 거절을 종료형으로 되물었나 (2026-09-20)
        self._asking_where = False       # 도착 질문을 "네, 어디로 모실까요?"로 바꿔 물었나 (2026-10-08)
        self._leaving_deadline: Optional[float] = None   # 떠나기 예고 유예
        self._wait_until: Optional[float] = None         # WAITING 만료 시각
        self._wait_minutes_requested = -1    # 대장(P1): 대기 요청 분. WAITING 밖에서는 -1
        # ── 대기 장소 (2026-10-07) ─────────────────────────────────────────
        # 로봇의 지금 방향(도, map). 노드가 /amcl_pose 로 넣어 준다. 도착 멘트 M1 계산용.
        self.robot_yaw_deg: Optional[float] = None
        # 마지막 도착 기준 입구 방향 말("앞"/"뒤"/"오른쪽"/"왼쪽"). 상황판 door_side.
        self.door_side: str = ""
        # 도착 후 대화·대기의 목적지. _ask_arrival 이 active_destination 을 비우므로 따로 든다.
        self._arrived_destination: Optional[Destination] = None
        # 마지막으로 사용자와 도착한 목적지(2026-10-08 결정 1). 홈으로 떠나도 남긴다 — 홈 가는
        # 중의 "기다려"는 이 목적지 대기 장소로 간다. 홈 도착·새 안내·안내 뒤가 아닌 복귀·비상에서 지운다.
        self._last_guided: Optional[Destination] = None
        # 도착 질문에 답이 없어 떠났을 때 그 질문이 종료형이었나 — 늦게 온 "네·아니요"의 뜻이다.
        # None 이면 늦은 답을 받을 질문이 없다(다 됐어·대기 만료로 떠났거나 이미 세웠다).
        self._late_answer_finish: Optional[bool] = None
        # 지금 대기가 어디서인가: "spot"(대기 장소) / "destination"(막혀 목적지) / ""(제자리).
        self._wait_place: str = ""
        self._beacon_next_at: Optional[float] = None   # 다음 M3 시각
        # 로봇의 지금 위치(map). 노드가 /amcl_pose 로 넣어 준다. 홈 알림 판정용.
        self.robot_pose: Optional[Pose2D] = None
        self._home_beacon_next_at: Optional[float] = None   # 다음 홈 알림 시각
        self._release_text: str = ""                   # M2 문장 — 재생 완료 대조
        self._release_spoken_at: Optional[float] = None
        self._release_entered_at: Optional[float] = None
        self._wait_finish_asked_at: Optional[float] = None   # 대기 중 "어디로 모실까요?" 시각
        self._wait_need_asked_at: Optional[float] = None     # 대기 중 "안내가 필요 없으신가요?" 시각
        self._wait_finish_reasked = False   # 대기 중 두 질문을 이미 다시 물었나 (2026-10-08)
        self._wait_need_reasked = False
        # 대기 중에 목적지 '제안'이 와서 확인 질문(CONFIRMING)으로 들어갔을 때 돌아갈
        # 대기 상태. 거절·시간초과·호출이면 이 상태로 되돌아간다(대기 시간·장소 유지).
        self._wait_hold: Optional[State] = None
        # 주행 중 목적지 바꾸기(2026-10-07, 작업 계획 탭 '주행 중 목적지 변경 흐름').
        # 안내 주행 중 다른 목적지 제안이 오면 멈추고 확인 질문(CONFIRMING)으로 들어가며
        # 원래 가던 목적지를 여기 든다. 확정되면 새 목적지로, 거절·무응답·호출이면 이
        # 목적지로 다시 출발한다(_fold_confirming). 대기 보류(_wait_hold)와 같은 틀이다.
        self._change_from: Optional[Destination] = None
        # 접근 질문·돌아서기 중에 사용자가 말한 목적지(2026-10-08 반응표). 손잡이를 내준 뒤
        # 온보딩 대신 이 목적지로 확인 질문을 한다. 그때 쓰고 비운다.
        self._approach_dest: Optional[Destination] = None
        # 지금 확인 질문이 그 접근 목적지에서 왔나(2026-10-09 검토 I-7). 그러면 아니요·무응답에
        # 끝내지 않고 온보딩 질문으로 어디 갈지 다시 묻는다. 확인 질문에 들어갈 때마다 새로 정한다.
        self._confirm_from_approach = False
        # 지금 확인 질문의 문장(2026-10-08 반응표). "다시 가자"·다시 묻기에서 같은 질문을 한다.
        self._confirm_prompt = ""
        # 다시 묻기(2026-10-08): 확인 질문·취소 확인을 다시 물을 시각과 이미 다시 물었는지.
        self._confirm_reask_at: Optional[float] = None
        self._confirm_reasked = False
        self._cancel_reask_at: Optional[float] = None
        self._cancel_reasked = False
        # 지금 안내 주행의 행동 트리 종류. 재시도·재개가 같은 트리를 쓰게 한다.
        self._nav_tree: str = NAV_TREE_DEFAULT
        # 귀 상태 (/vica/listen_state). 무응답 판정 전에 귀 사정을 본다.
        self._ear_busy = False
        self._ear_busy_since: Optional[float] = None
        self._ear_grace_until: Optional[float] = None
        # 사용자가 실제로 말을 시작했는가("speech") — 창이 열렸을 뿐("open")
        # 인 상태와 구분한다. 온보딩 되묻기 사다리의 시계는 이것만 본다.
        self._ear_speaking = False
        self._ear_speech_since: Optional[float] = None

        self.state: State = State.IDLE
        self.estop_active: bool = False
        self.active_destination: Optional[Destination] = None
        # 일시정지로 보관한 목적지. active_destination 은 _to_idle/_enter_estopped 에서
        # 비워지므로 재개할 목적지는 따로 들고 있어야 한다.
        self.paused_destination: Optional[Destination] = None
        # 보관한 목적지가 관리자 홈 복귀였는가(2026-09-03). 재개할 때 일반 안내
        # 주행(NAVIGATING)이 아니라 복귀(RETURNING)로 돌아가야 끝날 때
        # return_home_* 이벤트가 나간다 — 앱의 배송 카드는 홈 복귀 중에 그
        # 이름만 보고 '완료'로 넘어간다. 일반 주행으로 재개하면 도착이
        # goal_succeeded 로 나가 카드가 영영 '홈 복귀 중'에 남는다.
        self._paused_returning_home: bool = False
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
        # 접근 질문을 이번 접근에서 다시 물은 횟수(2026-10-09). APPROACH_REASK_MAX 를 다 쓴 뒤의 비답은 물러남이다.
        self._approach_reasks = 0
        # 질문에 LLM 이 답하는 중 — (그 답의 글자, 재생 끝 소식을 기다리는 상한 시각). 끝나면 다시 묻는다.
        self._approach_after_reply: Optional[tuple] = None
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

    # -- 음성 요청 반응 (미션 요청 반응표, 2026-10-08) -------------------------
    def on_voice_intent(
        self,
        intent: IntentData,
        now: float,
        lookup: Callable[[str], Optional[Destination]],
        bounds: Optional[MapBounds] = None,
        nav_ready: bool = True,
    ) -> list:
        """음성 요청 하나에 대한 반응. 노드는 나온 동작을 실행하고 기록만 한다.

        갈래(라우팅)는 2026-10-08 노드 _on_intent 에서 이리로 옮겼다 — 반응표(상태 18 ×
        요청 11)를 ROS 없이 순수 시험으로 전수 확인하려고. 호출 반응표(on_wake_call)와
        같은 방식이다. lookup 은 목적지 id → Destination(없으면 None)이다.
        설계: docs/superpowers/specs/2026-10-08-mission-request-reactions-design.md
        """
        reacted = self._react_by_table(intent, now, lookup, bounds, nav_ready)
        if reacted is not None:
            return reacted
        return self._route_voice_intent(intent, now, lookup, bounds, nav_ready)

    def _react_by_table(
        self,
        intent: IntentData,
        now: float,
        lookup: Callable[[str], Optional[Destination]],
        bounds: Optional[MapBounds],
        nav_ready: bool,
    ) -> Optional[list]:
        """반응표로 새로 정한 칸(2026-10-08). 맡지 않는 칸은 None — 옛 갈래가 처리한다."""
        if self.cancel_confirm_pending and intent.intent in ("affirm", "deny"):
            # "안내를 취소할까요?"의 네·아니요 — 예전엔 접근 질문 배선으로 가서 버려졌다.
            return self.on_cancel_confirm_answer(intent.intent == "affirm", now)
        if self.cancel_confirm_pending and intent.intent == "unknown":
            # 취소 확인에 못 알아들은 답 — 한 번 다시 묻고, 그 뒤는 흘려보낸다(다시 묻기).
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
        """홈 가는 중의 "기다려" — 직전 목적지 대기 장소로 가서 기다린다(2026-10-08 결정 1).
        대기 장소가 없으면 그 목적지 입구 앞으로, 직전 목적지가 없으면 그 자리에서 기다린다."""
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
        """안내 없음. 홈 가다 "비카야"로 세운 뒤(복귀 재개 사다리)면 기다려 = 직전 목적지 대기
        장소로(결정 1), 다 됐어·다시 가 = "안내를 종료합니다" 하고 바로 홈. 그냥 쉬는 중의
        기다려·다 됐어 = 이유를 말한다(규칙 1). 손잡이 잡기·온보딩 질문 중은 다루지 않는다."""
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
        """확인 질문. "네"만 출발이고 기다려·다 됐어·취소는 "아니요"와 같다 — 대기·주행 중
        바꾸기 질문이면 하던 대기·주행으로 돌아간다(규칙 1). 주행 중 바꾸기 질문의 다 됐어는 안내
        전체를 그만둘지 되묻는다. 잠깐 = "네?" 하고 질문을 그대로 둔다. 주행 중 바꾸기
        질문의 잠깐·기다려는 일시정지다(기다려는 2026-10-09 사용자 결정). 접근에서 온 확인
        질문의 다 됐어·취소는 그만두는 말이라 어디 갈지 다시 묻지 않고 접는다."""
        kind = intent.intent
        change = self._change_from is not None
        if kind == "finish" and change:
            return self._voice_mission_command("cancel", now, nav_ready)
        if kind == "wait" and change:
            # "기다려"도 "잠깐"처럼 선 채로 둔다 — 원래 목적지는 보관하고 "다시 가자"를 기다린다
            # (2026-10-09 사용자 결정, 검토 M-5. 전엔 아니요처럼 원래 목적지로 다시 출발했다).
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
            # 물어 둔 질문에 "다시 가자" — 출발해도 되는지 같은 질문으로 다시 묻는다.
            self._arm_confirm(now)
            return [self._ask(self._confirm_prompt)]
        if kind == "unknown":
            # 못 알아들은 답 — 같은 질문을 한 번 다시(2026-10-08 다시 묻기). 그 뒤의 이상한
            # 답은 흘려보내고 시간이 다 되면 지금처럼 접는다. LLM 이 되묻는 중(clarify)이면
            # 끼어들지 않는다 — 그 대화의 답이 곧 온다.
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
        """안내 주행. 기다려 = 잠깐과 같이 멈춘다, 다 됐어 = 취소처럼 되묻는다, 다시 가자 =
        이미 가는 중이라고 답한다(규칙 1, 옛 말은 "다시 출발할 안내가 없습니다")."""
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
        """일시정지. 잠깐·기다려 = 선 채로 "잠시 멈추겠습니다…"를 다시(규칙 1, 옛 말은 "안내
        중이 아닙니다"). 손 놓침 정지는 on_pause_request 가 보통 정지로 바꾼다. 다 됐어 =
        취소처럼 되묻는다."""
        if intent.intent == "finish":
            return self._voice_mission_command("cancel", now, nav_ready)
        if intent.intent in ("pause", "wait"):
            actions, reason = self.on_pause_request(now)
            if reason == GateReason.OK:
                return actions
            return [Say(MSG_PAUSED, priority="response")]
        return None

    def _react_asking(self, intent, now, lookup, bounds, nav_ready) -> Optional[list]:
        """도착 질문·시간 질문. 잠깐 = "네?" 하고 질문 유지 — 다시 묻기 기회를 쓰지 않는다.
        다시 가자 = "네, 어디로 모실까요?"로 묻는다(규칙 1, 옛 말은 알아듣고도 "잘 듣지
        못했습니다…"). 그 질문의 아니요 = 안내 종료, 네 = 못 알아들은 답이다."""
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
            # "몇 분쯤 걸리실까요?"에 "네"만 — 같은 질문을 한 번 다시 묻는다(결정 2). 다시 물은
            # 뒤에도 "네"면 기다려 달라는 뜻이 분명하니 기본 30분을 기다린다.
            if not self._arrival_retried:
                self._arrival_retried = True
                self._response_deadline = None
                self._asking_entered_at = now
                return [self._ask(MSG_ASK_WAIT_TIME)]
            return self._enter_waiting(WAIT_MINUTES_CAP, now, default_msg=True)
        if self.state == State.ASKING_WAIT_TIME and kind == "deny":
            # "몇 분쯤 걸리실까요?"에 "아니(기다리지 마)" — 끝낼지 한 번 확인한다.
            return self._ask_end_confirm(now)
        return None

    def _ask_end_confirm(self, now: float) -> list:
        """끝낼지 한 번 확인한다 — "여기까지 안내를 마칠까요?"(종료형). 네 = 종료·홈, 아니요 =
        대기, 침묵 = 같은 질문 한 번 더 뒤 떠나기 예고(도착 질문의 무응답 사다리 그대로)."""
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
        """손 놓기 기다림·대기 중. 기다려 = 시간을 지금부터 다시 세고 대기 안내를 다시 말한다.
        다시 가자 = 어디로 갈지 묻는다. 손 놓기 기다림의 잠깐 = "비카야"처럼 "네?" 하고
        듣는다 — 듣는 동안은 대기 장소로 떠나지 않는다(_ear_holds). 손 놓기 기다림의
        아니요·취소 = 끝낼지 확인. 대기 중 취소 = "안내가 필요 없으신가요?"(결정 3). 대기 중
        "어디로 모실까요?" 뒤 30초 안의 아니요 = 종료."""
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
                # 대기 안내(M2) 바로 뒤 "아니, 기다리지 마"·"취소" — 끝낼지 한 번 확인한다.
                return self._ask_end_confirm(now)
            return None
        if self._wait_need_asked_at is not None and kind in ("affirm", "deny", "cancel", "finish"):
            # "안내가 필요 없으신가요?"의 답(결정 3). 부정 질문이라 네(필요 없다)·다 됐어·두 번째
            # 취소 = 종료·홈, 아니요(필요하다) = 계속 기다린다.
            self._wait_need_asked_at = None
            if kind == "deny":
                return [Say(MSG_CANCEL_KEPT, priority="response")]
            self._reset_arrival_dialog()
            return [Say(MSG_FINISH, priority="response"), *self._go_home(now)]
        if self._wait_need_asked_at is not None and kind == "unknown":
            # 못 알아들은 답 — 한 번 다시 묻고, 또 그러면 계속 기다린다(다시 묻기).
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
            # "네, 어디로 모실까요?"에 "아니" — 갈 곳이 없다. "다 됐어"를 두 번 한 것과 같다.
            self._reset_arrival_dialog()
            return [Say(MSG_FINISH, priority="response"), *self._go_home(now)]
        if within and kind in ("unknown", "affirm"):
            # "어디로 모실까요?"에 못 알아들은 답·"네" — 한 번 다시 묻는다(다시 묻기).
            if not self._wait_finish_reasked:
                self._wait_finish_reasked = True
                self._wait_finish_asked_at = now
                return [self._ask(MSG_WAIT_FINISH_ASK)]
            return []
        return None

    def _wait_question_tick(self, now: float) -> list:
        """대기 중 미션 질문의 다시 묻기(2026-10-08). 15초 조용하면 한 번 더 묻는다. "안내가
        필요 없으신가요?"는 다시 물어도 조용하면 "안내를 계속하겠습니다." 하고 계속 기다린다."""
        if self._ear_holds(now):
            return []
        asked = self._wait_finish_asked_at
        if (asked is not None and not self._wait_finish_reasked
                and now - asked >= QUESTION_REASK_SEC):
            self._wait_finish_reasked = True
            self._wait_finish_asked_at = now    # 두 번째 "다 됐어"의 30초 창도 다시 연다
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
        """대기 중 "다시 가자" — 어디로 갈지 묻는다. "다 됐어"의 첫 질문과 같은 문장이지만
        두 번 말해도 안내를 끝내지 않는다(그건 "다 됐어"만)."""
        self._wait_finish_asked_at = now
        self._wait_finish_reasked = False
        self._wait_need_asked_at = None
        return [self._ask(MSG_WAIT_FINISH_ASK)]

    def _rewait(self, minutes: int, now: float) -> list:
        """대기 중 "기다려"·"20분 기다려" — 시간을 지금부터 다시 세고 대기 안내를 다시 말한다.
        시간을 안 말했으면 처음 정한 시간 그대로다."""
        if minutes is None or minutes <= 0:
            minutes = (self._wait_minutes_requested if self._wait_minutes_requested > 0
                       else WAIT_MINUTES_CAP)
        minutes = min(int(minutes), WAIT_MINUTES_CAP)
        self._wait_until = now + minutes * 60.0
        self._wait_minutes_requested = minutes
        # 새 대기 시간이 물어 둔 대기 질문("어디로 모실까요?"·"안내가 필요 없으신가요?")의 답이다.
        # 남기면 15초 뒤 같은 질문을 또 하고, 거기 "아니"면 홈으로 떠난다(2026-10-09 검토 I-3).
        self._wait_finish_asked_at = None
        self._wait_need_asked_at = None
        place = self.wait_place
        msg = (MSG_WAIT_SPOT_CONFIRM.format(minutes=minutes, place=place) if place
               else MSG_WAIT_CONFIRM.format(minutes=minutes))
        if self.state == State.WAITING_RELEASE:
            # 손 놓기 판정은 새 안내를 다 말한 뒤부터다(on_wait_speech_spoken 이 이 글자와 대조).
            self._release_text = msg
            self._release_spoken_at = None
            self._release_entered_at = now
        return [Say(msg, priority="response")]

    def _react_awaiting_user(self, intent, now, lookup, bounds, nav_ready) -> Optional[list]:
        """접근 질문. 목적지로 답하면("응, 화장실 가고 싶어") 수락으로 받고 그 목적지를 기억한다
        — 돌아서 손잡이를 내준 뒤 확인 질문으로 묻는다(규칙 1, 옛 동작은 버림·"다른 응대 중").
        취소 = 아니요(물러난다). 네·아니요는 옛 갈래(on_approach_answer)로 간다. 그 밖의 말(질문·못
        알아들음·잠깐·다시 가자·기다려·다 됐어)은 세 번까지 다시 묻고, 그 뒤면 물러난다(2026-10-09
        사용자 결정 — 옛 동작은 잠깐·다시 가자만 "네?", 나머지는 버려 8초 뒤 떠났다)."""
        if intent.intent == "navigate":
            self._approach_dest = self._approach_destination(intent, lookup, bounds, nav_ready)
            return self.on_approach_answer(True, now)
        if intent.intent == "cancel":
            return self.on_approach_answer(False, now)
        if intent.intent in ("affirm", "deny"):
            return None
        return self._approach_not_answered(getattr(intent, "reply", ""), now)

    def _react_turning(self, intent, now, lookup, bounds, nav_ready) -> Optional[list]:
        """돌아서는 중에 말한 목적지 — 기억했다가 손잡이를 내준 뒤 확인 질문으로 묻는다.
        도는 동안은 말을 얹지 않는다(옛 동작은 확정에 "지금은 다른 응대 중입니다")."""
        if intent.intent != "navigate":
            return None
        dest = self._approach_destination(intent, lookup, bounds, nav_ready)
        if dest is not None:
            self._approach_dest = dest
        return []

    def _approach_destination(self, intent, lookup, bounds, nav_ready) -> Optional[Destination]:
        """접근 대화 중 받은 목적지 — 갈 수 있는 곳만 기억한다. 못 가는 곳이면 None 이고,
        손잡이를 내준 뒤 평소 온보딩("어디로 가고 싶으신가요?")을 한다."""
        dest = lookup(intent.matched_destination_id)
        confirmed = replace(intent, need_confirm=False)
        if check_gate(confirmed, dest, bounds, self.estop_active, nav_ready) != GateReason.OK:
            return None
        return dest

    def _react_returning(self, intent, now, lookup, bounds, nav_ready) -> Optional[list]:
        """홈 복귀. 기다려 = 세우고 직전 목적지 대기 장소로(결정 1). 다 됐어 = 그대로 홈 + 한마디.
        답이 없어 떠난 뒤 늦게 온 네·아니요 = 그 질문의 뜻대로(대기형: 네 = 대기·아니요 = 홈,
        종료형: 반대). 잠깐 = "비카야"처럼 세우고 "네?"(규칙 1, 옛 말은 "안내 중이 아닙니다"로
        못 세웠다). 관리자 홈 복귀의 잠깐은 지금처럼 일시정지다."""
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
        # 복귀 주행 중 늦게 도착한 답 — 마지막 그물 (2026-08-30 실기: 무응답 오판으로
        # 떠난 직후 도착한 답이 버려져 세울 방법이 없었다). 복귀를 조용히 멈추고
        # (ASKING_NEXT) 아래 갈래가 그 뜻을 그대로 처리한다 — wait 는 대기, navigate
        # 제안은 확인 흐름.
        if self.state == State.RETURNING and kind in ("wait", "navigate"):
            actions.extend(self.on_return_brake(now, quiet=True))
        # 도착 후 대화 중이면 답을 on_arrival_answer 로 보낸다 — 같은 말이라도 이
        # 상태에선 뜻이 다르다(도착 후 cancel = 홈 복귀 등). 예외: 새 목적지 '제안'은
        # 대화를 닫고 아래 일반 확인 흐름(CONFIRMING)으로 합류한다(2026-08-30 실기).
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
        # 짧은 답(affirm/deny)은 어느 질문의 답인지 상태가 정한다. CONFIRMING 이면 확인
        # 질문의 답 — 목적지는 미션이 이미 안다(2026-08-31). 그 외에는 접근 질문 배선 —
        # AWAITING_USER 가 아니면 on_approach_answer 가 빈 목록을 돌려준다.
        if kind in ("affirm", "deny"):
            if self.state == State.CONFIRMING:
                dest = lookup(self.confirming_dest_id or "")
                return actions + self.on_confirm_answer(
                    kind == "affirm", dest, bounds, nav_ready, now)
            return actions + self.on_approach_answer(kind == "affirm", now)
        dest = lookup(intent.matched_destination_id)
        return actions + self.on_intent(intent, dest, bounds, nav_ready, now)

    def _voice_mission_command(self, kind: str, now: float, nav_ready: bool) -> list:
        """음성 취소·일시정지·재개(옛 노드 _on_voice_mission_command). 취소는 곧바로 하지
        않고 "취소할까요?"로 되묻는다 — 이미 되물은 상태의 두 번째 "취소"는 긍정이다.
        관문에 걸리면 이유를 말한다."""
        if kind == "cancel":
            if self.cancel_confirm_pending:
                return self.on_cancel_confirm_answer(True, now)
            actions, reason = self.on_cancel_confirm_request(now)
        elif kind == "pause":
            actions, reason = self.on_pause_request(now)
        else:
            actions, reason = self.on_resume_request(nav_ready, now)
        if reason != GateReason.OK:
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
        # LLM 이 뭐든 intent 를 만들어 왔다는 것 자체가 대화를 넘겨받았다는
        # 뜻이라 온보딩 되묻기 사다리부터 청산한다 — navigate 가 아닌 것도
        # 포함한다. LLM-unknown("안내와 관련된 요청이 아니에요")은 음성
        # 노드가 스스로 되묻고 재청취를 열므로, 미션까지 겹쳐 되물으면 두
        # 번 묻는 사고가 난다.
        self._forget_dest_prompt()
        if self._return_interrupted and self.state == State.IDLE:
            # 복귀가 끊긴 채 대화가 오갔다(되묻기·질문 등) — 무응답 시계를
            # 이 시각부터 다시 잰다. 예고를 이미 했으면 그것도 되돌린다
            # (2026-10-06 실기: clarify 직후 "응답이 없어…"로 떠남).
            self._return_resume_deadline = now + self.return_resume_sec
            self._return_notice_given = False
        if (intent.intent == "finish"
                and self.state in (State.WAITING, State.WAITING_RELEASE)):
            return self._wait_finish(now)
        if intent.intent != "navigate":
            # 질문/잡담 등은 LLM(reply)과 ros_tts_node 몫 — 여기선 관여하지 않는다.
            # 잡기 대기 중의 "손잡이 어디 있어요?"도 여기로 온다 — 대기는 그대로 둔다.
            return []
        if self._grip_wait_since is not None:
            # 잡기 전에 목적지부터 말했다. 대기를 접고 평소처럼 처리한다 —
            # 모드는 출발 순간에 정한다(_decide_handle_mode, 설계 4.3 (가)).
            self._grip_wait_since = None
            self._grip_pulse_at = None

        if self.state == State.ESTOPPED:
            return [Say(MSG_ESTOP_REJECT, priority="response")]

        if self.state in _WAIT_MOVING_STATES:
            # 대기 장소로 혼자 가는 중 — 대기 상태가 될 때까지 대답하지 않는다
            # (호출 반응표, 2026-10-07). 사용자는 도착 뒤 다시 부르면 된다.
            return []

        if self.state == State.NAVIGATING:
            # 안내 주행 중 새 목적지 — 멈추고 바꿀지 묻는다(2026-10-07 사용자 결정).
            # 옛 v1 정책(무조건 MSG_BUSY)은 관리자 주행에만 남는다.
            return self._navigating_destination_request(
                intent, dest, bounds, nav_ready, now)

        if self.state in _APPROACH_STATES:
            # 접근·질문·회전(TURNING)·탐색(SEEKING)·복귀 중에는 새 목적지를
            # 받지 않는다. 특히 AWAITING_USER 는 방금 던진 질문의 답을
            # 기다리는 구간이라, 그 자리에 다른 목적지를 끼워 넣으면 누구의
            # 요청인지 알 수 없게 된다(설계 4절). TURNING·SEEKING 은 진행 중인
            # SpinInPlace 를 취소하지 않은 채 Navigate 가 나가는 사고를 막는다
            # (설계 4절, 2026-09-10).
            # 제안(확인 질문 단계)에는 거절하지 않는다(2026-10-07 사용자 결정 '제안 단계
            # 침묵') — 음성 쪽이 이미 확인 질문을 했는데 그 위에 거절을 얹으면 두 목소리가
            # 연달아 나고, 로봇이 자기 거절을 다시 들었다(09-24 11:30 실기). 거절은 확정
            # 요청에만 한다.
            if intent.need_confirm:
                return []
            return [Say(MSG_APPROACH_BUSY, priority="response")]

        if intent.need_confirm:
            if (self.state == State.CONFIRMING
                    and self._confirming_dest_id
                    and intent.matched_destination_id == self._confirming_dest_id):
                # 확인 중인 그 목적지를 다시 말했다("응 화장실로 가자") —
                # 그건 답이다. LLM 이 재제안(confirm=True)으로 되돌려줘도
                # 확정한다 (2026-09-01, 야간 로그: 같은 확인 질문이 반복되다
                # 3수째에야 출발). 확인 답의 정식 통로로 보낸다 — 게이트
                # 포함 (need_confirm 그대로 흘리면 게이트가 거부한다).
                return self.on_confirm_answer(True, dest, bounds, nav_ready, now)
            else:
                # 확인 대기 시작/갱신. 확인 질문(confirm_prompt)은 LLM reply 로
                # ros_tts_node 가 이미 재생하므로 여기서 중복 발화하지 않는다.
                if self.state in _WAIT_STATES and self._wait_hold is None:
                    # 대기 중의 새 목적지 제안. 출발이 확정될 때까지 대기를 끝내지
                    # 않는다 — 거절·무응답이면 이 대기로 돌아간다(2026-10-07).
                    self._wait_hold = self.state
                self.state = State.CONFIRMING
                self._confirm_from_approach = False
                self._confirming_dest_id = intent.matched_destination_id or None
                self._arm_confirm(now)
                self._confirm_prompt = self._confirm_prompt_for(dest)
                return []

        # need_confirm == false (확정 요청)
        if (
            self.state == State.CONFIRMING
            and self._confirming_dest_id
            and intent.matched_destination_id != self._confirming_dest_id
        ):
            # 확인 질문 중 다른 목적지를 확정했다 — 새 목적지로 다시 묻는다(2026-10-08 사용자
            # 결정 4, "네, XX로 안내해드릴까요?"). 09-01 의 '멘트 없이 접기'는 규칙 1(꼭 한마디)
            # 로 바꿨다. 갈 수 없는 곳이면 이유를 말하고 묻던 질문은 그대로 둔다.
            reason = check_gate(intent, dest, bounds, self.estop_active, nav_ready)
            if reason != GateReason.OK:
                msg = _REJECT_MESSAGES.get(reason)
                return [Say(msg, priority="response")] if msg else []
            assert dest is not None  # check_gate 가 보장
            self._confirming_dest_id = dest.id
            self._arm_confirm(now)
            self._confirm_prompt = self._confirm_prompt_for(dest)
            return [self._ask(MSG_CONFIRM_SWITCH.format(prompt=self._confirm_prompt))]

        reason = check_gate(intent, dest, bounds, self.estop_active, nav_ready)
        if reason != GateReason.OK:
            folded: list = []
            if (reason == GateReason.NAV_NOT_READY and self._changing_destination()):
                # 주행 중 바꾸기 확정인데 Nav2 가 준비되지 않았다 — 원래 목적지로도 못
                # 간다. 일시정지로 두어 원래 목적지를 보관하고 "다시 가자"를 기다린다.
                self._hold_change_as_pause()
            elif self.state == State.CONFIRMING:
                folded = self._fold_confirming(now)
            elif self.state not in _WAIT_STATES:
                # 대기 중 확정 요청이 거절되면 대기를 그대로 둔다.
                self._to_idle()
            msg = _REJECT_MESSAGES.get(reason)
            # 거절을 먼저 말하고, 바꾸기 질문이었으면 그다음 원래 목적지로 다시 출발한다.
            return ([Say(msg, priority="response")] if msg else []) + folded

        assert dest is not None  # check_gate 가 보장
        if self.state in _WAIT_STATES or self._wait_hold is not None:
            # 대기 중에 목적지가 정해졌다 — 대기는 여기서 끝난다(시간·10초 알림 정리).
            self._reset_arrival_dialog()
        # 주행 중 바꾸기가 확정됐다 — 원래 주행은 질문할 때 이미 멈췄다(goal_paused).
        self._change_from = None
        # 새 안내다 — 옛 주행의 재시도 횟수·예약과 관리자 주행 표시를 넘겨받지 않는다
        # (2026-10-07 검토: 바꾸기·일시정지 뒤 확정은 _to_idle 을 거치지 않는다).
        self._nav_retry_count = 0
        self._retry_at = None
        self._nav_from_app = False
        self.state = State.NAVIGATING
        self.active_destination = dest
        # 사용자 안내 도착은 위치만 판정한다 — 방향 정렬 회전 없음(2026-10-07).
        self._nav_tree = NAV_TREE_GUIDED
        self.door_side = ""
        self.handle_active = self._decide_handle_mode(now)
        self._confirming_dest_id = None
        self._confirm_deadline = None
        self._announced_milestones = set()
        self._distance_baseline = None
        self._approach.reset()
        # 탐색 창(SEEKING)이 열린 채로 이 길을 타면 창이 살아남는다 — 이 길은
        # _to_idle() 을 거치지 않기 때문이다. 비우지 않으면 이번 안내가 끝나고
        # 한참 뒤 낡은 복귀각으로 갑자기 도는 사고가 난다(2026-09-10 재현).
        self._seek_deadline = None
        self._seek_return_yaw = None
        # 끊긴 복귀를 잊는다 — 이번이 그 재개다(음성으로 새 목적지를 받았으니
        # 사용자는 이미 응답한 것이다). 안 지우면 이번 안내를 마치고 한참 뒤
        # 낡은 복귀 사다리가 갑자기 홈으로 떠난다.
        self._forget_interrupted_return()
        # 새 안내다 — 지난 안내의 목적지·늦은 답은 넘겨받지 않는다(결정 1).
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
        """안내 주행 중 들어온 목적지 요청 — 멈추고 바꿀지 묻는다(2026-10-07 사용자 결정).

        옛 v1 정책은 무조건 MSG_BUSY 였다. 09-24 11:30 실기에서 음성의 확인 질문
        ("화장실로 안내해드릴까요?")과 이 거절이 연달아 나왔고, 로봇이 자기 거절을 다시
        들어 "안내를 취소할까요?"를 물었다. 이제는:

        - 제안(need_confirm): 질문은 음성 쪽이 이미 했다("지금 409호로 가는 중이에요.
          화장실로 바꿀까요?"). 미션은 말없이 멈추고 물은 목적지를 기억한다.
        - 확정(need_confirm=false): 아무도 묻지 않았다. 관문을 먼저 보고 못 가는 곳이면
          거절만 하고 하던 안내를 계속한다. 갈 수 있으면 멈추고 미션이 확인 질문을 한다.

        답은 CONFIRMING 의 정식 통로가 받는다 — 확정은 새 목적지로(on_intent 시작 블록),
        아니요·무응답·호출은 원래 목적지로 다시 출발(_fold_confirming).

        관리자 주행(원격·배송·가보기, _nav_from_app)은 바꾸지 않는다 — 손잡이 사용자가
        없고, 지나가던 사람이 배송을 가로채면 안 된다(지나가던 사람의 목적지 말은 이번
        범위 밖, 작업 계획 탭 '고려하지 않는 것'). 그때는 옛 정책에 '제안 단계 침묵'만 얹는다.
        """
        current = self.active_destination
        if self._nav_from_app or current is None:
            return [] if intent.need_confirm else [Say(MSG_BUSY, priority="response")]
        new_id = intent.matched_destination_id or ""
        if new_id and new_id == current.id:
            return []          # 이미 그리로 가는 중이다 — 하던 안내를 그대로 한다.
        ask: list = []
        if not intent.need_confirm:
            # 확정 요청은 관문부터 — 못 가는 곳(모르는 곳 포함)이면 거절만 하고 계속 간다.
            reason = check_gate(intent, dest, bounds, self.estop_active, nav_ready)
            if reason != GateReason.OK:
                msg = _REJECT_MESSAGES.get(reason)
                return [Say(msg, priority="response")] if msg else []
            assert dest is not None  # check_gate 가 보장
            prompt = (dest.confirm_prompt
                      or say_destination(MSG_CONFIRM_PROMPT_FALLBACK, dest.name))
            ask = [Say(prompt, priority="response", expects_reply=True)]
        elif dest is None or not new_id:
            # 미션이 모르는 곳의 제안 — 멈추지 않는다. 멈추면 확인 답이 와도 출발할 곳이
            # 없어 30초를 그냥 선다. 제안 단계라 말하지도 않는다.
            return []
        # 멈춘다. 일시정지와 같은 취소(goal_paused)라 앱이 '주행 끝'으로 오해하지 않고,
        # 원래 목적지는 _change_from 이 든다.
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
        # 앞서 물은 "안내를 취소할까요?"는 이 질문으로 대체됐다 — 남기면 새 목적지로
        # 출발한 뒤 "취소" 한마디가 되묻지 않고 바로 취소된다(2026-10-07 검토).
        self.cancel_confirm_pending = False
        self._cancel_confirm_deadline = None
        self._announced_milestones = set()
        self._distance_baseline = None
        self._approach.reset()
        return actions + ask

    def _wait_finish(self, now: float) -> list:
        """대기 중 "다 됐어"(finish) — 다음 목적지를 묻는다(2026-10-07 사용자 결정).

        돌아온 사용자의 "다 됐어"는 볼일이 끝났다는 말이다. 예전엔 미션도 LLM 도 말하지
        않아(finish 의 reply 는 계약상 빈 말) 대기와 10초 알림이 그대로 이어졌다. 대기는
        목적지가 정해져 출발할 때 끝나므로(호출 반응표) 여기서는 묻기만 한다. 물은 뒤
        WAIT_FINISH_REPEAT_SEC 안에 또 끝말이면 갈 곳이 없다는 뜻이라 같은 질문을 되풀이
        하지 않고 안내를 끝낸다 — 도착 질문의 finish 와 같은 길(MSG_FINISH → 홈).
        """
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
        """바꾸기 질문을 접고 일시정지로 둔다 — 원래 목적지를 보관해 "다시 가자"로 잇는다.

        "잠깐"이 왔거나 Nav2 가 준비되지 않아 원래 목적지로도 다시 출발할 수 없을 때다.
        로봇은 질문할 때 이미 섰다(goal_paused) — 새로 취소할 goal 이 없다.
        """
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
        """확인 질문(CONFIRMING)을 새 출발 없이 접는다. 질문 전에 하던 일로 돌아간다.

        - 대기 중에 들어온 질문: 그 대기로(시간·장소 그대로, 2026-10-07).
        - 안내 주행 중 바꾸기 질문: 원래 목적지로 다시 출발한다(2026-10-07 사용자 결정 —
          아니요·무응답이면 원래 목적지). announce 면 MSG_RESUMED 로 알린다. 호출("비카야")로
          접힐 때는 "네?"가 막 나가는 참이라 말을 보태지 않는다.
        - 그 밖: 평소처럼 IDLE.
        돌려주는 행동은 다시 출발할 때만 있다.
        """
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
                # 사용자 답에 대한 말이라 response — narration 은 6초 유통기한에 버려질 수 있다.
                actions.append(Say(say_destination(MSG_RESUMED, change_from.name),
                                   priority="response"))
            # 트리는 원래 주행 것 그대로다(_nav_tree 는 질문하는 동안 바뀌지 않는다).
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
        """접근에서 온 확인 질문의 아니요·무응답 — 안내를 끝내지 않고 온보딩 질문으로 어디 갈지
        다시 묻는다(2026-10-09 검토 I-7, 새 문장 없음). 다가가 손잡이까지 내준 사람이라 원하는 곳이
        따로 있을 수 있다. 그 뒤의 되묻기·떠나기는 온보딩 사다리가 맡는다."""
        self._confirm_from_approach = False
        self._fold_confirming(now)
        self._arm_dest_prompt(now)
        return [Say(MSG_APPROACH_ONBOARDING, priority="response", expects_reply=True)]

    @property
    def return_interrupted(self) -> bool:
        """복귀 재개 사다리가 도는 중인가. on_wake_doa 와 노드의 진단 로그가
        같은 값을 보게 하려고 공개한다(wake_guard_active 와 같은 이유)."""
        return self._return_interrupted

    def on_confirm_answer(
        self,
        affirmative: bool,
        dest: Optional[Destination],
        bounds: Optional[MapBounds],
        nav_ready: bool,
        now: float,
    ) -> list:
        """확인 질문("…로 안내해 드릴까요?")에 대한 네/아니오의 정식 통로.

        2026-08-31 실기 전까지 이 통로가 없어, "그래" 한마디를 LLM 이 대화
        기록으로 목적지를 추측해 확정 navigate 로 재구성해야만 출발했다.
        추측이 어긋나면 엉뚱한 곳으로 가고, affirm 으로 해석되면 접근 질문
        전용 배선이라 통째로 버려졌다(그날 밤 affirm 무시 4건·확인 타임아웃
        취소 7건). 확인 중인 목적지는 미션이 이미 알고 있으므로 여기서 직접
        확정한다 — LLM 이 목적지를 만들지 않는다는 원칙 그대로다.

        긍정이면 확정 요청과 완전히 같은 길(on_intent 의 게이트·시작 블록)을
        밟는다 — 확인 답이라고 게이트를 건너뛰면 안 된다(fail-closed).
        LLM 이 여전히 확정 navigate 를 보내는 경로도 그대로 살아 있다.
        """
        if self.state != State.CONFIRMING:
            return []
        if not affirmative:
            # "아니오" — 확인 중이던 요청을 접는다. 멘트는 타임아웃과 같은
            # 기존 문구를 쓴다(신규 멘트 금지, 2026-08-31 멘트 최소주의).
            # 대기 중에 들어온 질문이었으면 그 대기로 돌아간다. 주행 중 바꾸기
            # 질문이었으면 원래 목적지로 다시 출발하며 MSG_RESUMED 만 말한다 —
            # "안내 요청이 취소되었습니다"는 안내가 끝난 것으로 들린다.
            if self._change_from is not None:
                return self._fold_confirming(now)
            if self._confirm_from_approach:
                return self._fold_approach_confirm(now)
            self._fold_confirming(now)
            return [Say(MSG_CONFIRM_TIMEOUT, priority="response")]
        if dest is None or dest.id != (self._confirming_dest_id or ""):
            # 확인 중이던 목적지를 되찾지 못했다(id 미기록·저장소 갱신 등).
            # 아무 데나 출발하는 것보다 침묵이 낫다 — CONFIRMING 을 유지해
            # LLM 의 확정 navigate 나 타임아웃이 이어받게 둔다.
            return []
        confirmed = IntentData(
            intent="navigate",
            matched_destination_id=dest.id,
            need_confirm=False,
            safety_flag="normal",
        )
        return self.on_intent(confirmed, dest, bounds, nav_ready, now)

    # -- 취소 / 일시정지 / 재개 --------------------------------------------------
    #
    # 세 요청 모두 안전 사건이 아니라 목표 조작이다. E-stop 과 달리 래치도 reset 도
    # 없고, 처리 뒤 바로 새 요청을 받을 수 있다. 판정은 여기(Mission Manager)가 하고
    # 앱·LLM·CLI 는 요청만 보낸다.

    def _force_clear_all(self, now: float) -> list:
        """진행 중인 모든 활동을 강제 정리한다 (앱 선점·전체 취소 전용).

        상태별 정리가 흩어지면 하나를 빠뜨려 유령(낡은 타이머·보관 목적지)이
        생긴다 — 한곳에 모은다. goal 이 살아 있는 상태(_GOAL_ACTIVE_STATES) 는
        전부 CancelNav 를 낸다 — TURNING·SEEKING 의 spin 도 nav goal 과 같은
        Nav2 task 라 여기서 끊긴다(2026-09-10 재현: 예전엔 등록 목적지 상태만
        취소해 SEEKING 중 앱 선점이 spin 을 못 끊고 Navigate 와 함께 나가
        /cmd_vel_req 에 두 발행자가 붙었다). `CancelNav(None)` 이 안전한 것은
        on_emergency 의 E-stop 경로가 이미 증명한다.
        """
        # 말부터 끊는다 — 이후 붙는 새 멘트(취소 확인·새 안내 시작)가
        # 낡은 멘트 뒤에 줄 서지 않게 한다 (2026-09-01 큐 청소).
        actions: list = [StopSpeech(), SetNavSpeedLimit(NO_SPEED_LIMIT)]
        if self.state in _GOAL_ACTIVE_STATES:
            actions.append(CancelNav(self.active_destination))
        else:
            # 서 있던 안내(일시정지·주행 중 바꾸기 질문)를 끝낸다. 굴러가는 goal 이 없어
            # CancelNav 는 안 나가지만, 앱(일시정지 카드)과 대장(LLM 메모의 '안내 중')에는
            # 끝났다고 알려야 한다 — 안 알리면 둘 다 옛 안내를 붙들고 있었다(2026-10-07 검토).
            held = (self._change_from if self._changing_destination()
                    else self.paused_destination if self.state == State.PAUSED else None)
            if held is not None:
                actions.append(GoalEvent("goal_canceled", held))
        self._reset_arrival_dialog()
        self._to_idle()   # 보관 목적지·재시도 예약·확인 대기까지 전부 정리
        # 앱 선점·전체 취소 뒤의 주행은 손잡이 사용자의 안내가 아니다.
        self._handle_engaged = False
        # 끊긴 복귀 재개 사다리도 함께 청산한다(2026-09-10 사용자 결정) —
        # 앱 선점·취소는 사용자가 명시적으로 내린 지시라, 그 뒤 로봇이
        # 서 있는 것은 정당하다.
        self._forget_interrupted_return()
        return actions

    def on_app_destination(self, dest: Optional[Destination],
                           bounds: Optional[MapBounds], nav_ready: bool,
                           now: float, allow_private: bool = False,
                           is_delivery: bool = False) -> tuple:
        """앱(관리자) 새 목적지 — ESTOPPED 만 빼고 어느 상태든 선점한다.

        2026-08-31 사용자 결정: 관리자의 새 목적지는 내부적으로 전부 취소한
        뒤 즉시 출발한다. 멘트는 기존 출발 안내만 쓴다(신규 문구 없음 —
        사용자 결정). E-stop 래치만은 못 뚫는다(reset 경로 보호). 목적지
        품질 검증(공개·좌표·Nav2 준비)은 음성 게이트와 같은 기준이다.

        ``is_delivery`` 는 출발 멘트만 가른다 — 배송이면 "배달을 시작합니다",
        아니면 "안내를 시작합니다"(2026-09-03 사용자 승인). ``allow_private``
        와 지금은 늘 같이 켜지지만 뜻이 달라 인자를 나눠 둔다: 하나는 권한,
        하나는 말투다. 겸용하면 나중에 "배송인데 공개 목적지" 같은 경우에
        조용히 어긋난다.

        ``allow_private`` 는 **물류 배송 전용**이다(2026-09-02 사용자 결정).
        배송은 교수 사무실처럼 private 로 저장된 곳으로 가는 일이 많다. 이
        스위치는 `/vica/mission/request_delivery` 서비스만 켜며, 일반 주행
        요청과 음성 경로는 그대로 private 를 거부한다. 접근 가능 여부·좌표·
        Nav2 준비·E-stop 검사는 스위치와 무관하게 그대로다.
        """
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
        # 끊긴 복귀를 잊는다 — 관리자가 새 목적지로 선점했으니 이번이 그
        # 재개다(on_intent 와 같은 이유). _force_clear_all 이 한다.
        actions = self._force_clear_all(now)
        self.state = State.NAVIGATING
        # 배송은 입구 방향을 바라보고 선다(입구 화살표 하나를 같이 쓴다, 10-07 결정).
        # 원격 주행은 저장된 pose.yaw 그대로 — 앱이 저장할 때 같은 값을 넣는다.
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
        """관리자 '대기 장소로 가보기'(`/vica/mission/request_wait_spot`, 2026-10-07).

        등록이 맞는지 확인하는 이동이다 — 지도 설정 화면에서만 부른다('홈으로
        가보기'와 같은 자리). 안내 중에는 거절한다: 사용자가 손잡이를 잡고 있거나
        대기 중일 수 있다. 말하지 않는다(확인용 이동이라 새 멘트를 만들지 않는다).
        대기 장소와 같은 짧은 트리로 가서 나가는 방향까지 맞춰 선다.
        """
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
        actions = self._force_clear_all(now)
        self.state = State.NAVIGATING
        self.active_destination = wait_dest
        self._nav_from_app = True
        self._nav_tree = NAV_TREE_WAIT
        self.door_side = ""
        actions.append(Navigate(wait_dest, tree=NAV_TREE_WAIT))
        return actions, GateReason.OK

    def _voice_cancel_gate(self) -> GateReason:
        """음성 취소 게이트. 주행 중 바꾸기 질문으로 멈춘 동안(CONFIRMING)도 아직
        안내 중이라 취소할 수 있다(2026-10-07) — 아니면 "지금은 안내 중이 아닙니다"가 된다."""
        if self._changing_destination() and not self.estop_active:
            return GateReason.OK
        return check_cancel_gate(self.state, self.estop_active)

    def on_cancel_request(self, now: float) -> tuple:
        """음성 취소. 게이트(주행·일시정지·대기)를 지킨다 — 앱 취소는
        on_app_cancel. (actions, GateReason) 을 돌려준다."""
        reason = self._voice_cancel_gate()
        if reason != GateReason.OK:
            return [], reason
        actions = self._force_clear_all(now)
        actions.append(Say(MSG_CANCELED, priority="response"))
        return actions, GateReason.OK

    def on_app_cancel(self, now: float) -> tuple:
        """앱(관리자) 취소 — ESTOPPED 만 빼고 어느 상태든 전부 정리하고
        IDLE (2026-08-31 사용자 결정). 음성 취소와 달리 게이트가 없다:
        관리자 버튼은 오인식이 없고, 접근 응대·도착 질문도 끊을 권한이 있다.
        E-stop 래치만은 못 푼다(reset 경로 보호)."""
        if self.estop_active or self.state == State.ESTOPPED:
            return [], GateReason.ESTOP_ACTIVE
        if self.state == State.IDLE:
            # 끊긴 복귀 재개 사다리가 도는 중일 수 있다(2026-09-10 사용자
            # 결정) — 조용히 수락하는 척만 하고 사다리를 그대로 두면 18초
            # 뒤 로봇이 취소를 무시한 것처럼 보인다.
            self._forget_interrupted_return()
            return [], GateReason.OK   # 이미 대기 — 조용히 수락(멘트 최소주의)
        actions = self._force_clear_all(now)
        actions.append(Say(MSG_CANCELED, priority="response"))
        return actions, GateReason.OK

    def on_pause_request(self, now: float) -> tuple:
        """일시정지("잠깐"·앱). goal 을 취소하되 목적지는 보관해 재개할 수 있게 둔다.

        손 놓침으로 선 PAUSED 에서 "잠깐"이 오면 말로 재개하는 보통 일시정지로
        바꾼다 — 이제 쥐고 있어도 자동으로 출발하지 않는다(설계 4.3 (다)).
        """
        if self.state == State.PAUSED and self._handle_pause:
            self._clear_handle_pause()
            return [Say(MSG_PAUSED, priority="response")], GateReason.OK
        if self._changing_destination() and not self.estop_active:
            # 주행 중 바꾸기 질문으로 이미 서 있다 — 질문을 접고 일시정지로 바꾼다
            # (2026-10-07). 원래 목적지를 보관하므로 "다시 가자"로 이어 간다.
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
        # 홈 복귀를 멈춘 것이면 그 사실을 보관분 쪽으로 옮긴다. RETURNING 을
        # 떠나므로 현재 복귀 표시는 내리고, 재개할 때 _enter_returning 이 다시 켠다.
        self._paused_returning_home = (
            self.state == State.RETURNING and self._returning_home
        )
        self._returning_home = False
        self.state = State.PAUSED
        self.paused_destination = destination
        self.active_destination = None
        self.cancel_confirm_pending = False
        self._cancel_confirm_deadline = None
        # 재개하면 새 goal 이므로 거리 안내도 처음부터 다시 한다.
        self._announced_milestones = set()
        self._distance_baseline = None
        self._approach.reset()
        return actions, GateReason.OK

    def on_resume_request(self, nav_ready: bool, now: float) -> tuple:
        """다시 출발. 보관한 목적지로 새 goal 을 만든다.

        손 놓침으로 선 뒤의 재개는 두 길이다. 다시 잡아 자동으로(_handle_pause_tick)
        오면 활성 그대로 간다. 안 잡은 채 "다시 가자"로 오면 비활성으로 간다 —
        활성으로 두면 0.5초 만에 또 서는 반복이 된다(설계 4.3 (다)).
        """
        if self._changing_destination() and not self.estop_active:
            # 주행 중 바꾸기 질문 중의 "다시 가자" — 바꾸지 않고 원래 목적지로 다시
            # 출발한다(2026-10-07, 거절과 같은 길).
            if not nav_ready:
                return [], GateReason.NAV_NOT_READY
            return self._fold_confirming(now), GateReason.OK
        reason = check_resume_gate(
            self.state, self.paused_destination, self.estop_active, nav_ready
        )
        if reason != GateReason.OK:
            return [], reason
        # 다시 출발 = 취소하지 않겠다는 답이다 — 물어 둔 "안내를 취소할까요?"를 닫는다(2026-10-09
        # 검토 I-5). 남기면 다시 달리는 중 15초 뒤 그 질문을 또 한다.
        self.cancel_confirm_pending = False
        self._cancel_confirm_deadline = None
        if self._handle_pause:
            if not self._holding(now, self.grip_resume_window_sec,
                                 since=self._handle_lost_since):
                self.handle_active = False
            self._clear_handle_pause()

        destination = self.paused_destination
        assert destination is not None  # check_resume_gate 가 보장
        if self._paused_returning_home:
            # 홈 복귀를 이어간다. 관리자 홈 복귀는 출발도 말없이 나가므로
            # 재개도 말없이 간다(새 멘트 없음). 좌표는 지금의 홈을 다시 읽는다.
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
        """음성 취소 요청. 바로 취소하지 않고 사용자에게 되묻는다.

        확인을 기다리는 동안에도 주행은 계속된다. 확인 전에 goal 을 멈추면
        사용자가 "아니오"라고 답했을 때 되돌릴 수 없기 때문이다.
        """
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
        """"안내를 취소할까요?" 시계. 15초 조용하면 한 번 다시 묻고(2026-10-08 다시 묻기), 시간이
        다 되면 조용히 하던 대로 둔다(2026-09-01 감량, 취소하지 않는다). 예전엔 안내 주행 중에만
        시계를 봐서 일시정지·바꾸기 질문 중에 물은 확인은 끝나지 않았다."""
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
        self._approach_dest = None
        self.active_destination = destination
        self.approach_track_id = request.track_id
        self.approach_goal_pose = request.goal
        self._response_deadline = None
        self._announced_milestones = set()
        self._distance_baseline = None
        self._approach.reset()
        return (
            [
                # 소리가 움직임보다 먼저 — 다가오는 것이 무엇인지 먼저 알린다.
                Say(MSG_APPROACH_COMING, priority="response"),
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

        재생에 몇 초가 걸리는지는 TTS 만 알 수 있어 노드가 알려준다. 안 오면
        AWAITING_USER 진입 때 걸어 둔 안전망(APPROACH_QUESTION_STUCK_SEC)이
        탈출을 맡는다. tts_done 은 끊긴 발화에도 발행되므로(2026-08-31 수리)
        정상 경로에서는 반드시 온다.
        """
        if self.state != State.AWAITING_USER:
            return []
        self._response_deadline = now + self.approach_response_timeout_sec
        return []

    # -- 손잡이 터치 × 진동 (2026-09-30) -----------------------------------------

    def on_handle_state(self, contact: bool, fresh: bool, now: float) -> None:
        """/vica/smart_handle_state 한 건. 사실만 적고 판정은 on_tick 이 한다."""
        self._grip.update(now, contact, fresh)

    @property
    def dialog_state(self) -> str:
        """LLM 상황판에 내는 대화 단계(RobotState.dialog_state).

        대부분 state.value 그대로다. 두 곳만 더 잘게 가른다(설계 4.7) — 잡기
        대기(IDLE 의 하위 단계)와 손 놓침 PAUSED. is_paused 의 뜻은 그대로다.
        """
        if self.state == State.IDLE and self._grip_wait_since is not None:
            return DIALOG_GRIP_WAIT
        if self.state == State.PAUSED and self._handle_pause:
            return DIALOG_PAUSED_HANDLE
        return self.state.value

    def _holding(self, now: float, window: float, since: Optional[float] = None) -> bool:
        return (self._grip.fresh(now)
                and self._grip.ratio(now, window, since=since) >= self.grip_ratio)

    def _decide_handle_mode(self, now: float) -> bool:
        """음성으로 시작한 안내의 출발 순간 — 활성 모드인가(설계 4.3 (나)).

        잡기 대기를 통과한 사람이면 활성이다. 출발 순간 잠깐 손을 뗐더라도
        활성으로 두는 쪽이 안전하다 — 0.5초 뒤 서서 "다시 잡아주세요"라고
        말할 뿐, 시각장애인을 두고 떠나지 않는다. 대기를 거치지 않았으면
        출발 순간 쥐고 있는지만 본다. 센서가 끊겨 있으면 늘 비활성이다.
        """
        if self.grip_assume_held:
            return False   # 시연 스위치: 터치로 세우지 않는다(위 __init__ 주석)
        if not self._grip.fresh(now):
            return False
        if self._handle_engaged and self.user_attached_guard_active(now):
            return True
        return self._holding(now, GRIP_DEPART_WINDOW_SEC)

    def _start_grip_wait(self, now: float) -> list:
        """수락 뒤 손잡이를 내준 순간(회전 완료·실패·생략). 힌트 + 진동.

        온보딩("어디로 가고 싶으신가요?")은 아직 말하지 않는다 — 잡거나
        시간이 다 되면 _finish_grip_wait 가 말한다(D2). 센서가 없거나 끊겨
        있으면 기다릴 수단이 없으니 곧장 비활성으로 온보딩한다 — touch_enabled
        false 인 로봇에서는 09-11 흐름(힌트 → 진동 → 온보딩)과 같아진다.
        """
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
            # 접근 질문에 목적지로 답했다 — 온보딩 대신 그 목적지를 확인한다(2026-10-08 반응표).
            self.state = State.CONFIRMING
            self._confirm_from_approach = True
            self._confirming_dest_id = approach_dest.id
            self._arm_confirm(now)
            self._confirm_prompt = self._confirm_prompt_for(approach_dest)
            actions.append(self._ask(self._confirm_prompt))
            return actions
        # 온보딩 질문을 던지는 자리 — 빈손 되묻기 사다리를 켠다.
        self._arm_dest_prompt(now)
        actions.append(Say(MSG_APPROACH_ONBOARDING, priority="response",
                           expects_reply=True))
        return actions

    def _grip_wait_tick(self, now: float) -> list:
        since = self._grip_wait_since
        assert since is not None
        if not self._grip.fresh(now):
            # 대기 중 상향이 끊겼다 — 잡아도 알 길이 없다. 비활성으로 넘긴다.
            return self._finish_grip_wait(now, engaged=False)
        if self._holding(now, self.grip_enter_window_sec, since=since):
            return self._finish_grip_wait(now, engaged=True)
        if now - since >= self.grip_wait_timeout_sec:
            # 시연 스위치가 켜져 있으면 못 잡았어도 잡은 것으로 넘어간다.
            return self._finish_grip_wait(now, engaged=self.grip_assume_held)
        if (self._grip_pulse_at is not None
                and now - self._grip_pulse_at >= self.grip_hint_pulse_sec):
            self._grip_pulse_at = now
            return [Haptic(HAPTIC_PATTERN_HANDLE_HINT)]
        return []

    def _handle_nav_tick(self, now: float) -> list:
        """활성 주행 중 손 놓침·상향 두절을 본다. NAVIGATING·주행 중에만 부른다."""
        if not self._grip.fresh(now):
            # 두절은 놓침이 아니다 — 세우면 다시 잡아도 알 길이 없어 영원히
            # 못 간다. 비활성으로 내리고 계속 간다(4.3 (라)). 손잡이 당김
            # 감속(knob)은 CAN 경로라 그대로 살아 있다.
            self.handle_active = False
            return [Say(MSG_HANDLE_UNAVAILABLE, priority="response")]
        if self._grip.released_for(now) < self.grip_release_grace_sec:
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

    def _handle_pause_tick(self, now: float, nav_ready: bool) -> list:
        """손 놓침으로 선 PAUSED. 다시 잡으면 출발, 오래 안 잡으면 끝낸다."""
        lost_since = self._handle_lost_since
        assert lost_since is not None
        if not self._grip.fresh(now):
            # 서 있는 중에 상향이 끊겼다. 다시 잡아도 알 길이 없으므로 말로
            # 재개하는 보통 일시정지로 바꾼다. 움직이지는 않는다 — 손을 놓친
            # 사람을 두고 떠나면 안 된다. 멘트는 기존 일시정지 문구다.
            self.handle_active = False
            self._clear_handle_pause()
            return [Say(MSG_PAUSED, priority="response")]
        if self._holding(now, self.grip_resume_window_sec, since=lost_since):
            actions, reason = self.on_resume_request(nav_ready, now)
            if reason != GateReason.OK:
                return []   # Nav2 준비 전이면 다음 tick 에 다시 본다
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
        """질문을 던지고 AWAITING_USER 로 들어간다.

        걸어서 도착한 정상 접근(on_tick 의 APPROACHING→SUCCEEDED), 코앞이라
        걸어가지 않는 근접 호출(on_person_detection), 핸들 쪽에서 와 카메라
        확인도 없이 곧장 들어오는 호출(on_wake_doa) 셋 다 여기로 온다 — 질문
        멘트·재청취·탈출용 안전망(APPROACH_QUESTION_STUCK_SEC)이 세 경로에서
        완전히 같기 때문이다. 회전 여부(_near_call_no_spin)는 호출부가 이 함수
        호출 전후로 각자 정한다 — 여기서는 다루지 않는다.
        """
        self.state = State.AWAITING_USER
        # 여기서는 탈출용 안전망만 건다. 진짜 응답 8초는 질문 재생이 끝난
        # 시점(on_approach_question_spoken)부터 — 도착 후 대화와 같은 방식.
        # 큐 시각 기준 8초는 창이 0초가 되는 결함이었다.
        self._response_deadline = now + APPROACH_QUESTION_STUCK_SEC
        self._approach.reset()
        self._approach_reasks = 0
        self._approach_after_reply = None
        return [
            SetNavSpeedLimit(NO_SPEED_LIMIT),
            Say(MSG_APPROACH_QUESTION, priority="response", expects_reply=True),
        ]

    def _approach_not_answered(self, reply: str, now: float) -> list:
        """접근 질문에 예·아니요가 아닌 말(질문·못 알아들음·잠깐 등, 2026-10-09 사용자 결정).

        세 번까지 "안내를 받으시겠어요?"로 다시 묻고, 그 뒤면 물러난다. LLM 이 그 말에
        소리 내어 답하면(질문의 답) 그 답이 끝난 뒤에 한다 — 미션이 먼저 말하면 순서가 뒤바뀐다.
        """
        reply = (reply or "").strip()
        if reply:
            self._approach_after_reply = (reply, now + self.approach_response_timeout_sec)
            self._response_deadline = None
            return []
        return self._approach_follow_up(now)

    def _approach_follow_up(self, now: float) -> list:
        """다시 묻기(이번 접근에서 APPROACH_REASK_MAX 번까지) 또는 물러나기(다 썼으면 — 홈으로).

        물러날 때는 "알겠습니다"가 아니라 '비카야'를 알려 주고 떠난다 — 대답을 못 들었다(사용자 결정 '나').
        """
        self._approach_after_reply = None
        if self._approach_reasks < APPROACH_REASK_MAX:
            self._approach_reasks += 1
            # 재생이 끝나면(on_approach_question_spoken) 8초를 다시 센다. 그 소식이 안 오면 안전망.
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
    ) -> list:
        """/vica/person_detection 원본 결과 (근접 호출, 2026-09-10 확장).

        탐색 창(IDLE + `_seek_deadline` 살아있음) 중에만 본다 — 그 밖에서는
        기존 동작이 전부 그대로여야 한다. `approachable=true` 는 다루지 않는다
        — RequestApproach service 를 거치는 기존 경로(_on_approach_request)가
        그대로 처리한다.

        detection_gate 는 신뢰도·추적·안정(1초)·정지(3초창 0.3 m) 관문을 전부
        통과시킨 뒤 거리 하나만으로 TOO_NEAR 거절한다 — 즉 `approachable=false`
        인데 `stable=true`면 "코앞에 서 있는 진짜 사람"이라는 뜻이다. 그 값을
        안 쓰고 버리면 로봇이 8초 동안 사람을 보면서도 아무 말 없이 원위치로
        돌아가는 동작이 된다(부른 사람 관점에선 "쳐다보고 무시").

        `distance_m` 이 `near_call_max_m` 이상이면(또는 NaN·억제 중이면) 이
        경로가 관여할 일이 아니다 — 그 거리는 접근 goal(1.1 m)을 만들 수 있는
        자리라 기존 탐지→요청→접근 경로가 담당한다.
        """
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

        # 대화가 시작됐다 — 복귀 회전이 나가면 안 된다(사람에게 응대하러
        # 갔으므로). _to_idle() 을 부르지 않는다 — 그 함수는 state 도 IDLE 로
        # 내리는데, 여기서는 AWAITING_USER 로 곧장 들어가야 한다.
        self._seek_deadline = None
        self._seek_return_yaw = None
        self.approach_track_id = track_id
        self.active_destination = None
        self._near_call_no_spin = distance_m < self.near_call_no_spin_m
        return self._enter_awaiting_user(now)

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
            if self.approach_turn_yaw_rad == 0.0 or self._near_call_no_spin:
                # 회전이 없으면 회전 예고는 거짓말 — 온보딩으로 바로 간다.
                # 온보딩 끝은 질문이라 expects_reply 로 재청취 창이 열린다.
                # _near_call_no_spin(근접 호출 1.0 m 미만)도 같은 길을 탄다 —
                # 손잡이가 뒤로 길게 나와 있어 이 거리의 180도 회전은 손잡이가
                # 사람을 칠 수 있다(NEAR_CALL_NO_SPIN_M 근거 참고).
                self._to_idle()
                # 회전을 껐어도 사용자는 이미 승낙하고 그 자리에 있다 —
                # State.TURNING 을 거치는 길과 같은 억제를 건다.
                self._user_attached_until = now + USER_ATTACHED_SUPPRESS_SEC
                # 손잡이 위치 안내 (2026-09-10 사용자 결정): 회전이 없어도
                # 사용자는 손잡이가 어디인지 모른다 — 온보딩보다 먼저. 잡기
                # 대기를 거쳐 온보딩한다(TURNING 완료와 같은 길).
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
        """접근 질문을 접고 물러난다 — 아니요(MSG_APPROACH_DECLINED)든 대답 없음(MSG_APPROACH_UNANSWERED)이든
        같은 길이고 말만 다르다."""
        actions: list = [Say(message, priority="response")]
        if self._never_approached:
            # 걸어간 적이 없다(핸들 쪽 호출) — 물러날 곳이 없다. 사람이 이미
            # 손잡이 자리(로봇 뒤)에 서 있어, 복귀 주행을 걸면 출발 회전이
            # 손잡이로 그 사람을 훑는다(Ruling 10, I-1). 홈 미지정일 때의
            # 기존 동작과 같은 모양으로 제자리에서 끝낸다.
            self._to_idle()
        else:
            actions.extend(self._enter_returning(now))
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

    def on_listen_state(self, state: str, now: float) -> list:
        """/vica/listen_state (open/speech/closed/empty[:이유]). 발화 없음.

        closed = 발화가 STT 를 통과해 LLM 으로 가는 중 — 유예를 준다.
        empty[:이유] = 빈손("empty:ghost"·"empty:short-reject" 등) — 유예
        없이 시계가 그대로 흐른다.
        """
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
        else:   # "empty" 또는 "empty:이유"
            self._ear_busy = False
            self._ear_speaking = False
            self._ear_grace_until = None
        # 온보딩 뒤 빈손 되묻기 사다리(2026-09-11): 빈손이 실제로 도착하면
        # 폴백 시계(DEST_PROMPT_FALLBACK_SEC)까지 기다리지 않고 곧장
        # 전진한다. "leaving"·"notice" 단은 이미 되물은 뒤라 다시 듣지
        # 않는다 — 그 뒤는 시간(dest_retry_return_sec·LEAVING_GRACE_SEC)만
        # 으로 전진한다(on_tick).
        # IDLE 에서만 — 사다리는 온보딩 뒤 목적지를 기다리는 IDLE 의 일이다. 비상 정지 중
        # "비카야"를 무시하며 닫은 창(empty:wake-ignored)이 "잘 듣지 못했습니다…"를 끌어내던
        # 길을 막는다(2026-10-07 검토).
        if (state.startswith("empty") and self.state == State.IDLE
                and self._dest_prompt_stage in ("asked", "retried")):
            return self._advance_dest_prompt(now)
        return []

    def _dest_prompt_holds(self, now: float) -> bool:
        """온보딩 되묻기 사다리의 시계를 잡아둘 이유가 있는가.

        _ear_holds 와 달리 "창이 열려 있다(open)"는 잡지 않는다 — 질문 뒤
        청취 창은 30초라 열림을 잡으면 DEST_ANSWER_WAIT_SEC 가 무의미해진다.
        사용자가 말을 시작한 신호(speech)와, 말이 STT 를 지나 LLM 으로 가는
        유예(closed 뒤 EAR_GRACE_SEC)만 잡는다. 상한(EAR_HOLD_MAX_SEC)은
        닫힘 신호 유실 대비다.
        """
        if (self._ear_speaking and self._ear_speech_since is not None
                and now - self._ear_speech_since <= EAR_HOLD_MAX_SEC):
            return True
        return (self._ear_grace_until is not None
                and now < self._ear_grace_until)

    def on_dest_prompt_spoken(self, now: float) -> list:
        """온보딩(MSG_APPROACH_ONBOARDING)·되묻기(MSG_DEST_RETRY) 재생이
        끝났다 — 답 대기 시계(DEST_ANSWER_WAIT_SEC)는 여기서부터 센다.
        on_approach_question_spoken 과 같은 방식이다: 재생에 몇 초가 걸리는지는
        TTS 만 알아 노드가 tts_done 문구 대조로 알려준다. 안 오면
        _arm_dest_prompt·_advance_dest_prompt 가 걸어 둔 폴백(DEST_PROMPT_
        FALLBACK_SEC)이 탈출을 맡는다.
        """
        if self._dest_prompt_stage in ("asked", "retried"):
            self._dest_prompt_deadline = now + DEST_ANSWER_WAIT_SEC
        return []

    def _ear_holds(self, now: float) -> bool:
        """무응답 시계를 잡아둘 이유가 있는가. 상한(EAR_HOLD_MAX_SEC)은
        닫힘 신호 유실 대비 — 귀가 영영 바빠 보여도 결국 떠난다."""
        if (self._ear_busy and self._ear_busy_since is not None
                and now - self._ear_busy_since <= EAR_HOLD_MAX_SEC):
            return True
        return (self._ear_grace_until is not None
                and now < self._ear_grace_until)

    def exit_arrival_dialog(self) -> None:
        """도착 후 대화를 조용히 닫는다 — 새 목적지 '제안'(need_confirm=True)이
        왔을 때 노드가 부른다. navigate 는 2단계(제안→확정)라 제안에서 바로
        출발하면 확인 질문 전에 달린다(2026-08-30 실기). IDLE 로 내려가
        on_intent 게이트가 평소처럼 CONFIRMING 부터 밟게 한다."""
        if self.state in (State.ASKING_NEXT, State.ASKING_WAIT_TIME):
            self._reset_arrival_dialog()
            self.state = State.IDLE
            # _to_idle() 을 거치지 않는 유일한 IDLE 진입로라 탐색 창이 안
            # 비워진다 — 안내 한 판이 통째로 지난 뒤 낡은 복귀각으로 갑자기
            # 도는 사고로 이어진다(2026-09-10 재현).
            self._seek_deadline = None
            self._seek_return_yaw = None

    def _ask_arrival(self, dest: Optional[Destination], now: float,
                     arrival_text: str = "") -> list:
        """유형별 질문을 던지고 ASKING_NEXT 로 들어간다. "네"의 뜻(_asking_is_finish)
        을 함께 기억한다 — restroom 은 대기형(네=대기), 나머지는 종료형(네=끝).

        arrival_text 가 있으면 도착 멘트와 질문을 한 발화로 합쳐 낸다(순서 역전
        방지). 재질문(on_wake·복귀 브레이크)에서는 도착 멘트 없이 질문만 낸다.
        """
        category = (dest.category if dest else "") or ""
        # (질문, 종료형인가, 대기 수락 시 시간을 묻는가). 그 외 유형은 대기형
        # 으로 개편(2026-08-31) — "여기서 대기할까요?"라 네/아니오 판단이
        # 단순하다: 네=대기(시간 질문으로), 아니오=종료.
        if category == "restroom":
            question, is_finish, ask_time = MSG_ASK_RESTROOM, False, False
        elif category == "entrance":
            question, is_finish, ask_time = MSG_ASK_ENTRANCE, True, False
        else:
            question, is_finish, ask_time = MSG_ASK_GENERIC, False, True
        self.state = State.ASKING_NEXT
        # 도착했으니 취소할 안내 주행이 끝났다 — 물어 둔 "안내를 취소할까요?"를 닫는다. 남기면
        # 도착 질문의 "네"를 그 질문의 답으로 먹고, 15초 뒤 그 질문을 또 한다(2026-10-09 검토 I-5).
        self.cancel_confirm_pending = False
        self._cancel_confirm_deadline = None
        # 대기 장소는 이 목적지에 딸려 있다 — active_destination 을 비우기 전에 든다.
        # 목적지 없이 다시 묻는 자리(재질문)에서는 앞서 든 값을 그대로 둔다.
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
        self._response_deadline = None   # 재생완료(on_arrival_question_spoken)에서 시작
        text = f"{arrival_text} {question}".strip() if arrival_text else question
        return [Say(text, priority="response", expects_reply=True)]

    def on_arrival_question_spoken(self, now: float) -> list:
        """도착 후 질문 재생이 끝났다. 응답 대기 8초는 여기서부터 센다
        (AWAITING_USER 와 같은 방식). 재생시간을 TTS 만 아므로 노드가 알려준다."""
        if self.state in (State.ASKING_NEXT, State.ASKING_WAIT_TIME):
            self._response_deadline = now + self.approach_response_timeout_sec
        return []

    def on_wake_call(self, now: float) -> list:
        """"비카야" 한 번 — 호출 반응표대로 대답할지 정한다(2026-10-07 사용자 결정).

        음성 쪽은 호출 즉시 듣기 창을 열어 두고 이 판정(WakeReply)을 기다린다.
        - 대답하는 상태: 하던 말을 끊고 "네?"(MSG_WAKE_GREETING), 상태별 정리(on_wake),
          그리고 listen. 끊기와 "네?"는 같은 /vica/tts_request 로 순서대로 나간다(09-01
          순서 뒤집힘 사고). "네?"에 expects_reply 를 달지 않는다 — 창은 이미 열려 있고,
          질문 예약(followup)을 걸면 말이 끝날 때 그 창이 호출 창을 갈아치운다(10-06).
        - 대답하지 않는 상태(_WAKE_IGNORE_STATES): 아무 말 없이 ignore.
        - 비상 정지: "지금은 비상 멈춤 상태입니다."만 하고 ignore.
        복귀 중 호출(RETURNING)의 브레이크는 노드가 on_return_brake 로 먼저 건다. 창 안에서
        소리로 건진 호출(wake-rescue)은 노드가 on_wake 만 부른다 — 그때는 음성 쪽이 이미
        대답하고 듣는 중이다.
        """
        if self.estop_active or self.state == State.ESTOPPED:
            return [Say(MSG_ESTOP_WAKE, priority="response"), WakeReply(listen=False)]
        if self.state in _WAKE_IGNORE_STATES:
            return [WakeReply(listen=False)]
        if self.state == State.AWAITING_USER:
            # 접근 질문의 답을 기다리는 동안 '비카야'는 무시한다(2026-10-09 사용자 결정) — 말을 끊지
            # 않고 "네?"도 하지 않는다. 들은 말은 그대로 LLM 으로 넘겨 이 질문의 답으로 받는다. 음성 쪽
            # 귀도 이 단계에서는 호출을 확정하지 않으므로 이것은 새어 들어온 호출의 이중 장치다.
            # (2026-10-07: 같은 호출의 wake_doa 가 질문을 막 연 경우만 이렇게 했다.)
            return [WakeReply(listen=True)]
        # 판정은 "네?" 바로 뒤 — 상태 정리(재출발 등)보다 먼저 음성 쪽에 닿게 한다.
        actions: list = [StopSpeech(), Say(MSG_WAKE_GREETING, priority="response"),
                         WakeReply(listen=True)]
        actions.extend(self.on_wake(now))
        return actions

    def on_wake(self, now: float) -> list:
        """"비카야" — 새 대화의 시작 신호. 상태 정리만 한다("네?"·판정은 on_wake_call).

        확인 질문(CONFIRMING)·접근 질문은 옛 질문을 조용히 접는다(2026-09-01 사용자
        결정) — 답 자리가 살아 있으면 새 대화의 첫 마디(짧은 "그래" 오전사 등)가 옛
        질문의 답으로 오인 접수된다. 접는 멘트는 없다: 사용자는 이미 새 말을 하려는
        참이다. 도착 질문·대기 시간 질문과 대기(손 놓기 기다림 포함)는 2026-10-07 호출
        반응표대로 접지 않고 이어 간다.
        RETURNING 의 복귀 브레이크는 노드가 on_return_brake 로 따로 보낸다.

        접근 온보딩 직후 억제(`_user_attached_until`)가 살아 있으면 되감는다
        (USER_ATTACHED_SUPPRESS_SEC 근거 참고) — 붙어 있는 사용자가 계속
        말을 거는 동안은 대화가 이어지는 한 계속 막고, 조용해지면 그 값을
        새로 만들지 않으므로 결국 시간이 다 되어 풀린다.
        """
        # 새 대화다 — 온보딩 되묻기 사다리가 돌고 있었다면 청산한다.
        self._forget_dest_prompt()
        if self.state in (State.WAITING, State.WAITING_RELEASE):
            # 대기를 이어 가며 새 대화로 받는다(2026-10-07, 호출 반응표). 대기는
            # 목적지가 정해질 때 끝나고, 대기 시간도 그대로다 — 멀리서 로봇을 찾으려
            # 부른 "비카야"에 대기가 끝나 10초 알림이 멈추면 사용자가 로봇을 잃는다.
            # (옛 동작: 대기를 접고 IDLE. 각성 질문은 2026-09-01 삭제 그대로다.)
            # on_wake_doa 가 이 시각을 봐 이 소비를 새 호출로 오인하지 않는다.
            self._wake_consumed_at = now
            if self.state == State.WAITING:
                # 손잡이 위치 알림(2026-10-09 사용자) — WAITING 에서만. 손 놓기 기다림
                # (WAITING_RELEASE)은 사용자가 아직 손잡이 곁이라 떨지 않는다. 일반 호출
                # (on_wake_call)과 창 안에서 건진 호출(노드가 on_wake 만 부름) 모두 여기를
                # 지나므로 호출 한 번에 한 번 떤다. 비상 정지면 상태가 ESTOPPED 로 바뀌어
                # 이 분기에 오지 않는다(두 길 모두).
                return [Haptic(HAPTIC_PATTERN_WAKE_LOCATE)]
            return []
        if self.state in _WAIT_MOVING_STATES:
            # 혼자 대기 장소로 가는 중 — 대기 상태가 될 때까지 대답하지 않는다.
            return []
        if self.state == State.CONFIRMING:
            # 대기 중에 들어온 확인 질문이었으면 그 대기로 돌아간다. 주행 중 바꾸기
            # 질문이었으면 원래 목적지로 다시 출발한다 — 질문 전 하던 일(안내 주행)로
            # 돌아가는 것이고, "네?"가 나가는 참이라 출발 멘트는 보태지 않는다.
            # 이어 목적지를 말하면 주행 중 바꾸기가 처음부터 다시 열린다.
            actions = self._fold_confirming(now, announce=False)
            self._wake_consumed_at = now
            return actions
        if self.state in (State.ASKING_NEXT, State.ASKING_WAIT_TIME):
            # 질문을 접지 않는다(2026-10-07 사용자 결정, 호출 반응표). 옛 동작은 질문을
            # 접고 IDLE 이라, 부른 뒤의 "기다려 줘"·"5분"이 갈 곳 없이 무시됐다. 부른
            # 사람이 곁에 있으니 무응답 사다리는 처음부터 다시 센다 — 떠나기 예고 유예도
            # 거둔다. 8초 시계는 "네?" 재생이 끝난 때부터 다시 돈다(노드가 tts_done 마다
            # on_arrival_question_spoken 을 부른다).
            self._leaving_deadline = None
            self._arrival_retried = False
            self._response_deadline = None
            self._asking_entered_at = now
            self._wake_consumed_at = now
            return []
        if self.state == State.AWAITING_USER:
            # 접근 질문은 '비카야'에 접지 않는다(2026-10-09 사용자 결정, run82 17:14·17:17 — 접힌 뒤
            # 이어진 "그래"·"안내를 받을게요"가 갈 곳 없이 버려졌다). 옛 동작(2026-09-01): 질문을 접고
            # 트랙을 억제한 채 IDLE.
            return []
        if self.user_attached_guard_active(now):
            # 되감기. 새로 억제를 걸지는 않는다(이미 살아 있을 때만) — 없던
            # 억제를 여기서 새로 만들면 접근·온보딩과 무관한 "비카야"에도
            # USER_ATTACHED_SUPPRESS_SEC 짜리 억제가 생긴다.
            self._user_attached_until = now + USER_ATTACHED_SUPPRESS_SEC
        return []

    def wake_guard_active(self, now: float) -> bool:
        """`_wake_consumed_at` 직후 가드가 지금 유효한가.

        on_wake_doa 안과 진단 로그(mission_manager_node._on_wake_doa)가 정확히
        같은 조건으로 판정하게 하려고 메서드로 뺐다 — 로그가 이 조건을 따로
        베끼면 언젠가 어긋나고, 그러면 로그가 실제 관문과 다른 이야기를 하게
        된다.
        """
        return (self._wake_consumed_at is not None
                and now - self._wake_consumed_at < WAKE_CONSUMED_GUARD_SEC)

    def user_attached_guard_active(self, now: float) -> bool:
        """접근 온보딩 직후 억제(`_user_attached_until`)가 지금 유효한가.

        이유는 wake_guard_active 와 같다. 60초가 지나도 누가 손잡이를 쥐고 있는
        동안은 억제를 유지한다(2026-09-30, 설계 4.4) — 시계는 "사용자가 손잡이를
        잡고 있는가"의 어림이었고, 이제 그 답을 센서가 직접 준다. 시계를 지우지
        않고 덧대는 이유: 센서가 끊기면 규약상 '놓음'으로 읽혀 억제가 풀리는
        쪽으로 고장나므로, 시계가 바닥을 받친다.
        """
        if self._user_attached_until is None:
            return False
        return now < self._user_attached_until or self._grip.contact(now)

    def on_wake_doa(self, doa_deg: float, nav_ready: bool, now: float) -> list:
        """"비카야"가 온 방향으로 고개를 돌린다 (호출 접근 설계 §4).

        대기 중에만 연다. 다른 상태의 호출은 기존 on_wake 의 몫이다 —
        안내 중 "비카야"는 지금 안내받는 사용자의 명령이지 새 부름이 아니다.

        마이크는 거리를 모르고 각도도 ±4~16° 라, 소리로 목표점을 만들지
        않는다. 돌아서 카메라가 확인한 뒤에야 기존 접근 경로가 이어받는다.

        /vica/wake 와 이 토픽은 같은 호출에서 수 ms 간격으로 오고 처리 순서가
        보장되지 않는다(2026-09-10 실기 재현). wake 가 먼저 오면 위 state != IDLE
        관문이 거절하지만, wake 가 답-대기 상태를 IDLE 로 접은 **직후**라면
        state 만으로는 "방금 접힌 옛 대화"와 "진짜 새 호출"을 구분 못 한다.
        그래서 wake/on_return_brake 가 상태를 바꾼 시각을 함께 본다 — 그 직후
        WAKE_CONSUMED_GUARD_SEC 이내면 옛 대화의 여진으로 보고 거절한다.

        같은 이유로, 접근 온보딩 직후(USER_ATTACHED_SUPPRESS_SEC 이내)도
        거절한다. 이 전이는 wake 가 아니라 회전 완료(on_tick 의 TURNING
        분기)가 일으킨 것이라 _wake_consumed_at 도장이 없다 — 손잡이를 막
        받아든 사용자 옆에서 같은 사고가 재현되는 것을 막는다.

        복귀 재개 사다리가 도는 동안(_return_interrupted)도 통째로 거절한다
        (2026-09-10 사용자 결정). 회전이면 왕복 최대 16초, 못 찾으면 사다리가
        다시 18초를 센다 — 오탐 한 번이 30초 넘게 로봇을 통행로에 붙잡을 수
        있어 "부른 쪽을 본다"의 이득보다 위험이 크다는 판단이다. 이 관문
        하나로 무회전 분기(정면 호출, 아래 SEEK_MIN_YAW_RAD 미만)가 탐색
        창(_seek_deadline)만 여는 경로도 함께 막힌다 — 그 경로는 state 를
        IDLE 에 둔 채 _to_idle() 을 거치지 않아, 이 관문이 없으면 탐색 창과
        복귀 사다리(_return_resume_deadline)가 같은 IDLE 위에서 겹쳤다.
        SEEK_LOOK_SEC 이 RETURN_RESUME_SEC 보다 얼마나 작은지와 무관하게
        막히므로, 둘 중 어느 값을 나중에 올려도 이 관문은 그대로 유효하다.

        핸들 쪽 사각지대(HANDLE_SIDE_MIN_YAW_RAD, 위 정면 사각지대의 거울쌍):
        회전량이 180°에 가까우면(부채꼴 180°±45°) 소리가 핸들 옆에서 왔다는
        뜻이고, 그 자체가 "이미 핸들 옆에 서 있다"는 증거라 카메라 확인을
        기다리지 않고 곧바로 접근 질문으로 들어간다(2026-09-10 사용자 결정).
        여기서 만들어진 대화는 track_id 가 없다 — 카메라로 확인한 적이 없기
        때문이다(approach_track_id=None). 수락해도 회전하지 않는다
        (_near_call_no_spin 재사용 — 이름은 "근접"이지만 뜻은 같다: 이미
        올바른 자리에 있으니 180도를 돌리지 않는다).
        """
        if self.state != State.IDLE or self.estop_active or not nav_ready:
            return []
        if self._return_interrupted:
            return []
        if self.wake_guard_active(now):
            return []
        if self.user_attached_guard_active(now):
            return []
        yaw = doa_to_spin_yaw(doa_deg, self.wake_doa_sign)
        # 돌아야 할 만큼 돌았다고 치고 복귀각을 먼저 누적한다 — 창 중에 다시
        # 부르면 또 돌기 때문에 덮어쓰면 원래 자세로 못 돌아온다.
        back = wrap_to_pi((self._seek_return_yaw or 0.0) - yaw)
        if abs(yaw) < SEEK_MIN_YAW_RAD:
            # 이미 그쪽을 보고 있다. 돌지 않고 찾는 창만 연다.
            self._seek_return_yaw = back
            self._seek_deadline = now + self.seek_look_sec
            return []
        if abs(yaw) > self.handle_side_min_yaw_rad:
            # 핸들 쪽이다. 돌지 않고 곧바로 질문한다 — 탐색 창(_seek_deadline)
            # 도 열지 않는다: 카메라로 찾을 필요가 없다.
            self._seek_return_yaw = None
            self._seek_deadline = None
            self.approach_track_id = None
            self.active_destination = None
            self._near_call_no_spin = True
            # 걸어서건 코앞이건 이 사람 쪽으로 움직인 적이 없다 — 거절·
            # 무응답이 복귀 주행으로 새면 안 된다(Ruling 10, I-1).
            self._never_approached = True
            # 이 호출의 형제 신호인 /vica/wake 가 수 ms 뒤 따라올 수 있다
            # (2026-09-11 실기 재현) — 도장을 찍어 on_wake 의 AWAITING_USER
            # 분기가 방금 연 이 질문을 "옛 대화"로 오인해 접지 않게 한다.
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
        """도착 후 질문에 대한 답. 정리된 intent 만 받아 갈래를 만든다.

        next_dest 는 navigate 답일 때 노드가 찾아 준 다음 목적지다. 평소 목적지
        요청과 같은 관문(check_gate)을 여기서 거친다 — 예전에는 노드가 관문 없이
        넘겨 비공개·위치 미등록·주행 미준비 목적지로도 바로 출발했다(10-06 발견,
        10-07 수리). bounds·nav_ready 는 노드가 on_intent 와 같은 값을 넘긴다.
        """
        if self.state not in (State.ASKING_NEXT, State.ASKING_WAIT_TIME):
            return []
        kind = intent.intent

        # 다음 목적지(확정만) — 제안(need_confirm=True)은 여기 오기 전에
        # 노드가 exit_arrival_dialog 로 일반 확인 흐름에 합류시킨다.
        if kind == "navigate" and next_dest is not None:
            reason = check_gate(intent, next_dest, bounds, self.estop_active, nav_ready)
            if reason != GateReason.OK:
                # 갈 수 없는 곳 — 거절을 말하고 도착 대화에 남는다. 거절 멘트 재생이
                # 끝나면(tts_done) 8초 시계가 다시 돌아, 침묵이면 같은 질문을 다시 한다.
                msg = _REJECT_MESSAGES.get(reason)
                return [Say(msg, priority="response")] if msg else []
            self.state = State.NAVIGATING
            self.active_destination = next_dest
            # 도착하며 활성 모드는 끝났다. 다음 목적지는 출발 순간 다시 정한다.
            self.handle_active = self._decide_handle_mode(now)
            self._reset_arrival_dialog()
            self._nav_tree = NAV_TREE_GUIDED   # 사용자 안내 — 위치만 판정
            self.door_side = ""
            # 출발을 먼저 알린다(2026-10-07 사용자 결정) — 08-30 첫 구현부터 이 길만 말없이
            # 움직였다. 평소 목적지 요청(on_intent 시작 블록)과 같은 멘트다.
            return [SetNavSpeedLimit(NO_SPEED_LIMIT),
                    Say(say_destination(MSG_START, next_dest.name)),
                    Navigate(next_dest, tree=NAV_TREE_GUIDED)]

        # 종료: finish, 도착 후 cancel(=finish, 2026-08-30), 종료형 질문의 affirm.
        if (kind in ("finish", "cancel")
                or (kind == "affirm" and self._asking_is_finish)):
            self._reset_arrival_dialog()
            return [Say(MSG_FINISH, priority="response"), *self._go_home(now)]

        # 시간 질문(ASKING_WAIT_TIME)에 네/아니오는 답이 아니다 — 옛 질문
        # 태그로 해석하면 "네"가 종료(홈행)로 둔갑했다(2026-08-31 구멍 ②).
        # 숫자가 올 때까지 재질문 사다리로 보낸다.
        if self.state == State.ASKING_WAIT_TIME and kind in ("affirm", "deny"):
            return self._arrival_no_answer(now)

        # 대기: wait, 또는 대기형 질문의 affirm.
        if kind == "wait" or (kind == "affirm" and not self._asking_is_finish):
            minutes = intent.wait_minutes if kind == "wait" else -1
            if minutes is None or minutes < 0:
                # 시간 없음: 그 외 유형(ask_time)만 "몇 분쯤?" 후속 질문,
                # restroom·entrance 는 시간 안 묻고 기본 30분(스펙).
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

        # deny: 종료형이면 "안 끝났다"=대기, 대기형이면 "대기 싫다"=종료.
        if kind == "deny":
            if self._asking_is_finish:
                if self._wait_minutes_requested > 0:
                    # 대기 중 "기다리지 마"를 끝낼지 되물은 질문의 "아니요" — 하던 대기 시간
                    # 그대로 다시 기다린다(2026-10-08 반응표).
                    return self._enter_waiting(self._wait_minutes_requested, now)
                return self._enter_waiting(WAIT_MINUTES_CAP, now,
                                           default_msg=True)
            if not self._deny_reconfirmed:
                # 2026-09-20 사용자 결정: 대기형 질문의 '아니오' 한 번으로는 떠나지
                # 않는다 — "그럴래?"를 거절로 잘못 들어 가 버린 사건(실기 18:18).
                # 종료형 질문으로 한 번 더 묻는다: "네"=끝(홈행), "아니오"=대기.
                self._deny_reconfirmed = True
                self._asking_is_finish = True
                self._asking_time_after_yes = False
                self._asking_question = MSG_ASK_ENTRANCE
                self._asking_entered_at = now
                self._response_deadline = None
                return [Say(MSG_ASK_ENTRANCE, priority="response", expects_reply=True)]
            self._reset_arrival_dialog()
            return [Say(MSG_FINISH, priority="response"), *self._go_home(now)]

        # 정보 질문("화장실 어디야?"·"몇 시야?") — LLM 이 이미 답했다. 질문은 그대로 두고
        # 사다리를 쓰지 않는다: 미션까지 "잘 듣지 못했습니다…"를 얹으면 두 목소리가 나고
        # 재질문 한 번을 헛되이 쓴다(2026-10-07 검토 — 호출 뒤 질문 유지·도착 방향 답이
        # 이 길을 자주 만든다). 8초 시계는 LLM 대답의 재생이 끝날 때 다시 돈다.
        # LLM 이 되묻는 중(clarify, "어느 화장실이요?")도 같다 — 미션까지 다시 물으면 두 목소리가
        # 난다(2026-10-08 반응표). 못 알아들은 답(unknown)만 아래 사다리로 간다.
        if kind in ("question", "clarify"):
            self._response_deadline = None
            self._asking_entered_at = now
            return []

        # 못 알아들음(unknown 등): 무응답 사다리 ①.
        return self._arrival_no_answer(now)

    def _enter_waiting(self, minutes: int, now: float,
                       default_msg: bool = False, away: bool = False) -> list:
        """대기 확정 + 멘트. 사람접근은 대기 상태값으로 자연히 꺼진다.

        대기 장소가 있는 목적지면 M2/M2′ 를 말하고 손 놓기를 기다린다
        (WAITING_RELEASE). 없으면 지금처럼 그 자리에서 기다린다(WAITING). away(목적지를
        떠나 있음, 2026-10-08 결정 1)면 대기 장소가 없는 목적지는 그 입구 앞으로 돌아간다.
        대기 시간은 어느 쪽이든 이 순간(멘트를 말한 순간)부터 흐른다 — 말한
        "N분 동안"이 맞게(2026-10-07).
        """
        self._wait_until = now + minutes * 60.0
        self._wait_minutes_requested = int(minutes)
        self._response_deadline = None
        self._leaving_deadline = None
        dest = self._arrived_destination
        spot = dest.wait_spot if dest is not None else None
        place = WAIT_PLACE_PHRASES.get(spot.side, "") if spot is not None else ""
        if not place and away and dest is not None:
            # 목적지를 떠나 있다(홈 가는 중 "기다려", 결정 1) — 대기 장소가 없는 목적지면 그
            # 입구 앞으로 돌아가 기다린다. 장소 말은 대기 장소가 막혔을 때(M6)와 같다.
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
        """도착 순간의 M1 문장("화장실은 오른쪽에 있습니다."). 못 정하면 "".

        입구 방향이 없는 옛 목적지·로봇 방향을 아직 모를 때는 말하지 않는다 —
        짐작한 방향을 말하는 것보다 침묵이 낫다. 상황판 door_side 도 함께 정한다.
        """
        self.door_side = ""
        if (dest is None or dest.door_yaw_deg is None
                or self.robot_yaw_deg is None):
            return ""
        side = door_side_word(dest.door_yaw_deg, self.robot_yaw_deg)
        self.door_side = side
        return MSG_DOOR_SIDE.format(
            name=dest.name, eun=josa_eun_neun(dest.name), side=side)

    def on_wait_speech_spoken(self, text: str, now: float) -> list:
        """M2 재생이 끝났다(노드가 /vica/tts_done 문구로 알려 준다). 손 놓기 판정은
        이때부터다 — 말하는 도중에 떠나면 사용자가 장소 안내를 끝까지 못 듣는다."""
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
        # 터치 센서가 살아 있으면 손을 놓은 지 WAIT_RELEASE_SEC 를 기다린다. 센서가
        # 없거나 끊겼거나 시연 스위치면 놓았는지 알 수 없으니 M2 가 끝나자마자 떠난다.
        sensor = (not self.grip_assume_held) and self._grip.fresh(now)
        if sensor and self._grip.released_for(now) < WAIT_RELEASE_SEC:
            return []
        # 대화 중("비카야" 뒤 듣는 중 등)이면 떠나지 않는다 — 사용자가 옆에서 말하는
        # 중이다. 떠나면 뒤이어 온 목적지는 이동 중이라 버려진다.
        if self._ear_holds(now):
            return []
        return self._start_moving_to_wait_spot(now)

    def _start_moving_to_wait_spot(self, now: float) -> list:
        dest = self._arrived_destination
        if dest is None or dest.wait_spot is None:
            # 대기 장소를 잃었다(목적지 다시 읽기 등) — 그 자리에서 기다린다.
            self.state = State.WAITING
            self._wait_place = ""
            return []
        wait_dest = wait_spot_destination(dest)
        self.state = State.MOVING_TO_WAIT_SPOT
        self.active_destination = wait_dest
        self.handle_active = False
        self._release_text = ""
        # 입구 방향(M1·LLM 메모의 'OO 방향')은 도착한 자리의 로봇 기준이다 — 대기 장소로
        # 움직이면 낡은 말이 된다(2026-10-07 검토).
        self.door_side = ""
        # M3 박자는 대기 장소로 출발한 순간부터 센다.
        self._beacon_next_at = now + WAIT_BEACON_INTERVAL_SEC
        return [SetNavSpeedLimit(NO_SPEED_LIMIT), Navigate(wait_dest, tree=NAV_TREE_WAIT)]

    def _wait_spot_blocked(self, now: float) -> list:
        """대기 장소에 못 들어갔다(Nav2 실패 신호). 다시 출발시키지 않는다 —
        버티는 시간은 대기 장소 전용 짧은 트리의 복구 횟수가 정한다(19차 결정).
        M6 을 말하고 앱에 알린 뒤 목적지로 돌아가 거기서 기다린다."""
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
        # 박자를 잇는다 — 밀린 만큼 한꺼번에 몰아 말하지 않는다.
        while self._beacon_next_at <= now:
            self._beacon_next_at += WAIT_BEACON_INTERVAL_SEC
        if self._ear_holds(now):
            return []
        # 가장 낮은 ambient 등급(2026-10-07) — 다른 말이 나가거나 줄 서 있으면 TTS 가
        # 기다리지 않고 버리고, 재생 중 다른 말이 오면 비킨다(작업 계획 탭 M3). narration
        # 이면 최대 6초 기다렸다 나와 대화 끝에 끼어든다.
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
        """장애물 안내 후보를 말할지 정한다 → (actions, 뺀 이유). 말하면 이유는 "".

        판정(obstacle_judge)이 '앞에 진짜 물체가 있어 비켰다·줄였다'고 넘긴 후보다. 대화가 먼저다
        (2026-10-09 사용자 확인): 귀가 듣는 중이거나 취소 확인의 답을 기다리면 말하지 않는다 — 로봇 말이
        시작되면 귀가 열린 재청취 창을 접어 사용자 대답이 잘린다. 로봇이 다른 말을 하는 중이면 TTS 가
        버리도록 ambient 로 보낸다. 늦은 후보도 버린다. 버린 안내는 다시 하지 않는다.
        """
        text = {"avoid": MSG_OBSTACLE_AVOID, "slow": MSG_OBSTACLE_SLOW}.get(phrase)
        if text is None:
            return [], "unknown_phrase"
        if self.dialog_state != State.NAVIGATING.value:
            return [], "not_navigating"
        if self.cancel_confirm_pending:
            return [], "question_pending"
        if self._ear_holds(now):
            return [], "ear_busy"
        if now - onset > OBSTACLE_STALE_SEC:
            return [], "stale"
        return [Say(text, priority="ambient")], ""

    def _home_beacon_tick(self, now: float) -> list:
        """홈 알림 — 홈에서 쉬는 동안 1분마다 M3. 첫 마디는 홈에 선 지 1분 뒤.

        홈을 벗어나거나 IDLE 이 아니게 되면 박자를 지운다(돌아오면 다시 1분부터).
        대화 중이면 건너뛰고 박자는 잇는다 — M3(_beacon_tick)와 같은 규칙이다.
        """
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
        # ambient — 다른 말이 나가거나 줄 서 있으면 TTS 가 버린다(M3 와 같다).
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
        """무응답 사다리: 못 알아들으면 같은 질문을 1회 다시 묻고, 그 뒤엔 떠나기 예고.

        2026-10-08 다시 묻기: 예전엔 "잘 듣지 못했습니다. 계속 안내가 필요하시면 말씀해
        주세요."였다 — 사용자는 질문을 다시 들어야 답할 수 있다. 물어 둔 질문이 없으면
        (늦은 답 그물의 대화) 옛 문장을 쓴다."""
        if not self._arrival_retried:
            self._arrival_retried = True
            self._response_deadline = None
            self._asking_entered_at = now   # 재질문도 새 시계 유실 폴백 기준
            return [self._ask(self._asking_question or MSG_ARRIVAL_RETRY)]
        return self._leaving_notice(now)

    def _arrival_silence(self, now: float) -> list:
        """침묵 사다리 (2026-09-20 사용자 결정): 8초 침묵이면 같은 질문을 한 번 더
        묻고, 그래도 침묵이면 떠나기 예고. 옛 동작(침묵 → 바로 예고)은 귀가 답을
        놓쳤을 때(노드 사망·오전사) 사용자가 서 있는데 로봇이 가 버리게 했다."""
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
        """안내 종료의 홈 복귀(도착 후 대화의 종료 답·무응답 떠남·대기 만료).

        auto_return_home 을 보지 않는다 — 그 스위치는 로봇이 먼저 다가갔다
        거절당한 **접근 뒤** 복귀의 '제자리' 정책이고(2026-08-27 사용자 결정),
        여기는 사용자가 안내 종료를 고른 경우라 시나리오대로 홈으로 간다
        (2026-09-01 사용자 확인). 홈이 지정되지 않았으면 _enter_returning 이
        제자리로 처리한다.
        """
        return self._enter_returning(now, dialog_finish=True)

    def _arm_dest_prompt(self, now: float) -> None:
        """온보딩 질문(MSG_APPROACH_ONBOARDING)을 던진 직후 되묻기 사다리를
        켠다 — 회전 생략·회전 성공·회전 실패, 온보딩이 나가는 세 자리 모두
        여기를 부른다.

        deadline 은 우선 보험(DEST_PROMPT_FALLBACK_SEC)으로 건다 — 정상 경로는
        온보딩 재생이 끝날 때 노드가 on_dest_prompt_spoken 을 불러
        DEST_ANSWER_WAIT_SEC 짜리 진짜 시계로 갈아 끼우고, 그 전에 음성의
        빈손 신호(on_listen_state 의 empty:*)가 오면 그것이 더 일찍 전진시킨다.

        끊긴 옛 복귀 사다리(_return_interrupted)를 함께 잊는다 — 사람 접근은
        그 사다리가 도는 중에도 카메라로 열릴 수 있어(2026-09-10 설계), 새
        온보딩이 시작된 이상 두 사다리가 같은 IDLE 위에서 겹치면 안 된다.
        """
        self._dest_prompt_stage = "asked"
        self._dest_prompt_deadline = now + DEST_PROMPT_FALLBACK_SEC
        self._forget_interrupted_return()

    def _forget_dest_prompt(self) -> None:
        """온보딩 되묻기 사다리를 청산한다 — 대화가 다른 경로로 넘어가거나
        (on_wake·on_intent) 주행이 시작되거나(_enter_returning) IDLE 을 새로
        떠날 때(_to_idle) 부른다."""
        self._dest_prompt_stage = None
        self._dest_prompt_deadline = None

    def _advance_dest_prompt(self, now: float) -> list:
        """되묻기 사다리를 한 단 전진한다.

        빈손 신호(on_listen_state)가 불렀든 폴백 시계 만료(on_tick)가
        불렀든 전진 규칙은 하나다 — "이 단에서 더 기다릴 이유가 없다"는
        사실만 다르게 도착할 뿐이다.

        asked → retried  : 한 번 되묻는다(MSG_DEST_RETRY).
        retried → leaving: 조용히 기다린다(dest_retry_return_sec).
        leaving → notice : 떠남을 예고한다(MSG_LEAVING_NOTICE, 재사용).
        notice → (청산)  : 사다리를 접고 홈으로 돌아간다.
        """
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
        """홈 복귀 중 "비카야"/늦은 답 (작업 E). 복귀를 취소한다.

        quiet(늦은 답 선처리): 바로 뒤따르는 답이 on_arrival_answer 로
        처리되도록 ASKING_NEXT 로 들어간다. 호출("비카야")이면 질문 없이
        접고 IDLE — 각성 질문은 2026-09-01 삭제(A안, 비카야=새 대화).
        긴급어("멈춰")는 별도 경로로 어느 상태든 항상 통한다."""
        if self.state != State.RETURNING:
            return []
        # on_wake_doa 가 이 시각을 봐 이 소비를 새 호출로 오인하지 않는다.
        self._wake_consumed_at = now
        # 세웠다 — 그 뒤의 "네·아니요"는 늦은 답이 아니라 지금 대화다(결정 1).
        self._late_answer_finish = None
        cancel_dest = self.active_destination
        self.active_destination = None
        actions: list = [SetNavSpeedLimit(NO_SPEED_LIMIT)]
        if cancel_dest is not None:
            actions.append(CancelNav(cancel_dest))
        if quiet:
            self.state = State.ASKING_NEXT
            self._asking_is_finish = False   # "네" = 계속
            self._arrival_retried = False
            self._response_deadline = None
            self._asking_entered_at = now    # 낡은 진입 시각이면 폴백 즉발
        else:
            self._reset_arrival_dialog()
            self._to_idle()
            # 복귀가 끊긴 채로 IDLE 에 섰다 — 재개 사다리를 지금 이 시각
            # 기준으로 건다(청취 창이 몇 초든 이 시각과 무관하다, 근거는
            # RETURN_RESUME_SEC 주석). on_tick 의 IDLE 분기가 이어받는다.
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
        # 대기 장소(2026-10-07) — 대기가 끝나면 장소·알림 박자·확인 보류도 함께 끝이다.
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
        """복귀 재개 사다리를 청산한다 — "복귀가 끊겨 있다"는 사실과 그
        시각을 모두 지운다. 부르는 자리: 사다리 자신이 실제로 복귀를
        재개할 때 · 새 목적지로 주행을 시작할 때(음성·앱) · 관리자가
        복귀나 취소를 직접 명령할 때. `_to_idle()` 은 이 사실을 지우지
        않는다(탐색 회전 등 다른 경로로 IDLE 을 거칠 때도 복귀가 잊히면
        안 되기 때문) — 그래서 지우는 자리를 여기 한 곳에 모아 둔다.
        """
        self._return_interrupted = False
        self._return_resume_deadline = None
        self._return_notice_given = False

    def on_emergency(self, keyword: str, now: float) -> list:
        """/vica/emergency (긴급어). 하드 키워드만 처리 — LLM 을 거치지 않은 경로.

        모터 정지의 권위는 /emergency_stop 래치 체인(진행순서 ③에서 배선)이고,
        여기서의 goal 취소는 심층 방어 보조 경로다.
        """
        if keyword not in HARD_EMERGENCY_KEYWORDS:
            return []

        actions: list = []
        # 접근·복귀·탐색(SEEKING) 중에도 goal 이 살아 있다. 로봇이 사람을 향해
        # 움직이거나 소리 쪽으로 도는 구간이 있으므로 여기서 취소가 빠지면
        # 긴급어 경로가 죽는다(설계 7절).
        if self.state in _GOAL_ACTIVE_STATES:
            actions.append(SetNavSpeedLimit(NO_SPEED_LIMIT))
            actions.append(CancelNav(self.active_destination))
        already_estopped = self.state == State.ESTOPPED
        moving = self.state in _GOAL_ACTIVE_STATES
        self._enter_estopped(now)
        if not already_estopped:
            # 정지 중 "멈춰"도 침묵 — 움직임이 없으니 알릴 변화가 없다.
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
                # 움직이는 중에 걸렸을 때만 말한다 — 정지 중 걸림(통신 순단
                # 자동복구 포함)은 사용자에게 달라지는 게 없어 침묵한다.
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
        nav_ready: bool = True,
    ) -> list:
        """주기 처리. distance_remaining 은 Nav2 feedback 의 남은 거리(m)다.

        nav_ready 는 손 놓침 뒤 자동 재출발(_handle_pause_tick)의 재개 관문에만 쓴다.
        """
        actions: list = []
        self._prune_suppressed(now)

        # 활성 주행 중 손 놓침(설계 4.3 (다)). 주행 결과가 이번 tick 에 나왔으면
        # 결과가 먼저다 — 도착한 순간 손을 뗀 것을 놓침으로 세우면 안 된다.
        if (self.state == State.NAVIGATING and self.handle_active
                and nav_status not in (NavStatus.SUCCEEDED, NavStatus.FAILED,
                                       NavStatus.CANCELED)):
            handle_actions = self._handle_nav_tick(now)
            if self.state == State.PAUSED:
                return handle_actions
            actions.extend(handle_actions)

        if self.cancel_confirm_pending:
            actions.extend(self._cancel_confirm_tick(now))

        if self.state == State.CONFIRMING:
            if (not self._confirm_reasked and self._confirm_prompt
                    and self._confirm_reask_at is not None and now >= self._confirm_reask_at
                    and not self._ear_holds(now)):
                # 확인 질문에 15초 답이 없다 — 같은 질문을 한 번 더 묻는다(2026-10-08 다시 묻기).
                # 다시 물은 뒤에도 답할 시간을 남긴다(틱이 늦게 와도).
                self._confirm_reasked = True
                self._confirm_deadline = max(self._confirm_deadline or now,
                                             now + QUESTION_REASK_SEC)
                actions.append(self._ask(self._confirm_prompt))
            elif self._confirm_deadline is not None and now >= self._confirm_deadline:
                if self._change_from is not None:
                    # 주행 중 바꾸기 질문에 답이 없다 — 원래 목적지로 다시 출발한다
                    # (2026-10-07 사용자 결정, 아니요와 같은 길·같은 멘트).
                    actions.extend(self._fold_confirming(now))
                elif self._confirm_from_approach:
                    actions.extend(self._fold_approach_confirm(now))
                else:
                    self._fold_confirming(now)
                    actions.append(Say(MSG_CONFIRM_TIMEOUT))

        elif self.state == State.NAVIGATING:
            # 취소 재확인의 시계는 위 _cancel_confirm_tick 이 본다(어느 상태에서 물었든).
            if nav_status == NavStatus.SUCCEEDED:
                dest = self.active_destination
                text = (
                    dest.arrival_message
                    if dest and dest.arrival_message
                    else MSG_ARRIVED_FALLBACK.format(name=dest.name if dest else "목적지")
                )
                self._approach.reset()
                # 도착하면 활성 모드는 끝이다(손을 떼도 된다). 도착 진동(짧게 ×3)은
                # 펌웨어가 ARRIVED 상태 진입 때 스스로 낸다.
                self.handle_active = False
                actions.append(SetNavSpeedLimit(NO_SPEED_LIMIT))
                if dest is not None and is_wait_destination_id(dest.id):
                    # 관리자 '대기 장소로 가보기' 도착 — 확인용 이동이라 말하지 않는다.
                    self.state = State.ARRIVED
                    self._dwell_until = now + self.dwell_sec
                elif self.arrival_dialog and not self._nav_from_app:
                    # 도착 멘트·M1(입구 방향)·유형별 질문을 한 발화로 합쳐 낸다 — 따로
                    # 내면 우선순위(narration vs response)로 순서가 뒤집히고(실기 확인
                    # 2026-08-30), 앞 문장의 재생 완료가 8초 응답 창을 일찍 연다.
                    # M1 문장은 음성 쪽이 문장 단위로 미리 합성해 두므로 합쳐도 빠르다.
                    door = self._door_side_sentence(dest)
                    arrival = f"{text} {door}".strip() if door else text
                    actions.extend(self._ask_arrival(dest, now, arrival_text=arrival))
                else:
                    self.state = State.ARRIVED
                    self._dwell_until = now + self.dwell_sec
                    # 앱(관리자) 주행 도착엔 사용자가 없다 — 입구 방향은 사용자 안내에만.
                    door = "" if self._nav_from_app else self._door_side_sentence(dest)
                    actions.append(Say(f"{text} {door}".strip() if door else text))
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
                # 관리자 '가보기'(대기 장소)는 확인용 이동이라 다시 시도하지도, 말하지도
                # 않는다 — 실패는 앱의 주행 실패 팝업이 알린다.
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
                    # 재시도 안내 멘트는 2026-09-01 감량 — 침묵 후 재출발.
                else:
                    self._retry_destination = None
                    self._retry_at = None
                    # narration 은 큐 정원 초과 시 가장 먼저 버려진다
                    # (tts_queue._trim). 주행 실패는 사용자가 왜 멈췄는지 알
                    # 유일한 단서라 버려지면 안 된다.
                    if not wait_check:
                        actions.append(Say(MSG_NAV_FAILED, priority="response"))

        elif self.state == State.APPROACHING:
            if nav_status == NavStatus.SUCCEEDED:
                # 사람 앞 1.1 m 에 섰다. 여기서부터 주도권은 음성 쪽으로 넘어가고
                # Mission 은 타임아웃만 센다(설계 4절). 걸어서 도착했으니 정상
                # 접근이다 — 근접 호출(on_person_detection)의 회전 생략은 이
                # 경로와 무관하다(도착 거리 1.1 m > near_call_no_spin_m 1.0 m).
                self._near_call_no_spin = False
                actions.extend(self._enter_awaiting_user(now))
            elif nav_status in (NavStatus.FAILED, NavStatus.CANCELED):
                # 접근은 재시도하지 않는다. 등록 목적지는 제자리에 있지만 사람은
                # 3초 뒤 그 자리에 없다. 실패하면 돌아가서 다시 탐지하는 편이
                # 빠르고, 같은 자리로 되풀이 진입하면 통행에 방해가 된다.
                actions.extend(self._enter_returning(now))

        elif self.state == State.SEEKING:
            if nav_status in (NavStatus.SUCCEEDED, NavStatus.FAILED,
                              NavStatus.CANCELED):
                # 복귀각이 남아 있으면 방금 것은 '가는' 회전이다 — IDLE 로
                # 내려놓고 사람을 찾는 창을 연다. IDLE 이어야 접근 관문을
                # 그대로 통과한다. 없으면 방금 것이 복귀 회전이라 끝이다.
                #
                # 회전이 거부돼도(FAILED) 찾아는 본다. 카메라가 이미 사람을
                # 보고 있을 수 있고, 못 봐도 창이 닫히면 조용히 끝난다.
                back = self._seek_return_yaw
                self._to_idle()
                if back is not None:
                    self._seek_return_yaw = back
                    self._seek_deadline = now + self.seek_look_sec
            elif (self._turn_deadline is not None
                  and now >= self._turn_deadline):
                # spin 이 시작조차 안 됐다. 시계로 탈출한다.
                self._to_idle()

        elif self.state == State.PAUSED:
            # 손 놓침으로 선 경우만 시계를 본다. "잠깐"으로 선 PAUSED 는 말로
            # 재개할 때까지 그대로다(기존 동작).
            if self._handle_pause:
                actions.extend(self._handle_pause_tick(now, nav_ready))

        elif self.state == State.IDLE:
            # 잡기 대기(설계 4.3 (가)). 대기 중에는 탐색 창·복귀 사다리·되묻기
            # 사다리가 걸려 있지 않다(_to_idle 뒤에 열리고, 온보딩 전이다).
            if self._grip_wait_since is not None:
                actions.extend(self._grip_wait_tick(now))
            # 찾는 창이 닫혔다. 아무도 못 찾았으니 조용히 원래 자세로.
            # 되돌리지 않으면 오작동 한 번에 카메라가 벽만 보는 자세로 굳는다.
            if self._seek_deadline is not None and now >= self._seek_deadline:
                back = self._seek_return_yaw
                self._seek_deadline = None
                self._seek_return_yaw = None
                if back is not None and abs(back) >= SEEK_MIN_YAW_RAD:
                    self.state = State.SEEKING
                    self._turn_deadline = now + SEEK_TURN_TIMEOUT_SEC
                    actions.append(SpinInPlace(back, reason="못 찾아 원위치로"))

            # 복귀 재개 사다리 (2026-09-10). 위 탐색 창 처리가 이번 tick 에
            # SEEKING 을 새로 열었을 수 있으므로 state 를 다시 본다 — 그
            # 상태에서 아래를 마저 돌리면 방금 낸 SpinInPlace 위에 _go_home 의
            # Navigate 가 겹친다. 탐색 창(_seek_deadline)과는 필드가 달라
            # 서로 방해하지 않는다.
            if self.state == State.IDLE and self._return_interrupted:
                if self._return_resume_deadline is None:
                    # 회전이 끼어들었다 IDLE 로 막 돌아온 시점 — 여기서부터
                    # 다시 잰다(호출 시각부터 누적하지 않는다).
                    self._return_resume_deadline = now + self.return_resume_sec
                elif (now >= self._return_resume_deadline
                      and not self._ear_holds(now)):
                    # 귀가 열려 있거나 말이 LLM 으로 가는 중이면 기다린다 —
                    # 되물은 질문의 답을 듣는 중에 "응답이 없어"를 말하고
                    # 떠난 사고(2026-10-06 실기). 상한은 _ear_holds 몫.
                    if not self._return_notice_given:
                        self._return_notice_given = True
                        self._return_resume_deadline = now + LEAVING_GRACE_SEC
                        actions.append(Say(MSG_LEAVING_NOTICE, priority="response"))
                    else:
                        self._forget_interrupted_return()
                        actions.extend(self._go_home(now))

            # 온보딩 뒤 빈손 되묻기 사다리 (2026-09-11). 위 두 블록 중 하나가
            # 이번 tick 에 state 를 이미 바꿨을 수 있으므로(SEEKING 진입,
            # _go_home 의 RETURNING 전이) state 를 다시 본다 — 같은 이유로
            # 위 두 사다리도 서로 state 를 재확인한다. 사용자가 말하는 동안
            # (_dest_prompt_holds: speech·LLM 유예)은 시계가 끝나도 기다린다.
            # 창이 열렸다는 것(open)만으로는 기다리지 않는다 — 그 창은 30초다.
            if (self.state == State.IDLE and self._dest_prompt_stage is not None
                    and self._dest_prompt_deadline is not None
                    and now >= self._dest_prompt_deadline
                    and not self._dest_prompt_holds(now)):
                actions.extend(self._advance_dest_prompt(now))

        elif self.state == State.TURNING:
            if nav_status == NavStatus.SUCCEEDED:
                # 회전 완료 훅 (approach-voice-flow.md 확정 흐름): 완료를 알리고
                # 온보딩을 말한다. 온보딩 끝은 질문 -> expects_reply 로 재청취
                # 창이 열리고, 회전으로 사용자가 핸들 방향에 정렬됐으므로
                # DOA 방향 관문도 자연히 유효해진다.
                self._to_idle()
                # 사용자가 손잡이를 받아든 시점이다 — 재청취 창이 만료된 뒤
                # 다른 "비카야"가 이 사람을 새 호출로 오인하지 않도록 얼마간
                # wake_doa 를 거절한다. _to_idle() 은 이 값을 지우지 않으므로
                # 호출 순서는 상관없다.
                self._user_attached_until = now + USER_ATTACHED_SUPPRESS_SEC
                # 회전 완료 멘트는 2026-09-01 감량 — 바로 뒤 손잡이 안내가
                # 완료를 대신한다. 손잡이 안내 + 진동으로 잡기 대기를 연다.
                # 온보딩(빈손 되묻기 사다리 포함)은 잡거나 시간이 다 되면 나간다.
                actions.extend(self._start_grip_wait(now))
            elif nav_status in (NavStatus.FAILED, NavStatus.CANCELED):
                # 회전 실패에 "완료되었습니다"는 거짓말 - 생략한다. 다만 방금
                # 수락한 사람을 침묵 속에 버려두지 않도록 온보딩은 한다.
                # (핸들 방향은 어긋났을 수 있다 - 안내 실패는 아니다.)
                self._to_idle()
                # 회전이 실패해도 사용자는 이미 승낙하고 그 자리에 있다 —
                # 위와 같은 억제를 건다.
                self._user_attached_until = now + USER_ATTACHED_SUPPRESS_SEC
                # 회전이 실패해도 손잡이 안내·잡기 대기는 그대로다 — 사용자는
                # 이미 승낙하고 서 있다(2026-09-10 사용자 결정).
                actions.extend(self._start_grip_wait(now))
            elif (self._turn_deadline is not None
                  and now >= self._turn_deadline):
                # spin 이 시작조차 안 됐다(노드 결함 등). 시계로 탈출한다.
                self._approach_dest = None
                self._to_idle()

        elif self.state == State.AWAITING_USER:
            # 여기서 하는 일은 시계를 보는 것뿐이다. 말을 알아듣는 쪽은 음성이다.
            if self._approach_after_reply is not None:
                if now >= self._approach_after_reply[1]:
                    # LLM 답의 재생 끝 소식이 끝내 안 왔다 — 기다리지 않고 다시 묻거나 물러난다.
                    actions.extend(self._approach_follow_up(now))
            elif (self._response_deadline is not None and now >= self._response_deadline
                    and not self._ear_holds(now)):
                # 8초 침묵 — 세 번까지 다시 묻고, 그 뒤면 물러난다(2026-10-09 사용자 결정, 옛 동작은
                # 곧장 "실례했습니다"). 사람이 말하는 중이거나 방금 한 말을 알아듣는
                # 중이면 기다린다 — 도착 질문과 같은 귀 유예(run82 17:19, 대답 처리 중에 떠났다).
                actions.extend(self._approach_follow_up(now))

        elif self.state in (State.ASKING_NEXT, State.ASKING_WAIT_TIME):
            # 무응답 사다리 (arrival-dialog 3절). 떠나기 예고 후면 유예를 세고,
            # 아니면 8초 침묵을 센다. 침묵은 같은 질문을 한 번 더 묻고(2026-09-20,
            # _arrival_silence) 그래도 침묵이면 예고로 간다. 말을 알아듣는 쪽은 음성이다.
            if (self._leaving_deadline is not None
                    and now >= self._leaving_deadline
                    and not self._ear_holds(now)):
                # 답이 없어 떠난다 — 홈 가는 중 늦게 온 "네·아니요"를 이 질문의 뜻으로 받는다
                # (2026-10-08 결정 1). 물어 둔 질문이 없었으면 받을 뜻도 없다.
                self._late_answer_finish = (
                    self._asking_is_finish if self._asking_question else None)
                self._reset_arrival_dialog()
                # 복귀 멘트는 2026-09-01 감량 — 직전 떠나기 예고가 이미 말했다.
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
                # 질문 재생이 끊겨 tts_done(시계 기점)이 유실된 경우 —
                # 영구 대기 대신 강제로 무응답 절차를 연다 (구멍 ①).
                actions.extend(self._leaving_notice(now))

        elif self.state == State.WAITING_RELEASE:
            # M2 를 말했다. 끝나고 손을 놓으면 대기 장소로 떠난다(2026-10-07).
            # 손을 끝내 놓지 않아도(센서가 계속 '잡힘') 대기 시간은 흐르므로 만료를 본다 —
            # 안 보면 로봇이 M7·앱 알림 없이 영원히 선다.
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
                # E-stop 취소는 이미 ESTOPPED 로 빠졌으므로 여기 온 것은 못 들어간 것이다.
                actions.extend(self._wait_spot_blocked(now))
            actions.extend(self._beacon_tick(now))

        elif self.state == State.MOVING_BACK_TO_DEST:
            # 목적지 복귀는 다시 시도하지 않는다 — 도착이든 실패든 그 자리에서 기다린다.
            if nav_status in (NavStatus.SUCCEEDED, NavStatus.FAILED,
                              NavStatus.CANCELED):
                self.state = State.WAITING
                self.active_destination = None
            actions.extend(self._beacon_tick(now))

        elif self.state == State.WAITING:
            # 대기. 사람접근은 이 상태값으로 자연히 꺼진다(_GOAL_ACTIVE 아님·IDLE
            # 아님). 시간이 다 되면 M7 을 말하고 홈으로 간다(2026-10-07, 옛 동작은
            # 예고 없이 홈). 사용자와 말하는 중(귀가 바쁨)이면 끝날 때까지 미룬다.
            if (self._wait_until is not None and now >= self._wait_until
                    and not self._ear_holds(now)):
                actions.extend(self._wait_expired(now))
            else:
                # 질문을 다시 묻는 틱에는 M3 를 얹지 않는다 — 다음 틱의 M3 는 ambient 라 질문이
                # 나가는 동안 TTS 가 버린다.
                asked = self._wait_question_tick(now)
                actions.extend(asked)
                if not asked:
                    actions.extend(self._beacon_tick(now))

        elif self.state == State.RETURNING:
            # 복귀 실패도 완료로 친다. 대기 위치에 못 갔다고 접근 상태에 갇히면
            # 다음 사람을 아예 못 본다 — 복귀는 안전 사건이 아니다.
            #
            # active_destination 을 본다. return_destination 이 아니다 —
            # auto_return_home 이 꺼져 있으면 홈이 지정돼 있어도 이번 복귀는
            # 목적지가 없고, 그때 return_destination 을 보면 오지 않을 주행
            # 결과를 영영 기다린다.
            if self.active_destination is None or nav_status in (
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
                    actions.append(Navigate(dest, tree=self._nav_tree))
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
                    announced = self._estop_announced
                    self._estop_announced = False
                    self._to_idle()
                    if announced:
                        # 해제를 못 들으면 사용자는 계속 멈춰 있는 줄 안다.
                        # 침묵 걸림(정지 중)은 해제도 침묵 — 짝을 맞춘다.
                        # emergency 등급: 걸림 멘트가 아직 재생 중이면 끊고
                        # 즉시 해제를 알린다(2026-08-31). response 면 걸림
                        # 멘트 완주를 기다려 낡은 소식이 된다.
                        actions.append(Say(MSG_ESTOP_RELEASED, priority="emergency"))

        # 홈 알림은 상태 분기 뒤에 본다 — 이번 tick 에 IDLE 을 떠났으면 박자를 지운다.
        actions.extend(self._home_beacon_tick(now))
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

    def on_return_home_request(self, nav_ready: bool, now: float) -> tuple:
        """관리자가 앱에서 홈 복귀를 요청했다(`/vica/mission/return_home`).

        **사용자(음성)는 이 경로로 들어올 수 없다.** 홈은 목적지 카탈로그에
        없어서 UUID 가 없고, 음성은 목적지를 UUID 로 지목하기 때문이다.

        게이트를 통과하면 접근 뒤 복귀와 **같은 상태·같은 좌표**로 간다.
        가는 곳이 같으므로 상태를 새로 만들지 않고, 끝낼 때 할 일만
        `_returning_home` 플래그로 가른다.

        Returns:
            (허용 여부, 사유, 실행할 동작 목록)
        """
        reason = check_return_home_gate(
            self.state, self.return_destination, self.estop_active, nav_ready
        )
        if reason is not GateReason.OK:
            return False, reason, []
        # 끊긴 복귀를 잊는다 — 관리자가 지금 직접 복귀를 명령했으니 이번이
        # 그 재개다(on_intent 와 같은 이유).
        self._forget_interrupted_return()
        return True, reason, self._enter_returning(now, is_home=True)

    def _enter_returning(
        self, now: float, is_home: bool = False, dialog_finish: bool = False
    ) -> list:
        """대기 위치(= 홈)로 돌아간다.

        들어오는 길이 셋이다.
          - 접근을 거절당했거나 답을 못 들었을 때 (기본)
          - 관리자가 앱에서 홈 복귀를 눌렀을 때 (`is_home=True`)
          - 도착 후 대화에서 사용자가 안내 종료를 골랐을 때 (`dialog_finish=True`)

        가는 곳은 같다. 복귀는 접근이 아니므로 0.3 m/s 제한을 여기서 푼다.
        홈이 지정되지 않았으면 제자리에서 끝낸다 — 상태만 지나가고 다음 tick 에
        IDLE 로 내려간다.

        track_id 는 아직 지우지 않는다. 재접근 억제는 복귀가 끝난 시점부터
        세야 하므로 _finish_returning 까지 들고 간다(설계 4절).
        """
        # 목적지를 떠난다 — 도착 대화의 목적지는 비운다. 안내를 마치고 떠나는 복귀라면
        # _last_guided 가 그 목적지를 기억한다: 홈 가는 중 "기다려"는 그 대기 장소로 간다
        # (2026-10-08 사용자 결정 1 — 10-07 의 '그 자리에서 기다린다'를 바꿨다). 접근 뒤·
        # 관리자 복귀에는 돌아갈 안내 목적지가 없다.
        self._arrived_destination = None
        if not dialog_finish:
            self._last_guided = None
            self._late_answer_finish = None
        # 입구 방향(M1 기준)도 떠나면 낡은 말이다 — LLM 메모에 옛 목적지 방향이 남지 않게.
        self.door_side = ""
        # auto_return_home 게이트는 접근 뒤 복귀에만 걸린다 — 꺼져 있으면 그
        # 자리에 선 채로 상태만 정리한다. 관리자 복귀와 안내 종료는 늘 홈으로
        # 간다(2026-09-01 시나리오 확인).
        destination = (
            self.return_destination
            if is_home or dialog_finish or self.auto_return_home
            else None
        )
        self.state = State.RETURNING
        self._returning_home = is_home
        self.active_destination = destination
        # 복귀는 사용자를 태우지 않는다 — 손잡이 안내는 여기서 끝이다.
        self._handle_engaged = False
        self.handle_active = False
        self.approach_goal_pose = None
        self._response_deadline = None
        self._announced_milestones = set()
        self._distance_baseline = None
        self._approach.reset()
        # 어떤 이유로든 주행이 시작되면 온보딩 되묻기 사다리는 끝이다.
        self._forget_dest_prompt()
        actions: list = [SetNavSpeedLimit(NO_SPEED_LIMIT)]
        if destination is not None:
            actions.append(Navigate(destination))
        return actions

    def _finish_returning(self, now: float) -> None:
        """복귀 완료.

        접근 뒤 복귀였다면 이 시점부터 같은 사람에 대한 재접근 억제를 센다.
        관리자 홈 복귀였다면 **억제하지 않는다** — 억제할 사람이 없고, 그냥
        걸면 마지막에 만났던 track_id 가 걸려 **다음 사람을 이유 없이 무시한다.**
        """
        track_id = self.approach_track_id
        was_home = self._returning_home
        # 홈에 왔다 — 홈 가는 중의 "기다려"·늦은 답은 여기서 끝이다(결정 1).
        self._last_guided = None
        self._late_answer_finish = None
        self._to_idle()
        if not was_home:
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
        # 접근도 함께 버린다. 목적지를 보관하지 않으므로 해제 뒤 자동 재개는
        # 없고, 사람에게 다시 가려면 탐지부터 다시 해야 한다(설계 4절).
        #
        # 억제까지 거는 것은 설계에 없는 판단이다. 방금 비상 정지가 걸린 그
        # 사람에게 해제 직후 로봇이 다시 다가가는 것이 더 나쁘다고 봤다.
        self._suppress_track(self.approach_track_id, now)
        self.approach_track_id = None
        self.approach_goal_pose = None
        self._response_deadline = None
        self._nav_from_app = False
        # 손잡이 상태도 버린다 — 해제 뒤 자동 재개가 없으니 모드도 새로 정한다.
        self._handle_engaged = False
        self.handle_active = False
        self._grip_wait_since = None
        self._grip_pulse_at = None
        self._clear_handle_pause()
        # 대기 장소 흐름도 버린다(2026-10-07) — 해제 뒤 옛 목적지 대기 장소로 가지 않는다.
        self._arrived_destination = None
        self.door_side = ""
        # 대기 질문·대기 시간 기억도 버린다(2026-10-09 검토 I-4) — 남기면 해제 뒤 새 안내의
        # 대기에서 옛 "안내가 필요 없으신가요?"가 나오고 옛 대기 시간이 쓰인다.
        self._reset_arrival_dialog()
        # 주행 중 바꾸기 보류도 버린다(2026-10-07 검토). 온보딩 되묻기 사다리는 그대로
        # 멈춰 두었다가 해제 뒤 _to_idle 이 지운다(09-11 설계) — 비상 중 빈손 신호로
        # 전진하던 길은 on_listen_state 가 IDLE 에서만 전진시켜 막는다.
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
        # 복귀 재개 사다리의 "언제"만 지운다 — 탐색 회전이 여기를 지나갈 때마다
        # 시계를 멈추기 위해서다. "사실"(_return_interrupted)은 남겨 둔다:
        # 여기서 같이 지우면 회전 한 번으로 끊긴 복귀를 영영 잊는다(설계 요점).
        self._return_resume_deadline = None
        self._return_notice_given = False
        # 온보딩 되묻기 사다리는 위 복귀 재개 사다리와 달리 "사실"까지
        # 통째로 지운다 — 이어받을 회전이 없다. E-stop 해제처럼
        # on_wake·on_intent·_enter_returning 어느 것도 거치지 않고 곧장
        # 여기로 오는 경로가 있어(on_tick 의 ESTOPPED 분기), 이 함수 자체가
        # 마지막 청산 지점이다.
        self._dest_prompt_stage = None
        self._dest_prompt_deadline = None
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
        # 복귀 종류 표시도 함께 비운다. 남겨 두면 다음 접근 뒤 복귀가 홈 복귀로
        # 오인되어 재접근 억제가 걸리지 않는다.
        self._returning_home = False
        self.approach_goal_pose = None
        self._response_deadline = None
        self._nav_from_app = False
        # 다음 주행이 스스로 다시 정한다. 남겨 두면 재시도가 낡은 트리를 탄다.
        self._nav_tree = NAV_TREE_DEFAULT
        # 대기 장소의 흔적도 내린다. 특히 확인 보류(_wait_hold)가 남으면 E-stop 뒤
        # 평범한 확인 질문이 엉뚱하게 '대기로 돌아가기'가 된다. 대기에서 확인으로
        # 들어간 경우는 _fold_confirming 이 이 함수를 거치지 않고 되돌린다.
        self._wait_hold = None
        # 주행 중 바꾸기 보류도 같다 — 남으면 E-stop·취소 뒤 평범한 확인 질문의 거절이
        # 엉뚱하게 옛 목적지로의 재출발이 된다.
        self._change_from = None
        self._wait_place = ""
        self._beacon_next_at = None
        self._release_text = ""
        self._release_spoken_at = None
        self._release_entered_at = None
        self._wait_finish_asked_at = None
        # 다음 AWAITING_USER 진입(정상 접근·근접 호출 어느 쪽이든)이 각자 다시
        # 명시적으로 정하므로, 여기서 지우지 않아도 안전과는 무관하다 —
        # 다만 묵은 값을 들고 있을 이유도 없어 다른 접근 상태값들과 함께 비운다.
        self._near_call_no_spin = False
        self._never_approached = False
        # 손잡이: 주행이 끝났으니 모드·놓침 정지·잡기 대기를 내린다. 잡기 대기를
        # 여는 자리(_start_grip_wait)는 _to_idle() 뒤에 부르므로 지워지지 않는다.
        # _handle_engaged 는 남긴다 — 확인 시간초과처럼 IDLE 을 거쳐 다시 목적지를
        # 말하는 사람도 같은 사람이다(_decide_handle_mode 가 60초 자물쇠로 거른다).
        self.handle_active = False
        self._grip_wait_since = None
        self._grip_pulse_at = None
        self._clear_handle_pause()
