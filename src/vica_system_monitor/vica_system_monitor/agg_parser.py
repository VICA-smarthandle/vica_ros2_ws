"""Map /diagnostics_agg (or flat /diagnostics) items onto VICA components.

ROS 의존이 없다. `DiagnosticStatus`의 name·level·message 세 값만 다룬다.

두 가지 name 형태를 모두 파싱한다.

    계층형 (aggregator 출력)  : /VICA/Hardware/Motor/CAN link
    평면형 (aggregator 우회)  : mdrobot_can_keyboard_knob_node: CAN link

모니터의 `diagnostics_topic` 파라미터로 aggregator 없이 단독 디버깅할 수 있어야 하므로
두 형태를 함께 지원한다.

모르는 name은 버리지 않고 `monitor` 컴포넌트로 모아 표시한다. 버리면 다른 노드가 진단을
추가했을 때 조용히 사라져서, 나중에 "health가 정상이라고 했는데 왜 못 잡았나"가 생긴다.
"""

from typing import NamedTuple

from .fault_catalog import COMPONENTS


# diagnostic_msgs/DiagnosticStatus의 level 상수와 같은 값이다.
DIAG_OK = 0
DIAG_WARN = 1
DIAG_ERROR = 2
DIAG_STALE = 3

# 결함으로 취급하는 최소 level.
DIAG_FAULT_THRESHOLD = DIAG_WARN

# 알 수 없는 name을 모으는 컴포넌트.
FALLBACK_COMPONENT = 'monitor'

# name 조각(소문자) → 컴포넌트. 계층형·평면형 모두 이 표로 판정한다.
# 왼쪽 항이 name 안에 부분 문자열로 있으면 매칭한다. **위쪽이 우선이다.**
#
# 순서가 곧 규칙이므로 새 항목을 아무 데나 끼우면 안 된다. 한 이름이 여러 항에
# 걸릴 때 위에 있는 것이 이긴다. 2026-08-01에 이것으로 오분류가 나왔다 —
# `lifecycle_manager_localization: Nav2 Health`는 'localization'과 'nav2'에 모두
# 걸리는데 'localization'이 위에 있어서 위치추정 항목으로 분류됐다. Nav2
# lifecycle 상태가 위치추정 상태로 보고되면 정비하는 사람이 엉뚱한 곳을 본다.
#
# 규칙: **더 구체적인 이름을 위에** 둔다. 노드 이름(nav2, bt_navigator)이
# 컴포넌트 이름(localization)보다 구체적이다.
_NAME_HINTS = (
    # 하드웨어
    ('mdrobot', 'motor'),
    ('motor', 'motor'),
    ('can link', 'motor'),
    ('/scan', 'lidar'),
    ('lidar', 'lidar'),
    ('rplidar', 'lidar'),
    ('nvblox', 'perception'),
    ('camera', 'perception'),
    ('perception', 'perception'),
    # 주행. 위치추정보다 먼저 본다 — 위 주석의 lifecycle_manager 오분류 때문이다.
    ('nav2', 'navigation'),
    ('navigation', 'navigation'),
    ('controller_server', 'navigation'),
    ('planner_server', 'navigation'),
    ('bt_navigator', 'navigation'),
    # 위치추정
    ('/odom', 'localization'),
    ('wheel/odom', 'localization'),
    ('ekf', 'localization'),
    ('localization', 'localization'),
    # 안전 · 안내
    ('safety', 'safety'),
    ('emergency', 'safety'),
    ('smart_handle', 'guidance'),
    ('guidance', 'guidance'),
    ('turn_guide', 'guidance'),
    # 음성 · 앱 · 시스템
    ('stt', 'voice'),
    ('tts', 'voice'),
    ('voice', 'voice'),
    ('rosbridge', 'app'),
    ('supervisor', 'app'),
    ('app', 'app'),
    ('mission', 'navigation'),
    ('cpu', 'computer'),
    ('ram', 'computer'),
    ('disk', 'computer'),
    ('computer', 'computer'),
    ('health_monitor', 'monitor'),
    ('system_monitor', 'monitor'),
)


# diagnostic_aggregator가 스스로 만드는 영문 요약어 → 관리자용 한국어.
#
# 그룹 노드(`/VICA/Hardware/Motor`)의 요약과 보고되지 않은 항목에 이 문자열이 붙는다.
# 우리가 만든 문구가 아니므로 fault_catalog에 없고, 그대로 두면 한국어 화면에 "Missing"
# 한 단어만 뜬다. 정보량도 없어서 관리자가 무엇이 없는지 알 수 없다.
#
# 여기 없는 문구는 손대지 않는다. 우리 노드가 쓴 한국어이거나 서드파티가 남긴 구체적인
# 진단이며, 둘 다 버리면 정비 단서가 사라진다.
_AGG_MESSAGES = {
    'missing': '진단 항목이 보고되지 않았습니다.',
    'stale': '진단이 갱신되지 않았습니다.',
    'error': '오류를 보고했습니다.',
    'warning': '경고를 보고했습니다.',
    'warn': '경고를 보고했습니다.',
    'ok': '정상입니다.',
    'no events recorded.': '아직 한 건도 수신하지 못했습니다.',
    'no events recorded': '아직 한 건도 수신하지 못했습니다.',
}


def localize_message(message: object) -> str:
    """Translate an aggregator-generated English summary into Korean.

    아는 요약어만 바꾸고 나머지는 원문을 유지한다. 모르는 문구를 버리거나 일반 문구로
    덮으면 정비하는 사람이 실제 원인을 볼 수 없다.
    """
    if not message or not isinstance(message, str):
        return ''
    return _AGG_MESSAGES.get(message.strip().lower(), message)


# 무시할 외부 진단. 판정에 넣지 않는다.
#
# 남의 노드가 내는 진단을 함부로 버리면 진짜 고장을 놓친다. 그래서 **한 번도 참인
# 적이 없었다는 근거가 있을 때만** 여기 올린다. 지금 한 건뿐이다.
#
# robot_localization의 `odometry/filtered topic status`는 2026-08-01 실기에서
# 기동 이후 단 한 번도 카운터가 돌지 않았다.
#
#     Events since startup: 0
#     Actual frequency: 0.000000
#     Minimum acceptable: 25.2 Hz
#
# 같은 시각 `/odom`은 24.7 Hz로 정상 발행 중이었다. `print_diagnostics: false`를
# ekf.yaml에 넣어도 사라지지 않아(설정을 읽는 것은 확인했다) 소비 쪽에서 막는다.
# robot_localization의 .cpp가 설치돼 있지 않아 정확한 기전은 확인하지 못했다.
#
# 이것이 상시 ERROR로 남으면 앱에 "주행 불가 · 위치추정 오류"가 계속 떠서
# 관리자가 진짜 결함을 무시하게 된다. 6절이 경고한 "감시 도구가 스스로 오탐을
# 만든다"의 실제 사례다.
#
# `/odom` 주기 감시는 우리 프로브가 대신한다(probes.yaml, min 20 / max 35 Hz).
# 2026-08-01에 RViz가 두 개 떠 CPU가 모자랐을 때 15.5 Hz를 실제로 잡아냈다.
IGNORED_NAME_FRAGMENTS = (
    'odometry/filtered topic status',
    'odometry filtered topic status',
)


def is_ignored(name: str) -> bool:
    """Report whether this diagnostic should be dropped before judging."""
    if not name:
        return False
    lowered = name.lower()
    return any(frag in lowered for frag in IGNORED_NAME_FRAGMENTS)


def parse_name(name: str) -> str:
    """Return the VICA component that this diagnostic name belongs to.

    매칭에 실패하면 FALLBACK_COMPONENT를 돌려준다. 예외를 던지지 않는다 — 감시 노드가
    파싱 실패로 죽으면 상태 표시 자체가 사라진다.
    """
    if not name:
        return FALLBACK_COMPONENT

    lowered = name.lower()

    for hint, component in _NAME_HINTS:
        if hint in lowered:
            return component

    # 계층형 경로의 마지막 그룹 이름이 컴포넌트와 그대로 일치하는 경우.
    for token in reversed([t.strip() for t in lowered.split('/') if t.strip()]):
        if token in COMPONENTS:
            return token

    return FALLBACK_COMPONENT


def normalize_level(level: object) -> int:
    """Convert a diagnostic level to int no matter how it arrives.

    rclpy·rosbridge·bag 재생에 따라 int, bytes, str로 올 수 있다.
    vica_status_app_node.py의 `_diagnostic_level`이 쓰던 방어와 같은 이유다.
    """
    if isinstance(level, bool):
        return int(level)
    if isinstance(level, int):
        return level
    if isinstance(level, (bytes, bytearray)) and len(level) > 0:
        return level[0]
    if isinstance(level, str) and len(level) > 0:
        return ord(level[0])
    return DIAG_OK


def to_fault_code(level: int) -> str:
    """Map a diagnostic level onto the catalog passthrough code.

    Stale을 ERROR와 구분하는 이유: "오류가 났다"와 "소식이 없다"는 정비하는 사람에게
    다른 정보다. 전자는 장치를 보고, 후자는 노드 실행 여부를 본다.
    """
    if level >= DIAG_STALE:
        return 'DIAG_COMPONENT_STALE'
    if level >= DIAG_ERROR:
        return 'DIAG_COMPONENT_ERROR'
    if level >= DIAG_WARN:
        return 'DIAG_COMPONENT_WARN'
    return ''


class DiagItem(NamedTuple):
    """One diagnostic status entry, already normalized."""

    name: str
    level: int
    message: str

    @property
    def component(self) -> str:
        """VICA component this item belongs to."""
        return parse_name(self.name)

    @property
    def is_fault(self) -> bool:
        """Report whether this item should surface as a fault."""
        return self.level >= DIAG_FAULT_THRESHOLD

    @property
    def fault_code(self) -> str:
        """Catalog code for this item, or empty when healthy."""
        return to_fault_code(self.level)

    @property
    def detail(self) -> str:
        """Message ready to show an administrator.

        `message`를 직접 쓰지 않는다. aggregator의 영문 요약어가 그대로 화면에 뜨는
        것을 막는 지점이 여기 하나뿐이다.
        """
        return localize_message(self.message)


def from_status(name: str, level: object, message: str) -> DiagItem:
    """Build a DiagItem from raw DiagnosticStatus fields."""
    return DiagItem(name=name, level=normalize_level(level), message=message or '')
