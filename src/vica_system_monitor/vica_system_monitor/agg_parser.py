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
# 왼쪽 항이 name 안에 부분 문자열로 있으면 매칭한다. 위쪽이 우선이다.
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
    # 위치추정 · 주행
    ('/odom', 'localization'),
    ('wheel/odom', 'localization'),
    ('ekf', 'localization'),
    ('localization', 'localization'),
    ('nav2', 'navigation'),
    ('navigation', 'navigation'),
    ('controller_server', 'navigation'),
    ('planner_server', 'navigation'),
    ('bt_navigator', 'navigation'),
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


def from_status(name: str, level: object, message: str) -> DiagItem:
    """Build a DiagItem from raw DiagnosticStatus fields."""
    return DiagItem(name=name, level=normalize_level(level), message=message or '')
