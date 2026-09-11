"""Map /diagnostics_agg (or flat /diagnostics) items onto VICA components."""

from typing import NamedTuple

from .fault_catalog import COMPONENTS


DIAG_OK = 0
DIAG_WARN = 1
DIAG_ERROR = 2
DIAG_STALE = 3

DIAG_FAULT_THRESHOLD = DIAG_WARN

FALLBACK_COMPONENT = 'monitor'

_NAME_HINTS = (
    ('mdrobot', 'motor'),
    ('motor', 'motor'),
    ('can link', 'motor'),
    ('/scan', 'lidar'),
    ('lidar', 'lidar'),
    ('rplidar', 'lidar'),
    ('nvblox', 'perception'),
    ('camera', 'perception'),
    ('perception', 'perception'),
    ('nav2', 'navigation'),
    ('navigation', 'navigation'),
    ('controller_server', 'navigation'),
    ('planner_server', 'navigation'),
    ('bt_navigator', 'navigation'),
    ('/odom', 'localization'),
    ('wheel/odom', 'localization'),
    ('ekf', 'localization'),
    ('localization', 'localization'),
    ('safety', 'safety'),
    ('emergency', 'safety'),
    ('smart_handle', 'guidance'),
    ('guidance', 'guidance'),
    ('turn_guide', 'guidance'),
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
    """Translate an aggregator-generated English summary into Korean."""
    if not message or not isinstance(message, str):
        return ''
    return _AGG_MESSAGES.get(message.strip().lower(), message)


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
    """Return the VICA component that this diagnostic name belongs to."""
    if not name:
        return FALLBACK_COMPONENT

    lowered = name.lower()

    for hint, component in _NAME_HINTS:
        if hint in lowered:
            return component

    for token in reversed([t.strip() for t in lowered.split('/') if t.strip()]):
        if token in COMPONENTS:
            return token

    return FALLBACK_COMPONENT


def normalize_level(level: object) -> int:
    """Convert a diagnostic level to int no matter how it arrives."""
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
    """Map a diagnostic level onto the catalog passthrough code."""
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
        """Message ready to show an administrator."""
        return localize_message(self.message)


def from_status(name: str, level: object, message: str) -> DiagItem:
    """Build a DiagItem from raw DiagnosticStatus fields."""
    return DiagItem(name=name, level=normalize_level(level), message=message or '')
