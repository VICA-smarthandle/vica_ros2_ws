"""Judge Nav2 liveness from lifecycle_manager's own diagnostic."""

from .agg_parser import DIAG_OK
from .freshness import is_fresh_ns


ACTIVE_MESSAGE = 'nav2 is active'

NAV2_MANAGER_FRAGMENT = 'lifecycle_manager_navigation'
NAV2_HEALTH_FRAGMENT = 'nav2 health'


POLL_WAIT = 'wait'
POLL_ASK = 'ask'
POLL_FALLBACK = 'fallback'


def decide_poll_action(in_flight, call_started_ns, service_ready, now_ns, timeout_ns):
    """Decide what poll_nav2_state should do this tick."""
    if in_flight:
        if is_fresh_ns(call_started_ns, now_ns=now_ns, timeout_ns=timeout_ns):
            return POLL_WAIT
        return POLL_FALLBACK
    return POLL_ASK if service_ready else POLL_FALLBACK


def message_says_active(message) -> bool:
    """Report whether this diagnostic message says Nav2 is active."""
    if not message:
        return False
    return ' '.join(str(message).split()).lower() == ACTIVE_MESSAGE


def is_nav2_manager_diag(name) -> bool:
    """Report whether this diagnostic name is the navigation lifecycle manager's."""
    if not name:
        return False
    lowered = str(name).lower()
    return NAV2_MANAGER_FRAGMENT in lowered and NAV2_HEALTH_FRAGMENT in lowered


def is_nav2_active(diag_items, now_ns: int, timeout_ns: int) -> bool:
    """Report whether the navigation lifecycle manager says Nav2 is active."""
    for name, (item, seen_ns) in diag_items.items():
        if not is_nav2_manager_diag(name):
            continue
        if not is_fresh_ns(seen_ns, now_ns=now_ns, timeout_ns=timeout_ns):
            continue
        if item.level != DIAG_OK:
            continue
        if message_says_active(item.message):
            return True
    return False
