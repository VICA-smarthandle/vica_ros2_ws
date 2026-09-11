"""Unit tests for the Nav2 liveness fallback."""

from vica_system_monitor.agg_parser import (
    DIAG_ERROR,
    DIAG_OK,
    DIAG_STALE,
    DIAG_WARN,
    from_status,
)
from vica_system_monitor.nav2_liveness import (
    decide_poll_action,
    is_nav2_active,
    message_says_active,
    POLL_ASK,
    POLL_FALLBACK,
    POLL_WAIT,
)


NOW_NS = 1_000_000_000_000
TIMEOUT_NS = 5_000_000_000

NAV2_DIAG_NAME = '/VICA/Navigation/lifecycle_manager_navigation: Nav2 Health'


def _items(message, level=DIAG_OK, age_ns=0, name=NAV2_DIAG_NAME):
    """Build a diag_items mapping shaped like the monitor's own store."""
    return {name: (from_status(name, level, message), NOW_NS - age_ns)}


def test_active_message_reads_as_active():
    """lifecycle_manager가 활성이라고 말하면 활성이다."""
    assert message_says_active('Nav2 is active') is True


def test_inactive_message_does_not_read_as_active():
    """'inactive' 안에 'active'가 들어 있다고 활성으로 읽으면 안 된다."""
    assert message_says_active('Nav2 is inactive') is False


def test_message_matching_ignores_case_and_padding():
    """표기 흔들림은 흡수한다 — 판정이 뒤집히는 것은 'in' 접두사뿐이다."""
    assert message_says_active('  NAV2 IS ACTIVE  ') is True


def test_empty_message_is_not_active():
    """근거가 없으면 활성이라고 말하지 않는다."""
    assert message_says_active('') is False
    assert message_says_active(None) is False


def test_fresh_active_item_reports_active():
    """신선하고 OK이며 활성 문구인 항목만 활성 근거가 된다."""
    assert is_nav2_active(_items('Nav2 is active'), NOW_NS, TIMEOUT_NS) is True


def test_inactive_item_reports_not_active():
    """Nav2가 내려갔다고 말하는 항목은 활성이 아니다."""
    assert is_nav2_active(_items('Nav2 is inactive', level=DIAG_ERROR),
                          NOW_NS, TIMEOUT_NS) is False


def test_stale_timestamp_is_not_active():
    """마지막 수신이 오래된 항목은 활성 근거가 되지 못한다."""
    stale = _items('Nav2 is active', age_ns=TIMEOUT_NS + 1)
    assert is_nav2_active(stale, NOW_NS, TIMEOUT_NS) is False


def test_stale_level_is_not_active():
    """aggregator가 level만 STALE로 덮어써도 활성으로 읽지 않는다."""
    stale_level = _items('Nav2 is active', level=DIAG_STALE)
    assert is_nav2_active(stale_level, NOW_NS, TIMEOUT_NS) is False


def test_warn_level_is_not_active():
    """OK가 아닌 등급은 활성 근거로 쓰지 않는다 — 이 판정은 fail-safe다."""
    assert is_nav2_active(_items('Nav2 is active', level=DIAG_WARN),
                          NOW_NS, TIMEOUT_NS) is False


def test_unrelated_diagnostic_is_ignored():
    """lifecycle_manager의 Nav2 Health 항목이 아니면 보지 않는다."""
    other = _items('Nav2 is active', name='/VICA/Hardware/Motor/CAN link')
    assert is_nav2_active(other, NOW_NS, TIMEOUT_NS) is False


def test_localization_lifecycle_manager_is_not_nav2():
    """lifecycle_manager_localization은 주행 lifecycle이 아니다."""
    localization = _items(
        'Nav2 is active',
        name='/VICA/Navigation/lifecycle_manager_localization: Nav2 Health',
    )
    assert is_nav2_active(localization, NOW_NS, TIMEOUT_NS) is False


def test_active_localization_does_not_cover_a_dead_navigation():
    """위치추정 manager가 살아 있다고 주행 manager의 죽음을 덮으면 안 된다."""
    items = _items('Nav2 is inactive', level=DIAG_ERROR)
    items.update(_items(
        'Nav2 is active',
        name='/VICA/Navigation/lifecycle_manager_localization: Nav2 Health',
    ))
    assert is_nav2_active(items, NOW_NS, TIMEOUT_NS) is False


def test_no_items_is_not_active():
    """진단을 하나도 못 받았으면 활성이 아니다."""
    assert is_nav2_active({}, NOW_NS, TIMEOUT_NS) is False


def test_idle_and_ready_asks_the_service():
    """서비스가 준비돼 있으면 서비스에 묻는다 — 그쪽이 더 강한 근거다."""
    action = decide_poll_action(
        in_flight=False, call_started_ns=None, service_ready=True,
        now_ns=NOW_NS, timeout_ns=TIMEOUT_NS,
    )
    assert action == POLL_ASK


def test_idle_and_not_ready_falls_back():
    """서비스가 아직 없으면 진단으로 판정한다."""
    action = decide_poll_action(
        in_flight=False, call_started_ns=None, service_ready=False,
        now_ns=NOW_NS, timeout_ns=TIMEOUT_NS,
    )
    assert action == POLL_FALLBACK


def test_recent_call_still_waits():
    """방금 부른 호출은 기다린다. 매 tick마다 새로 부르면 안 된다."""
    action = decide_poll_action(
        in_flight=True, call_started_ns=NOW_NS - 1, service_ready=True,
        now_ns=NOW_NS, timeout_ns=TIMEOUT_NS,
    )
    assert action == POLL_WAIT


def test_hung_call_falls_back_instead_of_waiting_forever():
    """응답 없이 멈춘 호출은 포기하고 진단으로 판정한다."""
    action = decide_poll_action(
        in_flight=True, call_started_ns=NOW_NS - TIMEOUT_NS - 1, service_ready=True,
        now_ns=NOW_NS, timeout_ns=TIMEOUT_NS,
    )
    assert action == POLL_FALLBACK


def test_in_flight_without_a_start_stamp_falls_back():
    """시작 시각을 모르는 in_flight는 붙들지 않는다 — 영구 정지를 막는 fail-safe다."""
    action = decide_poll_action(
        in_flight=True, call_started_ns=None, service_ready=True,
        now_ns=NOW_NS, timeout_ns=TIMEOUT_NS,
    )
    assert action == POLL_FALLBACK
