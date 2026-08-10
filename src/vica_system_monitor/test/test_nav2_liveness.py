"""Unit tests for the Nav2 liveness fallback.

`<node>/get_state`가 응답하지 않을 때 lifecycle_manager 진단을 두 번째 근거로 쓴다.
그 판정이 틀리면 앱에 "주행 준비 완료"가 뜨는데 실제로는 Nav2가 내려가 있다 — 관리자가
로봇을 출발시키고 나서야 알게 된다. 그래서 세 가지를 모두 시험한다.

    문구  : "Nav2 is inactive"를 활성으로 읽지 않는가
    등급  : level이 ERROR·STALE인 항목을 활성 근거로 쓰지 않는가
    신선도: 낡은 항목을 계속 활성으로 붙들지 않는가
"""

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


# ---------------------------------------------------------------------------
# 문구 판정
# ---------------------------------------------------------------------------


def test_active_message_reads_as_active():
    """lifecycle_manager가 활성이라고 말하면 활성이다."""
    assert message_says_active('Nav2 is active') is True


def test_inactive_message_does_not_read_as_active():
    """'inactive' 안에 'active'가 들어 있다고 활성으로 읽으면 안 된다.

    nav2_lifecycle_manager는 system_active_가 false일 때 "Nav2 is inactive"를 낸다
    (libnav2_lifecycle_manager_core.so에서 문자열 확인). 부분 문자열로 보면 Nav2가
    죽은 바로 그 순간에 정상으로 보고한다.
    """
    assert message_says_active('Nav2 is inactive') is False


def test_message_matching_ignores_case_and_padding():
    """표기 흔들림은 흡수한다 — 판정이 뒤집히는 것은 'in' 접두사뿐이다."""
    assert message_says_active('  NAV2 IS ACTIVE  ') is True


def test_empty_message_is_not_active():
    """근거가 없으면 활성이라고 말하지 않는다."""
    assert message_says_active('') is False
    assert message_says_active(None) is False


# ---------------------------------------------------------------------------
# 항목 선별
# ---------------------------------------------------------------------------


def test_fresh_active_item_reports_active():
    """신선하고 OK이며 활성 문구인 항목만 활성 근거가 된다."""
    assert is_nav2_active(_items('Nav2 is active'), NOW_NS, TIMEOUT_NS) is True


def test_inactive_item_reports_not_active():
    """Nav2가 내려갔다고 말하는 항목은 활성이 아니다."""
    assert is_nav2_active(_items('Nav2 is inactive', level=DIAG_ERROR),
                          NOW_NS, TIMEOUT_NS) is False


def test_stale_timestamp_is_not_active():
    """마지막 수신이 오래된 항목은 활성 근거가 되지 못한다.

    diag_items는 만료 제거를 하지 않는다. 신선도를 보지 않으면 Nav2가 죽은 뒤에도
    마지막 "Nav2 is active"가 dict에 남아 활성 판정이 영구히 고정된다.
    """
    stale = _items('Nav2 is active', age_ns=TIMEOUT_NS + 1)
    assert is_nav2_active(stale, NOW_NS, TIMEOUT_NS) is False


def test_stale_level_is_not_active():
    """aggregator가 level만 STALE로 덮어써도 활성으로 읽지 않는다.

    diagnostic_aggregator는 발행이 끊긴 항목의 message는 그대로 둔 채 level만 STALE로
    바꿔 재발행한다. 문구만 보면 "Nav2 is active"가 계속 보인다.
    """
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
    """lifecycle_manager_localization은 주행 lifecycle이 아니다.

    두 manager 모두 nav2_lifecycle_manager라서 진단 이름이 똑같이 `Nav2 Health`이고
    문구도 똑같이 "Nav2 is active"다. agg_parser 주석의 2026-08-01 오분류와 같은 충돌이라
    manager 노드 이름으로 갈라야 한다.
    """
    localization = _items(
        'Nav2 is active',
        name='/VICA/Navigation/lifecycle_manager_localization: Nav2 Health',
    )
    assert is_nav2_active(localization, NOW_NS, TIMEOUT_NS) is False


def test_active_localization_does_not_cover_a_dead_navigation():
    """위치추정 manager가 살아 있다고 주행 manager의 죽음을 덮으면 안 된다.

    이 조합이 실제 사고 형태다. AMCL 쪽은 멀쩡한데 주행 스택만 내려간 상태에서
    앱에 "주행 준비 완료"가 뜬다.
    """
    items = _items('Nav2 is inactive', level=DIAG_ERROR)
    items.update(_items(
        'Nav2 is active',
        name='/VICA/Navigation/lifecycle_manager_localization: Nav2 Health',
    ))
    assert is_nav2_active(items, NOW_NS, TIMEOUT_NS) is False


def test_no_items_is_not_active():
    """진단을 하나도 못 받았으면 활성이 아니다."""
    assert is_nav2_active({}, NOW_NS, TIMEOUT_NS) is False


# ---------------------------------------------------------------------------
# 폴링 행동 결정
# ---------------------------------------------------------------------------


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
    """응답 없이 멈춘 호출은 포기하고 진단으로 판정한다.

    2026-08-01 실기 사건이 이것이다. `/bt_navigator/get_state`가 10분간 반환되지 않았다.
    서버는 떠 있으므로 service_is_ready()는 계속 True이고, future는 끝나지 않는다.
    포기 시한이 없으면 in_flight가 영원히 True로 남아 폴링이 멈추고, fallback도
    'service_ready가 False일 때'에만 달려 있어 끝내 실행되지 않는다.
    """
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
