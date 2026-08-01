"""Boundary tests for the shared steady-clock freshness predicate.

이 계약은 vica_safety·mdrobot_can_control과 동일해야 한다. 사본이 세 개이므로 경계
동작을 각 패키지에서 각각 고정한다.
"""

from vica_system_monitor.freshness import is_fresh_ns, sec_to_ns


SEC = 1_000_000_000


def test_sec_to_ns_converts_whole_and_fractional():
    """초를 정수 나노초로 바꾼다."""
    assert sec_to_ns(1.0) == SEC
    assert sec_to_ns(0.5) == SEC // 2
    assert sec_to_ns(0.0) == 0


def test_age_below_timeout_is_fresh():
    """Timeout 안이면 신선하다."""
    assert is_fresh_ns(0, now_ns=SEC // 2, timeout_ns=SEC)


def test_age_exactly_at_timeout_is_fresh():
    """경계값: age == timeout은 신선하다."""
    assert is_fresh_ns(0, now_ns=SEC, timeout_ns=SEC)


def test_age_one_ns_over_timeout_is_stale():
    """경계값: timeout + 1 ns는 stale이다."""
    assert not is_fresh_ns(0, now_ns=SEC + 1, timeout_ns=SEC)


def test_never_received_is_stale():
    """미수신은 None이며 stale이다. 0.0이 아니다."""
    assert not is_fresh_ns(None, now_ns=SEC, timeout_ns=SEC)


def test_time_reversal_is_stale():
    """음수 age는 stale로 처리해 clock jump가 fail-safe하게 만든다."""
    assert not is_fresh_ns(2 * SEC, now_ns=SEC, timeout_ns=SEC)


def test_zero_timeout_only_accepts_same_instant():
    """Timeout 0이면 같은 시각만 신선하다."""
    assert is_fresh_ns(SEC, now_ns=SEC, timeout_ns=0)
    assert not is_fresh_ns(SEC, now_ns=SEC + 1, timeout_ns=0)


def test_matches_safety_package_contract():
    """vica_safety 사본과 같은 식임을 명시한다.

    fresh = last_ns is not None and 0 <= (now_ns - last_ns) <= timeout_ns
    """
    for last, now, timeout, expected in (
        (None, 0, 0, False),
        (0, 0, 0, True),
        (0, 1, 0, False),
        (1, 0, 5, False),
        (0, 5, 5, True),
        (0, 6, 5, False),
    ):
        assert is_fresh_ns(last, now_ns=now, timeout_ns=timeout) is expected
