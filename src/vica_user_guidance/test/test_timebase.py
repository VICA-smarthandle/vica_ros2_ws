"""timebase 계약 테스트.

계약 정본은 vica_safety/freshness.py이며 이 테스트는 그쪽 test_freshness.py의
케이스를 그대로 복제한 것이다. 두 구현이 갈라지면 여기서 잡힌다.
"""

from vica_user_guidance.timebase import is_fresh_ns, sec_to_ns


def test_sec_to_ns_returns_integer_nanoseconds():
    """타임아웃은 정수 나노초로 변환한다."""
    assert sec_to_ns(1.0) == 1_000_000_000
    assert sec_to_ns(0.5) == 500_000_000
    assert isinstance(sec_to_ns(0.3), int)


def test_missing_timestamp_is_stale():
    """미수신은 None이며 stale이다. 0.0을 sentinel로 쓰지 않는다."""
    assert is_fresh_ns(None, 1_000_000_000, 500_000_000) is False


def test_age_zero_is_fresh():
    """방금 수신한 값은 fresh다."""
    assert is_fresh_ns(1_000_000_000, 1_000_000_000, 500_000_000) is True


def test_age_equal_to_timeout_is_fresh():
    """경계값(age == timeout)은 fresh다."""
    assert is_fresh_ns(500_000_000, 1_000_000_000, 500_000_000) is True


def test_age_over_timeout_is_stale():
    """경계 + 1ns는 stale이다."""
    assert is_fresh_ns(499_999_999, 1_000_000_000, 500_000_000) is False


def test_time_reversal_is_stale():
    """시간 역전(음수 age)은 stale로 처리해 clock jump에 fail-safe한다."""
    assert is_fresh_ns(2_000_000_000, 1_000_000_000, 500_000_000) is False
