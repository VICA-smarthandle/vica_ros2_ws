"""Unit tests for the shared steady-clock freshness predicate."""

import pytest

from vica_safety.freshness import is_fresh_ns, sec_to_ns


def test_never_received_is_stale():
    # 한 번도 수신하지 않음 (last_ns is None) → stale
    assert is_fresh_ns(None, now_ns=1_000, timeout_ns=500) is False


def test_just_before_timeout_is_fresh():
    # age == timeout_ns 경계 → fresh
    timeout_ns = 500_000_000
    last_ns = 1_000_000_000
    now_ns = last_ns + timeout_ns
    assert is_fresh_ns(last_ns, now_ns=now_ns, timeout_ns=timeout_ns) is True


def test_just_after_timeout_is_stale():
    # age == timeout_ns + 1 → stale
    timeout_ns = 500_000_000
    last_ns = 1_000_000_000
    now_ns = last_ns + timeout_ns + 1
    assert is_fresh_ns(last_ns, now_ns=now_ns, timeout_ns=timeout_ns) is False


def test_zero_age_is_fresh():
    # 방금 수신 (age == 0) → fresh
    assert is_fresh_ns(1_000, now_ns=1_000, timeout_ns=500) is True


def test_negative_age_is_stale():
    # 시간 역전 (now_ns < last_ns, age < 0) → stale (fail-safe)
    assert is_fresh_ns(2_000, now_ns=1_000, timeout_ns=500) is False


def test_sec_to_ns_converts_to_integer_nanoseconds():
    assert sec_to_ns(0.5) == 500_000_000
    assert isinstance(sec_to_ns(0.5), int)
    assert sec_to_ns(0.8) == 800_000_000
