import pytest

from vica_safety.emergency_latch import EmergencyLatch
from vica_safety.freshness import sec_to_ns

# 모든 시각 인자는 정수 나노초(steady clock) 기준이다.
T0 = 1_000_000_000  # 기준 now_ns
TIMEOUT_NS = sec_to_ns(0.5)


def test_releasing_physical_input_keeps_latch_until_reset():
    latch = EmergencyLatch(f1_timeout_ns=TIMEOUT_NS)

    latch.mark_physical_seen(True, T0)
    latch.mark_physical_seen(False, T0 + sec_to_ns(0.1))

    now = T0 + sec_to_ns(0.2)
    assert latch.evaluate(now).latched is True
    accepted, _ = latch.try_reset(now)
    assert accepted is True
    assert latch.evaluate(now).latched is False


def test_reset_rejects_active_physical_input():
    latch = EmergencyLatch(f1_timeout_ns=TIMEOUT_NS)
    latch.mark_physical_seen(True, T0)

    accepted, message = latch.try_reset(T0 + sec_to_ns(0.1))

    assert accepted is False
    assert "physical_f1" in message


def test_reset_rejects_stale_physical_input():
    latch = EmergencyLatch(f1_timeout_ns=TIMEOUT_NS)
    latch.mark_physical_seen(False, T0)

    # age = 0.6s > 0.5s timeout → stale
    accepted, message = latch.try_reset(T0 + sec_to_ns(0.6))

    assert accepted is False
    assert "physical_stale" in message


def test_never_received_physical_is_stale_and_fail_safe():
    # 물리 F1을 한 번도 수신하지 않음 → physical_stale, latch 유지
    latch = EmergencyLatch(f1_timeout_ns=TIMEOUT_NS)

    snapshot = latch.evaluate(T0)

    assert snapshot.physical_fresh is False
    assert "physical_stale" in snapshot.active_sources
    assert snapshot.latched is True
    accepted, message = latch.try_reset(T0)
    assert accepted is False
    assert "physical_stale" in message


def test_time_reversal_marks_physical_stale():
    # 시간 역전: now_ns < last_physical_ns → 음수 age → stale (fail-safe)
    latch = EmergencyLatch(f1_timeout_ns=TIMEOUT_NS)
    latch.mark_physical_seen(False, T0)

    snapshot = latch.evaluate(T0 - sec_to_ns(0.1))

    assert snapshot.physical_fresh is False
    assert "physical_stale" in snapshot.active_sources
    assert snapshot.latched is True


def test_timeout_boundary_is_fresh():
    # age == timeout_ns 경계 → fresh (정지 아님)
    latch = EmergencyLatch(f1_timeout_ns=TIMEOUT_NS)
    latch.mark_physical_seen(False, T0)

    snapshot = latch.evaluate(T0 + TIMEOUT_NS)

    assert snapshot.physical_fresh is True
    assert "physical_stale" not in snapshot.active_sources


@pytest.mark.parametrize("source", ["app", "voice"])
def test_reset_rejects_active_software_source(source):
    latch = EmergencyLatch(f1_timeout_ns=TIMEOUT_NS)
    latch.mark_physical_seen(False, T0)
    latch.update_source(source, True, T0 + sec_to_ns(0.1))

    accepted, message = latch.try_reset(T0 + sec_to_ns(0.2))

    assert accepted is False
    assert source in message


def test_source_reactivation_relatches_after_successful_reset():
    latch = EmergencyLatch(f1_timeout_ns=TIMEOUT_NS)
    latch.mark_physical_seen(False, T0)
    assert latch.try_reset(T0 + sec_to_ns(0.1))[0] is True

    latch.update_source("voice", True, T0 + sec_to_ns(0.2))

    snapshot = latch.evaluate(T0 + sec_to_ns(0.2))
    assert snapshot.latched is True
    assert snapshot.active_sources == ("voice",)


def test_unknown_software_source_is_rejected():
    latch = EmergencyLatch(f1_timeout_ns=TIMEOUT_NS)

    with pytest.raises(ValueError, match="unsupported source"):
        latch.update_source("unknown", True, T0)
