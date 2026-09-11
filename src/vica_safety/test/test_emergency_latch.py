import pytest

from vica_safety.emergency_latch import EmergencyLatch
from vica_safety.freshness import sec_to_ns

T0 = 1_000_000_000
TIMEOUT_NS = sec_to_ns(0.5)
MOTOR_CAN_TIMEOUT_NS = sec_to_ns(0.5)


def test_releasing_physical_input_keeps_latch_until_reset():
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
    )

    latch.mark_physical_seen(True, T0)
    latch.mark_physical_seen(False, T0 + sec_to_ns(0.1))

    now = T0 + sec_to_ns(0.2)
    latch.mark_motor_can_seen(True, now)
    assert latch.evaluate(now).latched is True
    accepted, _ = latch.try_reset(now)
    assert accepted is True
    assert latch.evaluate(now).latched is False


def test_reset_rejects_active_physical_input():
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
    )
    latch.mark_physical_seen(True, T0)

    latch.mark_motor_can_seen(True, T0 + sec_to_ns(0.1))
    accepted, message = latch.try_reset(T0 + sec_to_ns(0.1))

    assert accepted is False
    assert "physical_f1" in message


def test_reset_rejects_stale_physical_input():
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
    )
    latch.mark_physical_seen(False, T0)

    latch.mark_motor_can_seen(True, T0 + sec_to_ns(0.6))
    accepted, message = latch.try_reset(T0 + sec_to_ns(0.6))

    assert accepted is False
    assert "physical_stale" in message


def test_never_received_physical_is_stale_and_fail_safe():
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
    )

    latch.mark_motor_can_seen(True, T0)
    snapshot = latch.evaluate(T0)

    assert snapshot.physical_fresh is False
    assert "physical_stale" in snapshot.active_sources
    assert snapshot.latched is True
    accepted, message = latch.try_reset(T0)
    assert accepted is False
    assert "physical_stale" in message


def test_time_reversal_marks_physical_stale():
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
    )
    latch.mark_physical_seen(False, T0)

    now = T0 - sec_to_ns(0.1)
    latch.mark_motor_can_seen(True, now)
    snapshot = latch.evaluate(now)

    assert snapshot.physical_fresh is False
    assert "physical_stale" in snapshot.active_sources
    assert snapshot.latched is True


def test_timeout_boundary_is_fresh():
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
    )
    latch.mark_physical_seen(False, T0)

    now = T0 + TIMEOUT_NS
    latch.mark_motor_can_seen(True, now)
    snapshot = latch.evaluate(now)

    assert snapshot.physical_fresh is True
    assert "physical_stale" not in snapshot.active_sources


@pytest.mark.parametrize("source", ["app", "voice"])
def test_reset_rejects_active_software_source(source):
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
    )
    latch.mark_physical_seen(False, T0)
    latch.update_source(source, True, T0 + sec_to_ns(0.1))

    latch.mark_motor_can_seen(True, T0 + sec_to_ns(0.2))
    accepted, message = latch.try_reset(T0 + sec_to_ns(0.2))

    assert accepted is False
    assert source in message


def test_source_reactivation_relatches_after_successful_reset():
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
    )
    latch.mark_physical_seen(False, T0)
    latch.mark_motor_can_seen(True, T0 + sec_to_ns(0.1))
    assert latch.try_reset(T0 + sec_to_ns(0.1))[0] is True

    latch.update_source("voice", True, T0 + sec_to_ns(0.2))

    latch.mark_motor_can_seen(True, T0 + sec_to_ns(0.2))
    snapshot = latch.evaluate(T0 + sec_to_ns(0.2))
    assert snapshot.latched is True
    assert snapshot.active_sources == ("voice",)


def test_unknown_software_source_is_rejected():
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
    )

    with pytest.raises(ValueError, match="unsupported source"):
        latch.update_source("unknown", True, T0)


def test_motor_can_failure_latches():
    """Motor node가 CAN 장애를 보고하면 중앙 걸쇠가 걸린다."""
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
    )
    latch.mark_physical_seen(False, T0)

    latch.mark_motor_can_seen(False, T0)

    snapshot = latch.evaluate(T0)
    assert snapshot.latched is True
    assert "motor_can" in snapshot.active_sources


def test_missing_motor_can_report_is_stale():
    """Motor node가 죽어 보고가 끊기면 stale로 걸쇠가 걸린다."""
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
    )
    latch.mark_physical_seen(False, T0)
    latch.mark_motor_can_seen(True, T0)

    now = T0 + MOTOR_CAN_TIMEOUT_NS + 1
    latch.mark_physical_seen(False, now)

    snapshot = latch.evaluate(now)
    assert snapshot.latched is True
    assert "motor_can_stale" in snapshot.active_sources


def test_motor_can_boundary_is_fresh():
    """경계값 age == timeout은 fresh다."""
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
    )
    now = T0 + MOTOR_CAN_TIMEOUT_NS
    latch.mark_physical_seen(False, now)
    latch.mark_motor_can_seen(True, T0)

    snapshot = latch.evaluate(now)
    assert "motor_can_stale" not in snapshot.active_sources
    assert snapshot.reset_allowed is True


def test_never_reported_motor_can_is_stale():
    """한 번도 보고받지 못한 상태는 fail-closed로 stale이다."""
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
    )
    latch.mark_physical_seen(False, T0)

    snapshot = latch.evaluate(T0)
    assert "motor_can_stale" in snapshot.active_sources


def test_reset_rejected_while_motor_can_failed():
    """CAN이 비정상인 동안에는 관리자 reset도 거부된다."""
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
    )
    latch.mark_physical_seen(False, T0)
    latch.mark_motor_can_seen(False, T0)

    accepted, message = latch.try_reset(T0)

    assert accepted is False
    assert "motor_can" in message


def test_reset_allowed_after_can_recovers():
    """CAN 복구 후에는 관리자 reset으로 해제된다."""
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
    )
    latch.mark_physical_seen(False, T0)
    latch.mark_motor_can_seen(False, T0)

    now = T0 + sec_to_ns(0.1)
    latch.mark_physical_seen(False, now)
    latch.mark_motor_can_seen(True, now)

    accepted, _ = latch.try_reset(now)
    assert accepted is True
    assert latch.evaluate(now).latched is False


def test_time_reversal_marks_motor_can_stale():
    """시각이 뒤로 가면 음수 age가 되고, motor_can도 physical과 같이 stale이다."""
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
    )
    now = T0 - sec_to_ns(0.1)
    latch.mark_physical_seen(False, now)
    latch.mark_motor_can_seen(True, T0)

    snapshot = latch.evaluate(now)

    assert "motor_can_stale" in snapshot.active_sources


def test_dead_motor_node_rejects_reset():
    """모터 노드가 죽어 보고가 끊기면 관리자 reset도 거부된다."""
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
    )
    latch.mark_physical_seen(False, T0)
    latch.mark_motor_can_seen(True, T0)
    dead_at = T0 + MOTOR_CAN_TIMEOUT_NS + 1

    latch.mark_physical_seen(False, dead_at)
    accepted, message = latch.try_reset(dead_at)

    assert accepted is False
    assert "motor_can_stale" in message


GRACE_NS = sec_to_ns(15.0)


def test_missing_input_within_grace_is_waiting_not_stale():
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
        initially_latched=False,
        input_grace_ns=GRACE_NS,
        start_ns=T0,
    )

    snapshot = latch.evaluate(T0 + sec_to_ns(1.0))

    assert "physical_waiting" in snapshot.active_sources
    assert "motor_can_waiting" in snapshot.active_sources
    assert "physical_stale" not in snapshot.active_sources
    assert snapshot.latched is False


def test_missing_input_past_grace_becomes_stale_and_latches():
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
        initially_latched=False,
        input_grace_ns=GRACE_NS,
        start_ns=T0,
    )

    snapshot = latch.evaluate(T0 + GRACE_NS + 1)

    assert "physical_stale" in snapshot.active_sources
    assert "physical_waiting" not in snapshot.active_sources
    assert snapshot.latched is True


def test_disconnect_after_first_receipt_is_stale_even_within_grace():
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
        initially_latched=False,
        input_grace_ns=GRACE_NS,
        start_ns=T0,
    )
    latch.mark_physical_seen(False, T0)
    latch.mark_motor_can_seen(True, T0)

    snapshot = latch.evaluate(T0 + sec_to_ns(1.0))

    assert "physical_stale" in snapshot.active_sources
    assert "motor_can_stale" in snapshot.active_sources
    assert "physical_waiting" not in snapshot.active_sources
    assert snapshot.latched is True


def test_button_pressed_during_grace_latches_immediately():
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
        initially_latched=False,
        input_grace_ns=GRACE_NS,
        start_ns=T0,
    )

    latch.mark_physical_seen(True, T0 + sec_to_ns(1.0))
    snapshot = latch.evaluate(T0 + sec_to_ns(1.0))

    assert "physical_f1" in snapshot.active_sources
    assert snapshot.latched is True


def test_grace_disabled_by_default_keeps_current_behaviour():
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
        initially_latched=False,
    )

    snapshot = latch.evaluate(T0)

    assert "physical_stale" in snapshot.active_sources
    assert "physical_waiting" not in snapshot.active_sources
    assert snapshot.latched is True
