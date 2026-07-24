import pytest

from vica_safety.emergency_latch import EmergencyLatch


def test_releasing_physical_input_keeps_latch_until_reset():
    latch = EmergencyLatch(f1_timeout_sec=0.5)

    latch.mark_physical_seen(True, 1.0)
    latch.mark_physical_seen(False, 1.1)

    assert latch.evaluate(1.2).latched is True
    accepted, _ = latch.try_reset(1.2)
    assert accepted is True
    assert latch.evaluate(1.2).latched is False


def test_reset_rejects_active_physical_input():
    latch = EmergencyLatch(f1_timeout_sec=0.5)
    latch.mark_physical_seen(True, 1.0)

    accepted, message = latch.try_reset(1.1)

    assert accepted is False
    assert "physical_f1" in message


def test_reset_rejects_stale_physical_input():
    latch = EmergencyLatch(f1_timeout_sec=0.5)
    latch.mark_physical_seen(False, 1.0)

    accepted, message = latch.try_reset(1.6)

    assert accepted is False
    assert "physical_stale" in message


@pytest.mark.parametrize("source", ["app", "voice"])
def test_reset_rejects_active_software_source(source):
    latch = EmergencyLatch(f1_timeout_sec=0.5)
    latch.mark_physical_seen(False, 1.0)
    latch.update_source(source, True, 1.1)

    accepted, message = latch.try_reset(1.2)

    assert accepted is False
    assert source in message


def test_source_reactivation_relatches_after_successful_reset():
    latch = EmergencyLatch(f1_timeout_sec=0.5)
    latch.mark_physical_seen(False, 1.0)
    assert latch.try_reset(1.1)[0] is True

    latch.update_source("voice", True, 1.2)

    snapshot = latch.evaluate(1.2)
    assert snapshot.latched is True
    assert snapshot.active_sources == ("voice",)


def test_unknown_software_source_is_rejected():
    latch = EmergencyLatch(f1_timeout_sec=0.5)

    with pytest.raises(ValueError, match="unsupported source"):
        latch.update_source("unknown", True, 1.0)
