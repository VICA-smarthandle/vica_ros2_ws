import pytest

from vica_safety.safety_gate import SafetyGate, SafetyState


@pytest.mark.parametrize(
    "active,fresh,cmd_zero,reason",
    [
        (True, True, True, "emergency stop is active"),
        (False, False, True, "emergency stop is stale"),
        (False, True, False, "/cmd_vel_req is not zero"),
    ],
)
def test_reset_rejects_each_unsafe_condition(active, fresh, cmd_zero, reason):
    gate = SafetyGate()

    decision = gate.request_reset(active, fresh, cmd_zero)

    assert decision.accepted is False
    assert decision.reason == reason
    assert gate.reset_armed is False


def test_reset_accepts_only_fresh_released_zero_command():
    gate = SafetyGate()

    decision = gate.request_reset(False, True, True)

    assert decision.accepted is True
    assert decision.state is SafetyState.READY_TO_GO
    assert gate.reset_armed is True


def test_active_estop_revokes_previous_reset():
    gate = SafetyGate()
    assert gate.request_reset(False, True, True).accepted is True

    state = gate.state_for_command(True, True, True, False)

    assert state is SafetyState.ESTOP_ACTIVE
    assert gate.reset_armed is False


def test_released_estop_waits_for_new_reset():
    gate = SafetyGate()
    gate.state_for_command(True, True, False, True)

    state = gate.state_for_command(False, True, False, True)

    assert state is SafetyState.ESTOP_RELEASED_WAIT_RESET


def test_stale_estop_is_fault_and_zero_output_state():
    gate = SafetyGate()

    state = gate.state_for_command(False, False, True, False)

    assert state is SafetyState.FAULT
    assert gate.can_forward_command is False


def test_armed_gate_runs_only_for_live_nonzero_command():
    gate = SafetyGate()
    gate.request_reset(False, True, True)

    assert gate.state_for_command(False, True, False, True) is SafetyState.READY_TO_GO
    assert gate.state_for_command(False, True, True, True) is SafetyState.READY_TO_GO
    assert gate.state_for_command(False, True, True, False) is SafetyState.RUNNING
    assert gate.can_forward_command is True
