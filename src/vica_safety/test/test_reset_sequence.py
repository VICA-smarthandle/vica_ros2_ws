import pytest

from vica_safety.reset_sequence import RESET_STEPS, ResetSequence


def test_sequence_starts_with_nav_goal_check():
    sequence = ResetSequence()

    sequence.begin()

    assert sequence.next_step == "nav_goal_check"


def test_failure_stops_all_later_reset_steps():
    sequence = ResetSequence()
    sequence.begin()

    result = sequence.record("nav_goal_check", False, "active goal remains")

    assert result.success is False
    assert result.step == "nav_goal_check"
    assert result.message == "active goal remains"
    assert sequence.next_step is None


def test_success_requires_every_step_in_order():
    sequence = ResetSequence()
    sequence.begin()

    for step in RESET_STEPS[:-1]:
        result = sequence.record(step, True, "ok")
        assert result.success is False

    result = sequence.record("ready", True, "READY_TO_GO")

    assert result.success is True
    assert sequence.next_step is None


def test_out_of_order_step_is_rejected():
    sequence = ResetSequence()
    sequence.begin()

    with pytest.raises(ValueError, match="expected nav_goal_check"):
        sequence.record("estop_reset", True, "ok")


def test_begin_resets_a_previously_failed_sequence():
    sequence = ResetSequence()
    sequence.begin()
    sequence.record("nav_goal_check", False, "failed")

    sequence.begin()

    assert sequence.next_step == "nav_goal_check"
