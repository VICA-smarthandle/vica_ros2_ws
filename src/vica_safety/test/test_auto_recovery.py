"""통신 원인 자동 복구 정책의 순수 모델 시험."""

from vica_safety.auto_recovery import AutoRecoveryPolicy
from vica_safety.freshness import sec_to_ns


T0 = 1_000_000_000
SETTLE_NS = sec_to_ns(1.0)


def policy() -> AutoRecoveryPolicy:
    return AutoRecoveryPolicy(settle_ns=SETTLE_NS)


def test_comm_only_cause_recovers_after_settle():
    p = policy()
    p.observe_sources(("motor_can_stale",), T0)
    p.observe_sources((), T0 + sec_to_ns(0.1))

    assert p.should_recover(T0 + sec_to_ns(0.1)) is False
    assert p.should_recover(T0 + sec_to_ns(1.1)) is True


def test_boot_waiting_then_all_inputs_arrive_recovers():
    p = policy()
    p.observe_sources(("motor_can_waiting", "physical_waiting"), T0)
    p.observe_sources((), T0 + sec_to_ns(2.0))

    assert p.should_recover(T0 + sec_to_ns(3.1)) is True


def test_pressed_button_blocks_recovery_even_after_release():
    p = policy()
    p.observe_sources(("physical_f1",), T0)
    p.observe_sources((), T0 + sec_to_ns(0.1))

    assert p.should_recover(T0 + sec_to_ns(5.0)) is False


def test_app_estop_blocks_recovery():
    p = policy()
    p.observe_sources(("app",), T0)
    p.observe_sources((), T0 + sec_to_ns(0.1))

    assert p.should_recover(T0 + sec_to_ns(5.0)) is False


def test_voice_estop_blocks_recovery():
    p = policy()
    p.observe_sources(("voice",), T0)
    p.observe_sources((), T0 + sec_to_ns(0.1))

    assert p.should_recover(T0 + sec_to_ns(5.0)) is False


def test_comm_cause_mixed_with_button_blocks_recovery():
    p = policy()
    p.observe_sources(("motor_can_stale", "physical_f1"), T0)
    p.observe_sources((), T0 + sec_to_ns(0.1))

    assert p.should_recover(T0 + sec_to_ns(5.0)) is False


def test_disconnect_while_running_blocks_recovery():
    p = policy()
    p.observe_safety_state("RUNNING", "FAULT")
    p.observe_sources(("motor_can_stale",), T0)
    p.observe_sources((), T0 + sec_to_ns(0.1))

    assert p.should_recover(T0 + sec_to_ns(5.0)) is False


def test_disconnect_while_idle_does_not_block_recovery():
    p = policy()
    p.observe_safety_state("READY_TO_GO", "FAULT")
    p.observe_sources(("motor_can_stale",), T0)
    p.observe_sources((), T0 + sec_to_ns(0.1))

    assert p.should_recover(T0 + sec_to_ns(1.1)) is True


def test_settle_restarts_when_cause_returns():
    p = policy()
    p.observe_sources(("motor_can_stale",), T0)
    p.observe_sources((), T0 + sec_to_ns(0.1))
    p.observe_sources(("motor_can_stale",), T0 + sec_to_ns(0.5))
    p.observe_sources((), T0 + sec_to_ns(0.6))

    assert p.should_recover(T0 + sec_to_ns(1.2)) is False
    assert p.should_recover(T0 + sec_to_ns(1.7)) is True


def test_no_cause_ever_seen_does_not_recover():
    p = policy()
    p.observe_sources((), T0)

    assert p.should_recover(T0 + sec_to_ns(5.0)) is False


def test_successful_recovery_rearms_for_the_next_event():
    p = policy()
    p.observe_sources(("motor_can_stale",), T0)
    p.observe_sources((), T0 + sec_to_ns(0.1))
    assert p.should_recover(T0 + sec_to_ns(1.1)) is True

    p.mark_attempted(success=True)
    assert p.should_recover(T0 + sec_to_ns(1.2)) is False

    p.observe_sources(("motor_can_stale",), T0 + sec_to_ns(10.0))
    p.observe_sources((), T0 + sec_to_ns(10.1))
    assert p.should_recover(T0 + sec_to_ns(11.2)) is True


def test_failed_recovery_hands_over_to_the_operator():
    p = policy()
    p.observe_sources(("motor_can_stale",), T0)
    p.observe_sources((), T0 + sec_to_ns(0.1))
    assert p.should_recover(T0 + sec_to_ns(1.1)) is True

    p.mark_attempted(success=False)

    assert p.should_recover(T0 + sec_to_ns(1.2)) is False
    p.observe_sources(("motor_can_stale",), T0 + sec_to_ns(10.0))
    p.observe_sources((), T0 + sec_to_ns(10.1))
    assert p.should_recover(T0 + sec_to_ns(11.2)) is False


def test_manual_reset_clears_the_block():
    p = policy()
    p.observe_sources(("physical_f1",), T0)
    p.observe_sources((), T0 + sec_to_ns(0.1))
    assert p.should_recover(T0 + sec_to_ns(5.0)) is False

    p.notify_manual_reset()

    p.observe_sources(("motor_can_stale",), T0 + sec_to_ns(10.0))
    p.observe_sources((), T0 + sec_to_ns(10.1))
    assert p.should_recover(T0 + sec_to_ns(11.2)) is True
