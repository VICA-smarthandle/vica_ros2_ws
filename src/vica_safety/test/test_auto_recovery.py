"""통신 원인 자동 복구 정책의 순수 모델 시험.

이 정책이 지키는 것은 하나다: **사람이 개입한 E-stop과 통신이 끊긴 상태를
구분해서, 후자만 정지 중에 한 번 자동으로 푼다.**

근거는 2026-08-28 젯슨 로그 8월 전체 집계다.

    IDLE -> FAULT                229회   부팅 직후
    CLEARED -> *                 154회   대기 중
    READY_TO_GO -> *             142회   reset 후 출발 대기 중
    RUNNING -> ESTOP_ACTIVE       12회   주행 중
    RUNNING -> FAULT               2회   주행 중 통신 지연

앞의 세 줄(525회)이 이 정책의 대상이고, 마지막 두 줄은 대상이 아니다.
"""

from vica_safety.auto_recovery import AutoRecoveryPolicy
from vica_safety.freshness import sec_to_ns


T0 = 1_000_000_000
SETTLE_NS = sec_to_ns(1.0)


def policy() -> AutoRecoveryPolicy:
    return AutoRecoveryPolicy(settle_ns=SETTLE_NS)


# ---------------------------------------------------------------------------
# 자동 복구가 되는 경우
# ---------------------------------------------------------------------------


def test_comm_only_cause_recovers_after_settle():
    # motor node 보고가 끊겼다가 돌아왔다. 사람은 아무것도 누르지 않았다.
    p = policy()
    p.observe_sources(("motor_can_stale",), T0)
    p.observe_sources((), T0 + sec_to_ns(0.1))

    assert p.should_recover(T0 + sec_to_ns(0.1)) is False
    assert p.should_recover(T0 + sec_to_ns(1.1)) is True


def test_boot_waiting_then_all_inputs_arrive_recovers():
    # 부팅 시나리오. `*_waiting`으로 시작해 첫 신호가 다 들어오면 스스로 푼다.
    p = policy()
    p.observe_sources(("motor_can_waiting", "physical_waiting"), T0)
    p.observe_sources((), T0 + sec_to_ns(2.0))

    assert p.should_recover(T0 + sec_to_ns(3.1)) is True


# ---------------------------------------------------------------------------
# 자동 복구가 막히는 경우
# ---------------------------------------------------------------------------


def test_pressed_button_blocks_recovery_even_after_release():
    # 사람이 누른 것은 사람이 푼다. 버튼을 뺐다고 로봇이 스스로 출발하면 안 된다.
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
    # 통신 원인과 사람 개입이 섞이면 사람 쪽이 이긴다.
    p = policy()
    p.observe_sources(("motor_can_stale", "physical_f1"), T0)
    p.observe_sources((), T0 + sec_to_ns(0.1))

    assert p.should_recover(T0 + sec_to_ns(5.0)) is False


def test_disconnect_while_running_blocks_recovery():
    # 사용자가 고른 "정지 중일 때만" 조건. 주행 중 끊김은 관리자가 확인한다.
    p = policy()
    p.observe_safety_state("RUNNING", "FAULT")
    p.observe_sources(("motor_can_stale",), T0)
    p.observe_sources((), T0 + sec_to_ns(0.1))

    assert p.should_recover(T0 + sec_to_ns(5.0)) is False


def test_disconnect_while_idle_does_not_block_recovery():
    # 정지 중(IDLE/READY_TO_GO)에서의 전이는 막지 않는다. 로그 집계의 525회다.
    p = policy()
    p.observe_safety_state("READY_TO_GO", "FAULT")
    p.observe_sources(("motor_can_stale",), T0)
    p.observe_sources((), T0 + sec_to_ns(0.1))

    assert p.should_recover(T0 + sec_to_ns(1.1)) is True


def test_settle_restarts_when_cause_returns():
    # 원인이 깜빡이면 안정 시간을 다시 센다. 떨리는 접점에서 재출발하면 안 된다.
    p = policy()
    p.observe_sources(("motor_can_stale",), T0)
    p.observe_sources((), T0 + sec_to_ns(0.1))
    p.observe_sources(("motor_can_stale",), T0 + sec_to_ns(0.5))
    p.observe_sources((), T0 + sec_to_ns(0.6))

    # 첫 해소(0.1s)로부터는 1.1s 지났지만, 마지막 해소(0.6s)로부터는 0.6s다
    assert p.should_recover(T0 + sec_to_ns(1.2)) is False
    assert p.should_recover(T0 + sec_to_ns(1.7)) is True


def test_no_cause_ever_seen_does_not_recover():
    # 걸린 적이 없으면 풀 것도 없다. 빈 원인 목록만으로 reset을 부르면 안 된다.
    p = policy()
    p.observe_sources((), T0)

    assert p.should_recover(T0 + sec_to_ns(5.0)) is False


# ---------------------------------------------------------------------------
# 시도 이후
# ---------------------------------------------------------------------------


def test_successful_recovery_rearms_for_the_next_event():
    p = policy()
    p.observe_sources(("motor_can_stale",), T0)
    p.observe_sources((), T0 + sec_to_ns(0.1))
    assert p.should_recover(T0 + sec_to_ns(1.1)) is True

    p.mark_attempted(success=True)
    assert p.should_recover(T0 + sec_to_ns(1.2)) is False

    # 다음 통신 장애에는 다시 동작해야 한다
    p.observe_sources(("motor_can_stale",), T0 + sec_to_ns(10.0))
    p.observe_sources((), T0 + sec_to_ns(10.1))
    assert p.should_recover(T0 + sec_to_ns(11.2)) is True


def test_failed_recovery_hands_over_to_the_operator():
    # 한 사건당 한 번만 시도한다. 실패를 반복하면 로그만 더럽히고 원인은 그대로다.
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
    # 관리자가 직접 풀면 정책도 처음 상태로 돌아간다.
    p = policy()
    p.observe_sources(("physical_f1",), T0)
    p.observe_sources((), T0 + sec_to_ns(0.1))
    assert p.should_recover(T0 + sec_to_ns(5.0)) is False

    p.notify_manual_reset()

    p.observe_sources(("motor_can_stale",), T0 + sec_to_ns(10.0))
    p.observe_sources((), T0 + sec_to_ns(10.1))
    assert p.should_recover(T0 + sec_to_ns(11.2)) is True
