"""주행 중 추론 차단 판정 순수 로직 검증 — ROS 없이 돈다.

왜 이 게이트가 필요한가:

    `mission_logic.py` 의 접근 게이트는 `state != State.IDLE` 이면 요청을
    거절한다("안내를 받고 있는 사용자가 우선이다"). 즉 주행 중에 사람을 찾아
    요청을 보내도 **반드시 거절된다.** 버려질 것이 확정된 추론을 5 Hz 로
    돌리는 셈이라, 주행 중에는 추론 자체를 하지 않는다.

이 판정이 잘못되는 방향은 둘이고 둘 다 여기서 막는다.

1. **너무 많이 막는다** — 미션 상태를 못 받는 동안 영영 꺼져 있으면 사람접근
   기능이 통째로 죽는다. 이 노드는 안전 회로가 아니므로(탐지 토픽 구독자 0,
   장애물은 라이다·depth 담당) 모르면 **켜는** 쪽으로 넘어진다.
2. **너무 늦게 푼다** — 도착 직후 다시 켜지지 않으면 안내가 끝난 사람 앞에서
   로봇이 아무것도 못 본다. 상태가 바뀌는 즉시 풀려야 한다.

시각은 전부 정수 나노초(STEADY_TIME)다. 이 모듈은 `time.monotonic_ns()` 를
부르지 않으며, 테스트가 시각을 직접 만들어 넣는다.
"""
import pytest

from vica_perception.inference_gate import (
    DEFAULT_STATE_TIMEOUT_S,
    InferenceGate,
    InferenceReason,
    sec_to_ns,
)


def ns(seconds: float) -> int:
    return sec_to_ns(seconds)


# ---- 상태를 한 번도 못 받았을 때 --------------------------------------------


def test_상태를_못_받으면_추론한다():
    """미션이 아직 안 떴을 수 있다. 모르면 켜는 쪽이다(기능 보존)."""
    gate = InferenceGate()

    assert gate.should_infer(ns(0.0)) is True
    assert gate.reason(ns(0.0)) is InferenceReason.NO_STATE


# ---- 정상 판정 ---------------------------------------------------------------


def test_주행_중이면_추론하지_않는다():
    gate = InferenceGate()
    gate.observe_state(ns(1.0), is_moving=True, is_paused=False)

    assert gate.should_infer(ns(1.0)) is False
    assert gate.reason(ns(1.0)) is InferenceReason.MOVING


def test_일시정지_중이면_추론하지_않는다():
    """목적지를 기억한 채 멈춰 있다 — 미션이 살아 있으므로 접근은 거절된다."""
    gate = InferenceGate()
    gate.observe_state(ns(1.0), is_moving=False, is_paused=True)

    assert gate.should_infer(ns(1.0)) is False
    assert gate.reason(ns(1.0)) is InferenceReason.PAUSED


def test_대기_중이면_추론한다():
    gate = InferenceGate()
    gate.observe_state(ns(1.0), is_moving=False, is_paused=True)
    gate.observe_state(ns(2.0), is_moving=False, is_paused=False)

    assert gate.should_infer(ns(2.0)) is True
    assert gate.reason(ns(2.0)) is InferenceReason.OK


def test_주행이_끝나면_같은_시각에_바로_풀린다():
    """도착 재개는 다음 상태 한 건이면 끝이다 — 재구독 대기가 없다."""
    gate = InferenceGate()
    gate.observe_state(ns(1.0), is_moving=True, is_paused=False)
    assert gate.should_infer(ns(1.5)) is False

    gate.observe_state(ns(2.0), is_moving=False, is_paused=False)
    assert gate.should_infer(ns(2.0)) is True


# ---- 신선도 (stale) ----------------------------------------------------------


def test_상태가_끊기면_추론을_다시_켠다():
    """미션이 죽으면 주행도 못 한다. 켜져 있어도 무해하므로 기능을 살린다."""
    gate = InferenceGate()
    gate.observe_state(ns(1.0), is_moving=True, is_paused=False)

    stale_at = ns(1.0 + DEFAULT_STATE_TIMEOUT_S + 0.001)
    assert gate.should_infer(stale_at) is True
    assert gate.reason(stale_at) is InferenceReason.STATE_STALE


def test_timeout_경계에서는_아직_유효하다():
    """'초과'가 stale 이다. 경계값 자체는 살아 있는 상태로 본다."""
    gate = InferenceGate()
    gate.observe_state(ns(1.0), is_moving=True, is_paused=False)

    edge = ns(1.0) + sec_to_ns(DEFAULT_STATE_TIMEOUT_S)
    assert gate.should_infer(edge) is False
    assert gate.reason(edge) is InferenceReason.MOVING


def test_시간이_뒤로_가면_stale_로_본다():
    """`vica_safety/freshness.py` 계약 — 시간 역전은 안전한 쪽으로 넘어진다."""
    gate = InferenceGate()
    gate.observe_state(ns(10.0), is_moving=True, is_paused=False)

    assert gate.should_infer(ns(9.0)) is True
    assert gate.reason(ns(9.0)) is InferenceReason.STATE_STALE


def test_상태가_다시_오면_stale_에서_회복한다():
    gate = InferenceGate()
    gate.observe_state(ns(1.0), is_moving=True, is_paused=False)
    assert gate.should_infer(ns(10.0)) is True

    gate.observe_state(ns(10.0), is_moving=True, is_paused=False)
    assert gate.should_infer(ns(10.0)) is False


# ---- 게이트 자체를 끄는 길 ----------------------------------------------------


def test_게이트를_끄면_주행_중에도_추론한다():
    """파라미터 하나로 종전 동작(항상 추론)으로 되돌릴 수 있어야 한다."""
    gate = InferenceGate(enabled=False)
    gate.observe_state(ns(1.0), is_moving=True, is_paused=False)

    assert gate.should_infer(ns(1.0)) is True
    assert gate.reason(ns(1.0)) is InferenceReason.DISABLED


# ---- 잘못된 설정은 기동 시점에 죽는다 -----------------------------------------


def test_timeout_이_0_이하면_거부한다():
    with pytest.raises(ValueError):
        InferenceGate(state_timeout_s=0.0)


def test_timeout_이_음수면_거부한다():
    with pytest.raises(ValueError):
        InferenceGate(state_timeout_s=-1.0)
