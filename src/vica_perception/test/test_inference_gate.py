"""주행 중 추론 차단 판정 순수 로직 검증 — ROS 없이 돈다."""
import pytest

from vica_perception.inference_gate import (
    DEFAULT_STATE_TIMEOUT_S,
    InferenceGate,
    InferenceReason,
    sec_to_ns,
)


def ns(seconds: float) -> int:
    return sec_to_ns(seconds)


def test_상태를_못_받으면_추론한다():
    """미션이 아직 안 떴을 수 있다. 모르면 켜는 쪽이다(기능 보존)."""
    gate = InferenceGate()

    assert gate.should_infer(ns(0.0)) is True
    assert gate.reason(ns(0.0)) is InferenceReason.NO_STATE


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


def test_게이트를_끄면_주행_중에도_추론한다():
    """파라미터 하나로 종전 동작(항상 추론)으로 되돌릴 수 있어야 한다."""
    gate = InferenceGate(enabled=False)
    gate.observe_state(ns(1.0), is_moving=True, is_paused=False)

    assert gate.should_infer(ns(1.0)) is True
    assert gate.reason(ns(1.0)) is InferenceReason.DISABLED


def test_timeout_이_0_이하면_거부한다():
    with pytest.raises(ValueError):
        InferenceGate(state_timeout_s=0.0)


def test_timeout_이_음수면_거부한다():
    with pytest.raises(ValueError):
        InferenceGate(state_timeout_s=-1.0)
