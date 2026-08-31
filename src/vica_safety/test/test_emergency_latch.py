import pytest

from vica_safety.emergency_latch import EmergencyLatch
from vica_safety.freshness import sec_to_ns

# 모든 시각 인자는 정수 나노초(steady clock) 기준이다.
T0 = 1_000_000_000  # 기준 now_ns
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

    # age = 0.6s > 0.5s timeout → stale
    latch.mark_motor_can_seen(True, T0 + sec_to_ns(0.6))
    accepted, message = latch.try_reset(T0 + sec_to_ns(0.6))

    assert accepted is False
    assert "physical_stale" in message


def test_never_received_physical_is_stale_and_fail_safe():
    # 물리 F1을 한 번도 수신하지 않음 → physical_stale, latch 유지
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
    # 시간 역전: now_ns < last_physical_ns → 음수 age → stale (fail-safe)
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
    # age == timeout_ns 경계 → fresh (정지 아님)
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
    # 경계가 fresh라는 표시로 끝나지 않고 실제로 reset 문을 열어야 한다.
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
    """모터 노드가 죽어 보고가 끊기면 관리자 reset도 거부된다.

    이 브랜치의 핵심 주장이다. evaluate가 stale을 표시하는 것만으로는
    부족하고, try_reset이 실제로 막아야 한다.
    """
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


# ===========================================================================
# 부팅 유예 (input grace) — "아직 안 옴"과 "받다가 끊김"의 분리
# ===========================================================================
#
# 근거: 2026-08-28 젯슨 로그 8월 전체 집계에서 `IDLE -> FAULT` 229회가 최다였다.
# 노드가 뜬 직후에는 last_*_ns 가 None 인데 is_fresh_ns(None) 이 False 라서
# 첫 evaluate 에서 곧바로 *_stale 이 붙는다. 미수신은 고장이 아니라 대기다.
#
# input_grace_ns 기본값은 0 이므로 이 블록 밖의 기존 시험은 영향을 받지 않는다.

GRACE_NS = sec_to_ns(15.0)


def test_missing_input_within_grace_is_waiting_not_stale():
    # 부팅 유예 안의 미수신은 대기이지 고장이 아니다 → 래치를 새로 걸지 않는다.
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
    # 유예를 넘기면 고장으로 승격한다. 유예는 무한정 봐주는 장치가 아니다.
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
    # 유예는 '첫 수신 전'에만 있다. 한 번 받은 뒤의 단절은 진짜 고장이다.
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
        initially_latched=False,
        input_grace_ns=GRACE_NS,
        start_ns=T0,
    )
    latch.mark_physical_seen(False, T0)
    latch.mark_motor_can_seen(True, T0)

    # 유예 창(15 s) 안이지만 타임아웃(0.5 s)은 넘긴 시점
    snapshot = latch.evaluate(T0 + sec_to_ns(1.0))

    assert "physical_stale" in snapshot.active_sources
    assert "motor_can_stale" in snapshot.active_sources
    assert "physical_waiting" not in snapshot.active_sources
    assert snapshot.latched is True


def test_button_pressed_during_grace_latches_immediately():
    # 유예 중이라도 사람이 누른 것은 즉시 래치한다. 유예는 미수신에만 적용된다.
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
    # input_grace_ns 기본값 0 = 되돌리기 경로. 기존 동작과 완전히 같아야 한다.
    latch = EmergencyLatch(
        f1_timeout_ns=TIMEOUT_NS,
        motor_can_timeout_ns=MOTOR_CAN_TIMEOUT_NS,
        initially_latched=False,
    )

    snapshot = latch.evaluate(T0)

    assert "physical_stale" in snapshot.active_sources
    assert "physical_waiting" not in snapshot.active_sources
    assert snapshot.latched is True
