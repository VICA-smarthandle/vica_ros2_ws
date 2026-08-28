import pytest

from vica_safety.emergency_latch import LatchSnapshot
from vica_safety.emergency_stop_node import (
    classify_latch_state,
    describe_latch_transition,
    f1_frame_means_estop_active,
)


def test_f1_decoder_detects_pressed_button_from_both_0x10_bits():
    data = bytes.fromhex("f100484800000000")

    assert f1_frame_means_estop_active(data, 2, 3, 0x10, 0x00) is True


def test_f1_decoder_detects_released_button():
    data = bytes.fromhex("f100585800000000")

    assert f1_frame_means_estop_active(data, 2, 3, 0x10, 0x00) is False


def test_f1_decoder_requires_both_configured_bytes_to_be_active():
    data = bytes.fromhex("f100485800000000")

    assert f1_frame_means_estop_active(data, 2, 3, 0x10, 0x00) is False


def test_f1_decoder_rejects_out_of_range_byte_index():
    with pytest.raises(ValueError, match="byte index"):
        f1_frame_means_estop_active(bytes(8), 2, 8, 0x10, 0x00)


@pytest.mark.parametrize(
    "new_state,severity,marker",
    [
        ("ESTOP_ACTIVE", "error", "[ESTOP ACTIVE]"),
        ("FAULT", "error", "[FAULT]"),
        ("ESTOP_RELEASED_WAIT_RESET", "warn", "[WAIT RESET]"),
        ("CLEARED", "info", "[ESTOP CLEARED]"),
    ],
)
def test_latch_transition_has_expected_severity(new_state, severity, marker):
    assert describe_latch_transition("OLD", new_state) == (severity, marker)


def snapshot(active_sources, latched=True):
    """Build a LatchSnapshot carrying only what classification reads."""
    return LatchSnapshot(
        latched=latched,
        active_sources=tuple(active_sources),
        physical_fresh="physical_stale" not in active_sources,
        reset_allowed=False,
    )


def test_stale_motor_can_is_reported_as_fault_not_estop():
    """모터 노드 사망은 FAULT다. ESTOP_ACTIVE로 찍으면 물리 버튼을 찾게 된다."""
    assert classify_latch_state(snapshot(["motor_can_stale"])) == "FAULT"


def test_stale_physical_input_is_reported_as_fault():
    assert classify_latch_state(snapshot(["physical_stale"])) == "FAULT"


def test_pressed_button_is_reported_as_estop_active():
    assert classify_latch_state(snapshot(["physical_f1"])) == "ESTOP_ACTIVE"


def test_reported_motor_can_failure_is_estop_active_not_fault():
    """보고가 도착하는 한 원인은 알려져 있으므로 FAULT가 아니다."""
    assert classify_latch_state(snapshot(["motor_can"])) == "ESTOP_ACTIVE"


def test_latched_with_no_active_source_waits_for_reset():
    state = classify_latch_state(snapshot([], latched=True))

    assert state == "ESTOP_RELEASED_WAIT_RESET"


def test_unlatched_with_no_active_source_is_cleared():
    assert classify_latch_state(snapshot([], latched=False)) == "CLEARED"


# ===========================================================================
# 부팅 대기 (WAITING_INPUT)
# ===========================================================================
#
# `*_waiting`은 고장이 아니라 "아직 첫 신호를 못 받았다"이다. FAULT로 찍으면
# 관리자가 없는 고장을 찾는다 -- stale을 FAULT로 분리한 것과 같은 이유다.


def test_waiting_input_is_not_reported_as_fault():
    assert classify_latch_state(snapshot(["physical_waiting"])) == "WAITING_INPUT"
    assert (
        classify_latch_state(snapshot(["motor_can_waiting"])) == "WAITING_INPUT"
    )


def test_waiting_mixed_with_stale_is_still_a_fault():
    # 한쪽이 유예를 넘겨 고장이면 전체는 고장이다. 대기가 고장을 가리면 안 된다.
    state = classify_latch_state(
        snapshot(["motor_can_waiting", "physical_stale"])
    )

    assert state == "FAULT"


def test_waiting_mixed_with_pressed_button_is_estop_active():
    state = classify_latch_state(snapshot(["motor_can_waiting", "physical_f1"]))

    assert state == "ESTOP_ACTIVE"


def test_waiting_input_transition_is_info_level():
    assert describe_latch_transition("IDLE", "WAITING_INPUT") == (
        "info",
        "[WAITING INPUT]",
    )
