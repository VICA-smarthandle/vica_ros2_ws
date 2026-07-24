import pytest

from vica_safety.emergency_stop_node import (
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
