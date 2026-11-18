from encoder_feedback.encoder_feedback import int32_delta
from encoder_feedback.encoder_feedback import int32_le


def test_int32_le_decodes_positive_and_negative_values():
    assert int32_le([0x05, 0x00, 0x00, 0x00]) == 5
    assert int32_le([0xF0, 0xFF, 0xFF, 0xFF]) == -16


def test_int32_delta_handles_forward_rollover():
    assert int32_delta(-0x80000000, 0x7FFFFFFF) == 1


def test_int32_delta_handles_reverse_rollover():
    assert int32_delta(0x7FFFFFFF, -0x80000000) == -1


def test_int32_delta_keeps_normal_motion():
    assert int32_delta(130, 100) == 30
    assert int32_delta(70, 100) == -30
