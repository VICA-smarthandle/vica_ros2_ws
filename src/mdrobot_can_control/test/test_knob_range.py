"""줄이 실제로 움직이는 구간을 0~100 으로 펴는 환산 (순수 로직)."""

from mdrobot_can_control.motor_watchdog import normalize_knob_pct

import pytest


def test_default_range_changes_nothing():
    """0~100 이면 원값 그대로다 — 종전 동작을 지킨다."""
    for raw in (0, 5, 55, 76, 98, 100):
        assert normalize_knob_pct(raw, 0, 100) == raw


def test_minimum_becomes_zero():
    """줄을 완전히 당긴 자리가 0 이 되어야 정지 판정(deadzone)에 걸린다."""
    assert normalize_knob_pct(55, 55, 98) == 0


def test_maximum_becomes_hundred():
    """줄을 놓은 자리가 100 이어야 최고 속도가 나온다."""
    assert normalize_knob_pct(98, 55, 98) == 100


def test_middle_becomes_about_half():
    """가운데는 절반이다. 55~98 의 중앙은 76.5 이므로 76 은 약 49 %."""
    assert normalize_knob_pct(76, 55, 98) == 49
    assert normalize_knob_pct(77, 55, 98) == 51


def test_below_minimum_is_clamped_to_zero():
    """관측값이 하한을 밑돌아도 음수를 내지 않는다 — 속도가 뒤집히면 안 된다."""
    assert normalize_knob_pct(50, 55, 98) == 0
    assert normalize_knob_pct(0, 55, 98) == 0


def test_above_maximum_is_clamped_to_hundred():
    """상한을 넘겨도 100 을 넘지 않는다. 상한 위로 새면 최고속도를 넘긴다."""
    assert normalize_knob_pct(100, 55, 98) == 100


def test_inverted_range_falls_back_to_raw():
    """상한 <= 하한 은 설정 실수이므로 원값을 그대로 흘린다."""
    assert normalize_knob_pct(70, 98, 55) == 70
    assert normalize_knob_pct(70, 60, 60) == 70


def test_is_monotonic():
    """당길수록 값이 작아지고 놓을수록 커진다 — 순서가 뒤집히면 안 된다."""
    vals = [normalize_knob_pct(raw, 55, 98) for raw in range(55, 99)]
    assert vals == sorted(vals)
    assert vals[0] == 0
    assert vals[-1] == 100


@pytest.mark.parametrize('raw,expected', [(55, 0), (66, 26), (87, 74), (98, 100)])
def test_real_rope_positions(raw, expected):
    """실기에서 잰 줄 위치들. 이 표가 곧 '당긴 만큼 느려진다' 의 내용이다."""
    assert normalize_knob_pct(raw, 55, 98) == expected
