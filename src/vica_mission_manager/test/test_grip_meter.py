"""GripMeter — 손잡이 접촉 비율을 시간으로 잰다 (터치×진동 설계 4.2절)."""
import math

import pytest

from vica_mission_manager.grip_meter import GripMeter


def feed(meter, samples):
    for t, contact in samples:
        meter.update(t, contact, True)


def test_no_data_is_not_gripped():
    m = GripMeter(stale_sec=1.0)
    assert m.ratio(5.0, 2.0) == 0.0
    assert not m.fresh(5.0)
    assert not m.contact(5.0)
    assert math.isinf(m.released_for(5.0))


def test_steady_grip_counts_by_time_not_messages():
    """가만히 쥐면 메시지는 2 Hz 뿐이다. 개수가 아니라 시간으로 100 %."""
    m = GripMeter(stale_sec=1.0)
    feed(m, [(0.0, True), (0.5, True), (1.0, True), (1.5, True), (2.0, True)])
    assert m.ratio(2.0, 2.0) == pytest.approx(1.0)


def test_tapping_does_not_inflate_ratio():
    """두드리면 메시지가 쏟아지지만 잡은 시간은 절반이다."""
    m = GripMeter(stale_sec=1.0)
    t = 0.0
    while t < 2.0:
        m.update(t, True, True)
        m.update(t + 0.05, False, True)
        t += 0.1
    assert m.ratio(2.0, 2.0) == pytest.approx(0.5, abs=0.02)


def test_before_first_sample_counts_as_released():
    m = GripMeter(stale_sec=1.0)
    feed(m, [(1.0, True), (1.5, True), (2.0, True)])
    # 창 [0, 2] 중 [1, 2] 만 잡음
    assert m.ratio(2.0, 2.0) == pytest.approx(0.5)


def test_since_cuts_the_window_head():
    """대기 시작 전부터 쥐고 있었어도 시작 뒤 시간만 센다(창 길이는 그대로)."""
    m = GripMeter(stale_sec=1.0)
    feed(m, [(0.0, True), (0.5, True), (1.0, True), (1.5, True), (2.0, True)])
    assert m.ratio(2.0, 2.0, since=1.0) == pytest.approx(0.5)
    m.update(3.0, True, True)
    assert m.ratio(3.0, 2.0, since=1.0) == pytest.approx(1.0)


def test_stale_turns_contact_off():
    """드라이버가 죽으면 마지막 '잡음'이 붙들리지 않는다 — 시한 뒤 놓음."""
    m = GripMeter(stale_sec=1.0)
    feed(m, [(0.0, True), (0.5, True)])
    assert m.contact(1.4)
    assert not m.fresh(1.6)
    assert not m.contact(1.6)
    # 창 [0.5, 2.5] 중 잡음은 [0.5, 1.5] 한 번
    assert m.ratio(2.5, 2.0) == pytest.approx(0.5)
    assert m.released_for(2.5) == pytest.approx(1.0)


def test_unfresh_message_counts_as_released():
    m = GripMeter(stale_sec=1.0)
    m.update(0.0, True, True)
    m.update(1.0, True, False)   # 드라이버: uplink_fresh=false (user_contact 도 false 로 오지만 방어)
    assert not m.fresh(1.0)
    assert not m.contact(1.0)
    assert m.released_for(1.5) == pytest.approx(0.5)


def test_released_for_measures_continuous_release():
    m = GripMeter(stale_sec=1.0)
    feed(m, [(0.0, True), (1.0, False), (1.3, False)])
    assert m.released_for(1.3) == pytest.approx(0.3)
    m.update(1.4, True, True)
    assert m.released_for(1.4) == 0.0
    m.update(1.6, False, True)
    assert m.released_for(1.9) == pytest.approx(0.3)


def test_repeated_same_value_messages_keep_it_fresh():
    m = GripMeter(stale_sec=1.0)
    for i in range(20):
        m.update(i * 0.5, True, True)
    assert m.fresh(9.5)
    assert m.contact(9.5)
    assert m.ratio(9.5, 2.0) == pytest.approx(1.0)


def test_recovery_after_stale_gap():
    m = GripMeter(stale_sec=1.0)
    m.update(0.0, True, True)
    m.update(3.0, True, True)    # 1.0 ~ 3.0 은 끊겼던 구간
    assert m.ratio(3.0, 3.0) == pytest.approx(1.0 / 3.0)
    assert m.contact(3.0)


def test_history_is_bounded():
    m = GripMeter(stale_sec=1.0, history_sec=5.0)
    t = 0.0
    for i in range(2000):
        m.update(t, i % 2 == 0, True)
        t += 0.1
    assert len(m._points) < 100
