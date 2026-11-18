from encoder_feedback.encoder_feedback import int32_delta
from encoder_feedback.encoder_feedback import int32_le
from encoder_feedback.encoder_feedback import pick_dt
import pytest


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


def test_pick_dt_uses_frame_gap():
    assert pick_dt(10.067, 10.0, 0.05) == (pytest.approx(0.067), True)


def test_pick_dt_falls_back_to_timer():
    # 프레임 시각 없음 · 역행 · 0.5 s 넘는 공백
    assert pick_dt(0.0, 10.0, 0.05) == (0.05, False)
    assert pick_dt(10.0, None, 0.05) == (0.05, False)
    assert pick_dt(10.0, 10.0, 0.05) == (0.05, False)
    assert pick_dt(9.9, 10.0, 0.05) == (0.05, False)
    assert pick_dt(10.6, 10.0, 0.05) == (0.05, False)


def _mean_speed_ratio(use_frames, period=0.0667, timer=0.05, phase=0.013):
    """67 ms 마다 오는 위치를 50 ms 타이머로 읽을 때 속도 표본 평균 / 진짜 속도."""
    speed = 0.3
    arrivals = [k * period for k in range(300)]
    samples = []
    prev_arr = arrivals[0]
    last_tick = phase
    tick = phase
    i = 0
    while tick < arrivals[-1]:
        tick += timer
        new = i
        while new + 1 < len(arrivals) and arrivals[new + 1] <= tick:
            new += 1
        if new == i:
            continue
        frame_t = arrivals[new] if use_frames else 0.0
        dt, _ = pick_dt(frame_t, prev_arr, tick - last_tick)
        samples.append(speed * (arrivals[new] - prev_arr) / dt)
        prev_arr, last_tick, i = arrivals[new], tick, new
    return sum(samples) / len(samples) / speed


def test_timer_dt_inflates_speed_samples():
    # 2026-10-20 감사의 원인 재현: 타이머 시각으로 나누면 표본 평균이 부풀려진다
    assert _mean_speed_ratio(use_frames=False) > 1.03


def test_frame_dt_keeps_speed_samples_true():
    assert _mean_speed_ratio(use_frames=True) == pytest.approx(1.0, abs=0.01)
