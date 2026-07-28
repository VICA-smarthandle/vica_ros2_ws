"""회전 판정 순수 로직 테스트.

핵심은 ±pi 경계 처리다. yaw는 atan2 결과라 +pi와 -pi 사이에서 점프하므로,
연속 샘플 차이를 정규화해 누적하지 않으면 방향을 반대로 판정한다.
"""

import math

import pytest

from vica_user_guidance.turn_detector import (
    DIRECTION_LEFT,
    DIRECTION_NONE,
    DIRECTION_RIGHT,
    PHASE_COMPLETE,
    PHASE_IDLE,
    PHASE_NOW,
    TurnDetector,
    normalize_angle,
    yaw_from_quaternion,
)

MS = 1_000_000
SEC = 1_000_000_000


def make_detector(
    window_sec=1.5,
    enter_deg=25.0,
    exit_deg=10.0,
    min_duration_sec=0.6,
    odom_timeout_sec=0.5,
):
    """기본 파라미터로 detector를 만든다."""
    return TurnDetector(
        window_ns=int(window_sec * SEC),
        enter_threshold_rad=math.radians(enter_deg),
        exit_threshold_rad=math.radians(exit_deg),
        min_duration_ns=int(min_duration_sec * SEC),
        odom_timeout_ns=int(odom_timeout_sec * SEC),
    )


def feed_rotation(detector, start_yaw, total_deg, duration_sec, t0, step_ms=33):
    """등속 회전을 샘플로 주입하고 마지막 시각을 돌려준다."""
    steps = max(1, int(duration_sec * 1000 / step_ms))
    total_rad = math.radians(total_deg)
    now = t0
    for i in range(steps + 1):
        yaw = start_yaw + total_rad * i / steps
        # yaw를 (-pi, pi]로 wrap해서 실제 /odom이 주는 형태를 재현한다
        detector.add_odom(normalize_angle(yaw), now)
        now = t0 + int((i + 1) * step_ms * MS)
    return now


# ── 기본 유틸 ──────────────────────────────────────────


def test_normalize_angle_wraps_to_pi_range():
    """(-pi, pi] 범위로 정규화한다."""
    assert normalize_angle(0.0) == pytest.approx(0.0)
    assert normalize_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1, abs=1e-9)
    assert normalize_angle(-math.pi - 0.1) == pytest.approx(math.pi - 0.1, abs=1e-9)
    assert abs(normalize_angle(math.pi)) == pytest.approx(math.pi)


def test_yaw_from_quaternion_matches_known_values():
    """2D 회전 quaternion(z=sin(yaw/2), w=cos(yaw/2))에서 yaw를 복원한다."""
    for deg in (0.0, 45.0, 90.0, 179.0, -90.0):
        rad = math.radians(deg)
        z = math.sin(rad / 2.0)
        w = math.cos(rad / 2.0)
        assert yaw_from_quaternion(0.0, 0.0, z, w) == pytest.approx(rad, abs=1e-9)


# ── ±pi 경계 (회귀 방지) ───────────────────────────────


def test_left_turn_across_pi_boundary():
    """+pi를 넘는 좌회전(CCW)에서 방향이 뒤집히지 않는다.

    yaw가 3.0 -> 3.1 -> -3.1 로 점프해도 누적은 양수(LEFT)여야 한다.
    """
    d = make_detector()
    t = feed_rotation(d, start_yaw=3.0, total_deg=40.0, duration_sec=1.0, t0=SEC)
    out = d.evaluate(t)
    assert out.turn_angle_deg > 0
    assert out.direction == DIRECTION_LEFT


def test_right_turn_across_pi_boundary():
    """-pi를 넘는 우회전(CW)에서 방향이 뒤집히지 않는다."""
    d = make_detector()
    t = feed_rotation(d, start_yaw=-3.0, total_deg=-40.0, duration_sec=1.0, t0=SEC)
    out = d.evaluate(t)
    assert out.turn_angle_deg < 0
    assert out.direction == DIRECTION_RIGHT


def test_small_turn_across_boundary_is_not_inverted():
    """경계를 넘는 30도 회전이 -330도로 계산되지 않는다.

    단순 뺄셈으로 구현하면 이 테스트가 실패한다.
    """
    d = make_detector()
    t = feed_rotation(d, start_yaw=3.0, total_deg=30.0, duration_sec=1.0, t0=SEC)
    out = d.evaluate(t)
    assert out.turn_angle_deg == pytest.approx(30.0, abs=3.0)


def test_rotation_beyond_180_degrees_accumulates():
    """윈도우 안에서 190도를 돌면 -170도가 아니라 190도로 누적된다."""
    d = make_detector(window_sec=2.0)
    t = feed_rotation(d, start_yaw=0.0, total_deg=190.0, duration_sec=1.5, t0=SEC)
    out = d.evaluate(t)
    assert out.turn_angle_deg == pytest.approx(190.0, abs=5.0)


def test_ccw_positive_is_left_per_rep103():
    """REP-103: CCW(반시계) 양수 = 좌회전."""
    d = make_detector()
    t = feed_rotation(d, start_yaw=0.0, total_deg=40.0, duration_sec=1.0, t0=SEC)
    out = d.evaluate(t)
    assert out.turn_angle_deg > 0
    assert out.direction == DIRECTION_LEFT


# ── hysteresis ────────────────────────────────────────


def test_below_enter_threshold_gives_no_cue():
    """진입 임계값(25도) 미만이면 cue가 없다."""
    d = make_detector()
    t = feed_rotation(d, start_yaw=0.0, total_deg=20.0, duration_sec=1.0, t0=SEC)
    out = d.evaluate(t)
    assert out.direction == DIRECTION_NONE
    assert out.phase == PHASE_IDLE


def test_stays_active_between_exit_and_enter():
    """진입 후 exit(10도)와 enter(25도) 사이에서는 회전을 유지한다."""
    d = make_detector()
    t = feed_rotation(d, start_yaw=0.0, total_deg=40.0, duration_sec=1.0, t0=SEC)
    assert d.evaluate(t).direction == DIRECTION_LEFT

    # 회전을 멈춰 윈도우 누적이 서서히 줄어들되 exit 위로 유지되게 한다
    t = feed_rotation(d, start_yaw=math.radians(40.0), total_deg=0.0,
                      duration_sec=0.6, t0=t)
    out = d.evaluate(t)
    assert out.direction == DIRECTION_LEFT
    assert out.phase == PHASE_NOW


def test_complete_emitted_once_then_idle():
    """종료 시 COMPLETE를 1회만 내고 이후 IDLE로 떨어진다."""
    d = make_detector()
    t = feed_rotation(d, start_yaw=0.0, total_deg=40.0, duration_sec=1.0, t0=SEC)
    assert d.evaluate(t).direction == DIRECTION_LEFT

    # 정지 상태를 윈도우 이상 유지해 누적을 0으로 만든다
    t = feed_rotation(d, start_yaw=math.radians(40.0), total_deg=0.0,
                      duration_sec=2.0, t0=t)
    first = d.evaluate(t)
    assert first.phase == PHASE_COMPLETE
    assert first.direction == DIRECTION_NONE

    second = d.evaluate(t + 50 * MS)
    assert second.phase == PHASE_IDLE


def test_no_chattering_near_threshold():
    """임계값 근처를 오가도 sequence_id가 폭증하지 않는다."""
    d = make_detector()
    t = SEC
    for _ in range(10):
        t = feed_rotation(d, start_yaw=0.0, total_deg=24.0, duration_sec=0.7, t0=t)
        d.evaluate(t)
        t = feed_rotation(d, start_yaw=math.radians(24.0), total_deg=-24.0,
                          duration_sec=0.7, t0=t)
        d.evaluate(t)
    assert d.evaluate(t).sequence_id <= 2


# ── 최소 지속시간 ──────────────────────────────────────


def test_brief_spike_below_min_duration_ignored():
    """0.3초 만에 임계값을 넘겨도 min_duration(0.6초) 미달이면 무시한다."""
    d = make_detector()
    t = feed_rotation(d, start_yaw=0.0, total_deg=30.0, duration_sec=0.3, t0=SEC)
    out = d.evaluate(t)
    assert out.direction == DIRECTION_NONE


def test_sustained_turn_enters_and_increments_sequence():
    """min_duration을 채우면 진입하고 sequence_id가 1 증가한다."""
    d = make_detector()
    before = d.evaluate(SEC).sequence_id
    t = feed_rotation(d, start_yaw=0.0, total_deg=40.0, duration_sec=1.0, t0=SEC)
    out = d.evaluate(t)
    assert out.phase == PHASE_NOW
    assert out.sequence_id == before + 1


def test_sign_flip_resets_candidate_timer():
    """부호가 바뀌면 지속시간 후보가 리셋되어 양쪽 다 진입하지 않는다."""
    d = make_detector()
    t = feed_rotation(d, start_yaw=0.0, total_deg=20.0, duration_sec=0.4, t0=SEC)
    d.evaluate(t)
    t = feed_rotation(d, start_yaw=math.radians(20.0), total_deg=-40.0,
                      duration_sec=0.4, t0=t)
    out = d.evaluate(t)
    assert out.direction == DIRECTION_NONE


# ── stale 처리 ────────────────────────────────────────


def test_stale_odom_reports_source_stale():
    """/odom이 timeout을 넘기면 source_stale=True, IDLE이다."""
    d = make_detector()
    t = feed_rotation(d, start_yaw=0.0, total_deg=10.0, duration_sec=0.3, t0=SEC)
    out = d.evaluate(t + int(0.6 * SEC))
    assert out.source_stale is True
    assert out.direction == DIRECTION_NONE
    assert out.phase == PHASE_IDLE
    assert math.isnan(out.turn_angle_deg)


def test_never_received_is_stale_not_zero():
    """한 번도 수신하지 않은 상태는 stale이다. 0.0을 sentinel로 쓰지 않는다."""
    d = make_detector()
    out = d.evaluate(SEC)
    assert out.source_stale is True
    assert out.direction == DIRECTION_NONE


def test_stale_recovery_discards_accumulation():
    """stale 후 복구 시 누적을 버려 큰 델타로 즉시 오탐하지 않는다.

    2초 끊긴 사이 로봇이 90도 돌았다면, 복구 첫 샘플의 델타가 90도로 튄다.
    이를 그대로 누적하면 즉시 회전으로 오판한다.
    """
    d = make_detector()
    t = feed_rotation(d, start_yaw=0.0, total_deg=5.0, duration_sec=0.3, t0=SEC)

    # 2초 공백 후 90도 떨어진 yaw로 복귀
    t_gap = t + 2 * SEC
    assert d.evaluate(t_gap).source_stale is True

    d.add_odom(math.radians(90.0), t_gap)
    out = d.evaluate(t_gap)
    assert out.direction == DIRECTION_NONE
    assert out.turn_angle_deg == pytest.approx(0.0, abs=1e-6)


def test_stale_during_turn_does_not_emit_complete():
    """회전 중 stale이 되면 COMPLETE가 아니라 IDLE이다."""
    d = make_detector()
    t = feed_rotation(d, start_yaw=0.0, total_deg=40.0, duration_sec=1.0, t0=SEC)
    assert d.evaluate(t).direction == DIRECTION_LEFT

    out = d.evaluate(t + int(0.6 * SEC))
    assert out.phase == PHASE_IDLE
    assert out.source_stale is True


def test_time_reversal_is_stale_and_does_not_crash():
    """now_ns가 과거로 가도 stale로 처리하고 크래시하지 않는다."""
    d = make_detector()
    t = feed_rotation(d, start_yaw=0.0, total_deg=10.0, duration_sec=0.3, t0=10 * SEC)
    out = d.evaluate(t - 5 * SEC)
    assert out.source_stale is True


# ── 경계 조건 ─────────────────────────────────────────


def test_single_sample_does_not_crash():
    """샘플 1개일 때 누적은 0이고 크래시하지 않는다."""
    d = make_detector()
    d.add_odom(0.0, SEC)
    out = d.evaluate(SEC)
    assert out.turn_angle_deg == pytest.approx(0.0)
    assert out.direction == DIRECTION_NONE


def test_window_slides_and_drops_old_samples():
    """윈도우(1.5초) 밖 샘플은 폐기되어 누적이 전체 회전량이 되지 않는다."""
    d = make_detector(window_sec=1.5, enter_deg=200.0)
    t = feed_rotation(d, start_yaw=0.0, total_deg=90.0, duration_sec=3.0, t0=SEC)
    out = d.evaluate(t)
    # 3초간 90도 등속이면 최근 1.5초 분량은 약 45도다
    assert out.turn_angle_deg == pytest.approx(45.0, abs=8.0)


def test_s_curve_closes_then_opens_new_sequence():
    """S자 코너에서 반대 방향으로 넘어가면 별도 sequence로 진입한다."""
    d = make_detector()
    t = feed_rotation(d, start_yaw=0.0, total_deg=40.0, duration_sec=1.0, t0=SEC)
    first = d.evaluate(t)
    assert first.direction == DIRECTION_LEFT

    t = feed_rotation(d, start_yaw=math.radians(40.0), total_deg=-80.0,
                      duration_sec=1.5, t0=t)
    d.evaluate(t)
    t = feed_rotation(d, start_yaw=math.radians(-40.0), total_deg=-40.0,
                      duration_sec=1.0, t0=t)
    later = d.evaluate(t)
    assert later.direction == DIRECTION_RIGHT
    assert later.sequence_id > first.sequence_id
