"""정지 중 자이로 편향 추정기의 계약.

2026-08-01 Jetson 실측: 정지 상태에서 EKF yaw가 시간당 161° 드리프트했다.
원인은 base_link 기준 gyro.z 편향 +0.000917 rad/s(시간당 189°)이며, EKF의 두 입력
(/wheel/odom, /imu/base_link)이 모두 각'속도'만 주므로 편향이 적분되어 무한히 쌓인다.

이 추정기는 기동 직후 정지 구간에서 평균을 내어 그 값을 빼준다.

가장 중요한 계약은 마지막 절이다 — 보정 중 로봇이 움직였으면 **편향을 적용하지 않는다.**
틀린 상수를 빼는 것은 드리프트보다 나쁘다. 드리프트는 느리게 쌓이지만 틀린 상수는
즉시 모든 회전을 왜곡한다.
"""

import pytest

from vica_sensor_adapters.gyro_bias import GyroBiasEstimator


def _feed(estimator, count, value=(0.001, 0.002, 0.003)):
    for _ in range(count):
        estimator.add(*value)
    return estimator


def test_before_enough_samples_it_does_not_correct():
    """표본이 모자라면 보정하지 않는다. 원값을 그대로 돌려준다."""
    est = _feed(GyroBiasEstimator(sample_count=10, max_abs_rate=0.05), 9)

    assert not est.ready
    assert est.correct(0.5, 0.6, 0.7) == (0.5, 0.6, 0.7)


def test_exactly_enough_samples_becomes_ready():
    """경계값: 표본이 정확히 sample_count면 준비된다."""
    est = _feed(GyroBiasEstimator(sample_count=10, max_abs_rate=0.05), 10)

    assert est.ready


def test_bias_is_the_mean_of_collected_samples():
    """편향은 수집 구간의 축별 평균이다."""
    est = GyroBiasEstimator(sample_count=4, max_abs_rate=0.05)
    for gz in (0.001, 0.002, 0.003, 0.004):
        est.add(0.0, 0.0, gz)

    bx, by, bz = est.bias
    assert bx == pytest.approx(0.0)
    assert by == pytest.approx(0.0)
    assert bz == pytest.approx(0.0025)


def test_correction_subtracts_the_bias():
    """준비된 뒤에는 원값에서 편향을 뺀다."""
    est = _feed(GyroBiasEstimator(sample_count=5, max_abs_rate=0.05), 5,
                value=(0.0, 0.0, 0.001))

    _, _, gz = est.correct(0.010, 0.020, 0.030)
    assert gz == pytest.approx(0.029)


def test_motion_during_calibration_aborts_permanently():
    """보정 중 움직이면 포기한다. 틀린 편향을 박아넣지 않는다."""
    est = GyroBiasEstimator(sample_count=10, max_abs_rate=0.05)
    _feed(est, 5)
    est.add(0.0, 0.0, 0.5)          # 회전이 들어왔다
    _feed(est, 100)                  # 이후 아무리 조용해도

    assert est.aborted
    assert not est.ready
    assert est.correct(0.5, 0.6, 0.7) == (0.5, 0.6, 0.7)


def test_rate_exactly_at_threshold_is_accepted():
    """경계값: 임계값과 같은 각속도는 정지로 본다."""
    est = GyroBiasEstimator(sample_count=2, max_abs_rate=0.05)
    est.add(0.0, 0.0, 0.05)
    est.add(0.0, 0.0, 0.05)

    assert not est.aborted
    assert est.ready


def test_rate_just_over_threshold_aborts():
    """경계값: 임계값을 조금이라도 넘으면 포기한다."""
    est = GyroBiasEstimator(sample_count=2, max_abs_rate=0.05)
    est.add(0.0, 0.0, 0.0500001)

    assert est.aborted


def test_threshold_applies_to_every_axis():
    """임계 판정은 세 축 각각에 적용한다."""
    est = GyroBiasEstimator(sample_count=2, max_abs_rate=0.05)
    est.add(0.9, 0.0, 0.0)

    assert est.aborted


def test_zero_sample_count_disables_the_feature():
    """sample_count가 0이면 기능을 끈다. 보정도 포기도 하지 않는다."""
    est = _feed(GyroBiasEstimator(sample_count=0, max_abs_rate=0.05), 50)

    assert not est.ready
    assert not est.aborted
    assert est.correct(0.5, 0.6, 0.7) == (0.5, 0.6, 0.7)


def test_extra_samples_after_ready_do_not_change_bias():
    """준비된 뒤 들어오는 값은 편향을 바꾸지 않는다. 주행 중 값이 섞이면 안 된다."""
    est = _feed(GyroBiasEstimator(sample_count=3, max_abs_rate=0.05), 3,
                value=(0.0, 0.0, 0.001))
    before = est.bias

    est.add(0.0, 0.0, 0.04)
    assert est.bias == before


def test_progress_reports_collected_and_target():
    """진행 상황을 읽을 수 있어야 로그와 진단에 남길 수 있다."""
    est = _feed(GyroBiasEstimator(sample_count=10, max_abs_rate=0.05), 4)

    assert est.collected == 4
    assert est.sample_count == 10
