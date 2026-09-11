"""정지 중 자이로 편향 추정기의 계약."""

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
    est.add(0.0, 0.0, 0.5)
    _feed(est, 100)

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


def _quiet(estimator, count, value=0.01):
    """정차로 인정될 만큼 조용한 표본."""
    for _ in range(count):
        estimator.add(value, value, value)


def _shaky(estimator, count, center=0.01, amp=0.03):
    """직진 중처럼 흔들리는 표본. 평균은 center 지만 폭이 크다."""
    for i in range(count):
        d = amp if i % 2 == 0 else -amp
        estimator.add(center + d, center + d, center + d)


def _ready_estimator(bias=0.01, **kw):
    """초기 확정을 마친 추정기를 만든다."""
    est = GyroBiasEstimator(sample_count=10, max_abs_rate=0.05, **kw)
    _feed(est, 10, (bias, bias, bias))
    assert est.ready
    return est


def test_refresh_off_by_default_keeps_old_behaviour():
    """refresh_sample_count 0 이면 종전과 똑같이 한 번 확정하고 고정이다."""
    est = _ready_estimator(bias=0.01, refresh_sample_count=0)
    before = est.bias
    _quiet(est, 500, value=0.02)
    assert est.bias == before


def test_quiet_stretch_updates_the_bias():
    """정차가 충분히 이어지면 편향을 갱신한다."""
    est = _ready_estimator(bias=0.01, refresh_sample_count=20,
                           refresh_alpha=0.5, max_abs_dev=0.01)
    _quiet(est, 20, value=0.02)
    assert est.bias[2] == pytest.approx(0.015, abs=1e-9)


def test_update_is_blended_not_replaced():
    """한 번의 측정이 편향을 통째로 갈아치우지 않는다."""
    est = _ready_estimator(bias=0.0, refresh_sample_count=10,
                           refresh_alpha=0.2, max_abs_dev=0.01,
                           max_refresh_jump=1.0)
    _quiet(est, 10, value=0.04)
    assert est.bias[2] == pytest.approx(0.008, abs=1e-9)


def test_shaky_stretch_is_rejected():
    """직진처럼 흔들리는 구간은 정차로 보지 않는다 — 이 시험이 핵심이다."""
    est = _ready_estimator(bias=0.01, refresh_sample_count=20,
                           refresh_alpha=0.5, max_abs_dev=0.01)
    before = est.bias
    _shaky(est, 200, center=0.05, amp=0.03)
    assert est.bias == before


def test_motion_resets_the_stretch():
    """중간에 움직이면 모으던 표본을 버린다. 이어붙이지 않는다."""
    est = _ready_estimator(bias=0.01, refresh_sample_count=20,
                           refresh_alpha=0.5, max_abs_dev=0.01)
    before = est.bias
    _quiet(est, 15, value=0.02)
    est.add(0.5, 0.0, 0.0)
    _quiet(est, 15, value=0.02)
    assert est.bias == before


def test_sudden_jump_is_refused():
    """옛 값과 너무 다르면 그 회차를 버린다."""
    est = _ready_estimator(bias=0.01, refresh_sample_count=10,
                           refresh_alpha=0.5, max_abs_dev=0.01,
                           max_refresh_jump=0.02)
    before = est.bias
    _quiet(est, 10, value=0.04)
    assert est.bias == before


def test_refresh_works_even_if_startup_aborted():
    """기동 때 못 쟀어도 첫 정차에서 잡는다."""
    est = GyroBiasEstimator(sample_count=10, max_abs_rate=0.05,
                            refresh_sample_count=10, refresh_alpha=1.0,
                            max_abs_dev=0.01)
    est.add(0.5, 0.0, 0.0)
    assert est.aborted and not est.ready
    _quiet(est, 10, value=0.02)
    assert est.ready
    assert est.bias[2] == pytest.approx(0.02, abs=1e-9)


def test_refresh_count_is_reported():
    """몇 번 갱신했는지 볼 수 있어야 실주행에서 동작을 확인한다."""
    est = _ready_estimator(bias=0.01, refresh_sample_count=10,
                           refresh_alpha=0.5, max_abs_dev=0.01)
    assert est.refresh_count == 0
    _quiet(est, 10, value=0.02)
    assert est.refresh_count == 1
