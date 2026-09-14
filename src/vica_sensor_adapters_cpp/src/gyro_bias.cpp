// 정지 중 자이로 편향 추정 — 파이썬 판(gyro_bias.py)의 로직을 그대로 옮겼다.
//
// 옮기면서 바꾼 것은 없다. 계약 설명과 실측 근거는 파이썬 판 상단 주석과
// 시험 파일에 있으며, 이 파일에는 "왜 이 줄이 이렇게 생겼는가"만 남긴다.
#include "vica_sensor_adapters_cpp/gyro_bias.hpp"

#include <algorithm>
#include <cmath>

namespace vica_sensor_adapters_cpp
{

namespace
{
/// 세 축 절대값 중 최대. 정지 판정과 흔들림 판정에 같은 잣대를 쓴다.
double max_abs3(double a, double b, double c)
{
  return std::max({std::fabs(a), std::fabs(b), std::fabs(c)});
}
}  // namespace

GyroBiasEstimator::GyroBiasEstimator(
  int sample_count, double max_abs_rate, int refresh_sample_count,
  double refresh_alpha, double max_abs_dev, double max_refresh_jump)
: sample_count_(sample_count),
  max_abs_rate_(max_abs_rate),
  refresh_sample_count_(refresh_sample_count),
  refresh_alpha_(refresh_alpha),
  max_abs_dev_(max_abs_dev),
  max_refresh_jump_(max_refresh_jump)
{
}

void GyroBiasEstimator::add(double gx, double gy, double gz)
{
  if (sample_count_ <= 0) {
    return;                     // 기능을 끈 것이다. 보정도 포기도 하지 않는다.
  }
  if (ready_ || aborted_) {
    add_refresh(gx, gy, gz);    // 기동 확정이 끝났으면 ZUPT 가 이어받는다
    return;
  }

  // 세 축 중 하나라도 임계를 넘으면 정지 상태가 아니다.
  if (max_abs3(gx, gy, gz) > max_abs_rate_) {
    aborted_ = true;
    return;
  }

  sums_[0] += gx;
  sums_[1] += gy;
  sums_[2] += gz;
  ++collected_;

  if (collected_ >= sample_count_) {
    const double n = static_cast<double>(collected_);
    bias_ = Vec3{sums_[0] / n, sums_[1] / n, sums_[2] / n};
    ready_ = true;
  }
}

void GyroBiasEstimator::add_refresh(double gx, double gy, double gz)
{
  if (refresh_sample_count_ <= 0) {
    return;                     // 종전처럼 기동 시 한 번만 확정하고 고정한다
  }

  // ① 크기가 임계를 넘는다 — 확실히 움직인다.
  if (max_abs3(gx, gy, gz) > max_abs_rate_) {
    reset_refresh();
    return;
  }

  // ② 구간 첫 표본에서 크게 벗어난다 — 흔들린다(직진 중).
  //    이 검사가 없으면 직진을 정차로 착각해 마스트 진동이 편향으로 들어간다.
  if (!r_has_first_) {
    r_first_ = Vec3{gx, gy, gz};
    r_has_first_ = true;
  } else {
    const double dev = max_abs3(gx - r_first_[0], gy - r_first_[1], gz - r_first_[2]);
    if (dev > max_abs_dev_) {
      reset_refresh();
      return;
    }
  }

  r_sums_[0] += gx;
  r_sums_[1] += gy;
  r_sums_[2] += gz;
  ++r_count_;

  if (r_count_ < refresh_sample_count_) {
    return;
  }

  const double n = static_cast<double>(r_count_);
  const Vec3 mean{r_sums_[0] / n, r_sums_[1] / n, r_sums_[2] / n};
  reset_refresh();

  if (ready_) {
    // 옛 값과 너무 다르면 정차 판정이 틀렸다고 본다. 편향은 온도로 천천히
    // 변하지 정차 한 번에 뛰지 않는다.
    const double jump = max_abs3(
      mean[0] - bias_[0], mean[1] - bias_[1], mean[2] - bias_[2]);
    if (jump > max_refresh_jump_) {
      return;
    }
    const double a = refresh_alpha_;
    for (int i = 0; i < 3; ++i) {
      bias_[i] = (1.0 - a) * bias_[i] + a * mean[i];
    }
  } else {
    // 기동 때 못 쟀던 경우다. 첫 정차 값을 그대로 받는다.
    bias_ = mean;
    ready_ = true;
  }
  ++refresh_count_;
}

void GyroBiasEstimator::reset_refresh()
{
  // 모으던 정차 구간을 버린다. 끊긴 구간을 이어붙이지 않는다.
  r_sums_ = Vec3{0.0, 0.0, 0.0};
  r_has_first_ = false;
  r_count_ = 0;
}

Vec3 GyroBiasEstimator::correct(double gx, double gy, double gz) const
{
  if (!ready_) {
    return Vec3{gx, gy, gz};
  }
  return Vec3{gx - bias_[0], gy - bias_[1], gz - bias_[2]};
}

}  // namespace vica_sensor_adapters_cpp
