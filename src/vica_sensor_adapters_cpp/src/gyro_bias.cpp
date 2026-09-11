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
}

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
    return;
  }
  if (ready_ || aborted_) {
    add_refresh(gx, gy, gz);
    return;
  }

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
    return;
  }

  if (max_abs3(gx, gy, gz) > max_abs_rate_) {
    reset_refresh();
    return;
  }

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
    bias_ = mean;
    ready_ = true;
  }
  ++refresh_count_;
}

void GyroBiasEstimator::reset_refresh()
{
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

}
