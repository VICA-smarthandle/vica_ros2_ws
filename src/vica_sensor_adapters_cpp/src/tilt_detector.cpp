// 몸통 넘어짐 판정. 계약은 헤더 주석에 있다.
#include "vica_sensor_adapters_cpp/tilt_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vica_sensor_adapters_cpp
{

TiltDetector::TiltDetector(
  double tip_angle_deg, double release_angle_deg, double hold_sec, double max_gap_sec)
: tip_angle_deg_(tip_angle_deg),
  release_angle_deg_(std::min(release_angle_deg, tip_angle_deg)),
  hold_sec_(std::max(0.0, hold_sec)),
  max_gap_sec_(max_gap_sec)
{
}

double TiltDetector::tilt_deg(double ax, double ay, double az)
{
  const double norm = std::sqrt(ax * ax + ay * ay + az * az);
  if (!std::isfinite(norm) || norm < kMinAccelNorm) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double cos_tilt = std::clamp(az / norm, -1.0, 1.0);
  return std::acos(cos_tilt) * 180.0 / M_PI;
}

bool TiltDetector::update(double tilt, double now_sec)
{
  // 시간이 거꾸로 가거나 표본 사이가 벌어졌다 — 앞의 셈은 버리고 이 표본부터 센다.
  const bool broken = last_now_ &&
    (now_sec < *last_now_ || now_sec - *last_now_ > max_gap_sec_);
  last_now_ = now_sec;
  if (broken) {
    since_.reset();
  }
  if (!std::isfinite(tilt)) {
    since_.reset();
    return false;
  }

  // '넘어지는 각 이상'은 넘어짐 쪽, '풀리는 각 이하'는 서 있음 쪽이다. 둘 사이(30~45°)는
  // 어느 쪽도 아니다. 이미 그 상태면 셀 것이 없다.
  bool toward_tipped;
  if (tilt >= tip_angle_deg_) {
    toward_tipped = true;
  } else if (tilt <= release_angle_deg_) {
    toward_tipped = false;
  } else {
    since_.reset();
    return false;
  }
  if (known_ && toward_tipped == tipped_) {
    since_.reset();
    return false;
  }
  if (!since_ || pending_tipped_ != toward_tipped) {
    since_ = now_sec;
    pending_tipped_ = toward_tipped;
  }
  if (now_sec - *since_ < hold_sec_) {
    return false;
  }
  known_ = true;
  tipped_ = toward_tipped;
  since_.reset();
  return true;
}

}  // namespace vica_sensor_adapters_cpp
