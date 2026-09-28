#include "vica_vcc_controller/core/ultrasonic.hpp"

#include <algorithm>
#include <cmath>

namespace vica_vcc_controller::core
{
void UltrasonicChannel::push(const RangeReading & r)
{
  hist_.push_back(r);
  while (hist_.size() > 4) {hist_.pop_front();}
}

std::optional<RangeReading> UltrasonicChannel::confirmed(
  double now, double max_age, int count, double tol) const
{
  if (count < 1 || static_cast<int>(hist_.size()) < count) {return std::nullopt;}
  double lo = 1e9, hi = -1e9;
  for (int i = 0; i < count; ++i) {
    const RangeReading & r = hist_[hist_.size() - 1 - i];
    if (now - r.recv_time > max_age) {return std::nullopt;}
    // 드라이버는 에코 없음을 max_range 로 발행한다(user_guidance_driver_node US_CLEAR_MM)
    if (!(r.range > r.min_range && r.range < r.max_range - 1e-3)) {return std::nullopt;}
    lo = std::min(lo, r.range);
    hi = std::max(hi, r.range);
  }
  if (hi - lo > tol) {return std::nullopt;}
  return hist_.back();
}

std::vector<Point2D> rangeToArcPoints(double range, double fov, int n)
{
  std::vector<Point2D> pts;
  if (n < 1) {return pts;}
  for (int i = 0; i < n; ++i) {
    const double a = n == 1 ? 0.0 : -fov / 2.0 + fov * i / (n - 1);
    pts.push_back({range * std::cos(a), range * std::sin(a)});
  }
  return pts;
}
}  // namespace vica_vcc_controller::core
