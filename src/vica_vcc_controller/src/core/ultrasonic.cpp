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
  // 거리끼리가 아니라 호 중심점(전역 좌표)끼리 비교한다. 다가가는 중에는 거리가 0.42·v 씩 줄어
  // v > 0.36 m/s 면 거리 비교로는 영영 확인되지 않는다(최종 리뷰 I1).
  std::vector<Point2D> centers;
  for (int i = 0; i < count; ++i) {
    const RangeReading & r = hist_[hist_.size() - 1 - i];
    if (now - r.recv_time > max_age) {return std::nullopt;}
    // 드라이버는 에코 없음을 max_range 로 발행한다(user_guidance_driver_node US_CLEAR_MM)
    if (!(r.range > r.min_range && r.range < r.max_range - 1e-3)) {return std::nullopt;}
    centers.push_back(toParent(r.sensor, Point2D{r.range, 0.0}));
  }
  for (size_t i = 0; i < centers.size(); ++i) {
    for (size_t j = i + 1; j < centers.size(); ++j) {
      if (std::hypot(centers[i].x - centers[j].x, centers[i].y - centers[j].y) > tol) {
        return std::nullopt;
      }
    }
  }
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
std::vector<Point2D> readingToArcPoints(const RangeReading & r, int n)
{
  std::vector<Point2D> pts = rangeToArcPoints(r.range, r.fov, n);
  for (auto & p : pts) {p = toParent(r.sensor, p);}
  return pts;
}
}  // namespace vica_vcc_controller::core
