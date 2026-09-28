#include "vica_vcc_controller/core/ultrasonic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vica_vcc_controller::core
{
void UltrasonicChannel::push(const RangeReading & r)
{
  hist_.push_back(r);
  while (hist_.size() > 4) {hist_.pop_front();}
}

namespace
{
// 점 p 에서 측정 r 의 호(받은 순간 센서 자세 기준, 거리 range, 폭 fov)까지 가장 가까운 거리.
// 방위가 호 안이면 반지름 차, 밖이면 가까운 끝점까지다.
double distanceToArc(const RangeReading & r, const Point2D & p)
{
  const double dx = p.x - r.sensor.x;
  const double dy = p.y - r.sensor.y;
  const double d = std::hypot(dx, dy);
  const double half = std::max(0.0, r.fov) / 2.0;
  if (d < 1e-9) {return r.range;}
  const double bearing = std::remainder(std::atan2(dy, dx) - r.sensor.yaw, 2.0 * M_PI);
  if (std::fabs(bearing) <= half) {return std::fabs(d - r.range);}
  double best = std::numeric_limits<double>::infinity();
  for (const double a : {-half, half}) {
    const Point2D e = toParent(r.sensor, Point2D{r.range * std::cos(a), r.range * std::sin(a)});
    best = std::min(best, std::hypot(p.x - e.x, p.y - e.y));
  }
  return best;
}
}  // namespace

std::optional<RangeReading> UltrasonicChannel::confirmed(
  double now, double max_age, int count, double tol) const
{
  if (count < 1 || static_cast<int>(hist_.size()) < count) {return std::nullopt;}
  const size_t first = hist_.size() - static_cast<size_t>(count);
  for (size_t i = first; i < hist_.size(); ++i) {
    const RangeReading & r = hist_[i];
    if (now - r.recv_time > max_age) {return std::nullopt;}
    // 드라이버는 에코 없음을 max_range 로 발행한다(user_guidance_driver_node US_CLEAR_MM)
    if (!(r.range > r.min_range && r.range < r.max_range - 1e-3)) {return std::nullopt;}
  }
  // 연속한 두 측정마다, 새 측정의 호 중심점(전역 좌표)이 이전 측정의 호에서 tol 안이어야 한다.
  // 중심끼리 비교하면 옆 채널이 긴 벽을 지날 때 매번 벽의 다른 점을 보아 v > 0.36 m/s 에서
  // 영영 확인되지 않는다. 이전 원뿔이 본 면은 이전 호 어딘가를 지나므로 호까지 거리로 본다.
  // 정면 접근은 중심이 호 위에 오므로 그대로 통과하고, 한 번 튄 값은 이전 호에서 멀어 걸러진다.
  for (size_t i = first + 1; i < hist_.size(); ++i) {
    const RangeReading & newer = hist_[i];
    const Point2D c = toParent(newer.sensor, Point2D{newer.range, 0.0});
    if (distanceToArc(hist_[i - 1], c) > tol) {return std::nullopt;}
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
