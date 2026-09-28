#pragma once
#include <cmath>
#include <vector>

namespace vica_vcc_controller::core
{
struct Point2D { double x{0.0}; double y{0.0}; };
struct Pose2D { double x{0.0}; double y{0.0}; double yaw{0.0}; };
struct Twist2D { double v{0.0}; double w{0.0}; };
using Polygon = std::vector<Point2D>;
using Path = std::vector<Pose2D>;

inline double normalizeAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

// frame 안의 좌표 p 를 frame 의 부모 좌표로 옮긴다.
inline Point2D toParent(const Pose2D & frame, const Point2D & p)
{
  const double c = std::cos(frame.yaw), s = std::sin(frame.yaw);
  return {frame.x + c * p.x - s * p.y, frame.y + s * p.x + c * p.y};
}
inline Pose2D toParent(const Pose2D & frame, const Pose2D & p)
{
  const Point2D q = toParent(frame, Point2D{p.x, p.y});
  return {q.x, q.y, normalizeAngle(frame.yaw + p.yaw)};
}
// 부모 좌표 p 를 frame 안의 좌표로 옮긴다.
inline Point2D toChild(const Pose2D & frame, const Point2D & p)
{
  const double dx = p.x - frame.x, dy = p.y - frame.y;
  const double c = std::cos(frame.yaw), s = std::sin(frame.yaw);
  return {c * dx + s * dy, -s * dx + c * dy};
}
}  // namespace vica_vcc_controller::core
