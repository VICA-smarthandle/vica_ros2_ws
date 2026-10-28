#include "vica_nav2_bt_plugins/straight_path.hpp"

#include <algorithm>
#include <cmath>

namespace vica_nav2_bt_plugins
{

namespace
{
constexpr double kSamePoint = 1e-3;   // 1 mm 안이면 선 방향이 없다
}  // namespace

nav_msgs::msg::Path makeStraightPath(
  double rx, double ry, double robot_yaw, double gx, double gy, double spacing,
  const std::string & frame)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  if (!std::isfinite(rx) || !std::isfinite(ry) || !std::isfinite(robot_yaw) ||
    !std::isfinite(gx) || !std::isfinite(gy) || !(spacing > 0.0))
  {
    return path;
  }
  const double dx = gx - rx, dy = gy - ry;
  const double d = std::hypot(dx, dy);
  const double yaw = d < kSamePoint ? robot_yaw : std::atan2(dy, dx);
  // 끝점이 정확히 목적지에 오도록 간격을 조금 줄여 n 등분한다(마지막 칸이 짧게 남지 않게).
  const int n = std::max(1, static_cast<int>(std::ceil(d / spacing)));
  path.poses.reserve(static_cast<std::size_t>(n) + 1);
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = frame;
    ps.pose.position.x = rx + t * dx;
    ps.pose.position.y = ry + t * dy;
    ps.pose.orientation.z = std::sin(yaw / 2.0);
    ps.pose.orientation.w = std::cos(yaw / 2.0);
    path.poses.push_back(ps);
  }
  return path;
}

}  // namespace vica_nav2_bt_plugins
