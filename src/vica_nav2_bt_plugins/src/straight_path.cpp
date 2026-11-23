#include "vica_nav2_bt_plugins/straight_path.hpp"

#include <algorithm>
#include <cmath>

namespace vica_nav2_bt_plugins
{

namespace
{
constexpr double kSamePoint = 1e-3;
}

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

}
