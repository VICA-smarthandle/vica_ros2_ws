#include "vica_nav2_bt_plugins/route_path_check.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vica_nav2_bt_plugins
{
namespace
{
double segmentDistance(double px, double py, double ax, double ay, double bx, double by, double back)
{
  const double dx = bx - ax, dy = by - ay;
  const double len = std::hypot(dx, dy);
  if (len < 1e-9) {return std::hypot(px - ax, py - ay);}
  const double ux = dx / len, uy = dy / len;
  const double s = std::clamp((px - ax) * ux + (py - ay) * uy, -back, len);
  return std::hypot(px - (ax + s * ux), py - (ay + s * uy));
}
}

RoutePathVerdict checkRoutePath(
  const nav_msgs::msg::Path & path, double robot_x, double robot_y,
  std::size_t min_poses, double max_dist,
  double goal_x, double goal_y, double handoff)
{
  RoutePathVerdict v;
  v.poses = path.poses.size();
  v.dist_to_path = std::numeric_limits<double>::infinity();
  v.dist_to_points = std::numeric_limits<double>::infinity();
  v.dist_to_goal = (std::isfinite(goal_x) && std::isfinite(goal_y)) ?
    std::hypot(goal_x - robot_x, goal_y - robot_y) : std::numeric_limits<double>::infinity();

  for (const auto & ps : path.poses) {
    if (!std::isfinite(ps.pose.position.x) || !std::isfinite(ps.pose.position.y)) {
      v.reason = "non-finite pose";
      return v;
    }
    const double d = std::hypot(ps.pose.position.x - robot_x, ps.pose.position.y - robot_y);
    if (d < v.dist_to_points) {
      v.dist_to_points = d;
    }
  }
  v.dist_to_path = v.dist_to_points;
  const auto & poses = path.poses;
  for (std::size_t i = 1; i < poses.size(); ++i) {
    v.dist_to_path = std::min(
      v.dist_to_path, segmentDistance(
        robot_x, robot_y, poses[i - 1].pose.position.x, poses[i - 1].pose.position.y,
        poses[i].pose.position.x, poses[i].pose.position.y, 0.0));
  }
  if (poses.size() >= 2) {
    const auto & p0 = poses.front().pose.position;
    std::size_t k = 1;
    while (k + 1 < poses.size() &&
      std::hypot(poses[k].pose.position.x - p0.x, poses[k].pose.position.y - p0.y) < 0.2) {++k;}
    v.dist_to_path = std::min(
      v.dist_to_path, segmentDistance(
        robot_x, robot_y, p0.x, p0.y, poses[k].pose.position.x, poses[k].pose.position.y,
        kPrunedEdgeExtend));
  }

  if (v.poses < min_poses) {
    v.reason = "points<" + std::to_string(min_poses);
    return v;
  }
  if (v.dist_to_path > max_dist) {
    v.reason = "robot far from path";
    return v;
  }
  if (handoff > 0.0 && v.dist_to_goal < handoff) {
    v.reason = "near goal: freespace approach";
    return v;
  }
  v.usable = true;
  return v;
}

}
