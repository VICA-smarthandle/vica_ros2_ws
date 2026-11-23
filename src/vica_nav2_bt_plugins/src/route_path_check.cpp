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

double remainingTurnDeg(const nav_msgs::msg::Path & path, double robot_x, double robot_y)
{
  const auto & ps = path.poses;
  if (ps.size() < 2) {return 0.0;}
  std::size_t near = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < ps.size(); ++i) {
    const double d = std::hypot(ps[i].pose.position.x - robot_x, ps[i].pose.position.y - robot_y);
    if (d < best) {best = d; near = i;}
  }
  const auto & a = ps[near].pose.position;
  std::size_t j = near + 1;
  while (j < ps.size() && std::hypot(ps[j].pose.position.x - a.x, ps[j].pose.position.y - a.y) < 0.2) {++j;}
  const auto & z = ps.back().pose.position;
  std::size_t k = ps.size() - 1;
  while (k > near && std::hypot(z.x - ps[k].pose.position.x, z.y - ps[k].pose.position.y) < 0.2) {--k;}
  if (j >= ps.size() || k <= near || k < j) {return 0.0;}
  const double h0 = std::atan2(ps[j].pose.position.y - a.y, ps[j].pose.position.x - a.x);
  const double h1 = std::atan2(z.y - ps[k].pose.position.y, z.x - ps[k].pose.position.x);
  return std::abs(std::remainder(h1 - h0, 2.0 * M_PI)) * 180.0 / M_PI;
}
}

RoutePathVerdict checkRoutePath(
  const nav_msgs::msg::Path & path, double robot_x, double robot_y,
  std::size_t min_poses, double max_dist,
  double goal_x, double goal_y, double handoff,
  double keep_goal_dist, double keep_min_turn_deg)
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
    if (keep_goal_dist > 0.0) {
      for (std::size_t i = 1; i < poses.size(); ++i) {
        v.goal_to_path = std::min(
          v.goal_to_path, segmentDistance(
            goal_x, goal_y, poses[i - 1].pose.position.x, poses[i - 1].pose.position.y,
            poses[i].pose.position.x, poses[i].pose.position.y, 0.0));
      }
      v.remaining_turn_deg = remainingTurnDeg(path, robot_x, robot_y);
      if (v.goal_to_path <= keep_goal_dist && v.remaining_turn_deg >= keep_min_turn_deg) {
        v.kept_for_corner = true;
        v.usable = true;
        return v;
      }
    }
    v.reason = "near goal: freespace approach";
    return v;
  }
  v.usable = true;
  return v;
}

}
