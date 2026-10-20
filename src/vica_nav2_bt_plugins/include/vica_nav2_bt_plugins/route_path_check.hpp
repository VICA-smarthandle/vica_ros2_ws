#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <string>

#include "nav_msgs/msg/path.hpp"

namespace vica_nav2_bt_plugins
{

struct RoutePathVerdict
{
  bool usable{false};
  std::size_t poses{0};
  double dist_to_path{0.0};
  double dist_to_points{0.0};
  double dist_to_goal{0.0};
  std::string reason;
};

constexpr double kPrunedEdgeExtend = 1.2;

RoutePathVerdict checkRoutePath(
  const nav_msgs::msg::Path & path, double robot_x, double robot_y,
  std::size_t min_poses, double max_dist,
  double goal_x = std::numeric_limits<double>::quiet_NaN(),
  double goal_y = std::numeric_limits<double>::quiet_NaN(),
  double handoff = 0.0);

class RailDistanceGate
{
public:
  double threshold(double max_dist, double rejoin_dist) const
  {
    return (far_ && rejoin_dist > 0.0 && rejoin_dist < max_dist) ? rejoin_dist : max_dist;
  }
  void update(double dist_to_path, double threshold, double goal_x, double goal_y)
  {
    if (std::isfinite(goal_x) && std::isfinite(goal_y)) {
      if (!std::isfinite(goal_x_) || std::hypot(goal_x - goal_x_, goal_y - goal_y_) > 0.5) {
        far_ = false;
      }
      goal_x_ = goal_x;
      goal_y_ = goal_y;
    }
    if (std::isfinite(dist_to_path)) {far_ = dist_to_path > threshold;}
  }
  bool far() const {return far_;}

private:
  bool far_{false};
  double goal_x_{std::numeric_limits<double>::quiet_NaN()};
  double goal_y_{std::numeric_limits<double>::quiet_NaN()};
};

}
