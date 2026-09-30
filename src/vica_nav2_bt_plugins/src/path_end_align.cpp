#include "vica_nav2_bt_plugins/path_end_align.hpp"

#include <cmath>

namespace vica_nav2_bt_plugins
{

bool alignPathEndToGoal(
  nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & goal, double max_dist)
{
  if (path.poses.empty()) {return false;}
  const std::string & pf = path.header.frame_id;
  const std::string & gf = goal.header.frame_id;
  if (!pf.empty() && !gf.empty() && pf != gf) {return false;}
  auto & end = path.poses.back().pose;
  const double d = std::hypot(
    end.position.x - goal.pose.position.x, end.position.y - goal.pose.position.y);
  if (!(d <= max_dist)) {return false;}   // NaN 도 거른다
  const auto & q = goal.pose.orientation;
  if (end.orientation.x == q.x && end.orientation.y == q.y && end.orientation.z == q.z &&
    end.orientation.w == q.w)
  {
    return false;
  }
  end.orientation = q;
  return true;
}

}  // namespace vica_nav2_bt_plugins
