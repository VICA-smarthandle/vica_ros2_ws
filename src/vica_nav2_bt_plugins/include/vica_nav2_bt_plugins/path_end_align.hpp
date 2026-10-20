#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace vica_nav2_bt_plugins
{

bool alignPathEndToGoal(
  nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & goal, double max_dist);

}
