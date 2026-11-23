#pragma once

#include <string>

#include "nav_msgs/msg/path.hpp"

namespace vica_nav2_bt_plugins
{

nav_msgs::msg::Path makeStraightPath(
  double rx, double ry, double robot_yaw, double gx, double gy, double spacing,
  const std::string & frame);

}
