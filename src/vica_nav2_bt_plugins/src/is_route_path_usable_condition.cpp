#include "vica_nav2_bt_plugins/is_route_path_usable_condition.hpp"

#include <algorithm>
#include <limits>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"
#include "vica_nav2_bt_plugins/route_path_check.hpp"

namespace vica_nav2_bt_plugins
{

IsRoutePathUsableCondition::IsRoutePathUsableCondition(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
  // nav2 의 GoalReached 조건 노드와 같은 자리에서 같은 것을 꺼낸다.
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
}

BT::NodeStatus IsRoutePathUsableCondition::tick()
{
  nav_msgs::msg::Path path;
  if (!getInput("path", path)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[IsRoutePathUsable] path 포트가 비었다. 레일 경로를 못 쓴다");
    return BT::NodeStatus::FAILURE;
  }

  int min_poses = 2;
  double max_dist = 1.5;
  double handoff = 2.0;
  geometry_msgs::msg::PoseStamped goal;
  const bool has_goal = static_cast<bool>(getInput("goal", goal));
  getInput("handoff_dist_to_goal", handoff);
  double tol = 0.2;
  std::string global_frame = "map";
  std::string base_frame = "base_footprint";
  getInput("min_poses", min_poses);
  getInput("max_dist_from_path", max_dist);
  getInput("transform_tolerance", tol);
  getInput("global_frame", global_frame);
  getInput("robot_base_frame", base_frame);

  geometry_msgs::msg::PoseStamped robot;
  if (!nav2_util::getCurrentPose(robot, *tf_, global_frame, base_frame, tol)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[IsRoutePathUsable] 로봇 위치(%s->%s)를 못 읽었다. 레일 경로를 못 쓴다",
      global_frame.c_str(), base_frame.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto v = checkRoutePath(
    path, robot.pose.position.x, robot.pose.position.y,
    static_cast<std::size_t>(std::max(min_poses, 1)), max_dist,
    has_goal ? goal.pose.position.x : nan, has_goal ? goal.pose.position.y : nan, handoff);

  if (!v.usable) {
    // 1 Hz 로 다시 물으므로 매 틱 찍히면 로그가 넘친다. 2초에 한 번만.
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[IsRoutePathUsable] 레일 경로를 못 쓴다 -> 자유주행: %s (점 %zu개, 경로까지 %.2f m, 목적지까지 %.2f m)",
      v.reason.c_str(), v.poses, v.dist_to_path, v.dist_to_goal);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace vica_nav2_bt_plugins

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<vica_nav2_bt_plugins::IsRoutePathUsableCondition>("IsRoutePathUsable");
}
