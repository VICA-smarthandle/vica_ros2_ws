#include "vica_nav2_bt_plugins/straight_path_to_goal_action.hpp"

#include <cmath>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_util/robot_utils.hpp"
#include "vica_nav2_bt_plugins/straight_path.hpp"

namespace vica_nav2_bt_plugins
{

namespace
{
double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}

StraightPathToGoalAction::StraightPathToGoalAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
}

BT::NodeStatus StraightPathToGoalAction::tick()
{
  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal)) {
    return BT::NodeStatus::FAILURE;
  }
  double max_dist = 2.0;
  double spacing = 0.05;
  double tol = 0.2;
  std::string global_frame = "map";
  std::string base_frame = "base_footprint";
  getInput("max_dist", max_dist);
  getInput("spacing", spacing);
  getInput("transform_tolerance", tol);
  getInput("global_frame", global_frame);
  getInput("robot_base_frame", base_frame);

  if (!goal.header.frame_id.empty() && goal.header.frame_id != global_frame) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "[StraightPathToGoal] 목적지 frame '%s' 이 '%s' 가 아니다 -> 직선 대신 planner",
      goal.header.frame_id.c_str(), global_frame.c_str());
    return BT::NodeStatus::FAILURE;
  }
  geometry_msgs::msg::PoseStamped robot;
  if (!nav2_util::getCurrentPose(robot, *tf_, global_frame, base_frame, tol)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[StraightPathToGoal] 로봇 위치(%s->%s)를 못 읽었다 -> 직선 대신 planner",
      global_frame.c_str(), base_frame.c_str());
    return BT::NodeStatus::FAILURE;
  }
  const double rx = robot.pose.position.x, ry = robot.pose.position.y;
  const double gx = goal.pose.position.x, gy = goal.pose.position.y;
  const double d = std::hypot(gx - rx, gy - ry);
  if (!(d < max_dist)) {
    return BT::NodeStatus::FAILURE;
  }
  nav_msgs::msg::Path path = makeStraightPath(
    rx, ry, yawOf(robot.pose.orientation), gx, gy, spacing, global_frame);
  if (path.poses.size() < 2) {
    return BT::NodeStatus::FAILURE;
  }
  path.header.stamp = node_->now();
  for (auto & ps : path.poses) {ps.header.stamp = path.header.stamp;}
  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "[StraightPathToGoal] 목적지 %.2f m — 곧은 경로(점 %zu개)", d, path.poses.size());
  setOutput("path", path);
  return BT::NodeStatus::SUCCESS;
}

}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<vica_nav2_bt_plugins::StraightPathToGoalAction>("StraightPathToGoal");
}
