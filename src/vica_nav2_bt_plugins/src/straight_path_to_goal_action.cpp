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
}  // namespace

StraightPathToGoalAction::StraightPathToGoalAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  // IsRoutePathUsable 과 같은 자리에서 같은 것을 꺼낸다.
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
}

BT::NodeStatus StraightPathToGoalAction::tick()
{
  // 실패할 때는 출력 포트를 건드리지 않는다 — {path} 를 비우면 FollowPath 가 빈 경로를 받는다(레일 7판 교훈).
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

  // goal 을 frame 변환하지 않는다. 다른 frame 이면 직선을 포기하고 planner 에 맡긴다(planner 는 변환한다).
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
  // 레일 거름망(checkRoutePath)과 같은 식·같은 부등호(목적지까지 < handoff)다. NaN 도 여기서 거른다.
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
  // 실주행 뒤 bag(/rosout)에서 직선이 쓰였는지 셀 수 있게 남긴다. 1 Hz·두 자리라 2초에 한 번.
  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "[StraightPathToGoal] 목적지 %.2f m — 곧은 경로(점 %zu개)", d, path.poses.size());
  setOutput("path", path);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace vica_nav2_bt_plugins

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<vica_nav2_bt_plugins::StraightPathToGoalAction>("StraightPathToGoal");
}
