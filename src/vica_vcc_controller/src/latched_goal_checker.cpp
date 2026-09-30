#include "vica_vcc_controller/latched_goal_checker.hpp"

#include <limits>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

using nav2_util::declare_parameter_if_not_declared;

namespace vica_vcc_controller
{
namespace
{
core::Pose2D toPose2D(const geometry_msgs::msg::Pose & p)
{
  return {p.position.x, p.position.y, tf2::getYaw(p.orientation)};
}
}  // namespace

void LatchedGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap_ros*/)
{
  auto node = parent.lock();
  if (!node) {throw std::runtime_error("LatchedGoalChecker: unable to lock node");}
  logger_ = node->get_logger();
  auto dp = [&](const std::string & n, double def) {
      declare_parameter_if_not_declared(node, plugin_name + "." + n, rclcpp::ParameterValue(def));
      double v = def;
      node->get_parameter(plugin_name + "." + n, v);
      return v;
    };
  core::ArrivalParams p;
  p.xy_tol = dp("xy_goal_tolerance", 0.15);
  p.yaw_tol = dp("yaw_goal_tolerance", 0.25);
  p.trans_stopped = dp("trans_stopped_velocity", 0.03);
  p.rot_stopped = dp("rot_stopped_velocity", 0.05);
  p.unlatch_dist = dp("unlatch_distance", 0.5);
  latch_ = core::ArrivalLatch(p);
  RCLCPP_INFO(logger_, "LatchedGoalChecker: xy %.2f yaw %.2f unlatch %.2f",
    p.xy_tol, p.yaw_tol, p.unlatch_dist);
}

bool LatchedGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  const bool was = latch_.latched();
  const bool ok = latch_.check(
    toPose2D(query_pose), toPose2D(goal_pose), velocity.linear.x, velocity.linear.y,
    velocity.angular.z);
  // 실주행 뒤 bag(/rosout)에서 도장 시점을 셀 수 있게 바뀔 때만 한 줄 남긴다.
  if (latch_.latched() != was) {
    RCLCPP_INFO(logger_, "LatchedGoalChecker: %s", latch_.latched() ? "xy latched" : "xy unlatched");
  }
  return ok;
}

bool LatchedGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance)
{
  const double invalid = std::numeric_limits<double>::lowest();
  const auto & p = latch_.params();
  pose_tolerance.position.x = p.xy_tol;
  pose_tolerance.position.y = p.xy_tol;
  pose_tolerance.position.z = invalid;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, p.yaw_tol);
  pose_tolerance.orientation.x = q.x();
  pose_tolerance.orientation.y = q.y();
  pose_tolerance.orientation.z = q.z();
  pose_tolerance.orientation.w = q.w();
  vel_tolerance.linear.x = p.trans_stopped;
  vel_tolerance.linear.y = p.trans_stopped;
  vel_tolerance.linear.z = invalid;
  vel_tolerance.angular.x = invalid;
  vel_tolerance.angular.y = invalid;
  vel_tolerance.angular.z = p.rot_stopped;
  return true;
}
}  // namespace vica_vcc_controller

PLUGINLIB_EXPORT_CLASS(vica_vcc_controller::LatchedGoalChecker, nav2_core::GoalChecker)
