#pragma once
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "vica_vcc_controller/core/arrival_latch.hpp"

namespace vica_vcc_controller
{
// controller_server 의 goal checker 플러그인. 계산은 core::ArrivalLatch 가 한다.
// getTolerances 는 StoppedGoalChecker 와 같은 칸에 같은 뜻으로 채운다 — VCC 가 매 주기 읽는다.
class LatchedGoalChecker : public nav2_core::GoalChecker
{
public:
  LatchedGoalChecker() = default;
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  // 새 경로마다 불린다(Humble setPlannerPath). 도장을 지우지 않는다 — 이 플러그인의 이유다.
  void reset() override {}
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance) override;

private:
  core::ArrivalLatch latch_;
  rclcpp::Logger logger_{rclcpp::get_logger("LatchedGoalChecker")};
};
}  // namespace vica_vcc_controller
