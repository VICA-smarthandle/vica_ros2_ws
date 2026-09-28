#pragma once
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "vica_vcc_controller/core/path_window.hpp"
#include "vica_vcc_controller/core/ultrasonic.hpp"
#include "vica_vcc_controller/core/vcc_core.hpp"

namespace vica_vcc_controller
{
class VccController : public nav2_core::Controller
{
public:
  VccController() = default;
  ~VccController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  core::Path windowPlan(const geometry_msgs::msg::PoseStamped & pose, core::Pose2D & goal_robot);
  bool transformPose(
    const std::string & frame, const geometry_msgs::msg::PoseStamped & in,
    geometry_msgs::msg::PoseStamped & out) const;
  void fillClearance(const geometry_msgs::msg::PoseStamped & pose);
  void fillUltrasonic(double now);
  double steadyNow() const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("VccController")};
  rclcpp::Clock::SharedPtr clock_;
  // Humble Clock::now() 는 const 가 아니다. steadyNow() 는 const 로 두고 여기만 mutable.
  mutable rclcpp::Clock steady_{RCL_STEADY_TIME};

  core::CoreParams params_;
  core::VccCore core_;
  core::ClearanceField field_;
  core::PathWindow path_window_;
  std::string plan_frame_;
  double transform_tolerance_{0.2};
  double base_speed_{0.5};
  double speed_cap_{0.5};
  double clearance_window_{5.0};
  double reset_gap_{0.5};
  double last_compute_{-1.0};
  double last_goal_x_{1e9}, last_goal_y_{1e9};
  bool publish_state_{true};

  // 초음파(구독 콜백 스레드와 제어 스레드가 공유)
  std::mutex us_mutex_;
  std::vector<core::UltrasonicChannel> us_channels_;
  std::vector<std::string> us_frames_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> us_subs_;
  double us_max_age_{1.0};
  int us_confirm_count_{2};
  double us_confirm_tol_{0.15};
  int us_arc_points_{7};

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> state_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> lane_pub_;
};
}  // namespace vica_vcc_controller
