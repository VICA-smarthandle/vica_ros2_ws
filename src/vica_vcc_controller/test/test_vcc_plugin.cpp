#include <gtest/gtest.h>
#include <memory>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
// NO_SPEED_LIMIT 는 costmap_2d_ros.hpp 가 전이 include 하지 않는다(Humble 실측) — 직접 포함한다.
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_core/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "vica_vcc_controller/vcc_controller.hpp"

class VccPluginTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(VccPluginTest, ConfigureDeclaresParametersAndLifecycleWorks)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("vcc_test_node");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("vcc_test_costmap");
  costmap->on_configure(rclcpp_lifecycle::State());

  auto ctrl = std::make_shared<vica_vcc_controller::VccController>();
  ctrl->configure(node, "FollowPath", tf, costmap);
  ctrl->activate();

  double lane_rate = 0.0;
  ASSERT_TRUE(node->get_parameter("FollowPath.lane_rate", lane_rate));
  EXPECT_NEAR(lane_rate, 0.10, 1e-9);
  std::vector<double> radii;
  ASSERT_TRUE(node->get_parameter("FollowPath.turn_radii", radii));
  EXPECT_EQ(radii.size(), 2u);

  ctrl->setSpeedLimit(50.0, true);
  ctrl->setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);
  ctrl->deactivate();
  ctrl->cleanup();
}

TEST_F(VccPluginTest, EmptyPlanThrowsPlannerException)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("vcc_test_node2");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("vcc_test_costmap2");
  costmap->on_configure(rclcpp_lifecycle::State());
  auto ctrl = std::make_shared<vica_vcc_controller::VccController>();
  ctrl->configure(node, "FollowPath", tf, costmap);
  ctrl->activate();

  ctrl->setPlan(nav_msgs::msg::Path());
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  EXPECT_THROW(
    ctrl->computeVelocityCommands(pose, geometry_msgs::msg::Twist(), nullptr),
    nav2_core::PlannerException);
}

TEST_F(VccPluginTest, StraightPlanProducesForwardCommand)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("vcc_test_node3");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("vcc_test_costmap3");
  costmap->on_configure(rclcpp_lifecycle::State());

  // map -> base_link : (2.5, 2.5) 에 놓는다(기본 costmap 원점 0,0)
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  t.transform.translation.x = 2.5;
  t.transform.translation.y = 2.5;
  t.transform.rotation.w = 1.0;
  tf->setTransform(t, "test", true);

  auto ctrl = std::make_shared<vica_vcc_controller::VccController>();
  ctrl->configure(node, "FollowPath", tf, costmap);
  ctrl->activate();

  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  for (int i = 0; i <= 40; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = 2.5 + 0.05 * i;
    ps.pose.position.y = 2.5;
    ps.pose.orientation.w = 1.0;
    plan.poses.push_back(ps);
  }
  ctrl->setPlan(plan);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 2.5;
  pose.pose.position.y = 2.5;
  pose.pose.orientation.w = 1.0;
  const auto cmd = ctrl->computeVelocityCommands(pose, geometry_msgs::msg::Twist(), nullptr);
  EXPECT_GT(cmd.twist.linear.x, 0.0);
  EXPECT_LE(cmd.twist.linear.x, 0.25);
  EXPECT_NEAR(cmd.twist.angular.z, 0.0, 1e-3);
}
