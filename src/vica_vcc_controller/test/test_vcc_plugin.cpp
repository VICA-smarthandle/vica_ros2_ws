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

TEST_F(VccPluginTest, ResetGapResetsOnlyAfterLongPause)
{
  // Review Focus 3 / 최종 리뷰 I3: 호출이 reset_gap(1.5 s) 넘게 끊기면 새 실행으로 보고 초기화한다.
  // 초기화는 실측 속도에서 이어 가므로, 실측을 0.5 로 주면 초기화 여부가 명령에 드러난다.
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("vcc_test_node4");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("vcc_test_costmap4");
  costmap->on_configure(rclcpp_lifecycle::State());
  auto ctrl = std::make_shared<vica_vcc_controller::VccController>();
  double t = 100.0;
  ctrl->setTimeSourceForTest([&t]() {return t;});
  ctrl->configure(node, "FollowPath", tf, costmap);
  ctrl->activate();
  double gap = 0.0;
  ASSERT_TRUE(node->get_parameter("FollowPath.reset_gap", gap));
  EXPECT_NEAR(gap, 1.5, 1e-9);

  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  for (int i = 0; i <= 60; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = 1.0 + 0.05 * i;
    ps.pose.position.y = 2.5;
    ps.pose.orientation.w = 1.0;
    plan.poses.push_back(ps);
  }
  ctrl->setPlan(plan);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 2.5;
  pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist still, cruise;
  cruise.linear.x = 0.5;

  ctrl->computeVelocityCommands(pose, still, nullptr);   // 0.05
  t += 0.1;
  const double v1 = ctrl->computeVelocityCommands(pose, still, nullptr).twist.linear.x;   // 0.10
  t += 0.1;   // 0.1 s 간격: 초기화 없음 -> 직전 명령에서 이어 간다
  const double v2 = ctrl->computeVelocityCommands(pose, cruise, nullptr).twist.linear.x;
  EXPECT_NEAR(v2, v1 + 0.05, 1e-6);
  t += 1.0;   // 1.0 s 간격(BT Wait 1 s): 아직 초기화하지 않는다
  const double v3 = ctrl->computeVelocityCommands(pose, cruise, nullptr).twist.linear.x;
  EXPECT_LT(v3, 0.3);
  t += 2.0;   // 2 s 간격: 초기화 -> 실측 0.5 에서 이어 간다
  const double v4 = ctrl->computeVelocityCommands(pose, cruise, nullptr).twist.linear.x;
  EXPECT_GE(v4, 0.45);
}
