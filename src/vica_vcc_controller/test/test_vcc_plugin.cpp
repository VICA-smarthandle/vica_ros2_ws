#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <unistd.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_core/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include "vica_vcc_controller/latched_goal_checker.hpp"
#include "vica_vcc_controller/vcc_controller.hpp"

class VccPluginTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::InitOptions opts;
    opts.set_domain_id(30 + static_cast<size_t>(::getpid()) % 70);
    rclcpp::init(0, nullptr, opts);
    anchor_ = std::make_shared<rclcpp::Node>("vcc_test_anchor");
  }
  static void TearDownTestSuite()
  {
    anchor_.reset();
    rclcpp::shutdown();
  }
  static inline rclcpp::Node::SharedPtr anchor_;

  void make(const std::string & suffix)
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("vcc_test_node" + suffix);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("vcc_test_costmap" + suffix);
    costmap_->on_configure(rclcpp_lifecycle::State());
    ctrl_ = std::make_shared<vica_vcc_controller::VccController>();
  }

  void TearDown() override
  {
    if (costmap_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      costmap_->on_cleanup(rclcpp_lifecycle::State());
      costmap_->on_shutdown(rclcpp_lifecycle::State());
    }
    ctrl_.reset();
    costmap_.reset();
    tf_.reset();
    node_.reset();
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<vica_vcc_controller::VccController> ctrl_;
  double t_{100.0};
};

TEST_F(VccPluginTest, ConfigureDeclaresParametersAndLifecycleWorks)
{
  make("");
  auto & node = node_;
  auto & tf = tf_;
  auto & costmap = costmap_;
  auto & ctrl = ctrl_;
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
  make("2");
  auto & node = node_;
  auto & tf = tf_;
  auto & costmap = costmap_;
  auto & ctrl = ctrl_;
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
  make("3");
  auto & node = node_;
  auto & tf = tf_;
  auto & costmap = costmap_;

  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  t.transform.translation.x = 2.5;
  t.transform.translation.y = 2.5;
  t.transform.rotation.w = 1.0;
  tf->setTransform(t, "test", true);

  auto & ctrl = ctrl_;
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
  make("4");
  double & t = t_;
  auto & node = node_;
  auto & tf = tf_;
  auto & costmap = costmap_;
  auto & ctrl = ctrl_;
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

  ctrl->computeVelocityCommands(pose, still, nullptr);
  t += 0.1;
  const double v1 = ctrl->computeVelocityCommands(pose, still, nullptr).twist.linear.x;
  t += 0.1;
  const double v2 = ctrl->computeVelocityCommands(pose, cruise, nullptr).twist.linear.x;
  EXPECT_NEAR(v2, v1 + 0.05, 1e-6);
  t += 1.0;
  const double v3 = ctrl->computeVelocityCommands(pose, cruise, nullptr).twist.linear.x;
  EXPECT_LT(v3, 0.3);
  t += 2.0;
  const double v4 = ctrl->computeVelocityCommands(pose, cruise, nullptr).twist.linear.x;
  EXPECT_GE(v4, 0.45);
  ctrl->deactivate();
  ctrl->cleanup();
}

TEST_F(VccPluginTest, DeactivateClearsPlan)
{
  make("5");
  auto & node = node_;
  auto & tf = tf_;
  auto & costmap = costmap_;
  auto & ctrl = ctrl_;
  ctrl->configure(node, "FollowPath", tf, costmap);
  ctrl->activate();
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  for (int i = 0; i <= 20; ++i) {
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
  EXPECT_NO_THROW(ctrl->computeVelocityCommands(pose, geometry_msgs::msg::Twist(), nullptr));
  ctrl->deactivate();
  ctrl->activate();
  EXPECT_THROW(
    ctrl->computeVelocityCommands(pose, geometry_msgs::msg::Twist(), nullptr),
    nav2_core::PlannerException);
}

TEST_F(VccPluginTest, LatchedGoalCheckerReadsParamsAndKeepsLatchAcrossReset)
{
  make("_gc");
  node_->declare_parameter("general_goal_checker.xy_goal_tolerance", 0.15);
  node_->declare_parameter("general_goal_checker.unlatch_distance", 0.5);
  vica_vcc_controller::LatchedGoalChecker gc;
  gc.initialize(node_, "general_goal_checker", costmap_);

  geometry_msgs::msg::Pose pt;
  geometry_msgs::msg::Twist vt;
  ASSERT_TRUE(gc.getTolerances(pt, vt));
  EXPECT_DOUBLE_EQ(pt.position.x, 0.15);
  EXPECT_NEAR(tf2::getYaw(pt.orientation), 0.25, 1e-9);
  EXPECT_DOUBLE_EQ(vt.linear.x, 0.03);
  EXPECT_DOUBLE_EQ(vt.angular.z, 0.05);

  geometry_msgs::msg::Pose goal, robot;
  goal.orientation.w = 1.0;
  robot = goal;
  robot.position.x = 0.10;
  robot.orientation.z = std::sin(0.3);
  robot.orientation.w = std::cos(0.3);
  geometry_msgs::msg::Twist still;
  EXPECT_FALSE(gc.isGoalReached(robot, goal, still));
  gc.reset();
  robot.position.x = 0.30;
  robot.orientation = goal.orientation;
  EXPECT_TRUE(gc.isGoalReached(robot, goal, still));
}
