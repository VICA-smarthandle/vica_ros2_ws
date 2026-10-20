#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "vica_nav2_bt_plugins/route_path_check.hpp"

using vica_nav2_bt_plugins::checkRoutePath;

static nav_msgs::msg::Path line(double x0, double x1, double y, int n)
{
  nav_msgs::msg::Path p;
  for (int i = 0; i < n; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = x0 + (x1 - x0) * i / (n - 1);
    ps.pose.position.y = y;
    p.poses.push_back(ps);
  }
  return p;
}

TEST(RoutePathCheck, EmptyPathIsUnusable)
{
  nav_msgs::msg::Path p;
  auto v = checkRoutePath(p, 0, 0, 2, 1.5);
  EXPECT_FALSE(v.usable);
  EXPECT_EQ(v.poses, 0u);
}

TEST(RoutePathCheck, SinglePoseIsUnusableEvenIfFree)
{
  nav_msgs::msg::Path p;
  geometry_msgs::msg::PoseStamped ps;
  ps.pose.position.x = 3.55;
  ps.pose.position.y = 5.41;
  p.poses.push_back(ps);
  auto v = checkRoutePath(p, -1.408, 5.620, 2, 1.5);
  EXPECT_FALSE(v.usable);
  EXPECT_EQ(v.poses, 1u);
  EXPECT_NE(v.reason.find("points"), std::string::npos);
}

TEST(RoutePathCheck, TwoPosesNearRobotIsUsable)
{
  auto p = line(0.0, 1.0, 0.0, 2);
  auto v = checkRoutePath(p, 0.1, 0.2, 2, 1.5);
  EXPECT_TRUE(v.usable);
  EXPECT_NEAR(v.dist_to_path, 0.2, 1e-9);
  EXPECT_NEAR(v.dist_to_points, 0.2236, 1e-3);
}

TEST(RoutePathCheck, RobotFarFromPathIsUnusable)
{
  auto p = line(5.0, 8.0, 0.0, 61);
  auto v = checkRoutePath(p, 0.0, 0.0, 2, 1.5);
  EXPECT_FALSE(v.usable);
  EXPECT_NEAR(v.dist_to_path, 5.0 - vica_nav2_bt_plugins::kPrunedEdgeExtend, 1e-9);
  EXPECT_NEAR(v.dist_to_points, 5.0, 1e-9);
  EXPECT_NE(v.reason.find("far"), std::string::npos);
}

TEST(RoutePathCheck, PrunedStartNodeIsMeasuredSideways)
{
  auto p = line(0.8, 3.8, 0.0, 61);
  auto v = checkRoutePath(p, 0.0, -0.72, 2, 0.8);
  EXPECT_TRUE(v.usable);
  EXPECT_NEAR(v.dist_to_path, 0.72, 1e-9);
  EXPECT_NEAR(v.dist_to_points, std::hypot(0.8, 0.72), 1e-9);
}

TEST(RoutePathCheck, SidewaysFromTheMiddleOfASegment)
{
  auto p = line(0.0, 2.0, 0.0, 2);
  auto v = checkRoutePath(p, 1.0, 0.3, 2, 0.8);
  EXPECT_TRUE(v.usable);
  EXPECT_NEAR(v.dist_to_path, 0.3, 1e-9);
}

TEST(RoutePathCheck, TrulyFarSidewaysIsStillUnusable)
{
  auto p = line(0.8, 3.8, 0.0, 61);
  auto v = checkRoutePath(p, 1.5, 1.0, 2, 0.8);
  EXPECT_FALSE(v.usable);
  EXPECT_NEAR(v.dist_to_path, 1.0, 1e-9);
}

TEST(RoutePathCheck, FirstSegmentDirectionIgnoresTinyJitter)
{
  auto p = line(1.0, 4.0, 0.0, 61);
  p.poses[1].pose.position.y = 0.03;
  auto v = checkRoutePath(p, 0.0, -0.5, 2, 0.8);
  EXPECT_TRUE(v.usable);
  EXPECT_NEAR(v.dist_to_path, 0.5, 1e-9);
}

TEST(RoutePathCheck, SlightlyOffTheRailIsUsable)
{
  auto p = line(-3.0, 3.0, 0.0, 121);
  auto v = checkRoutePath(p, 0.0, 0.3, 2, 1.5);
  EXPECT_TRUE(v.usable);
  EXPECT_NEAR(v.dist_to_path, 0.3, 1e-9);
}

TEST(RoutePathCheck, MinPosesIsHonored)
{
  auto p = line(0.0, 1.0, 0.0, 3);
  EXPECT_TRUE(checkRoutePath(p, 0.0, 0.0, 3, 1.5).usable);
  EXPECT_FALSE(checkRoutePath(p, 0.0, 0.0, 4, 1.5).usable);
}

TEST(RoutePathCheck, NonFinitePoseIsUnusable)
{
  auto p = line(0.0, 3.0, 0.0, 61);
  p.poses[30].pose.position.x = std::numeric_limits<double>::quiet_NaN();
  auto v = checkRoutePath(p, 0.0, 0.1, 2, 1.5);
  EXPECT_FALSE(v.usable);
  EXPECT_NE(v.reason.find("non-finite"), std::string::npos);
}

TEST(RoutePathCheck, NearGoalHandsOffToFreespace)
{
  auto p = line(0.0, 3.0, 0.0, 61);
  auto v = checkRoutePath(p, 1.5, 0.1, 2, 1.5, 3.0, 0.0, 2.0);
  EXPECT_FALSE(v.usable);
  EXPECT_NEAR(v.dist_to_goal, 1.5033, 1e-3);
  EXPECT_NE(v.reason.find("near goal"), std::string::npos);
  EXPECT_TRUE(checkRoutePath(p, 0.0, 0.1, 2, 1.5, 3.0, 0.0, 2.0).usable);
  EXPECT_TRUE(checkRoutePath(p, 1.5, 0.1, 2, 1.5).usable);
  EXPECT_TRUE(checkRoutePath(p, 1.5, 0.1, 2, 1.5, 3.0, 0.0, 0.0).usable);
}

using vica_nav2_bt_plugins::RailDistanceGate;

TEST(RailDistanceGate, OneThresholdUntilLeavingThenRejoinThreshold)
{
  RailDistanceGate g;
  EXPECT_DOUBLE_EQ(g.threshold(0.8, 0.5), 0.8);
  g.update(0.75, g.threshold(0.8, 0.5), 10.0, 0.0);
  EXPECT_FALSE(g.far());
  g.update(0.85, g.threshold(0.8, 0.5), 10.0, 0.0);
  EXPECT_TRUE(g.far());
  EXPECT_DOUBLE_EQ(g.threshold(0.8, 0.5), 0.5);
  g.update(0.75, g.threshold(0.8, 0.5), 10.0, 0.0);
  EXPECT_TRUE(g.far());
  g.update(0.45, g.threshold(0.8, 0.5), 10.0, 0.0);
  EXPECT_FALSE(g.far());
  EXPECT_DOUBLE_EQ(g.threshold(0.8, 0.5), 0.8);
}

TEST(RailDistanceGate, DisabledWhenRejoinNotSetOrNotSmaller)
{
  RailDistanceGate g;
  g.update(1.0, 0.8, 10.0, 0.0);
  ASSERT_TRUE(g.far());
  EXPECT_DOUBLE_EQ(g.threshold(0.8, -1.0), 0.8);
  EXPECT_DOUBLE_EQ(g.threshold(0.8, 0.9), 0.8);
}

TEST(RailDistanceGate, NewGoalStartsFresh)
{
  RailDistanceGate g;
  g.update(1.0, 0.8, 10.0, 0.0);
  ASSERT_TRUE(g.far());
  g.update(std::numeric_limits<double>::infinity(), 0.5, 20.0, 5.0);
  EXPECT_FALSE(g.far());
}
