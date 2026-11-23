#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "vica_nav2_bt_plugins/straight_path.hpp"

using vica_nav2_bt_plugins::makeStraightPath;

namespace
{
double yawOf(const geometry_msgs::msg::PoseStamped & p)
{
  const auto & q = p.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}

TEST(StraightPath, RunsFromRobotToGoalAtSpacing)
{
  const auto p = makeStraightPath(1.0, 2.0, 0.0, 2.2, 3.6, 0.05, "map");
  ASSERT_GE(p.poses.size(), 2u);
  EXPECT_EQ(p.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(p.poses.front().pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(p.poses.front().pose.position.y, 2.0);
  EXPECT_NEAR(p.poses.back().pose.position.x, 2.2, 1e-12);
  EXPECT_NEAR(p.poses.back().pose.position.y, 3.6, 1e-12);
  for (std::size_t i = 1; i < p.poses.size(); ++i) {
    const double step = std::hypot(
      p.poses[i].pose.position.x - p.poses[i - 1].pose.position.x,
      p.poses[i].pose.position.y - p.poses[i - 1].pose.position.y);
    EXPECT_LE(step, 0.05 + 1e-9);
    EXPECT_GT(step, 0.0);
  }
}

TEST(StraightPath, EveryPoseFacesAlongTheLine)
{
  const auto p = makeStraightPath(0.0, 0.0, 1.0, 1.0, 1.0, 0.05, "map");
  for (const auto & ps : p.poses) {
    EXPECT_NEAR(yawOf(ps), M_PI / 4.0, 1e-9);
  }
}

TEST(StraightPath, ShortLineStillHasTwoPoses)
{
  const auto p = makeStraightPath(0.0, 0.0, 0.0, 0.03, 0.0, 0.05, "map");
  ASSERT_EQ(p.poses.size(), 2u);
  EXPECT_NEAR(p.poses.back().pose.position.x, 0.03, 1e-12);
}

TEST(StraightPath, SamePointKeepsRobotYaw)
{
  const auto p = makeStraightPath(0.0, 0.0, 0.7, 0.0005, 0.0, 0.05, "map");
  ASSERT_EQ(p.poses.size(), 2u);
  EXPECT_NEAR(yawOf(p.poses.back()), 0.7, 1e-9);
}

TEST(StraightPath, BadInputGivesEmptyPath)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(makeStraightPath(nan, 0.0, 0.0, 1.0, 0.0, 0.05, "map").poses.empty());
  EXPECT_TRUE(makeStraightPath(0.0, 0.0, 0.0, 1.0, nan, 0.05, "map").poses.empty());
  EXPECT_TRUE(makeStraightPath(0.0, 0.0, nan, 1.0, 0.0, 0.05, "map").poses.empty());
  EXPECT_TRUE(makeStraightPath(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, "map").poses.empty());
  EXPECT_TRUE(makeStraightPath(0.0, 0.0, 0.0, 1.0, 0.0, -0.05, "map").poses.empty());
}
