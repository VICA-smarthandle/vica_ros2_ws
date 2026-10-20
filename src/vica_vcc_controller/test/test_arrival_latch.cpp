#include <gtest/gtest.h>
#include <cmath>
#include "vica_vcc_controller/core/arrival_latch.hpp"

using namespace vica_vcc_controller::core;

namespace
{
const Pose2D kGoal{10.0, 5.0, M_PI / 2};
Pose2D at(double dx, double yaw) {return {kGoal.x + dx, kGoal.y, yaw};}
}

TEST(ArrivalLatch, NeedsXyThenYawThenStop)
{
  ArrivalLatch a;
  EXPECT_FALSE(a.check(at(0.20, kGoal.yaw), kGoal, 0.0, 0.0, 0.0));
  EXPECT_FALSE(a.latched());
  EXPECT_FALSE(a.check(at(0.10, 0.0), kGoal, 0.0, 0.0, 0.0));
  EXPECT_TRUE(a.latched());
  EXPECT_FALSE(a.check(at(0.10, kGoal.yaw), kGoal, 0.0, 0.0, 0.2));
  EXPECT_FALSE(a.check(at(0.10, kGoal.yaw), kGoal, 0.05, 0.0, 0.0));
  EXPECT_TRUE(a.check(at(0.10, kGoal.yaw), kGoal, 0.0, 0.0, 0.0));
}

TEST(ArrivalLatch, LatchSurvivesDriftInsideUnlatchDistance)
{
  ArrivalLatch a;
  a.check(at(0.10, 0.0), kGoal, 0.0, 0.0, 0.0);
  ASSERT_TRUE(a.latched());
  EXPECT_TRUE(a.check(at(0.30, kGoal.yaw), kGoal, 0.0, 0.0, 0.0));
  const Pose2D moved_end{kGoal.x + 0.14, kGoal.y, kGoal.yaw};
  EXPECT_TRUE(a.check(at(0.30, kGoal.yaw), moved_end, 0.0, 0.0, 0.0));
}

TEST(ArrivalLatch, UnlatchesWhenRobotLeaves)
{
  ArrivalLatch a;
  a.check(at(0.10, 0.0), kGoal, 0.0, 0.0, 0.0);
  ASSERT_TRUE(a.latched());
  EXPECT_FALSE(a.check(at(0.60, kGoal.yaw), kGoal, 0.0, 0.0, 0.0));
  EXPECT_FALSE(a.latched());
  EXPECT_FALSE(a.check(at(0.30, kGoal.yaw), kGoal, 0.0, 0.0, 0.0));
}

TEST(ArrivalLatch, UnlatchesWhenGoalMoves)
{
  ArrivalLatch a;
  a.check(at(0.10, kGoal.yaw), kGoal, 0.0, 0.0, 0.0);
  ASSERT_TRUE(a.latched());
  const Pose2D other{kGoal.x + 0.40, kGoal.y + 0.40, kGoal.yaw};
  EXPECT_FALSE(a.check(at(0.10, kGoal.yaw), other, 0.0, 0.0, 0.0));
  EXPECT_FALSE(a.latched());
}

TEST(ArrivalLatch, GoalStepsAreMeasuredFromTheLatchedEnd)
{
  ArrivalLatch a;
  const Pose2D robot{kGoal.x, kGoal.y, kGoal.yaw};
  a.check(robot, kGoal, 0.0, 0.0, 0.0);
  ASSERT_TRUE(a.latched());
  Pose2D g = kGoal;
  g.y += 0.2; a.check(robot, g, 0.0, 0.0, 0.0);
  EXPECT_TRUE(a.latched());
  g.y += 0.2; a.check(robot, g, 0.0, 0.0, 0.0);
  EXPECT_TRUE(a.latched());
  g.y += 0.2; a.check(robot, g, 0.0, 0.0, 0.0);
  EXPECT_FALSE(a.latched());
}

TEST(ArrivalLatch, ClearDropsTheLatch)
{
  ArrivalLatch a;
  a.check(at(0.10, 0.0), kGoal, 0.0, 0.0, 0.0);
  a.clear();
  EXPECT_FALSE(a.latched());
}
