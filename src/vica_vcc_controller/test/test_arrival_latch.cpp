#include <gtest/gtest.h>
#include <cmath>
#include "vica_vcc_controller/core/arrival_latch.hpp"

using namespace vica_vcc_controller::core;

namespace
{
const Pose2D kGoal{10.0, 5.0, M_PI / 2};
Pose2D at(double dx, double yaw) {return {kGoal.x + dx, kGoal.y, yaw};}
}  // namespace

TEST(ArrivalLatch, NeedsXyThenYawThenStop)
{
  ArrivalLatch a;
  EXPECT_FALSE(a.check(at(0.20, kGoal.yaw), kGoal, 0.0, 0.0, 0.0));   // 0.15 밖
  EXPECT_FALSE(a.latched());
  EXPECT_FALSE(a.check(at(0.10, 0.0), kGoal, 0.0, 0.0, 0.0));          // 안에 들었지만 방향 90° 어긋남
  EXPECT_TRUE(a.latched());
  EXPECT_FALSE(a.check(at(0.10, kGoal.yaw), kGoal, 0.0, 0.0, 0.2));    // 아직 돈다
  EXPECT_FALSE(a.check(at(0.10, kGoal.yaw), kGoal, 0.05, 0.0, 0.0));   // 아직 간다
  EXPECT_TRUE(a.check(at(0.10, kGoal.yaw), kGoal, 0.0, 0.0, 0.0));
}

TEST(ArrivalLatch, LatchSurvivesDriftInsideUnlatchDistance)
{
  // run49 교착: 정렬하는 동안 AMCL·끝점이 흔들려 0.25~0.35 띠로 밀려도 도착을 못 했다.
  ArrivalLatch a;
  a.check(at(0.10, 0.0), kGoal, 0.0, 0.0, 0.0);
  ASSERT_TRUE(a.latched());
  EXPECT_TRUE(a.check(at(0.30, kGoal.yaw), kGoal, 0.0, 0.0, 0.0));
  const Pose2D moved_end{kGoal.x + 0.14, kGoal.y, kGoal.yaw};          // 새 경로 끝점이 0.14 m 옮겨짐
  EXPECT_TRUE(a.check(at(0.30, kGoal.yaw), moved_end, 0.0, 0.0, 0.0));
}

TEST(ArrivalLatch, UnlatchesWhenRobotLeaves)
{
  ArrivalLatch a;
  a.check(at(0.10, 0.0), kGoal, 0.0, 0.0, 0.0);
  ASSERT_TRUE(a.latched());
  EXPECT_FALSE(a.check(at(0.60, kGoal.yaw), kGoal, 0.0, 0.0, 0.0));
  EXPECT_FALSE(a.latched());
  EXPECT_FALSE(a.check(at(0.30, kGoal.yaw), kGoal, 0.0, 0.0, 0.0));   // 돌아와도 0.15 안이어야 다시 찍는다
}

TEST(ArrivalLatch, UnlatchesWhenGoalMoves)
{
  // 다른 목적지(끝점이 0.5 m 넘게 옮겨짐)는 도장을 물려받지 않는다. 레일 당근도 같다.
  ArrivalLatch a;
  a.check(at(0.10, kGoal.yaw), kGoal, 0.0, 0.0, 0.0);
  ASSERT_TRUE(a.latched());
  const Pose2D other{kGoal.x + 0.40, kGoal.y + 0.40, kGoal.yaw};
  EXPECT_FALSE(a.check(at(0.10, kGoal.yaw), other, 0.0, 0.0, 0.0));
  EXPECT_FALSE(a.latched());
}

TEST(ArrivalLatch, GoalStepsAreMeasuredFromTheLatchedEnd)
{
  // 기준은 도장 찍을 때의 끝점이다. 0.2 m 씩 옮기면 두 번(0.4)까지는 유지, 세 번째(0.6)에 푼다.
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
