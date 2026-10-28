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
}  // namespace

// 끝점은 목적지 자리, 시작점은 로봇 자리, 점 간격은 spacing 이하.
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

// 핵심: 모든 점의 방향은 선 방향이다. 목적지에 저장된 yaw 는 입력에 없다 — 끝에서 휘지 않는다.
TEST(StraightPath, EveryPoseFacesAlongTheLine)
{
  const auto p = makeStraightPath(0.0, 0.0, 1.0, 1.0, 1.0, 0.05, "map");
  for (const auto & ps : p.poses) {
    EXPECT_NEAR(yawOf(ps), M_PI / 4.0, 1e-9);
  }
}

// 목적지가 로봇 바로 앞(0.03 m)이어도 점 2개(시작·끝). IsRailAheadClear 는 점 2개 미만을 막힘으로 본다.
TEST(StraightPath, ShortLineStillHasTwoPoses)
{
  const auto p = makeStraightPath(0.0, 0.0, 0.0, 0.03, 0.0, 0.05, "map");
  ASSERT_EQ(p.poses.size(), 2u);
  EXPECT_NEAR(p.poses.back().pose.position.x, 0.03, 1e-12);
}

// 같은 자리(1 mm 안)면 선 방향이 없으므로 로봇 방향을 쓴다.
TEST(StraightPath, SamePointKeepsRobotYaw)
{
  const auto p = makeStraightPath(0.0, 0.0, 0.7, 0.0005, 0.0, 0.05, "map");
  ASSERT_EQ(p.poses.size(), 2u);
  EXPECT_NEAR(yawOf(p.poses.back()), 0.7, 1e-9);
}

// 못 쓰는 입력은 빈 경로 — 노드는 FAILURE 를 내고 planner 갈래로 넘어간다.
TEST(StraightPath, BadInputGivesEmptyPath)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(makeStraightPath(nan, 0.0, 0.0, 1.0, 0.0, 0.05, "map").poses.empty());
  EXPECT_TRUE(makeStraightPath(0.0, 0.0, 0.0, 1.0, nan, 0.05, "map").poses.empty());
  EXPECT_TRUE(makeStraightPath(0.0, 0.0, nan, 1.0, 0.0, 0.05, "map").poses.empty());
  EXPECT_TRUE(makeStraightPath(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, "map").poses.empty());
  EXPECT_TRUE(makeStraightPath(0.0, 0.0, 0.0, 1.0, 0.0, -0.05, "map").poses.empty());
}
