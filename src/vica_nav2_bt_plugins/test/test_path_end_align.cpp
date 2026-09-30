#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "vica_nav2_bt_plugins/path_end_align.hpp"

using vica_nav2_bt_plugins::alignPathEndToGoal;

namespace
{
geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw, const char * frame = "map")
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}
nav_msgs::msg::Path path(double end_x, double end_yaw)
{
  nav_msgs::msg::Path p;
  p.header.frame_id = "map";
  p.poses.push_back(pose(0.0, 0.0, 0.0));
  p.poses.push_back(pose(end_x, 0.0, end_yaw));
  return p;
}
}  // namespace

// run49 409호 재현: 끝점이 목적지 0.14 m 옆, 방향 −90°.
TEST(PathEndAlign, NearEndTakesGoalYawButKeepsPosition)
{
  auto p = path(1.14, -M_PI / 2);
  const auto goal = pose(1.0, 0.0, 0.0);
  EXPECT_TRUE(alignPathEndToGoal(p, goal, 0.5));
  EXPECT_DOUBLE_EQ(p.poses.back().pose.position.x, 1.14);
  EXPECT_DOUBLE_EQ(p.poses.back().pose.orientation.z, goal.pose.orientation.z);
  EXPECT_DOUBLE_EQ(p.poses.back().pose.orientation.w, goal.pose.orientation.w);
  EXPECT_DOUBLE_EQ(p.poses.front().pose.orientation.w, 1.0);   // 다른 점은 그대로
}

TEST(PathEndAlign, AlreadyAlignedIsUnchanged)
{
  auto p = path(1.0, 0.3);
  EXPECT_FALSE(alignPathEndToGoal(p, pose(1.0, 0.0, 0.3), 0.5));
}

TEST(PathEndAlign, FarEndIsLeftAlone)
{
  // 당근·중간 경로는 끝점이 목적지가 아니다.
  auto p = path(3.0, 1.0);
  EXPECT_FALSE(alignPathEndToGoal(p, pose(1.0, 0.0, 0.0), 0.5));
  EXPECT_NEAR(p.poses.back().pose.orientation.z, std::sin(0.5), 1e-12);
}

TEST(PathEndAlign, EmptyPathOrOtherFrameIsLeftAlone)
{
  nav_msgs::msg::Path empty;
  EXPECT_FALSE(alignPathEndToGoal(empty, pose(0.0, 0.0, 0.0), 0.5));
  auto p = path(1.0, 1.0);
  EXPECT_FALSE(alignPathEndToGoal(p, pose(1.0, 0.0, 0.0, "odom"), 0.5));
}

TEST(PathEndAlign, NanGoalIsLeftAlone)
{
  auto p = path(1.0, 1.0);
  EXPECT_FALSE(alignPathEndToGoal(
    p, pose(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0), 0.5));
}

TEST(PathEndAlign, SubDegreeDifferenceIsLeftAlone)
{
  // run50: planner 사원수의 끝자리 차이만으로 "고쳤다" 로그가 화장실에서 3~4번씩 나왔다.
  auto p = path(1.0, -M_PI / 2 + 0.5 * M_PI / 180.0);   // 0.5° 차이
  EXPECT_FALSE(alignPathEndToGoal(p, pose(1.0, 0.0, -M_PI / 2), 0.5));
  auto q = path(1.0, -M_PI / 2 + 2.0 * M_PI / 180.0);   // 2° 차이
  EXPECT_TRUE(alignPathEndToGoal(q, pose(1.0, 0.0, -M_PI / 2), 0.5));
}
