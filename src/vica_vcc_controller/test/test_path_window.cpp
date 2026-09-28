#include <gtest/gtest.h>
#include <cmath>
#include "vica_vcc_controller/core/path_window.hpp"

using namespace vica_vcc_controller::core;

namespace
{
Path straight(double length)
{
  Path p;
  for (double x = 0.0; x <= length + 1e-9; x += 0.05) {p.push_back({x, 0.0, 0.0});}
  return p;
}

// 1.0 m 나갔다가 옆으로 2 cm 옮겨 되돌아오는 경로(유턴). 되돌아오는 쪽이 로봇에 약간 더 가깝다.
Path uturn()
{
  Path p;
  for (double x = 0.0; x <= 1.0 + 1e-9; x += 0.05) {p.push_back({x, 0.0, 0.0});}
  for (double x = 1.0; x >= -1e-9; x -= 0.05) {p.push_back({x, 0.02, M_PI});}
  return p;
}
}  // namespace

TEST(PathWindow, EmptyPlanIsReported)
{
  PathWindow w;
  Path out;
  EXPECT_EQ(w.window({0, 0, 0}, 3.0, out), WindowStatus::EmptyPlan);
}

TEST(PathWindow, UnitPathIsAcceptedUnlessRejected)
{
  // ③ 기본 꺼짐: 지금 동작(1점 경로를 받는다)을 바꾸지 않는다.
  PathWindow on_default;
  on_default.setPlan({{0.1, 0.0, 0.3}});
  Path out;
  EXPECT_EQ(on_default.window({0, 0, 0}, 3.0, out), WindowStatus::Ok);
  EXPECT_EQ(out.size(), 1u);

  PathWindowParams p;
  p.reject_unit_path = true;
  PathWindow strict(p);
  strict.setPlan({{0.1, 0.0, 0.3}});
  EXPECT_EQ(strict.window({0, 0, 0}, 3.0, out), WindowStatus::UnitPath);
}

TEST(PathWindow, StartsAtClosestPointAndPrunesThePast)
{
  PathWindow w;
  w.setPlan(straight(3.0));
  Path out;
  ASSERT_EQ(w.window({1.0, 0.03, 0.0}, 3.0, out), WindowStatus::Ok);
  EXPECT_NEAR(out.front().x, 1.0, 1e-9);
  EXPECT_NEAR(w.plan().front().x, 1.0, 1e-9);   // 지나온 1.0 m 는 지워졌다
  // 다음 주기에 로봇이 조금 뒤로 밀려도 지운 길로 돌아가지 않는다
  ASSERT_EQ(w.window({0.8, 0.0, 0.0}, 3.0, out), WindowStatus::Ok);
  EXPECT_NEAR(out.front().x, 1.0, 1e-9);
}

TEST(PathWindow, KeepsTwoPointsAtTheEnd)
{
  // ① 경로 끝을 지나쳐도 끝 방향을 계산할 2점이 남는다.
  PathWindow w;
  w.setPlan(straight(0.5));
  Path out;
  ASSERT_EQ(w.window({0.9, 0.0, 0.0}, 3.0, out), WindowStatus::Ok);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_NEAR(out[0].x, 0.45, 1e-9);
  EXPECT_NEAR(out[1].x, 0.5, 1e-9);
}

TEST(PathWindow, ClipsToCostmapExtent)
{
  PathWindow w;
  w.setPlan(straight(10.0));
  Path out;
  ASSERT_EQ(w.window({0.0, 0.0, 0.0}, 3.0, out), WindowStatus::Ok);
  EXPECT_LE(out.back().x, 3.0 + 1e-9);
  EXPECT_GE(out.back().x, 2.95 - 1e-9);
}

TEST(PathWindow, DefaultSearchCanJumpToTheReturnLegOfATightUturn)
{
  // ② 가 왜 설정으로 빠져 있는지를 고정한다. 기본 3.0 m 는 되돌아오는 구간(경로 거리 1.7 m)까지 찾는다.
  PathWindow w;
  w.setPlan(uturn());
  Path out;
  ASSERT_EQ(w.window({0.3, 0.015, 0.0}, 3.0, out), WindowStatus::Ok);
  EXPECT_NEAR(out.front().yaw, M_PI, 1e-9);     // 유턴을 건너뛰었다
}

TEST(PathWindow, ShorterSearchDistanceKeepsTheOutboundLeg)
{
  PathWindowParams p;
  p.max_robot_pose_search_dist = 0.5;
  PathWindow w(p);
  w.setPlan(uturn());
  Path out;
  ASSERT_EQ(w.window({0.3, 0.015, 0.0}, 3.0, out), WindowStatus::Ok);
  EXPECT_NEAR(out.front().yaw, 0.0, 1e-9);
  EXPECT_NEAR(out.front().x, 0.3, 1e-9);
}

TEST(PathWindow, RobotFrameConversion)
{
  const Pose2D robot{1.0, 1.0, M_PI / 2};
  const Pose2D q = toRobotFrame(robot, {1.0, 2.0, M_PI / 2});
  EXPECT_NEAR(q.x, 1.0, 1e-9);
  EXPECT_NEAR(q.y, 0.0, 1e-9);
  EXPECT_NEAR(q.yaw, 0.0, 1e-9);
}
