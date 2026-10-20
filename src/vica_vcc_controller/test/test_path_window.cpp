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

Path uturn()
{
  Path p;
  for (double x = 0.0; x <= 1.0 + 1e-9; x += 0.05) {p.push_back({x, 0.0, 0.0});}
  for (double x = 1.0; x >= -1e-9; x -= 0.05) {p.push_back({x, 0.02, M_PI});}
  return p;
}
}

TEST(PathWindow, EmptyPlanIsReported)
{
  PathWindow w;
  Path out;
  EXPECT_EQ(w.window({0, 0, 0}, 3.0, out), WindowStatus::EmptyPlan);
}

TEST(PathWindow, UnitPathIsAcceptedUnlessRejected)
{
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
  EXPECT_NEAR(w.plan().front().x, 1.0, 1e-9);
  ASSERT_EQ(w.window({0.8, 0.0, 0.0}, 3.0, out), WindowStatus::Ok);
  EXPECT_NEAR(out.front().x, 1.0, 1e-9);
}

TEST(PathWindow, KeepsTwoPointsAtTheEnd)
{
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
  PathWindow w;
  w.setPlan(uturn());
  Path out;
  ASSERT_EQ(w.window({0.3, 0.015, 0.0}, 3.0, out), WindowStatus::Ok);
  EXPECT_NEAR(out.front().yaw, M_PI, 1e-9);
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
