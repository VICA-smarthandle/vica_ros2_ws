#include <gtest/gtest.h>
#include <cmath>
#include "vica_vcc_controller/core/pure_pursuit.hpp"

using namespace vica_vcc_controller::core;

namespace
{
Path straight(double y, double length = 3.0)
{
  Path p;
  for (double x = 0.0; x <= length + 1e-9; x += 0.05) {p.push_back({x, y, 0.0});}
  return p;
}
}  // namespace

TEST(PurePursuit, LookaheadIsClampedVelocityTimesTime)
{
  LookaheadParams lp;
  EXPECT_NEAR(lookaheadDistance(0.0, lp), 0.6, 1e-9);
  EXPECT_NEAR(lookaheadDistance(0.44, lp), 1.1, 1e-9);   // run36: 순항 0.44 m/s -> 1.10 m
  EXPECT_NEAR(lookaheadDistance(0.5, lp), 1.2, 1e-9);
}

TEST(PurePursuit, CarrotIsExactlyLookaheadAwayOnStraight)
{
  const Point2D c = carrotOnPath(straight(0.0), 1.0);
  EXPECT_NEAR(c.x, 1.0, 1e-6);
  EXPECT_NEAR(c.y, 0.0, 1e-6);
  EXPECT_NEAR(curvatureTo(c), 0.0, 1e-9);
}

TEST(PurePursuit, CarrotFallsBackToLastPoint)
{
  const Point2D c = carrotOnPath(straight(0.0, 0.3), 1.0);
  EXPECT_NEAR(c.x, 0.3, 1e-6);
}

TEST(PurePursuit, CurvatureTowardsOffsetRail)
{
  // 레일이 왼쪽 0.3 m -> 왼쪽(+)으로 도는 곡률
  const Point2D c = carrotOnPath(straight(0.3), 1.0);
  EXPECT_GT(curvatureTo(c), 0.0);
  EXPECT_NEAR(carrotTangent(straight(0.3), 1.0), 0.0, 1e-6);
}

TEST(PurePursuit, OffsetPathShiftsLinearlyToTheLeft)
{
  const Path base = straight(0.0);
  const Path p = offsetPath(base, 0.0, 0.3, 1.5);
  EXPECT_NEAR(p.front().y, 0.0, 1e-9);
  EXPECT_NEAR(p.back().y, 0.3, 1e-9);
  // 가운데(0.75 m)에서 절반
  const size_t mid = static_cast<size_t>(0.75 / 0.05);
  EXPECT_NEAR(p[mid].y, 0.15, 0.01);
  // 옆 이동이 단조 증가(되돌아가지 않음)
  for (size_t i = 1; i < p.size(); ++i) {EXPECT_GE(p[i].y + 1e-12, p[i - 1].y);}
}

TEST(PurePursuit, PrefixStopsAtLength)
{
  const Path p = pathPrefix(straight(0.0), 1.0);
  EXPECT_NEAR(pathLength(p), 1.0, 0.051);
}
