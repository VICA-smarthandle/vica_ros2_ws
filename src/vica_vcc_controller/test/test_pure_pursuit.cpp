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

// ── 2026-10-05 해결안 가: 경로 끝 연장 ───────────────────────────────────────────────────
namespace
{
// x 축과 나란히 end_x 에서 끝나는 경로(옆 y). VCC 경로 창처럼 로봇 근처(끝 0.4 m 전)에서 시작한다.
Path ending(double y, double end_x, double start_x = 1e9)
{
  if (start_x > 1e8) {start_x = end_x - 0.4;}
  Path p;
  for (double x = start_x; x <= end_x + 1e-9; x += 0.05) {p.push_back({x, y, 0.0});}
  return p;
}
}  // namespace

// run64 834 s 재현: 끝점이 0.05 m 앞·옆 4 cm. 예전 조준(끝점)은 39°, 연장은 0.6 m 앞이라 4° 안팎.
TEST(PurePursuitEndExtension, NearTheEndAimStaysFarAndGentle)
{
  const Path p = ending(-0.04, 0.05);
  const Point2D old_aim = carrotOnPath(p, 0.6);
  EXPECT_GT(std::abs(std::atan2(old_aim.y, old_aim.x)), 0.6);   // 34° 넘게
  bool ext = false;
  const Point2D aim = carrotWithEndExtension(p, 0.6, 0.08, 0.3, &ext);
  ASSERT_TRUE(ext);
  EXPECT_NEAR(std::hypot(aim.x, aim.y), 0.6, 1e-9);
  EXPECT_LT(std::abs(std::atan2(aim.y, aim.x)), 5.0 * M_PI / 180.0);
  EXPECT_LT(std::abs(curvatureTo(aim)), 0.25);                  // 예전 2y/d^2 ≈ 19 1/m
}

TEST(PurePursuitEndExtension, LargeLateralOffsetFallsBackToTheEndPoint)
{
  const Path p = ending(-0.12, 0.3);
  bool ext = true;
  const Point2D aim = carrotWithEndExtension(p, 0.6, 0.08, 0.3, &ext);
  EXPECT_FALSE(ext);
  EXPECT_NEAR(aim.x, 0.3, 1e-6);
  EXPECT_NEAR(aim.y, -0.12, 1e-9);
}

TEST(PurePursuitEndExtension, ShortPathIsNotExtended)
{
  const Path p = ending(0.0, 0.2, 0.0);   // 길이 0.2 m < 0.3
  bool ext = true;
  carrotWithEndExtension(p, 0.6, 0.08, 0.3, &ext);
  EXPECT_FALSE(ext);
}

TEST(PurePursuitEndExtension, EndBehindTheRobotIsNotExtended)
{
  const Path p = ending(0.0, -0.05);
  bool ext = true;
  const Point2D aim = carrotWithEndExtension(p, 0.6, 0.08, 0.3, &ext);
  EXPECT_FALSE(ext);
  EXPECT_NEAR(aim.x, -0.05, 1e-6);
}

TEST(PurePursuitEndExtension, FarFromTheEndBehavesLikeBefore)
{
  const Path p = straight(0.1);
  bool ext = true;
  const Point2D aim = carrotWithEndExtension(p, 0.6, 0.08, 0.3, &ext);
  const Point2D old_aim = carrotOnPath(p, 0.6);
  EXPECT_FALSE(ext);
  EXPECT_NEAR(aim.x, old_aim.x, 1e-12);
  EXPECT_NEAR(aim.y, old_aim.y, 1e-12);
}
