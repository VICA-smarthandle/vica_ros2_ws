#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include "vica_vcc_controller/core/geometry.hpp"
#include "vica_vcc_controller/core/turn_planner.hpp"

using namespace vica_vcc_controller::core;

namespace
{
const Polygon kFootprint{{0.151, 0.225}, {0.151, -0.225}, {-0.459, -0.225},
  {-0.569, -0.035}, {-0.569, 0.035}, {-0.459, 0.225}};
const Polygon kPadded = padFootprint(kFootprint, 0.05);

// y = lo, y = hi 두 벽 사이 복도. 여유 = 몸 윤곽에서 가까운 벽까지(정확한 해석식).
ClearanceFn corridor(double lo, double hi)
{
  const Polygon outline = densifyOutline(kPadded, 0.01);
  return [outline, lo, hi](const Pose2D & p) {
      double c = 1e9;
      for (const auto & q : outline) {
        const Point2D g = toParent(p, q);
        c = std::min({c, hi - g.y, g.y - lo});
      }
      return c;
    };
}
ClearanceFn open() {return [](const Pose2D &) {return 5.0;};}
}  // namespace

TEST(TurnPlanner, OpenSpaceUsesTwentyCentimetreArc)
{
  const TurnPlan t = planTurn(M_PI * 0.95, 0.0, false, open(), TurnParams{});
  EXPECT_EQ(t.mode, TurnMode::Arc);
  EXPECT_NEAR(t.radius, 0.2, 1e-9);
  EXPECT_EQ(t.direction, 1);
}

TEST(TurnPlanner, NarrowCorridorFallsBackToTenCentimetres)
{
  const auto [lo1, hi1] = sweptLateralExtent(kPadded, 0.1, M_PI * 0.95, 190);
  const TurnPlan t = planTurn(M_PI * 0.95, 0.0, false, corridor(lo1 - 0.06, hi1 + 0.06), TurnParams{});
  EXPECT_EQ(t.mode, TurnMode::Arc);
  EXPECT_NEAR(t.radius, 0.1, 1e-9);
}

TEST(TurnPlanner, TighterCorridorFallsBackToPivot)
{
  const auto [lo0, hi0] = sweptLateralExtent(kPadded, 0.0, M_PI * 0.95, 190);
  const TurnPlan t = planTurn(M_PI * 0.95, 0.0, false, corridor(lo0 - 0.06, hi0 + 0.06), TurnParams{});
  EXPECT_EQ(t.mode, TurnMode::Pivot);
}

TEST(TurnPlanner, NoRoomIsBlocked)
{
  const auto [lo0, hi0] = sweptLateralExtent(kPadded, 0.0, M_PI * 0.95, 190);
  const TurnPlan t = planTurn(M_PI * 0.95, 0.0, false, corridor(lo0 + 0.05, hi0 - 0.05), TurnParams{});
  EXPECT_EQ(t.mode, TurnMode::Blocked);
}

TEST(TurnPlanner, StationarySmallTurnPrefersPivot)
{
  // 사용자 확인: 정지 상태에서 60도 까지는 제자리
  const TurnPlan t = planTurn(0.8, 0.0, true, open(), TurnParams{});
  EXPECT_EQ(t.mode, TurnMode::Pivot);
}

TEST(TurnPlanner, NearHalfTurnPicksTheSideWithRoom)
{
  // 175도: 왼쪽으로 돌면 몸이 +y 로, 오른쪽이면 -y 로 쓴다. 왼쪽 벽을 가깝게 두면 오른쪽을 골라야 한다.
  const auto [lo, hi] = sweptLateralExtent(kPadded, 0.2, M_PI, 180);
  const double margin = 0.3;
  // 왼쪽 회전의 hi 는 막고(hi - 0.1), 오른쪽 회전이 쓰는 -hi 쪽은 넉넉히
  const TurnPlan t = planTurn(175.0 * M_PI / 180.0, 0.0, false,
      corridor(-hi - margin, hi - 0.1), TurnParams{});
  EXPECT_EQ(t.direction, -1);
  (void)lo;
}

TEST(TurnPlanner, ArcCommandKeepsRadius)
{
  TurnPlan t;
  t.mode = TurnMode::Arc; t.radius = 0.2; t.direction = -1;
  const Twist2D c = turnCommand(t, TurnParams{});
  EXPECT_NEAR(c.v, 0.45 * 0.2, 1e-9);
  EXPECT_NEAR(c.w, -0.45, 1e-9);
}

TEST(TurnPlanner, OnlyDirForcesDirection)
{
  // 최종 리뷰 M2: Turn 중에는 들어갈 때 고른 방향으로만 다시 계획한다.
  const TurnPlan t = planTurn(-175.0 * M_PI / 180.0, 0.0, false, open(), TurnParams{}, 1);
  EXPECT_NE(t.mode, TurnMode::Blocked);
  EXPECT_EQ(t.direction, 1);
  const TurnPlan u = planTurn(-100.0 * M_PI / 180.0, 0.0, false, open(), TurnParams{}, 1);
  EXPECT_EQ(u.direction, 1);   // 반대쪽이라도 고정 방향(남은 각 260°)
}

TEST(TurnPlanner, WiderRadiiListPicksTheWidestThatFits)
{
  // 2026-09-30 run50: U턴 5회 전부 R 0.20(목록 최대). 목록을 [0.4, 0.3, 0.2, 0.1] 로 넓히면
  // 앞에서부터 들어가는 첫 반지름을 쓴다 — 트인 곳은 0.4, 좁아질수록 자동으로 작아진다.
  TurnParams p;
  p.radii = {0.4, 0.3, 0.2, 0.1};
  EXPECT_NEAR(planTurn(M_PI * 0.95, 0.0, false, open(), p).radius, 0.4, 1e-9);
  const auto [lo3, hi3] = sweptLateralExtent(kPadded, 0.3, M_PI * 0.95, 190);
  const TurnPlan t3 = planTurn(M_PI * 0.95, 0.0, false, corridor(lo3 - 0.06, hi3 + 0.06), p);
  EXPECT_EQ(t3.mode, TurnMode::Arc);
  EXPECT_NEAR(t3.radius, 0.3, 1e-9);
  const auto [lo2, hi2] = sweptLateralExtent(kPadded, 0.2, M_PI * 0.95, 190);
  const TurnPlan t2 = planTurn(M_PI * 0.95, 0.0, false, corridor(lo2 - 0.06, hi2 + 0.06), p);
  EXPECT_EQ(t2.mode, TurnMode::Arc);
  EXPECT_NEAR(t2.radius, 0.2, 1e-9);
}
