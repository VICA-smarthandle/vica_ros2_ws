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
}

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
  const TurnPlan t = planTurn(0.8, 0.0, true, open(), TurnParams{});
  EXPECT_EQ(t.mode, TurnMode::Pivot);
}

TEST(TurnPlanner, NearHalfTurnPicksTheSideWithRoom)
{
  const auto [lo, hi] = sweptLateralExtent(kPadded, 0.2, M_PI, 180);
  const double margin = 0.3;
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
  const TurnPlan t = planTurn(-175.0 * M_PI / 180.0, 0.0, false, open(), TurnParams{}, 1);
  EXPECT_NE(t.mode, TurnMode::Blocked);
  EXPECT_EQ(t.direction, 1);
  const TurnPlan u = planTurn(-100.0 * M_PI / 180.0, 0.0, false, open(), TurnParams{}, 1);
  EXPECT_EQ(u.direction, 1);
}

TEST(TurnPlanner, WiderRadiiListPicksTheWidestThatFits)
{
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

TEST(TurnPlanner, KeepsCurrentRadiusWhileAboveKeepClearance)
{
  TurnParams p;
  p.radii = {0.4, 0.3, 0.2, 0.1};
  p.clearance = 0.30;
  p.keep_clearance = 0.15;
  const auto [lo4, hi4] = sweptLateralExtent(kPadded, 0.4, M_PI * 0.95, 190);
  const ClearanceFn f = corridor(lo4 - 0.20, hi4 + 0.20);
  const TurnPlan fresh = planTurn(M_PI * 0.95, 0.0, false, f, p, 1);
  EXPECT_LT(fresh.radius, 0.4 - 1e-9);
  TurnPlan cur{TurnMode::Arc, 0.4, 1, 0.0};
  const TurnPlan kept = planTurn(M_PI * 0.95, 0.0, false, f, p, 1, &cur);
  EXPECT_EQ(kept.mode, TurnMode::Arc);
  EXPECT_NEAR(kept.radius, 0.4, 1e-9);
  EXPECT_GE(kept.min_clearance, 0.15);
}

TEST(TurnPlanner, DropsRadiusWhenBelowKeepClearance)
{
  TurnParams p;
  p.radii = {0.4, 0.3, 0.2, 0.1};
  p.clearance = 0.30;
  p.keep_clearance = 0.15;
  const auto [lo4, hi4] = sweptLateralExtent(kPadded, 0.4, M_PI * 0.95, 190);
  const ClearanceFn f = corridor(lo4 - 0.08, hi4 + 0.08);
  TurnPlan cur{TurnMode::Arc, 0.4, 1, 0.0};
  const TurnPlan t = planTurn(M_PI * 0.95, 0.0, false, f, p, 1, &cur);
  EXPECT_LT(t.radius, 0.4 - 1e-9);
}

TEST(TurnPlanner, KeepClearanceDefaultsToSelectClearance)
{
  TurnParams p;
  p.clearance = 0.30;
  EXPECT_DOUBLE_EQ(keepClearance(p), 0.30);
  p.keep_clearance = 0.15;
  EXPECT_DOUBLE_EQ(keepClearance(p), 0.15);
}
