#include <gtest/gtest.h>
#include <cmath>
#include "vica_vcc_controller/core/clearance.hpp"
#include "vica_vcc_controller/core/geometry.hpp"
#include "vica_vcc_controller/core/lanes.hpp"

using namespace vica_vcc_controller::core;

namespace
{
const Polygon kFootprint{{0.151, 0.225}, {0.151, -0.225}, {-0.459, -0.225},
  {-0.569, -0.035}, {-0.569, 0.035}, {-0.459, 0.225}};

Path rail()
{
  Path p;
  for (double x = 0.0; x <= 3.0 + 1e-9; x += 0.05) {p.push_back({x, 0.0, 0.0});}
  return p;
}

struct World
{
  ClearanceField f;
  World()
  {
    f.setFootprint(padFootprint(kFootprint, 0.05));
    f.grid().reset(-2.5, -2.5, 0.05, 100, 100);
    f.grid().compute();
  }
  void box(double x0, double x1, double y0, double y1)
  {
    for (int ix = 0; ix < 100; ++ix) {
      for (int iy = 0; iy < 100; ++iy) {
        const double cx = -2.5 + (ix + 0.5) * 0.05, cy = -2.5 + (iy + 0.5) * 0.05;
        if (cx >= x0 && cx <= x1 && cy >= y0 && cy <= y1) {f.grid().markLethal(ix, iy);}
      }
    }
    f.grid().compute();
  }
  ClearanceFn fn() const {return [this](const Pose2D & p) {return f.clearance(p);};}
};

void run(
  LaneSelector & ls, const World & w, int cycles, double & now, double v = 0.4, double v_des = 0.5)
{
  for (int i = 0; i < cycles; ++i) {now += 0.1; ls.update(rail(), v, v_des, now, 0.1, w.fn());}
}
}

TEST(Lanes, EmptyCorridorStaysOnRail)
{
  World w;
  LaneSelector ls;
  double now = 0.0;
  run(ls, w, 10, now);
  EXPECT_NEAR(ls.target(), 0.0, 1e-9);
  EXPECT_NEAR(ls.offset(), 0.0, 1e-9);
  EXPECT_FALSE(ls.blocked());
}

TEST(Lanes, PoleOnRailIsPassedWithTwentyCentimetres)
{
  World w;
  w.box(1.5, 1.6, -0.05, 0.05);
  LaneSelector ls;
  double now = 0.0;
  run(ls, w, 3, now);
  EXPECT_NEAR(std::abs(ls.target()), 0.6, 1e-9);
  const auto & s = ls.scores();
  for (const auto & sc : s) {
    if (std::abs(sc.offset - ls.target()) < 1e-9) {
      EXPECT_GE(sc.min_clearance, 0.2 - 0.03);
      EXPECT_NEAR(sc.speed, 0.2, 1e-9);
    }
  }
  EXPECT_NEAR(ls.speedCap(), 0.2, 1e-9);
}

TEST(Lanes, SwitchWaitsForPersistence)
{
  World w;
  w.box(1.5, 1.6, 0.3, 0.4);
  LaneSelector ls;
  double now = 0.0;
  run(ls, w, 2, now);
  EXPECT_NEAR(ls.target(), 0.0, 1e-9);
  run(ls, w, 1, now);
  EXPECT_NEAR(ls.target(), -0.2, 1e-9);
}

TEST(Lanes, OffsetMovesAtLaneRate)
{
  World w;
  w.box(1.5, 1.6, -0.05, 0.05);
  LaneSelector ls;
  double now = 0.0;
  run(ls, w, 3, now, 0.2);
  const double before = ls.offset();
  run(ls, w, 1, now, 0.2);
  EXPECT_NEAR(std::abs(ls.offset() - before), 0.01, 1e-9);
  const double still = ls.offset();
  run(ls, w, 1, now, 0.0);
  EXPECT_NEAR(ls.offset(), still, 1e-12);
}

TEST(Lanes, ReturnsToRailOnlyAfterOneSecondClear)
{
  World w;
  w.box(1.5, 1.6, -0.05, 0.05);
  LaneSelector ls;
  double now = 0.0;
  run(ls, w, 3, now);
  ASSERT_NE(ls.target(), 0.0);
  World empty;
  run(ls, empty, 9, now);
  EXPECT_NE(ls.target(), 0.0);
  run(ls, empty, 2, now);
  EXPECT_NEAR(ls.target(), 0.0, 1e-9);
}

TEST(Lanes, NearlyEqualLanesDoNotFlipFlop)
{
  LaneSelector ls;
  double now = 0.0;
  int switches = 0;
  double last = ls.target();
  for (int i = 0; i < 50; ++i) {
    const double wobble = (i % 2 == 0) ? 0.01 : -0.01;
    ClearanceFn fn = [wobble](const Pose2D & p) {return 0.10 + wobble * (p.y > 0 ? 1 : -1);};
    now += 0.1;
    ls.update(rail(), 0.4, 0.5, now, 0.1, fn);
    if (ls.target() != last) {++switches; last = ls.target();}
  }
  EXPECT_LE(switches, 1);
}

TEST(Lanes, FarObstacleBeyondHorizonIsIgnored)
{
  World w;
  w.box(2.2, 2.4, -0.3, 0.3);
  LaneSelector ls;
  double now = 0.0;
  run(ls, w, 10, now);
  EXPECT_NEAR(ls.target(), 0.0, 1e-9);
}

TEST(Lanes, WallAcrossCorridorIsBlocked)
{
  World w;
  w.box(1.0, 1.1, -2.5, 2.5);
  LaneSelector ls;
  double now = 0.0;
  run(ls, w, 1, now);
  EXPECT_TRUE(ls.blocked());
}

TEST(Lanes, TieGoesRight)
{
  World w;
  w.box(1.5, 1.6, -0.05, 0.05);
  LaneSelector ls;
  double now = 0.0;
  run(ls, w, 3, now);
  EXPECT_LT(ls.target(), 0.0);
}

TEST(Lanes, SyncOffsetClampsToMaxOffset)
{
  LaneSelector ls;
  ls.syncOffset(0.4);
  EXPECT_NEAR(ls.offset(), 0.4, 1e-9);
  EXPECT_NEAR(ls.target(), 0.0, 1e-9);
  ls.syncOffset(-0.9);
  EXPECT_NEAR(ls.offset(), -0.6, 1e-9);
}

TEST(Lanes, ShiftSpeedStartsFromDesiredNotMeasured)
{
  World w;
  LaneSelector ls;
  ls.syncOffset(0.4);
  double now = 0.0;
  run(ls, w, 1, now, 0.1, 0.5);
  ASSERT_GT(std::abs(ls.target() - ls.offset()), 0.01);
  EXPECT_GE(ls.speedCap(), 0.5);
}

TEST(Lanes, PoleOnRailAtLowMeasuredSpeedStillShiftsAtTwo)
{
  World w;
  w.box(1.5, 1.6, -0.05, 0.05);
  LaneSelector ls;
  double now = 0.0;
  run(ls, w, 3, now, 0.1, 0.5);
  EXPECT_NEAR(std::abs(ls.target()), 0.6, 1e-9);
  EXPECT_NEAR(ls.speedCap(), 0.2, 1e-9);
}
