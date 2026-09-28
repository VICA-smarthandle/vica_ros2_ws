#include <gtest/gtest.h>
#include <cmath>
#include "vica_vcc_controller/core/clearance.hpp"
#include "vica_vcc_controller/core/geometry.hpp"
#include "vica_vcc_controller/core/vcc_core.hpp"

using namespace vica_vcc_controller::core;

namespace
{
const Polygon kFootprint{{0.151, 0.225}, {0.151, -0.225}, {-0.459, -0.225},
  {-0.569, -0.035}, {-0.569, 0.035}, {-0.459, 0.225}};

CoreParams params()
{
  CoreParams p;
  p.footprint = padFootprint(kFootprint, 0.05);
  return p;
}

Path line(double y0, double yaw, double length = 3.0)
{
  Path p;
  for (double s = 0.0; s <= length + 1e-9; s += 0.05) {
    p.push_back({s * std::cos(yaw), y0 + s * std::sin(yaw), yaw});
  }
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

CoreInputs inputs(const Path & path, const World & w, double now, double v, double w_meas = 0.0)
{
  CoreInputs in;
  in.now = now; in.dt = 0.1; in.path = path; in.goal = {10.0, 0.0, 0.0};
  in.measured = {v, w_meas}; in.speed_cap = 0.5; in.clearance = w.fn();
  return in;
}
}  // namespace

TEST(VccCore, StraightStartRampsWithoutTurning)
{
  VccCore c; c.configure(params());
  World w;
  double v = 0.0;
  CoreOutput out;
  for (int i = 0; i < 5; ++i) {out = c.step(inputs(line(0.0, 0.0), w, 0.1 * i, v)); v = out.cmd.v;}
  EXPECT_EQ(out.state, State::Track);
  EXPECT_NEAR(out.cmd.v, 0.25, 0.051);
  EXPECT_NEAR(out.cmd.w, 0.0, 1e-6);
  EXPECT_EQ(out.failure, Failure::None);
}

TEST(VccCore, ParallelOffsetRejoinsWithoutStopping)
{
  // 요구 3: 레일에서 0.55 m 떨어져도 멈추고 틀지 않고 곡선으로 합류
  VccCore c; c.configure(params());
  World w;
  double v = 0.3;
  for (int i = 0; i < 20; ++i) {
    const CoreOutput out = c.step(inputs(line(-0.55, 0.0), w, 0.1 * i, v));
    EXPECT_EQ(out.state, State::Track) << i;
    EXPECT_GT(out.cmd.v, 0.0) << i;
    EXPECT_LT(out.cmd.w, 0.0) << i;   // 오른쪽(레일 쪽)으로
    v = out.cmd.v;
  }
}

TEST(VccCore, UturnWhileMovingUsesArc)
{
  VccCore c; c.configure(params());
  World w;
  // 레일이 뒤쪽으로: 0.1 m 앞에서 되돌아온다
  Path back;
  for (double s = 0.0; s <= 0.1; s += 0.05) {back.push_back({s, 0.0, 0.0});}
  for (double s = 0.05; s <= 2.0; s += 0.05) {back.push_back({0.1 - s, 0.0, M_PI});}
  const CoreOutput out = c.step(inputs(back, w, 1.0, 0.3));
  EXPECT_EQ(out.state, State::Turn);
  EXPECT_EQ(out.turn_mode, TurnMode::Arc);
}

TEST(VccCore, PoleIsPassedWithoutHold)
{
  VccCore c; c.configure(params());
  World w;
  w.box(1.2, 1.3, -0.05, 0.05);
  double v = 0.4;
  for (int i = 0; i < 10; ++i) {
    const CoreOutput out = c.step(inputs(line(0.0, 0.0), w, 0.1 * i, v));
    EXPECT_NE(out.state, State::Hold) << i;
    v = out.cmd.v;
  }
}

TEST(VccCore, WallInsideStoppingDistanceIsCollisionAhead)
{
  VccCore c; c.configure(params());
  World w;
  w.box(0.35, 0.40, -2.5, 2.5);   // 몸 앞면 0.201 에서 0.15 m
  const CoreOutput out = c.step(inputs(line(0.0, 0.0), w, 1.0, 0.4));
  EXPECT_EQ(out.failure, Failure::CollisionAhead);
  EXPECT_EQ(out.cmd.v, 0.0);
  EXPECT_EQ(out.state, State::Hold);
}

TEST(VccCore, BlockedCorridorHoldsAndReportsBlockedWhenStopped)
{
  VccCore c; c.configure(params());
  World w;
  w.box(1.2, 1.3, -2.5, 2.5);
  const CoreOutput out = c.step(inputs(line(0.0, 0.0), w, 1.0, 0.0));
  EXPECT_EQ(out.state, State::Hold);
  EXPECT_EQ(out.failure, Failure::Blocked);
}

TEST(VccCore, ArrivalWithWrongYawRotatesInPlace)
{
  VccCore c; c.configure(params());
  World w;
  CoreInputs in = inputs(line(0.0, 0.0, 0.1), w, 1.0, 0.0);
  in.goal = {0.1, 0.0, M_PI / 2};
  const CoreOutput out = c.step(in);
  EXPECT_EQ(out.state, State::Align);
  EXPECT_EQ(out.cmd.v, 0.0);
  EXPECT_GT(out.cmd.w, 0.0);
}

TEST(VccCore, ArrivalWithGoodYawStops)
{
  VccCore c; c.configure(params());
  World w;
  CoreInputs in = inputs(line(0.0, 0.0, 0.1), w, 1.0, 0.0);
  in.goal = {0.1, 0.0, 0.1};
  const CoreOutput out = c.step(in);
  EXPECT_EQ(out.state, State::Track);
  EXPECT_EQ(out.cmd.v, 0.0);
}

TEST(VccCore, SpeedCapFromSpeedLimitIsRespected)
{
  VccCore c; c.configure(params());
  World w;
  CoreInputs in = inputs(line(0.0, 0.0), w, 1.0, 0.2);
  in.speed_cap = 0.2;
  double v = 0.2;
  CoreOutput out;
  for (int i = 0; i < 30; ++i) {in.now = 1.0 + 0.1 * i; in.measured.v = v; out = c.step(in); v = out.cmd.v;}
  EXPECT_LE(out.cmd.v, 0.2 + 1e-9);
}
