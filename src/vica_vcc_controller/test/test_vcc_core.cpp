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
}

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
  VccCore c; c.configure(params());
  World w;
  double v = 0.3;
  for (int i = 0; i < 20; ++i) {
    const CoreOutput out = c.step(inputs(line(-0.55, 0.0), w, 0.1 * i, v));
    EXPECT_EQ(out.state, State::Track) << i;
    EXPECT_GT(out.cmd.v, 0.0) << i;
    EXPECT_LT(out.cmd.w, 0.0) << i;
    v = out.cmd.v;
  }
}

TEST(VccCore, UturnWhileMovingUsesArc)
{
  VccCore c; c.configure(params());
  World w;
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
  w.box(0.35, 0.40, -2.5, 2.5);
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

TEST(VccCore, BlockedUturnInNarrowCorridorStaysInHoldWithoutCreeping)
{
  VccCore c; c.configure(params());
  World w;
  w.box(-2.5, 2.5, 0.40, 0.45);
  w.box(-2.5, 2.5, -0.45, -0.40);
  double X = 0.0, Y = 0.0, TH = 0.0, v = 0.0, wz = 0.0;
  for (int i = 0; i < 50; ++i) {
    Path back;
    for (double s = 0.0; s <= 2.0 + 1e-9; s += 0.05) {
      const Point2D q = toChild({X, Y, TH}, Point2D{-s, 0.0});
      back.push_back({q.x, q.y, normalizeAngle(M_PI - TH)});
    }
    CoreInputs in = inputs(back, w, 0.1 * i, v, wz);
    const Point2D g = toChild({X, Y, TH}, Point2D{-2.0, 0.0});
    in.goal = {g.x, g.y, normalizeAngle(M_PI - TH)};
    const Pose2D R{X, Y, TH};
    in.clearance = [&w, R](const Pose2D & q) {return w.f.clearance(toParent(R, q));};
    const CoreOutput out = c.step(in);
    const bool ok = out.failure == Failure::None;
    v = ok ? out.cmd.v : 0.0;
    wz = ok ? out.cmd.w : 0.0;
    EXPECT_LE(v, 1e-9) << i;
    EXPECT_EQ(out.state, State::Hold) << i;
    EXPECT_EQ(out.failure, Failure::Blocked) << i;
    X += v * std::cos(TH) * 0.1; Y += v * std::sin(TH) * 0.1; TH += wz * 0.1;
  }
  EXPECT_NEAR(X, 0.0, 1e-9);
}

TEST(VccCore, NewGoalKeepsSpeedAndLane)
{
  VccCore a; a.configure(params());
  VccCore b; b.configure(params());
  World w;
  w.box(1.2, 1.3, -0.05, 0.05);
  double v = 0.5;
  CoreOutput oa, ob;
  for (int i = 0; i < 8; ++i) {
    oa = a.step(inputs(line(0.0, 0.0), w, 0.1 * i, v));
    ob = b.step(inputs(line(0.0, 0.0), w, 0.1 * i, v));
    v = oa.cmd.v;
  }
  ASSERT_GT(std::abs(oa.target), 0.05);
  b.onNewGoal();
  oa = a.step(inputs(line(0.0, 0.0), w, 0.8, v));
  ob = b.step(inputs(line(0.0, 0.0), w, 0.8, v));
  EXPECT_NEAR(ob.offset, oa.offset, 1e-9);
  EXPECT_NEAR(ob.target, oa.target, 1e-9);
  EXPECT_NEAR(ob.cmd.v, oa.cmd.v, 1e-9);
  EXPECT_EQ(ob.state, oa.state);
}

TEST(VccCore, NewGoalWhileCruisingKeepsHalfSpeed)
{
  VccCore c; c.configure(params());
  World w;
  double v = 0.0;
  CoreOutput out;
  for (int i = 0; i < 40; ++i) {out = c.step(inputs(line(0.0, 0.0), w, 0.1 * i, v)); v = out.cmd.v;}
  ASSERT_NEAR(v, 0.5, 1e-6);
  c.onNewGoal();
  out = c.step(inputs(line(0.0, 0.0), w, 4.0, v));
  EXPECT_GE(out.cmd.v, 0.45);
}

TEST(VccCore, NewGoalLeavesHoldAndResetsAlign)
{
  VccCore c; c.configure(params());
  World w;
  CoreInputs in = inputs(line(0.0, 0.0, 0.1), w, 1.0, 0.0);
  in.goal = {0.1, 0.0, M_PI / 2};
  CoreOutput out = c.step(in);
  ASSERT_EQ(out.state, State::Align);
  ASSERT_EQ(out.align_attempts, 1);
  c.onNewGoal();
  in = inputs(line(0.0, 0.0), w, 1.1, 0.0);
  out = c.step(in);
  EXPECT_EQ(out.state, State::Track);
  EXPECT_EQ(out.align_attempts, 0);
}

TEST(VccCore, MotionProjectionCatchesWallOnTurningArc)
{
  World w;
  w.box(0.30, 0.40, 0.33, 0.40);
  const ClearanceFn f = w.fn();
  for (const auto & ps : pathPrefix(line(0.0, 0.0), 0.5)) {ASSERT_GE(f(ps), 0.0);}
  EXPECT_TRUE(motionCollides({0.4, 0.5}, f, 1.25, 1.2, 0.3));
  EXPECT_FALSE(motionCollides({0.4, 0.0}, f, 1.25, 1.2, 0.3));
  EXPECT_FALSE(motionCollides({0.4, -0.5}, f, 1.25, 1.2, 0.3));
  EXPECT_FALSE(motionCollides({0.0, 0.0}, f, 1.25, 1.2, 0.3));
}

TEST(VccCore, MotionProjectionCatchesRearSweepOfPureRotation)
{
  World w;
  w.box(-0.75, -0.30, -0.40, -0.30);
  const ClearanceFn f = w.fn();
  ASSERT_GE(f({0.0, 0.0, 0.0}), 0.0);
  EXPECT_TRUE(motionCollides({0.0, 0.35}, f, 1.25, 1.2, 0.3));
  EXPECT_FALSE(motionCollides({0.0, -0.35}, f, 1.25, 1.2, 0.3));
}

TEST(VccCore, OffsetResyncsToRobotAfterUturnExit)
{
  VccCore c; c.configure(params());
  World w;
  const CoreOutput out = c.step(inputs(line(-0.4, 0.0), w, 1.0, 0.1));
  EXPECT_NEAR(out.offset, 0.4, 0.011);
  ASSERT_FALSE(out.lane_path.empty());
  EXPECT_NEAR(out.lane_path.front().y, 0.0, 0.011);
}

TEST(VccCore, BlockedAlignSweepHoldsWithoutRotating)
{
  VccCore c; c.configure(params());
  World w;
  w.box(-0.60, 0.30, -0.55, -0.45);
  ASSERT_GE(w.fn()({0.0, 0.0, 0.0}), 0.0);
  for (int i = 0; i < 20; ++i) {
    CoreInputs in = inputs(line(0.0, 0.0, 0.1), w, 1.0 + 0.1 * i, 0.0);
    in.goal = {0.1, 0.0, 150.0 * M_PI / 180.0};
    const CoreOutput out = c.step(in);
    EXPECT_EQ(out.state, State::Hold) << i;
    EXPECT_EQ(out.cmd.w, 0.0) << i;
    EXPECT_EQ(out.failure, Failure::Blocked) << i;
  }
}

TEST(VccCore, TurnDirectionIsLockedWhileTurning)
{
  auto back = [](double yoff) {
      Path p;
      for (double s = 0.0; s <= 0.1; s += 0.05) {p.push_back({s, 0.0, 0.0});}
      for (double s = 0.05; s <= 2.0; s += 0.05) {p.push_back({0.1 - s, yoff, M_PI});}
      return p;
    };
  VccCore c; c.configure(params());
  World w;
  CoreOutput out = c.step(inputs(back(0.1), w, 1.0, 0.3));
  ASSERT_EQ(out.state, State::Turn);
  ASSERT_GT(out.cmd.w, 0.0);
  for (int i = 1; i <= 3; ++i) {
    out = c.step(inputs(back(i % 2 ? -0.1 : 0.1), w, 1.0 + 0.1 * i, 0.3, out.cmd.w));
    EXPECT_EQ(out.state, State::Turn) << i;
    EXPECT_GT(out.cmd.w, 0.0) << i;
  }
}

TEST(VccCore, AlignAttemptsSurvivePushOffAndReturn)
{
  VccCore c; c.configure(params());
  World w;
  auto at = [&](double now, double dist, double yaw) {
      CoreInputs in = inputs(line(0.0, 0.0, dist), w, now, 0.0);
      in.goal = {dist, 0.0, yaw};
      return c.step(in);
    };
  CoreOutput out = at(1.0, 0.1, 0.3);
  ASSERT_EQ(out.state, State::Align);
  out = at(1.1, 0.1, -0.3);
  for (double t = 1.2; t < 1.55; t += 0.1) {out = at(t, 0.1, -0.3);}
  ASSERT_EQ(out.align_attempts, 2);
  out = at(1.6, 0.6, -0.3);
  ASSERT_EQ(out.state, State::Track);
  out = at(1.7, 0.1, -0.3);
  ASSERT_EQ(out.state, State::Align);
  EXPECT_EQ(out.align_attempts, 2);
}

TEST(VccCore, AcceleratesWhileReturningToRailAfterUturn)
{
  VccCore c; c.configure(params());
  World w;
  double X = 0.0, Y = 0.4, TH = 0.0, v = 0.1, wz = 0.0;
  double t_fast = -1.0;
  for (int i = 0; i < 20; ++i) {
    Path rail;
    for (double s = 0.0; s <= 3.0 + 1e-9; s += 0.05) {
      const Point2D q = toChild({X, Y, TH}, Point2D{X + s, 0.0});
      rail.push_back({q.x, q.y, normalizeAngle(-TH)});
    }
    CoreInputs in = inputs(rail, w, 0.1 * i, v, wz);
    const Point2D g = toChild({X, Y, TH}, Point2D{X + 10.0, 0.0});
    in.goal = {g.x, g.y, normalizeAngle(-TH)};
    const CoreOutput out = c.step(in);
    ASSERT_EQ(out.failure, Failure::None) << i;
    v = out.cmd.v;
    wz = out.cmd.w;
    if (t_fast < 0.0 && v > 0.25) {t_fast = 0.1 * (i + 1);}
    X += v * std::cos(TH) * 0.1; Y += v * std::sin(TH) * 0.1; TH += wz * 0.1;
  }
  EXPECT_GT(t_fast, 0.0);
  EXPECT_LE(t_fast, 2.0);
}

TEST(VccCore, FarFirstPointOnLineThroughRobotDoesNotResync)
{
  VccCore c; c.configure(params());
  World w;
  const double a = 30.0 * M_PI / 180.0;
  Path p;
  for (double s = 0.6; s <= 3.0 + 1e-9; s += 0.05) {p.push_back({s * std::cos(a), s * std::sin(a), a});}
  p.front().yaw = 0.0;
  for (int i = 0; i < 3; ++i) {
    const CoreOutput out = c.step(inputs(p, w, 1.0 + 0.1 * i, 0.3));
    EXPECT_NEAR(out.offset, 0.0, 0.02) << i;
  }
}

TEST(VccCore, OneCycleLateralGlitchDoesNotResync)
{
  VccCore c; c.configure(params());
  World w;
  double t = 1.0;
  CoreOutput out;
  for (int i = 0; i < 3; ++i, t += 0.1) {out = c.step(inputs(line(0.0, 0.0), w, t, 0.3));}
  out = c.step(inputs(line(-0.4, 0.0), w, t, 0.3));
  t += 0.1;
  EXPECT_NEAR(out.offset, 0.0, 0.02);
  for (int i = 0; i < 3; ++i, t += 0.1) {
    out = c.step(inputs(line(0.0, 0.0), w, t, 0.3));
    EXPECT_NEAR(out.offset, 0.0, 0.02) << i;
  }
}

TEST(VccCore, PersistentLateralErrorResyncsOnSecondCycle)
{
  VccCore c; c.configure(params());
  World w;
  double t = 1.0;
  CoreOutput out;
  for (int i = 0; i < 3; ++i, t += 0.1) {out = c.step(inputs(line(0.0, 0.0), w, t, 0.3));}
  out = c.step(inputs(line(-0.4, 0.0), w, t, 0.3));
  EXPECT_NEAR(out.offset, 0.0, 0.02);
  out = c.step(inputs(line(-0.4, 0.0), w, t + 0.1, 0.3));
  EXPECT_NEAR(out.offset, 0.4, 0.011);
}

TEST(VccCore, TurnExitKeepsNearestLaneAsTarget)
{
  VccCore c; c.configure(params());
  World w;
  Path back;
  for (double s = 0.0; s <= 0.1; s += 0.05) {back.push_back({s, 0.0, 0.0});}
  for (double s = 0.05; s <= 2.0; s += 0.05) {back.push_back({0.1 - s, 0.0, M_PI});}
  CoreOutput out = c.step(inputs(back, w, 1.0, 0.3));
  ASSERT_EQ(out.state, State::Turn);
  bool exited = false;
  for (int i = 1; i <= 10 && !exited; ++i) {
    out = c.step(inputs(line(-0.4, 0.0), w, 1.0 + 0.1 * i, 0.4));
    exited = out.state == State::Track;
  }
  ASSERT_TRUE(exited);
  EXPECT_NEAR(out.target, 0.4, 1e-9);
  EXPECT_NEAR(out.offset, 0.4, 0.011);
}

TEST(VccCore, StopsInsideTheGoalCheckerCircle)
{
  VccCore c; c.configure(params());
  World w;
  CoreInputs in = inputs(line(0.0, 0.0, 0.14), w, 1.0, 0.1);
  in.goal = {0.14, 0.0, 0.0};
  in.xy_tol = 0.15;
  CoreOutput out = c.step(in);
  EXPECT_EQ(out.state, State::Track);
  EXPECT_GT(out.cmd.v, 0.0);
  in = inputs(line(0.0, 0.0, 0.11), w, 1.1, 0.05);
  in.goal = {0.11, 0.0, 0.0};
  in.xy_tol = 0.15;
  out = c.step(in);
  EXPECT_EQ(out.state, State::Track);
  EXPECT_LT(out.cmd.v, 0.05);
}

TEST(VccCore, AlignFailureRearmsAfterRearmTime)
{
  CoreParams p = params();
  p.align.max_attempts = 1;
  VccCore c; c.configure(p);
  World w;
  auto at = [&](double now, double yaw) {
      CoreInputs in = inputs(line(0.0, 0.0, 0.1), w, now, 0.0);
      in.goal = {0.1, 0.0, yaw};
      return c.step(in);
    };
  CoreOutput out = at(1.0, 0.5);
  ASSERT_EQ(out.state, State::Align);
  double t = 1.1;
  for (; t < 3.0 && out.failure != Failure::AlignFailed; t += 0.1) {out = at(t, -0.5);}
  ASSERT_EQ(out.failure, Failure::AlignFailed);
  const double failed_at = t - 0.1;
  for (; t < failed_at + 2.9; t += 0.1) {
    out = at(t, -0.5);
    ASSERT_EQ(out.failure, Failure::AlignFailed) << t;
  }
  for (; t < failed_at + 3.6; t += 0.1) {out = at(t, -0.5);}
  EXPECT_NE(out.failure, Failure::AlignFailed);
  EXPECT_EQ(out.state, State::Align);
  EXPECT_EQ(out.align_attempts, 1);
  EXPECT_LT(out.cmd.w, 0.0);
}

namespace
{
Path aimPath(double rel)
{
  Path p;
  for (double s = 0.0; s <= 3.0 + 1e-9; s += 0.05) {p.push_back({s * std::cos(rel), s * std::sin(rel), rel});}
  return p;
}

CoreParams paramsC(bool lock, double cap, double persist)
{
  CoreParams p = params();
  p.turn_lock_target = lock;
  p.turn_max_rotation = cap;
  p.turn_enter_persist = persist;
  return p;
}

struct Sim
{
  VccCore c;
  World w;
  double th{0.0}, v{0.3}, wz{0.0}, t{1.0};
  double rot_in_first_turn{0.0};
  bool left_first_turn{false};
  bool seen_turn{false};
  CoreOutput step(double rel_aim)
  {
    CoreInputs in = inputs(aimPath(rel_aim), w, t, v, wz);
    in.robot_yaw = th;
    const CoreOutput out = c.step(in);
    v = out.cmd.v; wz = out.cmd.w;
    if (out.state == State::Turn) {seen_turn = true;}
    if (seen_turn && !left_first_turn) {
      if (out.state == State::Turn) {rot_in_first_turn += std::abs(wz) * 0.1;} else {left_first_turn = true;}
    }
    th = normalizeAngle(th + wz * 0.1);
    t += 0.1;
    return out;
  }
};
}

TEST(VccCoreTurnEnd, OldTurnChasesItsTailForever)
{
  Sim s; s.c.configure(paramsC(false, 0.0, 0.0));
  for (int i = 0; i < 200; ++i) {s.step(100.0 * M_PI / 180.0);}
  EXPECT_FALSE(s.left_first_turn);
  EXPECT_GT(s.rot_in_first_turn, 2.0 * M_PI);
}

TEST(VccCoreTurnEnd, LockedTargetEndsTheTurnAtTheEntryAim)
{
  Sim s; s.c.configure(paramsC(true, 0.0, 0.0));
  for (int i = 0; i < 200 && !s.left_first_turn; ++i) {s.step(100.0 * M_PI / 180.0);}
  ASSERT_TRUE(s.left_first_turn);
  EXPECT_GT(s.rot_in_first_turn, 60.0 * M_PI / 180.0);
  EXPECT_LT(s.rot_in_first_turn, 120.0 * M_PI / 180.0);
}

TEST(VccCoreTurnEnd, RotationCapHoldsAndReportsOverrun)
{
  Sim s; s.c.configure(paramsC(false, 2.0 * M_PI, 0.0));
  bool held = false;
  for (int i = 0; i < 250 && !held; ++i) {
    const CoreOutput out = s.step(100.0 * M_PI / 180.0);
    held = out.state == State::Hold && std::string(out.reason) == "turn_overrun";
  }
  EXPECT_TRUE(held);
  EXPECT_LT(s.rot_in_first_turn, 2.0 * M_PI + 0.3);
}

TEST(VccCoreTurnEnd, RetargetsOnlyWhenTheNewAimPersists)
{
  Sim s; s.c.configure(paramsC(true, 0.0, 0.0));
  const double north = 150.0 * M_PI / 180.0, south_east = -60.0 * M_PI / 180.0;
  CoreOutput out;
  for (int i = 0; i < 5; ++i) {out = s.step(normalizeAngle(north - s.th));}
  ASSERT_EQ(out.state, State::Turn);
  ASSERT_GT(out.cmd.w, 0.0);
  for (int i = 0; i < 3; ++i) {out = s.step(normalizeAngle(south_east - s.th)); EXPECT_GT(out.cmd.w, 0.0) << i;}
  for (int i = 0; i < 3; ++i) {out = s.step(normalizeAngle(north - s.th));}
  EXPECT_GT(out.cmd.w, 0.0);
  bool flipped = false;
  for (int i = 0; i < 12; ++i) {out = s.step(normalizeAngle(south_east - s.th)); if (out.cmd.w < 0.0) {flipped = true;}}
  EXPECT_TRUE(flipped);
}

TEST(VccCoreTurnEnd, EntryNeedsPersistentHeadingError)
{
  Sim s; s.c.configure(paramsC(true, 0.0, 0.3));
  CoreOutput out = s.step(0.0);
  out = s.step(170.0 * M_PI / 180.0);
  EXPECT_NE(out.state, State::Turn);
  out = s.step(0.0);
  EXPECT_NE(out.state, State::Turn);
  for (int i = 0; i < 6; ++i) {out = s.step(normalizeAngle(170.0 * M_PI / 180.0));}
  EXPECT_EQ(out.state, State::Turn);
}

namespace
{
CoreParams paramsEnd(bool on)
{
  CoreParams p = params();
  p.end_extend = on;
  p.pass_arrival = on;
  return p;
}
CoreInputs endInputs(const World & w, double t, double v, double wz, double x_end, double y, double yaw_end)
{
  Path p;
  for (double x = x_end - 0.4; x <= x_end + 1e-9; x += 0.05) {p.push_back({x, y, 0.0});}
  p.back().yaw = yaw_end;
  CoreInputs in = inputs(p, w, t, v, wz);
  in.goal = {x_end, y, yaw_end};
  in.xy_tol = 0.15;
  return in;
}
}

TEST(VccCoreEndExtension, TurnsLittleRightBeforeArrival)
{
  World w;
  auto run = [&](bool on) {
      VccCore c; c.configure(paramsEnd(on));
      double v = 0.1, wz = 0.0, peak = 0.0;
      for (int i = 0; i < 8; ++i) {
        const CoreOutput out = c.step(endInputs(w, 1.0 + 0.1 * i, v, wz, 0.17, -0.04, 2.08));
        v = out.cmd.v; wz = out.cmd.w;
        if (out.state == State::Track) {peak = std::max(peak, std::abs(wz));}
      }
      return peak;
    };
  const double old_peak = run(false), new_peak = run(true);
  EXPECT_GT(old_peak, 0.3);
  EXPECT_LT(new_peak, 0.15);
}

TEST(VccCoreEndExtension, PassingTheEndInsideTheCheckerCircleArrives)
{
  World w;
  VccCore old_c; old_c.configure(paramsEnd(false));
  VccCore new_c; new_c.configure(paramsEnd(true));
  const CoreOutput o = old_c.step(endInputs(w, 1.0, 0.05, 0.0, -0.13, 0.0, 1.0));
  const CoreOutput n = new_c.step(endInputs(w, 1.0, 0.05, 0.0, -0.13, 0.0, 1.0));
  EXPECT_NE(o.state, State::Align);
  EXPECT_EQ(n.state, State::Align);
}

TEST(VccCoreEndExtension, PassingOutsideTheCheckerCircleIsNotArrival)
{
  World w;
  VccCore c; c.configure(paramsEnd(true));
  const CoreOutput n = c.step(endInputs(w, 1.0, 0.05, 0.0, -0.16, 0.0, 1.0));
  EXPECT_NE(n.state, State::Align);
}

namespace
{
CoreParams paramsStraight(double lateral)
{
  CoreParams p = paramsEnd(true);
  p.end_straight_lateral = lateral;
  return p;
}
struct EndSim
{
  VccCore c;
  World w;
  double ex, ey, eyaw;
  double x{0.0}, y{0.0}, th{0.0}, v{0.1}, wz{0.0}, t{1.0};
  double rot_before_align{0.0};
  double dist_at_align{-1.0};
  bool straight_seen{false};
  EndSim(const CoreParams & p, double ex_, double ey_, double eyaw_) : ex(ex_), ey(ey_), eyaw(eyaw_)
  {
    c.configure(p);
  }
  Point2D toRobot(double px, double py) const
  {
    const double dx = px - x, dy = py - y;
    return {dx * std::cos(th) + dy * std::sin(th), -dx * std::sin(th) + dy * std::cos(th)};
  }
  CoreOutput step()
  {
    Path p;
    for (double px = std::min(x, ex); px <= ex + 1e-9; px += 0.05) {
      const Point2D q = toRobot(px, ey);
      p.push_back({q.x, q.y, -th});
    }
    const Point2D g = toRobot(ex, ey);
    if (p.empty() || std::hypot(p.back().x - g.x, p.back().y - g.y) > 1e-9) {p.push_back({g.x, g.y, 0.0});}
    p.back().yaw = normalizeAngle(eyaw - th);
    CoreInputs in = inputs(p, w, t, v, wz);
    in.goal = {g.x, g.y, normalizeAngle(eyaw - th)};
    in.xy_tol = 0.15;
    in.yaw_tol = 0.25;
    in.robot_yaw = th;
    const CoreOutput out = c.step(in);
    if (out.end_straight) {straight_seen = true;}
    if (out.state == State::Align && dist_at_align < 0.0) {dist_at_align = std::hypot(g.x, g.y);}
    if (dist_at_align < 0.0) {rot_before_align += out.cmd.w * 0.1;}
    v = out.cmd.v; wz = out.cmd.w;
    x += v * std::cos(th) * 0.1; y += v * std::sin(th) * 0.1;
    th = normalizeAngle(th + wz * 0.1);
    t += 0.1;
    return out;
  }
  void run(int n = 80) {for (int i = 0; i < n && dist_at_align < 0.0; ++i) {step();}}
};
}

TEST(VccCoreEndStraight, OffByDefault)
{
  EXPECT_EQ(CoreParams{}.end_straight_lateral, 0.0);
}

TEST(VccCoreEndStraight, NoWrongWayTurnBeforeArrival)
{
  EndSim off(paramsStraight(0.0), 0.25, -0.04, 0.6);
  off.run();
  EndSim on(paramsStraight(0.08), 0.25, -0.04, 0.6);
  on.run();
  ASSERT_GT(off.dist_at_align, 0.0);
  ASSERT_GT(on.dist_at_align, 0.0);
  EXPECT_LT(off.rot_before_align, -3.0 * M_PI / 180.0);
  EXPECT_NEAR(on.rot_before_align, 0.0, 0.5 * M_PI / 180.0);
  EXPECT_TRUE(on.straight_seen);
  EXPECT_FALSE(off.straight_seen);
  EXPECT_LT(on.dist_at_align, 0.12 + 1e-9);
}

TEST(VccCoreEndStraight, ArrivesAtTheLateralLimit)
{
  EndSim on(paramsStraight(0.08), 0.25, -0.08, 0.6);
  on.run();
  EXPECT_TRUE(on.straight_seen);
  EXPECT_GT(on.dist_at_align, 0.0);
  EXPECT_LT(on.dist_at_align, 0.12 + 1e-9);
  EXPECT_NEAR(on.rot_before_align, 0.0, 0.5 * M_PI / 180.0);
}

TEST(VccCoreEndStraight, FarSidewaysEndKeepsAiming)
{
  World w;
  VccCore off; off.configure(paramsStraight(0.0));
  VccCore on; on.configure(paramsStraight(0.08));
  Path p;
  for (double x = 0.05; x <= 0.25 + 1e-9; x += 0.05) {p.push_back({x, -0.12, 0.0});}
  CoreInputs in = inputs(p, w, 1.0, 0.1);
  in.goal = {0.25, -0.12, 0.6};
  in.xy_tol = 0.15;
  in.yaw_tol = 0.25;
  const CoreOutput o = off.step(in);
  const CoreOutput n = on.step(in);
  EXPECT_FALSE(n.end_straight);
  EXPECT_LT(n.cmd.w, -0.05);
  EXPECT_DOUBLE_EQ(n.cmd.w, o.cmd.w);
  EXPECT_DOUBLE_EQ(n.cmd.v, o.cmd.v);
}

TEST(VccCoreEndStraight, LongRemainingPathKeepsEndExtension)
{
  World w;
  VccCore off; off.configure(paramsStraight(0.0));
  VccCore on; on.configure(paramsStraight(0.08));
  const CoreOutput o = off.step(endInputs(w, 1.0, 0.1, 0.0, 0.45, -0.04, 0.6));
  const CoreOutput n = on.step(endInputs(w, 1.0, 0.1, 0.0, 0.45, -0.04, 0.6));
  EXPECT_FALSE(n.end_straight);
  EXPECT_DOUBLE_EQ(n.cmd.w, o.cmd.w);
  EXPECT_DOUBLE_EQ(n.cmd.v, o.cmd.v);
}

TEST(VccCoreEndStraight, EndBehindIsNotStraight)
{
  World w;
  VccCore on; on.configure(paramsStraight(0.08));
  Path p;
  for (double x = -0.23; x <= -0.13 + 1e-9; x += 0.05) {p.push_back({x, 0.0, 0.0});}
  p.back().yaw = 1.0;
  CoreInputs in = inputs(p, w, 1.0, 0.05);
  in.goal = {-0.13, 0.0, 1.0};
  in.xy_tol = 0.15;
  in.yaw_tol = 0.25;
  const CoreOutput n = on.step(in);
  EXPECT_FALSE(n.end_straight);
  EXPECT_EQ(n.state, State::Align);
}

TEST(VccCoreEndStraight, PositionOnlyArrivalIsUntouched)
{
  World w;
  VccCore off; off.configure(paramsStraight(0.0));
  VccCore on; on.configure(paramsStraight(0.08));
  Path p;
  for (double x = 0.05; x <= 0.25 + 1e-9; x += 0.05) {p.push_back({x, -0.04, 0.0});}
  CoreInputs in = inputs(p, w, 1.0, 0.1);
  in.goal = {0.25, -0.04, 0.6};
  in.xy_tol = 0.15;
  in.yaw_tol = 3.141;
  const CoreOutput o = off.step(in);
  const CoreOutput n = on.step(in);
  EXPECT_FALSE(n.end_straight);
  EXPECT_LT(n.cmd.w, -0.01);
  EXPECT_DOUBLE_EQ(n.cmd.w, o.cmd.w);
  EXPECT_DOUBLE_EQ(n.cmd.v, o.cmd.v);
}

namespace
{
double cruise(VccCore & c, const World & w, CoreInputs & in)
{
  double v = 0.0;
  for (int i = 0; i < 40; ++i) {
    in.now = 1.0 + 0.1 * i; in.measured.v = v;
    v = c.step(in).cmd.v;
  }
  return v;
}
}

TEST(VccCorePlannedDecel, SpeedCapDropIsSoft)
{
  CoreParams p = params(); p.output.planned_decel = 0.5;
  VccCore c; c.configure(p);
  World w;
  CoreInputs in = inputs(line(0.0, 0.0), w, 1.0, 0.0);
  const double v = cruise(c, w, in);
  ASSERT_NEAR(v, 0.5, 1e-9);
  in.speed_cap = 0.2; in.now += 0.1; in.measured.v = v;
  const CoreOutput out = c.step(in);
  EXPECT_EQ(out.state, State::Track);
  EXPECT_NEAR(out.cmd.v, 0.5 - 0.05, 1e-9);
}

TEST(VccCorePlannedDecel, BlockedHoldStillBrakesHard)
{
  CoreParams p = params(); p.output.planned_decel = 0.5;
  VccCore c; c.configure(p);
  World open, blocked;
  blocked.box(1.2, 1.3, -2.5, 2.5);
  CoreInputs in = inputs(line(0.0, 0.0), open, 1.0, 0.0);
  const double v = cruise(c, open, in);
  ASSERT_NEAR(v, 0.5, 1e-9);
  in.clearance = blocked.fn(); in.now += 0.1; in.measured.v = v;
  const CoreOutput out = c.step(in);
  EXPECT_EQ(out.state, State::Hold);
  EXPECT_NEAR(out.cmd.v, 0.5 - 0.125, 1e-9);
}

TEST(VccCorePositionOnly, StopsStraightAtTheCheckerCircle)
{
  World w;
  auto make = [&](double yaw_tol) {
      CoreInputs in = inputs(line(0.0, 0.0, 0.14), w, 1.0, 0.17);
      in.path.back().y = 0.04;
      in.goal = {0.135, 0.04, 2.0};
      in.xy_tol = 0.15; in.yaw_tol = yaw_tol;
      return in;
    };
  VccCore pos; pos.configure(params());
  const CoreOutput p = pos.step(make(3.141));
  EXPECT_EQ(p.state, State::Track);
  EXPECT_LT(p.cmd.v, 0.17);
  EXPECT_NEAR(p.cmd.w, 0.0, 1e-9);
  VccCore yaw; yaw.configure(params());
  const CoreOutput y = yaw.step(make(0.25));
  EXPECT_EQ(y.state, State::Track);
  EXPECT_GT(y.cmd.v, p.cmd.v);
  EXPECT_GT(std::abs(y.cmd.w), 0.0);
}

TEST(VccCorePositionOnly, OutsideTheCircleStillApproaches)
{
  World w;
  VccCore c; c.configure(params());
  CoreInputs in = inputs(line(0.0, 0.0, 0.3), w, 1.0, 0.1);
  in.goal = {0.3, 0.0, 2.0};
  in.xy_tol = 0.15; in.yaw_tol = 3.141;
  EXPECT_GT(c.step(in).cmd.v, 0.0);
}

TEST(VccCorePositionOnly, ZeroDisablesIt)
{
  World w;
  CoreParams p = params(); p.position_only_yaw_tol = 0.0;
  VccCore c; c.configure(p);
  CoreInputs in = inputs(line(0.0, 0.0, 0.14), w, 1.0, 0.1);
  in.goal = {0.14, 0.0, 0.0};
  in.xy_tol = 0.15; in.yaw_tol = 3.141;
  EXPECT_GT(c.step(in).cmd.v, 0.0);
}

namespace
{
struct SideWall
{
  World w;
  SideWall() {w.box(-2.5, 2.4, -0.75, -0.65);}
};

CoreParams confirmParams(bool on)
{
  CoreParams p = params();
  p.new_path_confirm = on;
  return p;
}

double cruise(VccCore & c, const World & w)
{
  double t = 1.0;
  for (int i = 0; i < 3; ++i, t += 0.1) {c.step(inputs(line(0.0, 0.0), w, t, 0.4));}
  return t;
}
}

TEST(VccCore, StaleLaneOnNewPathStopsWithoutConfirm)
{
  VccCore c; c.configure(confirmParams(false));
  SideWall s;
  const double t = cruise(c, s.w);
  c.onNewPath();
  const CoreOutput out = c.step(inputs(line(-0.4, 0.0), s.w, t, 0.4));
  EXPECT_EQ(out.failure, Failure::CollisionAhead);
}

TEST(VccCore, StaleLaneOnNewPathIsConfirmedAndPasses)
{
  VccCore c; c.configure(confirmParams(true));
  SideWall s;
  double t = cruise(c, s.w);
  c.onNewPath();
  CoreOutput out = c.step(inputs(line(-0.4, 0.0), s.w, t, 0.4));
  EXPECT_EQ(out.failure, Failure::None);
  EXPECT_TRUE(out.collision_deferred);
  EXPECT_GT(out.cmd.v, 0.0);
  out = c.step(inputs(line(-0.4, 0.0), s.w, t + 0.1, 0.4));
  EXPECT_EQ(out.failure, Failure::None);
  EXPECT_FALSE(out.collision_deferred);
}

TEST(VccCore, CollisionStillThereOnSecondCycleStops)
{
  CoreParams p = confirmParams(true);
  p.resync_offset = 10.0;
  VccCore c; c.configure(p);
  SideWall s;
  double t = cruise(c, s.w);
  c.onNewPath();
  CoreOutput out = c.step(inputs(line(-0.4, 0.0), s.w, t, 0.4));
  EXPECT_TRUE(out.collision_deferred);
  c.onNewPath();
  out = c.step(inputs(line(-0.4, 0.0), s.w, t + 0.1, 0.4));
  EXPECT_FALSE(out.collision_deferred);
  EXPECT_EQ(out.state, State::Hold);
  EXPECT_EQ(out.cmd.v, 0.0);
}

TEST(VccCore, WallStraightAheadOnNewPathStillStopsAtOnce)
{
  VccCore c; c.configure(confirmParams(true));
  World w;
  w.box(0.35, 0.40, -2.5, 2.5);
  c.onNewPath();
  const CoreOutput out = c.step(inputs(line(0.0, 0.0), w, 1.0, 0.4));
  EXPECT_EQ(out.state, State::Hold);
  EXPECT_EQ(out.cmd.v, 0.0);
  EXPECT_EQ(out.cmd.w, 0.0);
}

TEST(VccCore, OldPathIsNotDeferred)
{
  CoreParams p = confirmParams(true);
  p.resync_offset = 10.0;
  VccCore c; c.configure(p);
  SideWall s;
  const double t = cruise(c, s.w);
  const CoreOutput out = c.step(inputs(line(-0.4, 0.0), s.w, t, 0.4));
  EXPECT_EQ(out.failure, Failure::CollisionAhead);
  EXPECT_FALSE(out.collision_deferred);
}
