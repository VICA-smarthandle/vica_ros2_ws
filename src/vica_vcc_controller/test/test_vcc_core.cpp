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

TEST(VccCore, BlockedUturnInNarrowCorridorStaysInHoldWithoutCreeping)
{
  // 최종 리뷰 I2: 0.80 m 통로에서 레일이 뒤로 — 유턴(제자리 1.25 m 필요) 불가.
  // Hold <-> Track 을 오가며 앞으로 기어가면 안 되고, 멈춘 뒤 매 주기 Blocked 여야 한다.
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
  // 최종 리뷰 I3: 레일 BT 당근 모드가 경로 끝을 ~1 Hz 로 옮겨 새 goal 판정이 반복된다.
  // 새 goal 은 도착 정렬만 초기화하고 속도(출력단)·차선은 이어 간다.
  VccCore a; a.configure(params());
  VccCore b; b.configure(params());
  World w;
  w.box(1.2, 1.3, -0.05, 0.05);   // 레일 위 기둥 -> 차선이 옮겨 간다
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
  // 최종 리뷰 I4: 차선 경로(직선)로는 비어 있어도, 실제로 나가는 (v, w) 호 위 벽을 잡는다.
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
  // 제자리 회전: 각 = |w|·(|w|/각가속 + 지연). 오른쪽 뒤 모서리(반지름 0.58)가 도는 쪽 벽을 쓸고 간다.
  World w;
  w.box(-0.75, -0.30, -0.40, -0.30);
  const ClearanceFn f = w.fn();
  ASSERT_GE(f({0.0, 0.0, 0.0}), 0.0);
  EXPECT_TRUE(motionCollides({0.0, 0.35}, f, 1.25, 1.2, 0.3));
  EXPECT_FALSE(motionCollides({0.0, -0.35}, f, 1.25, 1.2, 0.3));
}

TEST(VccCore, OffsetResyncsToRobotAfterUturnExit)
{
  // 최종 리뷰 I4: 유턴을 마치고 레일에서 0.4 m 왼쪽에 나왔다. 차선 d 가 실제 위치로 다시 맞춰지고
  // 차선 경로가 로봇에서 시작해야 한다(d = 0 이면 차선 검사가 로봇이 아닌 레일 위를 본다).
  VccCore c; c.configure(params());
  World w;
  const CoreOutput out = c.step(inputs(line(-0.4, 0.0), w, 1.0, 0.1));
  EXPECT_NEAR(out.offset, 0.4, 0.011);
  ASSERT_FALSE(out.lane_path.empty());
  EXPECT_NEAR(out.lane_path.front().y, 0.0, 0.011);
}

TEST(VccCore, BlockedAlignSweepHoldsWithoutRotating)
{
  // 최종 리뷰 I5: 150° 를 왼쪽으로 돌면 오른쪽 뒤 모서리(반지름 0.58)가 y = -0.58 까지 쓸고 간다.
  // 그 안쪽 0.13 m 에 벽이 있으면 돌기 시작하지 않고 Hold, 멈춘 뒤 Blocked.
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
  // 최종 리뷰 M2: 조준점이 ±170° 사이를 오가도 Turn 중 방향은 들어갈 때 고른 쪽으로 고정.
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
  // 최종 리뷰 M3: 같은 goal 안에서 Align -> Track(밀림) -> Align 이어도 도착 횟수는 이어 센다.
  VccCore c; c.configure(params());
  World w;
  auto at = [&](double now, double dist, double yaw) {
      CoreInputs in = inputs(line(0.0, 0.0, dist), w, now, 0.0);
      in.goal = {dist, 0.0, yaw};
      return c.step(in);
    };
  CoreOutput out = at(1.0, 0.1, 0.3);
  ASSERT_EQ(out.state, State::Align);
  out = at(1.1, 0.1, -0.3);   // 넘침 -> 끊고 멈춤 확인
  for (double t = 1.2; t < 1.55; t += 0.1) {out = at(t, 0.1, -0.3);}
  ASSERT_EQ(out.align_attempts, 2);
  out = at(1.6, 0.5, -0.3);   // 끝점 밖으로 밀림
  ASSERT_EQ(out.state, State::Track);
  out = at(1.7, 0.1, -0.3);
  ASSERT_EQ(out.state, State::Align);
  EXPECT_EQ(out.align_attempts, 2);
}

TEST(VccCore, AcceleratesWhileReturningToRailAfterUturn)
{
  // run48 F1: 유턴을 마치고 레일 왼쪽 0.4 m 에서 0.1 m/s. 빈 복도에서 레일로 돌아가는 동안에도
  // 속도를 올려야 한다(옛 코드는 옮김 속도 = 실측 0.1 에 묶여 2~10 s 를 0.1 m/s 로 달렸다).
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
