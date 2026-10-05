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
  out = at(1.6, 0.6, -0.3);   // 끝점 밖으로 밀림(align_exit_dist 0.5 너머)
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

TEST(VccCore, FarFirstPointOnLineThroughRobotDoesNotResync)
{
  // run48 F2: 경로 첫 점이 로봇 0.2~0.74 m 앞이고 그 점의 방향이 앞 구간과 달라(코너 노드) 옆 오차를
  // 잘못 쟀다. 옆 오차는 첫 구간(p0->p1)을 뒤로 늘인 직선까지의 수직 거리다. 로봇을 지나는 30° 직선이면 0.
  VccCore c; c.configure(params());
  World w;
  const double a = 30.0 * M_PI / 180.0;
  Path p;
  for (double s = 0.6; s <= 3.0 + 1e-9; s += 0.05) {p.push_back({s * std::cos(a), s * std::sin(a), a});}
  p.front().yaw = 0.0;   // 앞 구간 방향이 남은 첫 점
  for (int i = 0; i < 3; ++i) {
    const CoreOutput out = c.step(inputs(p, w, 1.0 + 0.1 * i, 0.3));
    EXPECT_NEAR(out.offset, 0.0, 0.02) << i;
  }
}

TEST(VccCore, OneCycleLateralGlitchDoesNotResync)
{
  // run48 F2: 가짜 되튐 75회 중 64 % 가 새 경로 뒤 0.25 s 안. 한 주기만 튄 오차로는 d 를 옮기지 않는다.
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
  EXPECT_NEAR(out.offset, 0.0, 0.02);   // 1주기: 아직
  out = c.step(inputs(line(-0.4, 0.0), w, t + 0.1, 0.3));
  EXPECT_NEAR(out.offset, 0.4, 0.011);  // 2주기 연속: 다시 맞춘다
}

TEST(VccCore, TurnExitKeepsNearestLaneAsTarget)
{
  // run48 F2: 유턴을 마치고 레일 왼쪽 0.4 m 에 나오면 목표 차선도 그 자리(0.4)로 둔다. 목표가 0 이면
  // 나오자마자 레일로 되튄다. 레일 복귀는 보통 규칙(return_clear_time)이 맡는다.
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
  // 멈춤 반경은 checker 원(xy_tol)보다 arrive_margin 안쪽이다. 경계 0.14 에서는 아직 다가간다 —
  // 그래야 checker 가 움직이는 동안 먼저 도장을 찍는다(run49 교착의 한 갈래).
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
  EXPECT_LT(out.cmd.v, 0.05);   // 0.12 안 — 목표 0, 출력단이 줄여 간다
}

TEST(VccCore, AlignFailureRearmsAfterRearmTime)
{
  // run49 409호: 정렬 3번을 다 쓰고 Failed 가 되자 같은 goal 을 BT 가 바로 다시 보내 17 s 동안
  // Hold(예외 172회). 이제 align_rearm_time(3 s) 뒤 횟수를 지우고 다시 돈다.
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
  // 돌다가 반대쪽으로 넘침(-0.5) -> 끊고 멈춤 확인 -> 여전히 어긋나 횟수(1)를 다 써 Failed.
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
  EXPECT_LT(out.cmd.w, 0.0);   // 새 기회로 남은 쪽(-0.5)을 향해 돈다
}

// ── 2026-10-05 해결안 C: 유턴(Turn)의 끝을 정한다 ──────────────────────────────────────────
namespace
{
// 로봇 좌표계에서 rel 방향으로 곧게 뻗은 3 m 경로(경로가 매 주기 로봇 위치에서 다시 그려지는 상황).
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
}  // namespace

// run60 사람 접근(73 s·5바퀴)·run63 시작→창구(28 s·687°) 의 꼬리 잡기: 조준점이 늘 로봇 왼쪽 100° 에
// 있으면(경로가 매 주기 로봇 자리에서 다시 그려져 로봇과 함께 돈다) 예전 Turn 은 끝나지 않는다.
TEST(VccCoreTurnEnd, OldTurnChasesItsTailForever)
{
  Sim s; s.c.configure(paramsC(false, 0.0, 0.0));
  for (int i = 0; i < 200; ++i) {s.step(100.0 * M_PI / 180.0);}
  EXPECT_FALSE(s.left_first_turn);
  EXPECT_GT(s.rot_in_first_turn, 2.0 * M_PI);   // 20 s 동안 한 바퀴 넘게
}

TEST(VccCoreTurnEnd, LockedTargetEndsTheTurnAtTheEntryAim)
{
  Sim s; s.c.configure(paramsC(true, 0.0, 0.0));
  for (int i = 0; i < 200 && !s.left_first_turn; ++i) {s.step(100.0 * M_PI / 180.0);}
  ASSERT_TRUE(s.left_first_turn);
  // 들어갈 때 조준 100°, 끝 문턱 25° -> 약 75° 돌고 끝(지연·최소 유지 0.5 s 로 조금 더).
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

// 경로가 정말 바뀌면(지도 기준 조준 방향이 90° 넘게, 0.5 s 넘게) 다시 고른다 — 방향도 가까운 쪽으로.
TEST(VccCoreTurnEnd, RetargetsOnlyWhenTheNewAimPersists)
{
  Sim s; s.c.configure(paramsC(true, 0.0, 0.0));
  const double north = 150.0 * M_PI / 180.0, south_east = -60.0 * M_PI / 180.0;   // 둘은 150° 떨어져 있다
  CoreOutput out;
  for (int i = 0; i < 5; ++i) {out = s.step(normalizeAngle(north - s.th));}
  ASSERT_EQ(out.state, State::Turn);
  ASSERT_GT(out.cmd.w, 0.0);                       // 왼쪽으로 150° 를 향해
  // 0.3 s 만 다른 곳(-60°)을 가리켰다 돌아오면 방향을 안 바꾼다
  for (int i = 0; i < 3; ++i) {out = s.step(normalizeAngle(south_east - s.th)); EXPECT_GT(out.cmd.w, 0.0) << i;}
  for (int i = 0; i < 3; ++i) {out = s.step(normalizeAngle(north - s.th));}
  EXPECT_GT(out.cmd.w, 0.0);
  // 계속 -60° 를 가리키면 0.5 s 뒤 다시 고른다 — 지금 방향(약 +30°)에서 -60° 는 오른쪽이 가깝다
  bool flipped = false;
  for (int i = 0; i < 12; ++i) {out = s.step(normalizeAngle(south_east - s.th)); if (out.cmd.w < 0.0) {flipped = true;}}
  EXPECT_TRUE(flipped);
}

// 진입 지속: 경로가 바뀌는 한 주기만 튄 조준각으로는 Turn 에 안 들어간다.
TEST(VccCoreTurnEnd, EntryNeedsPersistentHeadingError)
{
  Sim s; s.c.configure(paramsC(true, 0.0, 0.3));
  CoreOutput out = s.step(0.0);
  out = s.step(170.0 * M_PI / 180.0);             // 한 주기 뒤쪽
  EXPECT_NE(out.state, State::Turn);
  out = s.step(0.0);
  EXPECT_NE(out.state, State::Turn);
  for (int i = 0; i < 6; ++i) {out = s.step(normalizeAngle(170.0 * M_PI / 180.0));}
  EXPECT_EQ(out.state, State::Turn);              // 0.3 s 넘게 이어지면 들어간다
}
