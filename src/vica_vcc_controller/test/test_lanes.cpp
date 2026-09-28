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

// v = 실측 속도, v_des = 차선 제한 전 목표 속도(min(desired, speed_cap))
void run(
  LaneSelector & ls, const World & w, int cycles, double & now, double v = 0.4, double v_des = 0.5)
{
  for (int i = 0; i < cycles; ++i) {now += 0.1; ls.update(rail(), v, v_des, now, 0.1, w.fn());}
}
}  // namespace

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
  // 10 cm 기둥이 레일 1.5 m 앞(avoid_horizon 끝에서 처음 보인 상황).
  // 필요한 옆 간격 = 0.05 + 0.275 + 0.20 = 0.525 -> 0.6 차선. 0.4 m/s 로는 못 옮겨 0.2 m/s 로 줄인다.
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
  // 레일 차선이 '막히지는 않고 20 cm 가 모자란' 경우에만 줏대 규칙이 걸린다(막히면 즉시 옮긴다).
  // 물체를 레일 왼쪽 0.3~0.4 m 에 둔다: 레일 차선 여유 0.025 m -> 오른쪽 -0.2 차선이 가장 낫다.
  World w;
  w.box(1.5, 1.6, 0.3, 0.4);
  LaneSelector ls;
  double now = 0.0;
  run(ls, w, 2, now);
  EXPECT_NEAR(ls.target(), 0.0, 1e-9);   // 0.2 s 로는 아직
  run(ls, w, 1, now);
  EXPECT_NEAR(ls.target(), -0.2, 1e-9);  // 3주기(0.3 s)에 전환
}

TEST(Lanes, OffsetMovesAtLaneRate)
{
  // 옮김 속도(0.2)로 달리면 옆 이동은 정확히 lane_rate: 0.10 m/s x 0.1 s = 0.01 m
  World w;
  w.box(1.5, 1.6, -0.05, 0.05);
  LaneSelector ls;
  double now = 0.0;
  run(ls, w, 3, now, 0.2);
  const double before = ls.offset();
  run(ls, w, 1, now, 0.2);
  EXPECT_NEAR(std::abs(ls.offset() - before), 0.01, 1e-9);
  // 멈춰 있으면 옆으로 미끄러지지 않는다
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
  run(ls, empty, 9, now);                 // 0.9 s
  EXPECT_NE(ls.target(), 0.0);
  run(ls, empty, 2, now);                 // 1.1 s
  EXPECT_NEAR(ls.target(), 0.0, 1e-9);
}

TEST(Lanes, NearlyEqualLanesDoNotFlipFlop)
{
  // 두 차선이 거의 같은 점수로 매 주기 번갈아 1등 — MPPI 식 비틀거림 방지(switch_margin)
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
  // 요구 1: 3~4 m 앞 사람에 반응하지 않는다. 2.2 m 앞 물체도 horizon 1.5 밖이다.
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
  w.box(1.5, 1.6, -0.05, 0.05);   // 좌우 대칭
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
  EXPECT_NEAR(ls.target(), 0.0, 1e-9);   // 목표는 그대로 — 복귀 규칙이 되돌린다
  ls.syncOffset(-0.9);
  EXPECT_NEAR(ls.offset(), -0.6, 1e-9);
}

TEST(Lanes, ShiftSpeedStartsFromDesiredNotMeasured)
{
  // run48 F1: 유턴 뒤 0.09 m/s 에서 옮김 속도 후보가 실측(0.1)부터 시작해 속도 상한이 0.1 에 묶였다
  // (TRACK 의 31 %). 빈 복도라면 목표 속도로 옮겨도 20 cm 가 나오므로 상한이 없어야 한다.
  World w;
  LaneSelector ls;
  ls.syncOffset(0.4);   // 유턴을 마치고 레일 왼쪽 0.4 m, 목표는 레일
  double now = 0.0;
  run(ls, w, 1, now, 0.1, 0.5);
  ASSERT_GT(std::abs(ls.target() - ls.offset()), 0.01);   // 옮기는 중
  EXPECT_GE(ls.speedCap(), 0.5);
}

TEST(Lanes, PoleOnRailAtLowMeasuredSpeedStillShiftsAtTwo)
{
  // 느리게 달리던 중(0.1)이라도 기둥을 20 cm 로 비키는 가장 빠른 옮김 속도(0.2)를 고른다.
  World w;
  w.box(1.5, 1.6, -0.05, 0.05);
  LaneSelector ls;
  double now = 0.0;
  run(ls, w, 3, now, 0.1, 0.5);
  EXPECT_NEAR(std::abs(ls.target()), 0.6, 1e-9);
  EXPECT_NEAR(ls.speedCap(), 0.2, 1e-9);
}
