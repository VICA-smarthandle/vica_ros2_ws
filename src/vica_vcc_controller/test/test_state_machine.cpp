#include <gtest/gtest.h>
#include <cmath>
#include "vica_vcc_controller/core/state_machine.hpp"

using namespace vica_vcc_controller::core;

namespace
{
constexpr double kDeg = M_PI / 180.0;
StateInputs base(double now)
{
  StateInputs in;
  in.now = now;
  in.dist_to_end = 5.0;
  return in;
}
}  // namespace

TEST(StateMachine, TransitionTableEveryCell)
{
  const State all[] = {State::Track, State::Turn, State::Align, State::Hold};
  // 허용: 설계서 6.1 표
  const bool allowed[4][4] = {
    //            Track  Turn   Align  Hold
    /* Track */ {false, true,  true,  true},
    /* Turn  */ {true,  false, true,  true},
    /* Align */ {true,  false, false, true},
    /* Hold  */ {true,  false, false, false},
  };
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      EXPECT_EQ(transitionAllowed(all[i], all[j]), allowed[i][j]) << i << "->" << j;
    }
  }
}

TEST(StateMachine, MovingUturnEntersAboveSixtyDegrees)
{
  StateMachine sm;
  StateInputs in = base(1.0);
  in.heading_error = 70 * kDeg; in.path_heading_error = 170 * kDeg;
  EXPECT_EQ(sm.update(in), State::Turn);
  StateMachine sm2;
  in.heading_error = 55 * kDeg;
  EXPECT_EQ(sm2.update(in), State::Track);
}

TEST(StateMachine, ParallelOffsetRailIsNotAUturn)
{
  // Review Focus 1: 레일과 나란히 0.55 m 떨어짐 -> 조준점 66도, 레일 방향 0도
  StateMachine sm;
  StateInputs in = base(1.0);
  in.heading_error = -66 * kDeg; in.path_heading_error = 0.0;
  EXPECT_EQ(sm.update(in), State::Track);
}

TEST(StateMachine, StationaryPivotFromThirtyFiveDegrees)
{
  StateMachine sm;
  StateInputs in = base(1.0);
  in.stationary = true; in.heading_error = 45 * kDeg; in.path_heading_error = 45 * kDeg;
  EXPECT_EQ(sm.update(in), State::Turn);
}

TEST(StateMachine, TurnHasHysteresis)
{
  StateMachine sm;
  StateInputs in = base(1.0);
  in.heading_error = 90 * kDeg; in.path_heading_error = 180 * kDeg;
  ASSERT_EQ(sm.update(in), State::Turn);
  in.now = 2.0; in.heading_error = 40 * kDeg;
  EXPECT_EQ(sm.update(in), State::Turn);    // 25도 밑이 아니면 유지
  in.now = 2.1; in.heading_error = 20 * kDeg;
  EXPECT_EQ(sm.update(in), State::Track);
}

TEST(StateMachine, MinimumStateTimeBlocksQuickFlip)
{
  StateMachine sm;
  StateInputs in = base(1.0);
  in.heading_error = 90 * kDeg; in.path_heading_error = 180 * kDeg;
  ASSERT_EQ(sm.update(in), State::Turn);
  in.now = 1.3; in.heading_error = 10 * kDeg;
  EXPECT_EQ(sm.update(in), State::Turn);    // 0.3 s < 0.5 s
  in.now = 1.6;
  EXPECT_EQ(sm.update(in), State::Track);
}

TEST(StateMachine, SafetyStopIsImmediateFromAnyState)
{
  StateMachine sm;
  StateInputs in = base(1.0);
  in.heading_error = 90 * kDeg; in.path_heading_error = 180 * kDeg;
  ASSERT_EQ(sm.update(in), State::Turn);
  in.now = 1.05; in.collision_imminent = true;
  EXPECT_EQ(sm.update(in), State::Hold);    // 최소 유지 시간 무시
}

TEST(StateMachine, ArrivalBeatsUturnAndCannotTurnAfterwards)
{
  StateMachine sm;
  StateInputs in = base(1.0);
  in.dist_to_end = 0.1; in.yaw_error_end = 120 * kDeg;
  in.heading_error = 170 * kDeg; in.path_heading_error = 170 * kDeg;
  EXPECT_EQ(sm.update(in), State::Align);
  in.now = 5.0;
  EXPECT_EQ(sm.update(in), State::Align);   // Align -> Turn 금지
}

TEST(StateMachine, AlignExitNeedsMargin)
{
  StateMachine sm;
  StateInputs in = base(1.0);
  in.dist_to_end = 0.1; in.yaw_error_end = 1.0;
  ASSERT_EQ(sm.update(in), State::Align);
  in.now = 2.0; in.dist_to_end = 0.30;
  EXPECT_EQ(sm.update(in), State::Align);   // 0.25 + 0.10 안
  in.now = 2.1; in.dist_to_end = 0.36;
  EXPECT_EQ(sm.update(in), State::Track);
}

TEST(StateMachine, AlignFailureHoldsUntilReset)
{
  StateMachine sm;
  StateInputs in = base(1.0);
  in.dist_to_end = 0.1; in.yaw_error_end = 1.0;
  ASSERT_EQ(sm.update(in), State::Align);
  in.now = 2.0; in.align_failed = true;
  EXPECT_EQ(sm.update(in), State::Hold);
  in.now = 5.0;
  EXPECT_EQ(sm.update(in), State::Hold);
  sm.reset();
  EXPECT_EQ(sm.state(), State::Track);
}

TEST(StateMachine, HoldReleasesWhenPathOpens)
{
  StateMachine sm;
  StateInputs in = base(1.0);
  in.lanes_blocked = true;
  ASSERT_EQ(sm.update(in), State::Hold);
  in.now = 1.2; in.lanes_blocked = false;
  EXPECT_EQ(sm.update(in), State::Hold);    // 최소 유지
  in.now = 1.6;
  EXPECT_EQ(sm.update(in), State::Track);
}

TEST(StateMachine, BlockedTurnGoesToHold)
{
  StateMachine sm;
  StateInputs in = base(1.0);
  in.heading_error = 90 * kDeg; in.path_heading_error = 180 * kDeg; in.turn_blocked = true;
  EXPECT_EQ(sm.update(in), State::Hold);
}
