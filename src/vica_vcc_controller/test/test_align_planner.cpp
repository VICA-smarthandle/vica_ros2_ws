#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include "vica_vcc_controller/core/align_planner.hpp"

using namespace vica_vcc_controller::core;

namespace
{
struct Result { AlignPhase phase; int attempts; double final_err; double seconds; };

// 모터 모형: 명령이 delay 만큼 늦게 도착하고, 각가속 2.0 rad/s^2 로 따라간다(run41 지연 0.3~0.5 s).
Result simulate(double err0, double delay, double tol = 0.25, AlignParams ap = {})
{
  AlignPlanner a(ap);
  const double dt = 0.1;
  std::deque<double> q(static_cast<size_t>(std::round(delay / dt)), 0.0);
  double yaw = 0.0, w_act = 0.0;
  for (int k = 0; k < 600; ++k) {
    const double t = k * dt;
    const double cmd = a.update(err0 - yaw, w_act, tol, t, dt);
    q.push_back(cmd);
    const double c = q.front();
    q.pop_front();
    w_act += std::clamp(c - w_act, -2.0 * dt, 2.0 * dt);
    yaw += w_act * dt;
    if ((a.phase() == AlignPhase::Done || a.phase() == AlignPhase::Failed) && std::abs(w_act) < 1e-3) {
      return {a.phase(), a.attempts(), err0 - yaw, t};
    }
  }
  return {a.phase(), a.attempts(), err0 - yaw, 60.0};
}
}  // namespace

TEST(AlignPlanner, OneShotForTypicalErrorsAndLags)
{
  for (double deg : {20.0, 45.0, 90.0, 145.0, -100.0, 180.0}) {
    for (double delay : {0.3, 0.35, 0.5}) {
      const Result r = simulate(deg * M_PI / 180.0, delay);
      EXPECT_EQ(r.phase, AlignPhase::Done) << deg << " " << delay;
      EXPECT_EQ(r.attempts, 1) << deg << " " << delay;
      EXPECT_LE(std::abs(r.final_err), 6.0 * M_PI / 180.0) << deg << " " << delay;
    }
  }
}

TEST(AlignPlanner, AlreadyAlignedDoesNothing)
{
  AlignPlanner a;
  EXPECT_EQ(a.update(0.1, 0.0, 0.25, 0.0, 0.1), 0.0);
  EXPECT_EQ(a.phase(), AlignPhase::Done);
  EXPECT_EQ(a.attempts(), 0);
}

TEST(AlignPlanner, LearnsLagAndCorrectsOnSecondAttempt)
{
  // 추정 지연 0(틀림), 실제 0.5 s, 허용오차 0.1 rad -> 첫 회 넘침, 배운 지연으로 2회째에 맞춘다.
  AlignParams ap;
  ap.motor_lag = 0.0;
  const Result r = simulate(M_PI / 2, 0.5, 0.1, ap);
  EXPECT_EQ(r.phase, AlignPhase::Done);
  EXPECT_EQ(r.attempts, 2);
  EXPECT_LE(std::abs(r.final_err), 0.1);
}

TEST(AlignPlanner, WorstCaseFinishesInsideProgressCheckerWindow)
{
  // 진행 감시(SimpleProgressChecker movement_time_allowance 20 s)는 제자리 회전을 진행으로 세지 않는다.
  // 180도 + 보정 3회를 다 써도 그 안에 끝나야 한다. 도착 직전 감속 시간을 위해 5 s 를 남긴다.
  const Result r = simulate(M_PI, 0.5, 0.001);
  EXPECT_EQ(r.phase, AlignPhase::Failed);
  EXPECT_EQ(r.attempts, 3);
  EXPECT_LE(r.seconds, 15.0);
}

TEST(AlignPlanner, StoppedVelocityFollowsGoalChecker)
{
  // 정지 기준이 0.2 로 느슨하면, 0.1 rad/s 로 도는 중에도 '멈췄다'로 보고 확인 단계로 넘어간다.
  AlignPlanner a;
  a.setStoppedVelocity(0.2);
  EXPECT_GT(a.update(1.0, 0.0, 0.25, 0.0, 0.1), 0.0);   // 회전 시작
  // 이미 끊는 지점이면 Settling 으로 간다
  a.update(0.01, 0.1, 0.25, 0.1, 0.1);
  ASSERT_EQ(a.phase(), AlignPhase::Settling);
  a.update(0.01, 0.1, 0.25, 0.2, 0.1);
  a.update(0.01, 0.1, 0.25, 0.6, 0.1);
  EXPECT_EQ(a.phase(), AlignPhase::Done);      // 0.1 < 0.2 라 정지로 인정
}

TEST(AlignPlanner, FailsAfterMaxAttempts)
{
  // 허용오차를 비현실적으로 좁혀 매번 실패하게 한다.
  const Result r = simulate(M_PI / 2, 0.35, 0.001);
  EXPECT_EQ(r.phase, AlignPhase::Failed);
  EXPECT_EQ(r.attempts, 3);
}
