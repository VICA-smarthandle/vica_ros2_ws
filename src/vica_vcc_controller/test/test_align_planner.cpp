#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include "vica_vcc_controller/core/align_planner.hpp"

using namespace vica_vcc_controller::core;

namespace
{
struct Result { AlignPhase phase; int attempts; double final_err; double seconds; };

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
}

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
  AlignParams ap;
  ap.motor_lag = 0.0;
  const Result r = simulate(M_PI / 2, 0.5, 0.1, ap);
  EXPECT_EQ(r.phase, AlignPhase::Done);
  EXPECT_EQ(r.attempts, 2);
  EXPECT_LE(std::abs(r.final_err), 0.1);
}

TEST(AlignPlanner, WorstCaseFinishesInsideProgressCheckerWindow)
{
  const Result r = simulate(M_PI, 0.5, 0.001);
  EXPECT_EQ(r.phase, AlignPhase::Failed);
  EXPECT_EQ(r.attempts, 3);
  EXPECT_LE(r.seconds, 15.0);
}

TEST(AlignPlanner, StoppedVelocityFollowsGoalChecker)
{
  AlignPlanner a;
  a.setStoppedVelocity(0.2);
  EXPECT_GT(a.update(1.0, 0.0, 0.25, 0.0, 0.1), 0.0);
  a.update(0.01, 0.1, 0.25, 0.1, 0.1);
  ASSERT_EQ(a.phase(), AlignPhase::Settling);
  a.update(0.01, 0.1, 0.25, 0.2, 0.1);
  a.update(0.01, 0.1, 0.25, 0.6, 0.1);
  EXPECT_EQ(a.phase(), AlignPhase::Done);
}

TEST(AlignPlanner, FailsAfterMaxAttempts)
{
  const Result r = simulate(M_PI / 2, 0.35, 0.001);
  EXPECT_EQ(r.phase, AlignPhase::Failed);
  EXPECT_EQ(r.attempts, 3);
}
