#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "vica_vcc_controller/core/output_stage.hpp"

using namespace vica_vcc_controller::core;

TEST(OutputStage, StartRampHalfSpeedThenSlow)
{
  OutputStage o;
  double t = 0.0, t_half = -1.0, t_full = -1.0;
  for (int i = 0; i < 60; ++i) {
    t += 0.1;
    const Twist2D c = o.apply(Desired{{0.5, 0.0}}, o.last().v, 0.1);
    if (t_half < 0 && c.v >= 0.25 - 1e-9) {t_half = t;}
    if (t_full < 0 && c.v >= 0.5 - 1e-9) {t_full = t;}
  }
  EXPECT_GE(t_half, 0.4);
  EXPECT_LE(t_half, 0.6);
  EXPECT_GE(t_full - t_half, 1.5);
  EXPECT_LE(t_full - t_half, 2.0);
}

TEST(OutputStage, BrakingIsNeverSoftened)
{
  OutputStage o;
  for (int i = 0; i < 60; ++i) {o.apply(Desired{{0.5, 0.0}}, o.last().v, 0.1);}
  const Twist2D c = o.apply(Desired{{0.0, 0.0}}, 0.5, 0.1);
  EXPECT_NEAR(c.v, 0.5 - 0.125, 1e-9);
}

TEST(OutputStage, ResyncsToMeasuredSpeedAfterExternalStop)
{
  OutputStage o;
  for (int i = 0; i < 60; ++i) {o.apply(Desired{{0.5, 0.0}}, o.last().v, 0.1);}
  const Twist2D c = o.apply(Desired{{0.5, 0.0}}, 0.0, 0.1);
  EXPECT_LE(c.v, OutputParams{}.resync_margin + 0.05 + 1e-9);
}

TEST(OutputStage, NoReverseAndSymmetricTurnLimit)
{
  OutputStage o;
  EXPECT_GE(o.apply(Desired{{-0.3, 0.0}}, 0.0, 0.1).v, 0.0);
  OutputStage a, b;
  for (int i = 0; i < 20; ++i) {a.apply(Desired{{0.0, 3.0}}, 0.0, 0.1); b.apply(Desired{{0.0, -3.0}}, 0.0, 0.1);}
  EXPECT_NEAR(a.last().w, 0.5, 1e-9);
  EXPECT_NEAR(b.last().w, -0.5, 1e-9);
}

TEST(OutputStage, AngularAccelIsLimited)
{
  OutputStage o;
  EXPECT_NEAR(o.apply(Desired{{0.0, 0.5}}, 0.0, 0.1).w, 0.12, 1e-9);
}

TEST(OutputStage, CurvatureIsKeptWhileAccelerating)
{
  OutputStage o;
  Desired d{{0.5, 0.5}};
  d.curvature = 1.0;
  const Twist2D c = o.apply(d, 0.0, 0.1);
  EXPECT_NEAR(c.w, c.v * 1.0, 1e-9);
}

TEST(OutputStage, ArcRadiusIsKept)
{
  OutputStage o;
  Desired d{{0.5, 0.45}};
  d.radius = 0.2;
  for (int i = 0; i < 10; ++i) {o.apply(d, o.last().v, 0.1);}
  EXPECT_NEAR(o.last().v, std::abs(o.last().w) * 0.2, 1e-9);
}

TEST(OutputStage, ResetStartsFromMeasuredTwist)
{
  OutputStage o;
  o.reset({0.4, 0.1});
  EXPECT_NEAR(o.last().v, 0.4, 1e-9);
  EXPECT_NEAR(o.last().w, 0.1, 1e-9);
  const Twist2D c = o.apply(Desired{{0.5, 0.0}}, 0.4, 0.1);
  EXPECT_NEAR(c.v, 0.4 + 0.0143, 1e-9);
  o.reset({0.9, -2.0});
  EXPECT_NEAR(o.last().v, 0.5, 1e-9);
  EXPECT_NEAR(o.last().w, -0.5, 1e-9);
  o.reset({-0.2, 0.0});
  EXPECT_NEAR(o.last().v, 0.0, 1e-9);
}

TEST(OutputStage, CurvatureKeptByReducingSpeedAtTurnLimit)
{
  OutputStage o;
  for (int i = 0; i < 60; ++i) {o.apply(Desired{{0.5, 0.0}}, o.last().v, 0.1);}
  Desired d{{0.5, 0.0}};
  d.curvature = 2.0;
  Twist2D c;
  for (int i = 0; i < 10; ++i) {
    c = o.apply(d, o.last().v, 0.1);
    EXPECT_LE(c.v, 0.25 + 1e-9) << i;
  }
  EXPECT_NEAR(c.w, 0.5, 1e-9);
  EXPECT_NEAR(c.w, c.v * 2.0, 1e-9);
}

namespace
{
struct DelayedPlant
{
  std::vector<double> cmds;
  double measured() const {return cmds.size() >= 5 ? cmds[cmds.size() - 5] : 0.0;}
};
}

TEST(OutputStage, StartRampSurvivesMotorLag)
{
  OutputStage o;
  DelayedPlant m;
  double t = 0.0, t_half = -1.0, t_full = -1.0, prev = 0.0;
  for (int i = 0; i < 40; ++i) {
    t += 0.1;
    const Twist2D c = o.apply(Desired{{0.5, 0.0}}, m.measured(), 0.1);
    m.cmds.push_back(c.v);
    EXPECT_GE(c.v, prev - 1e-12) << i;
    prev = c.v;
    if (t_half < 0 && c.v >= 0.25 - 1e-9) {t_half = t;}
    if (t_full < 0 && c.v >= 0.5 - 1e-9) {t_full = t;}
  }
  EXPECT_GT(t_half, 0.0);
  EXPECT_LE(t_half, 0.6 + 1e-9);
  EXPECT_GT(t_full, 0.0);
  EXPECT_LE(t_full, 0.5 + 1.75 + 0.3 + 1e-9);
}

TEST(OutputStage, RealStopStillRestartsFromNearZeroWithMotorLag)
{
  OutputStage o;
  for (int i = 0; i < 60; ++i) {o.apply(Desired{{0.5, 0.0}}, o.last().v, 0.1);}
  Twist2D c;
  for (int i = 0; i < 5; ++i) {c = o.apply(Desired{{0.5, 0.0}}, 0.0, 0.1);}
  EXPECT_LE(c.v, OutputParams{}.resync_margin + 0.05 + 1e-9);
  EXPECT_LE(c.v, 0.2 + 1e-9);
}

TEST(OutputStage, PlannedSlowdownUsesPlannedDecel)
{
  OutputParams p; p.planned_decel = 0.5;
  OutputStage o(p);
  for (int i = 0; i < 60; ++i) {o.apply(Desired{{0.5, 0.0}}, o.last().v, 0.1);}
  Desired d{{0.2, 0.0}};
  d.planned = true;
  EXPECT_NEAR(o.apply(d, 0.5, 0.1).v, 0.5 - 0.05, 1e-9);
}

TEST(OutputStage, EmergencyStopIgnoresPlannedDecel)
{
  OutputParams p; p.planned_decel = 0.5;
  OutputStage o(p);
  for (int i = 0; i < 60; ++i) {o.apply(Desired{{0.5, 0.0}}, o.last().v, 0.1);}
  EXPECT_NEAR(o.apply(Desired{{0.0, 0.0}}, 0.5, 0.1).v, 0.5 - 0.125, 1e-9);
}

TEST(OutputStage, PlannedDecelIsNeverHarderThanEmergency)
{
  OutputParams p; p.planned_decel = 2.0;
  OutputStage o(p);
  for (int i = 0; i < 60; ++i) {o.apply(Desired{{0.5, 0.0}}, o.last().v, 0.1);}
  Desired d{{0.0, 0.0}};
  d.planned = true;
  EXPECT_NEAR(o.apply(d, 0.5, 0.1).v, 0.5 - 0.125, 1e-9);
}
