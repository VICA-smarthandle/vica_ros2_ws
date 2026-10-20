#include <gtest/gtest.h>
#include <cmath>
#include "vica_vcc_controller/core/speed_profile.hpp"

using namespace vica_vcc_controller::core;

namespace
{
Path cornerPath(double start, double R)
{
  Path p;
  for (double x = 0.0; x < start; x += 0.05) {p.push_back({x, 0.0, 0.0});}
  for (double th = 0.0; th <= M_PI / 2 + 1e-9; th += 0.05 / R) {
    p.push_back({start + R * std::sin(th), R - R * std::cos(th), th});
  }
  return p;
}
}

TEST(SpeedProfile, StraightKeepsDesired)
{
  Path p;
  for (double x = 0.0; x <= 3.0; x += 0.05) {p.push_back({x, 0.0, 0.0});}
  EXPECT_NEAR(curvePreviewLimit(p, SpeedParams{}), 0.5, 1e-6);
}

TEST(SpeedProfile, CornerIsAnticipatedNotSudden)
{
  SpeedParams sp;
  EXPECT_NEAR(curveLimitAt(1.0 / 0.5, sp), 0.208, 0.002);
  const double v02 = curvePreviewLimit(cornerPath(0.2, 0.5), sp);
  EXPECT_LE(v02, 0.434);
  EXPECT_GE(v02, 0.35);
  EXPECT_NEAR(curvePreviewLimit(cornerPath(1.0, 0.5), sp), 0.5, 1e-6);
  EXPECT_NEAR(curvePreviewLimit(cornerPath(0.2, 2.0), sp), 0.5, 1e-6);
}

TEST(SpeedProfile, ClearanceSlowdownUsesMeasuredDistance)
{
  SpeedParams sp;
  EXPECT_NEAR(clearanceLimit(0.50, sp), 0.5, 1e-9);
  EXPECT_NEAR(clearanceLimit(0.175, sp), 0.25, 1e-9);
  EXPECT_NEAR(clearanceLimit(0.01, sp), 0.12, 1e-9);
}

TEST(SpeedProfile, ApproachMatchesCurrentRpp)
{
  SpeedParams sp;
  EXPECT_NEAR(approachLimit(1.0, 0.5, sp), 0.5, 1e-9);
  EXPECT_NEAR(approachLimit(0.3, 0.5, sp), 0.25, 1e-9);
  EXPECT_NEAR(approachLimit(0.01, 0.5, sp), 0.05, 1e-9);
}
