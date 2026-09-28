#include <gtest/gtest.h>
#include <cmath>
#include "vica_vcc_controller/core/speed_profile.hpp"

using namespace vica_vcc_controller::core;

namespace
{
// start 까지 직진한 뒤 반지름 R 로 왼쪽 90도
Path cornerPath(double start, double R)
{
  Path p;
  for (double x = 0.0; x < start; x += 0.05) {p.push_back({x, 0.0, 0.0});}
  for (double th = 0.0; th <= M_PI / 2 + 1e-9; th += 0.05 / R) {
    p.push_back({start + R * std::sin(th), R - R * std::cos(th), th});
  }
  return p;
}
}  // namespace

TEST(SpeedProfile, StraightKeepsDesired)
{
  Path p;
  for (double x = 0.0; x <= 3.0; x += 0.05) {p.push_back({x, 0.0, 0.0});}
  EXPECT_NEAR(curvePreviewLimit(p, SpeedParams{}), 0.5, 1e-6);
}

TEST(SpeedProfile, CornerIsAnticipatedNotSudden)
{
  // R 0.5 코너 한계 = 0.5 x 0.5/1.2 = 0.208 (RPP 곡률 감속과 같은 값)
  SpeedParams sp;
  EXPECT_NEAR(curveLimitAt(1.0 / 0.5, sp), 0.208, 0.002);
  // 0.2 m 앞 코너: 이론값 sqrt(0.208^2 + 2*0.3*0.2) = 0.404 -> 미리 줄이기 시작.
  // 0.2 m 간격 표본의 첫 세 점에는 코너 앞 직선이 섞여 곡률이 옅게 잡힌다(k≈1.1, 거리 0 에서 0.374).
  // 이론값보다 조금 이르고 느리게 줄이는 쪽이라 안전하다. 지키는 것: 코너 앞에서 이미 줄이되(≤ 0.434),
  // 코너 한계 0.208 보다 한참 위(≥ 0.35)에서 시작한다.
  const double v02 = curvePreviewLimit(cornerPath(0.2, 0.5), sp);
  EXPECT_LE(v02, 0.434);
  EXPECT_GE(v02, 0.35);
  // 1.0 m 앞 코너: sqrt(0.043 + 0.6) = 0.80 > 0.5 -> 아직 안 줄인다
  EXPECT_NEAR(curvePreviewLimit(cornerPath(1.0, 0.5), sp), 0.5, 1e-6);
  // 완만한 R 2.0 코너는 줄이지 않는다
  EXPECT_NEAR(curvePreviewLimit(cornerPath(0.2, 2.0), sp), 0.5, 1e-6);
}

TEST(SpeedProfile, ClearanceSlowdownUsesMeasuredDistance)
{
  SpeedParams sp;
  EXPECT_NEAR(clearanceLimit(0.50, sp), 0.5, 1e-9);
  EXPECT_NEAR(clearanceLimit(0.175, sp), 0.25, 1e-9);
  EXPECT_NEAR(clearanceLimit(0.01, sp), 0.12, 1e-9);   // RPP regulated_linear_scaling_min_speed
}

TEST(SpeedProfile, ApproachMatchesCurrentRpp)
{
  SpeedParams sp;
  EXPECT_NEAR(approachLimit(1.0, 0.5, sp), 0.5, 1e-9);
  EXPECT_NEAR(approachLimit(0.3, 0.5, sp), 0.25, 1e-9);
  EXPECT_NEAR(approachLimit(0.01, 0.5, sp), 0.05, 1e-9);
}
