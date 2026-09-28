#include <gtest/gtest.h>
#include <cmath>
#include "vica_vcc_controller/core/output_stage.hpp"

using namespace vica_vcc_controller::core;

TEST(OutputStage, StartRampHalfSpeedThenSlow)
{
  // 요구 8: 0 -> 0.25 약 0.5 s, 0.25 -> 0.5 는 1.5~2 s
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
  EXPECT_NEAR(c.v, 0.5 - 0.125, 1e-9);   // smoother max_decel 1.25 와 같은 세기
}

TEST(OutputStage, ResyncsToMeasuredSpeedAfterExternalStop)
{
  // collision_monitor·예외로 실제 속도가 0 이 되면, 출발은 0 에서 다시 램프한다.
  OutputStage o;
  for (int i = 0; i < 60; ++i) {o.apply(Desired{{0.5, 0.0}}, o.last().v, 0.1);}
  const Twist2D c = o.apply(Desired{{0.5, 0.0}}, 0.0, 0.1);
  EXPECT_LE(c.v, 0.05 + 0.05 + 1e-9);
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
  EXPECT_NEAR(o.apply(Desired{{0.0, 0.5}}, 0.0, 0.1).w, 0.12, 1e-9);   // 1.2 rad/s^2
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
  // 최종 리뷰 I3: reset 뒤 0 에서 다시 램프하지 않고 실제 속도에서 이어 간다.
  OutputStage o;
  o.reset({0.4, 0.1});
  EXPECT_NEAR(o.last().v, 0.4, 1e-9);
  EXPECT_NEAR(o.last().w, 0.1, 1e-9);
  const Twist2D c = o.apply(Desired{{0.5, 0.0}}, 0.4, 0.1);
  EXPECT_NEAR(c.v, 0.4 + 0.0143, 1e-9);
  o.reset({0.9, -2.0});   // 한계 밖 실측은 한계로 자른다
  EXPECT_NEAR(o.last().v, 0.5, 1e-9);
  EXPECT_NEAR(o.last().w, -0.5, 1e-9);
  o.reset({-0.2, 0.0});   // 후진 없음
  EXPECT_NEAR(o.last().v, 0.0, 1e-9);
}

TEST(OutputStage, CurvatureKeptByReducingSpeedAtTurnLimit)
{
  // 최종 리뷰 M6: |v·κ| > max_w 면 w 만 자르지 않고 v 를 max_w/|κ| 로 낮춰 곡률을 지킨다.
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
