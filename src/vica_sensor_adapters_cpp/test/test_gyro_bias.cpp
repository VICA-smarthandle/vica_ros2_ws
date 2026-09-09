// 정지 중 자이로 편향 추정기의 계약 — 파이썬 판 시험 19 개를 그대로 옮겼다.
//
// 왜 그대로 옮기는가: 이 로직은 EKF yaw 정확도를 지탱한다(C안 = 자이로 단독
// yaw). 2026-08-01 실측에서 편향 +0.000917 rad/s 가 시간당 189° 드리프트를
// 만들었다. 숫자가 하나라도 달라지면 주행이 휜다. 그래서 언어를 바꾸되
// **계약은 한 줄도 바꾸지 않는다.**
//
// 원본: vica_sensor_adapters/test/test_gyro_bias.py

#include <gtest/gtest.h>

#include "vica_sensor_adapters_cpp/gyro_bias.hpp"

using vica_sensor_adapters_cpp::GyroBiasEstimator;
using vica_sensor_adapters_cpp::Vec3;

namespace
{

void feed(GyroBiasEstimator & est, int count, double x = 0.001, double y = 0.002,
  double z = 0.003)
{
  for (int i = 0; i < count; ++i) {
    est.add(x, y, z);
  }
}

/// 정차로 인정될 만큼 조용한 표본.
void quiet(GyroBiasEstimator & est, int count, double value = 0.01)
{
  for (int i = 0; i < count; ++i) {
    est.add(value, value, value);
  }
}

/// 직진 중처럼 흔들리는 표본. 평균은 center 지만 폭이 크다.
void shaky(GyroBiasEstimator & est, int count, double center = 0.01, double amp = 0.03)
{
  for (int i = 0; i < count; ++i) {
    const double d = (i % 2 == 0) ? amp : -amp;
    est.add(center + d, center + d, center + d);
  }
}

/// 초기 확정을 마친 추정기.
GyroBiasEstimator ready_estimator(
  double bias = 0.01, int refresh_sample_count = 0, double refresh_alpha = 0.2,
  double max_abs_dev = 0.01, double max_refresh_jump = 0.02)
{
  GyroBiasEstimator est(10, 0.05, refresh_sample_count, refresh_alpha,
    max_abs_dev, max_refresh_jump);
  feed(est, 10, bias, bias, bias);
  EXPECT_TRUE(est.ready());
  return est;
}

}  // namespace

// ---- 기동 확정 -------------------------------------------------------------

TEST(GyroBias, BeforeEnoughSamplesItDoesNotCorrect)
{
  // 표본이 모자라면 보정하지 않는다. 원값을 그대로 돌려준다.
  GyroBiasEstimator est(10, 0.05);
  feed(est, 9);

  EXPECT_FALSE(est.ready());
  const Vec3 out = est.correct(0.5, 0.6, 0.7);
  EXPECT_DOUBLE_EQ(out[0], 0.5);
  EXPECT_DOUBLE_EQ(out[1], 0.6);
  EXPECT_DOUBLE_EQ(out[2], 0.7);
}

TEST(GyroBias, ExactlyEnoughSamplesBecomesReady)
{
  // 경계값: 표본이 정확히 sample_count 면 준비된다.
  GyroBiasEstimator est(10, 0.05);
  feed(est, 10);

  EXPECT_TRUE(est.ready());
}

TEST(GyroBias, BiasIsTheMeanOfCollectedSamples)
{
  // 편향은 수집 구간의 축별 평균이다.
  GyroBiasEstimator est(4, 0.05);
  for (double gz : {0.001, 0.002, 0.003, 0.004}) {
    est.add(0.0, 0.0, gz);
  }

  const Vec3 b = est.bias();
  EXPECT_NEAR(b[0], 0.0, 1e-12);
  EXPECT_NEAR(b[1], 0.0, 1e-12);
  EXPECT_NEAR(b[2], 0.0025, 1e-12);
}

TEST(GyroBias, CorrectionSubtractsTheBias)
{
  // 준비된 뒤에는 원값에서 편향을 뺀다.
  GyroBiasEstimator est(5, 0.05);
  feed(est, 5, 0.0, 0.0, 0.001);

  const Vec3 out = est.correct(0.010, 0.020, 0.030);
  EXPECT_NEAR(out[2], 0.029, 1e-12);
}

TEST(GyroBias, MotionDuringCalibrationAbortsPermanently)
{
  // 보정 중 움직이면 포기한다. 틀린 편향을 박아넣지 않는다.
  GyroBiasEstimator est(10, 0.05);
  feed(est, 5);
  est.add(0.0, 0.0, 0.5);      // 회전이 들어왔다
  feed(est, 100);              // 이후 아무리 조용해도

  EXPECT_TRUE(est.aborted());
  EXPECT_FALSE(est.ready());
  const Vec3 out = est.correct(0.5, 0.6, 0.7);
  EXPECT_DOUBLE_EQ(out[0], 0.5);
  EXPECT_DOUBLE_EQ(out[2], 0.7);
}

TEST(GyroBias, RateExactlyAtThresholdIsAccepted)
{
  // 경계값: 임계값과 같은 각속도는 정지로 본다.
  GyroBiasEstimator est(2, 0.05);
  est.add(0.0, 0.0, 0.05);
  est.add(0.0, 0.0, 0.05);

  EXPECT_FALSE(est.aborted());
  EXPECT_TRUE(est.ready());
}

TEST(GyroBias, RateJustOverThresholdAborts)
{
  // 경계값: 임계값을 조금이라도 넘으면 포기한다.
  GyroBiasEstimator est(2, 0.05);
  est.add(0.0, 0.0, 0.0500001);

  EXPECT_TRUE(est.aborted());
}

TEST(GyroBias, ThresholdAppliesToEveryAxis)
{
  // 임계 판정은 세 축 각각에 적용한다.
  GyroBiasEstimator est(2, 0.05);
  est.add(0.9, 0.0, 0.0);

  EXPECT_TRUE(est.aborted());
}

TEST(GyroBias, ZeroSampleCountDisablesTheFeature)
{
  // sample_count 가 0 이면 기능을 끈다. 보정도 포기도 하지 않는다.
  GyroBiasEstimator est(0, 0.05);
  feed(est, 50);

  EXPECT_FALSE(est.ready());
  EXPECT_FALSE(est.aborted());
  const Vec3 out = est.correct(0.5, 0.6, 0.7);
  EXPECT_DOUBLE_EQ(out[1], 0.6);
}

TEST(GyroBias, ExtraSamplesAfterReadyDoNotChangeBias)
{
  // 준비된 뒤 들어오는 값은 편향을 바꾸지 않는다. 주행 중 값이 섞이면 안 된다.
  GyroBiasEstimator est(3, 0.05);
  feed(est, 3, 0.0, 0.0, 0.001);
  const Vec3 before = est.bias();

  est.add(0.0, 0.0, 0.04);
  EXPECT_DOUBLE_EQ(est.bias()[2], before[2]);
}

TEST(GyroBias, ProgressReportsCollectedAndTarget)
{
  // 진행 상황을 읽을 수 있어야 로그와 진단에 남길 수 있다.
  GyroBiasEstimator est(10, 0.05);
  feed(est, 4);

  EXPECT_EQ(est.collected(), 4);
  EXPECT_EQ(est.sample_count(), 10);
}

// ---- ZUPT — 정차할 때마다 다시 재기 ----------------------------------------
//
// 가장 큰 위험은 **직진을 정차로 착각하는 것**이다. 직진 중에도 자이로 평균은
// 0 에 가깝지만 마스트 진동 때문에 폭이 52 배 크다(정지 sigma 0.0018,
// 직진 sigma 0.0939 rad/s). 그래서 크기와 흔들림 폭을 함께 본다.

TEST(GyroBias, RefreshOffByDefaultKeepsOldBehaviour)
{
  // refresh_sample_count 0 이면 종전과 똑같이 한 번 확정하고 고정이다.
  GyroBiasEstimator est = ready_estimator(0.01, 0);
  const Vec3 before = est.bias();
  quiet(est, 500, 0.02);

  EXPECT_DOUBLE_EQ(est.bias()[2], before[2]);
}

TEST(GyroBias, QuietStretchUpdatesTheBias)
{
  // 정차가 충분히 이어지면 편향을 갱신한다. 0.01 과 0.02 를 반씩 섞는다.
  GyroBiasEstimator est = ready_estimator(0.01, 20, 0.5, 0.01);
  quiet(est, 20, 0.02);

  EXPECT_NEAR(est.bias()[2], 0.015, 1e-9);
}

TEST(GyroBias, UpdateIsBlendedNotReplaced)
{
  // 한 번의 측정이 편향을 통째로 갈아치우지 않는다. alpha 만큼만 옮겨간다.
  GyroBiasEstimator est = ready_estimator(0.0, 10, 0.2, 0.01, 1.0);
  quiet(est, 10, 0.04);            // max_abs_rate 0.05 안쪽

  EXPECT_NEAR(est.bias()[2], 0.008, 1e-9);   // 0.04 의 20 %
}

TEST(GyroBias, ShakyStretchIsRejected)
{
  // 직진처럼 흔들리는 구간은 정차로 보지 않는다 — 이 시험이 핵심이다.
  GyroBiasEstimator est = ready_estimator(0.01, 20, 0.5, 0.01);
  const Vec3 before = est.bias();
  shaky(est, 200, 0.05, 0.03);

  EXPECT_DOUBLE_EQ(est.bias()[2], before[2]);
}

TEST(GyroBias, MotionResetsTheStretch)
{
  // 중간에 움직이면 모으던 표본을 버린다. 이어붙이지 않는다.
  GyroBiasEstimator est = ready_estimator(0.01, 20, 0.5, 0.01);
  const Vec3 before = est.bias();
  quiet(est, 15, 0.02);
  est.add(0.5, 0.0, 0.0);          // 회전
  quiet(est, 15, 0.02);            // 합치면 30 이지만 이어지지 않았다

  EXPECT_DOUBLE_EQ(est.bias()[2], before[2]);
}

TEST(GyroBias, SuddenJumpIsRefused)
{
  // 옛 값과 너무 다르면 그 회차를 버린다. 편향은 온도로 천천히 변한다.
  GyroBiasEstimator est = ready_estimator(0.01, 10, 0.5, 0.01, 0.02);
  const Vec3 before = est.bias();
  quiet(est, 10, 0.04);            // 0.01 -> 0.04, 차이 0.03 > 0.02

  EXPECT_DOUBLE_EQ(est.bias()[2], before[2]);
}

TEST(GyroBias, RefreshWorksEvenIfStartupAborted)
{
  // 기동 때 못 쟀어도 첫 정차에서 잡는다.
  GyroBiasEstimator est(10, 0.05, 10, 1.0, 0.01);
  est.add(0.5, 0.0, 0.0);          // 기동 구간에 움직였다
  ASSERT_TRUE(est.aborted());
  ASSERT_FALSE(est.ready());

  quiet(est, 10, 0.02);
  EXPECT_TRUE(est.ready());
  EXPECT_NEAR(est.bias()[2], 0.02, 1e-9);
}

TEST(GyroBias, RefreshCountIsReported)
{
  // 몇 번 갱신했는지 볼 수 있어야 실주행에서 동작을 확인한다.
  GyroBiasEstimator est = ready_estimator(0.01, 10, 0.5, 0.01);
  EXPECT_EQ(est.refresh_count(), 0);

  quiet(est, 10, 0.02);
  EXPECT_EQ(est.refresh_count(), 1);
}
