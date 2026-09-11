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

}

TEST(GyroBias, BeforeEnoughSamplesItDoesNotCorrect)
{
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
  GyroBiasEstimator est(10, 0.05);
  feed(est, 10);

  EXPECT_TRUE(est.ready());
}

TEST(GyroBias, BiasIsTheMeanOfCollectedSamples)
{
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
  GyroBiasEstimator est(5, 0.05);
  feed(est, 5, 0.0, 0.0, 0.001);

  const Vec3 out = est.correct(0.010, 0.020, 0.030);
  EXPECT_NEAR(out[2], 0.029, 1e-12);
}

TEST(GyroBias, MotionDuringCalibrationAbortsPermanently)
{
  GyroBiasEstimator est(10, 0.05);
  feed(est, 5);
  est.add(0.0, 0.0, 0.5);
  feed(est, 100);

  EXPECT_TRUE(est.aborted());
  EXPECT_FALSE(est.ready());
  const Vec3 out = est.correct(0.5, 0.6, 0.7);
  EXPECT_DOUBLE_EQ(out[0], 0.5);
  EXPECT_DOUBLE_EQ(out[2], 0.7);
}

TEST(GyroBias, RateExactlyAtThresholdIsAccepted)
{
  GyroBiasEstimator est(2, 0.05);
  est.add(0.0, 0.0, 0.05);
  est.add(0.0, 0.0, 0.05);

  EXPECT_FALSE(est.aborted());
  EXPECT_TRUE(est.ready());
}

TEST(GyroBias, RateJustOverThresholdAborts)
{
  GyroBiasEstimator est(2, 0.05);
  est.add(0.0, 0.0, 0.0500001);

  EXPECT_TRUE(est.aborted());
}

TEST(GyroBias, ThresholdAppliesToEveryAxis)
{
  GyroBiasEstimator est(2, 0.05);
  est.add(0.9, 0.0, 0.0);

  EXPECT_TRUE(est.aborted());
}

TEST(GyroBias, ZeroSampleCountDisablesTheFeature)
{
  GyroBiasEstimator est(0, 0.05);
  feed(est, 50);

  EXPECT_FALSE(est.ready());
  EXPECT_FALSE(est.aborted());
  const Vec3 out = est.correct(0.5, 0.6, 0.7);
  EXPECT_DOUBLE_EQ(out[1], 0.6);
}

TEST(GyroBias, ExtraSamplesAfterReadyDoNotChangeBias)
{
  GyroBiasEstimator est(3, 0.05);
  feed(est, 3, 0.0, 0.0, 0.001);
  const Vec3 before = est.bias();

  est.add(0.0, 0.0, 0.04);
  EXPECT_DOUBLE_EQ(est.bias()[2], before[2]);
}

TEST(GyroBias, ProgressReportsCollectedAndTarget)
{
  GyroBiasEstimator est(10, 0.05);
  feed(est, 4);

  EXPECT_EQ(est.collected(), 4);
  EXPECT_EQ(est.sample_count(), 10);
}

TEST(GyroBias, RefreshOffByDefaultKeepsOldBehaviour)
{
  GyroBiasEstimator est = ready_estimator(0.01, 0);
  const Vec3 before = est.bias();
  quiet(est, 500, 0.02);

  EXPECT_DOUBLE_EQ(est.bias()[2], before[2]);
}

TEST(GyroBias, QuietStretchUpdatesTheBias)
{
  GyroBiasEstimator est = ready_estimator(0.01, 20, 0.5, 0.01);
  quiet(est, 20, 0.02);

  EXPECT_NEAR(est.bias()[2], 0.015, 1e-9);
}

TEST(GyroBias, UpdateIsBlendedNotReplaced)
{
  GyroBiasEstimator est = ready_estimator(0.0, 10, 0.2, 0.01, 1.0);
  quiet(est, 10, 0.04);

  EXPECT_NEAR(est.bias()[2], 0.008, 1e-9);
}

TEST(GyroBias, ShakyStretchIsRejected)
{
  GyroBiasEstimator est = ready_estimator(0.01, 20, 0.5, 0.01);
  const Vec3 before = est.bias();
  shaky(est, 200, 0.05, 0.03);

  EXPECT_DOUBLE_EQ(est.bias()[2], before[2]);
}

TEST(GyroBias, MotionResetsTheStretch)
{
  GyroBiasEstimator est = ready_estimator(0.01, 20, 0.5, 0.01);
  const Vec3 before = est.bias();
  quiet(est, 15, 0.02);
  est.add(0.5, 0.0, 0.0);
  quiet(est, 15, 0.02);

  EXPECT_DOUBLE_EQ(est.bias()[2], before[2]);
}

TEST(GyroBias, SuddenJumpIsRefused)
{
  GyroBiasEstimator est = ready_estimator(0.01, 10, 0.5, 0.01, 0.02);
  const Vec3 before = est.bias();
  quiet(est, 10, 0.04);

  EXPECT_DOUBLE_EQ(est.bias()[2], before[2]);
}

TEST(GyroBias, RefreshWorksEvenIfStartupAborted)
{
  GyroBiasEstimator est(10, 0.05, 10, 1.0, 0.01);
  est.add(0.5, 0.0, 0.0);
  ASSERT_TRUE(est.aborted());
  ASSERT_FALSE(est.ready());

  quiet(est, 10, 0.02);
  EXPECT_TRUE(est.ready());
  EXPECT_NEAR(est.bias()[2], 0.02, 1e-9);
}

TEST(GyroBias, RefreshCountIsReported)
{
  GyroBiasEstimator est = ready_estimator(0.01, 10, 0.5, 0.01);
  EXPECT_EQ(est.refresh_count(), 0);

  quiet(est, 10, 0.02);
  EXPECT_EQ(est.refresh_count(), 1);
}
