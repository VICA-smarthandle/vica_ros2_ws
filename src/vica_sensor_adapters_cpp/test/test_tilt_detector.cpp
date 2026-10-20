#include <gtest/gtest.h>

#include <cmath>

#include "vica_sensor_adapters_cpp/tilt_detector.hpp"

using vica_sensor_adapters_cpp::TiltDetector;

namespace
{

constexpr double kG = 9.67;
constexpr double kDt = 1.0 / 36.0;

/// 같은 기울기를 seconds 동안 넣고, 상태가 바뀐 횟수를 센다.
int feed(TiltDetector & det, double tilt, double & now, double seconds)
{
  int changes = 0;
  const int n = static_cast<int>(std::round(seconds / kDt));
  for (int i = 0; i < n; ++i) {
    now += kDt;
    if (det.update(tilt, now)) {
      ++changes;
    }
  }
  return changes;
}

TiltDetector standard() {return TiltDetector(45.0, 30.0, 1.0);}

}

TEST(TiltAngle, UprightIsZero)
{
  EXPECT_NEAR(TiltDetector::tilt_deg(0.0, 0.0, kG), 0.0, 1e-9);
}

TEST(TiltAngle, OnItsSideIsNinety)
{
  EXPECT_NEAR(TiltDetector::tilt_deg(kG, 0.0, 0.0), 90.0, 1e-9);
  EXPECT_NEAR(TiltDetector::tilt_deg(0.0, -kG, 0.0), 90.0, 1e-9);
}

TEST(TiltAngle, UpsideDownIsOneEighty)
{
  EXPECT_NEAR(TiltDetector::tilt_deg(0.0, 0.0, -kG), 180.0, 1e-9);
}

TEST(TiltAngle, FortyFiveDegrees)
{
  const double c = kG / std::sqrt(2.0);
  EXPECT_NEAR(TiltDetector::tilt_deg(c, 0.0, c), 45.0, 1e-9);
}

TEST(TiltAngle, MaxSmootherAccelLooksLikeFourteenDegrees)
{
  const double tilt = TiltDetector::tilt_deg(2.5, 0.0, kG);
  EXPECT_GT(tilt, 13.0);
  EXPECT_LT(tilt, 16.0);
}

TEST(TiltAngle, TooSmallOrInvalidIsNaN)
{
  EXPECT_TRUE(std::isnan(TiltDetector::tilt_deg(0.0, 0.0, 0.0)));
  EXPECT_TRUE(std::isnan(TiltDetector::tilt_deg(0.5, 0.5, 0.5)));
  EXPECT_TRUE(std::isnan(TiltDetector::tilt_deg(NAN, 0.0, kG)));
  EXPECT_TRUE(std::isnan(TiltDetector::tilt_deg(INFINITY, 0.0, kG)));
}

TEST(TiltDetector, StartsUnknownAndNotTipped)
{
  const TiltDetector det = standard();
  EXPECT_FALSE(det.known());
  EXPECT_FALSE(det.tipped());
}

TEST(TiltDetector, FirstUprightStateIsReportedOnceAfterHold)
{
  TiltDetector det = standard();
  double now = 0.0;
  EXPECT_EQ(feed(det, 1.0, now, 0.9), 0);
  EXPECT_FALSE(det.known());
  EXPECT_EQ(feed(det, 1.0, now, 0.2), 1);
  EXPECT_TRUE(det.known());
  EXPECT_FALSE(det.tipped());
  EXPECT_EQ(feed(det, 1.0, now, 10.0), 0);
}

TEST(TiltDetector, RestartWhileLyingNeverReportsUpright)
{
  TiltDetector det = standard();
  double now = 0.0;
  int upright_reports = 0;
  for (int i = 0; i < 36 * 3; ++i) {
    now += kDt;
    if (det.update(90.0, now) && !det.tipped()) {
      ++upright_reports;
    }
  }
  EXPECT_EQ(upright_reports, 0);
  EXPECT_TRUE(det.known());
  EXPECT_TRUE(det.tipped());
}

TEST(TiltDetector, StallBetweenTwoSpikesDoesNotTip)
{
  TiltDetector det = standard();
  double now = 0.0;
  feed(det, 1.0, now, 2.0);
  ASSERT_TRUE(det.known());
  EXPECT_FALSE(det.update(52.0, now + kDt));
  EXPECT_FALSE(det.update(52.0, now + kDt + 1.2));
  EXPECT_FALSE(det.tipped());
}

TEST(TiltDetector, GapRestartsTheCountFromTheNextSample)
{
  TiltDetector det = standard();
  double now = 0.0;
  feed(det, 1.0, now, 2.0);
  EXPECT_EQ(feed(det, 90.0, now, 0.5), 0);
  now += 0.3;
  EXPECT_EQ(feed(det, 90.0, now, 0.9), 0);
  EXPECT_FALSE(det.tipped());
  EXPECT_EQ(feed(det, 90.0, now, 0.2), 1);
  EXPECT_TRUE(det.tipped());
}

TEST(TiltDetector, TipsAfterHoldingOneSecond)
{
  TiltDetector det = standard();
  double now = 100.0;
  EXPECT_EQ(feed(det, 90.0, now, 0.9), 0);
  EXPECT_FALSE(det.tipped());
  EXPECT_EQ(feed(det, 90.0, now, 0.2), 1);
  EXPECT_TRUE(det.tipped());
}

TEST(TiltDetector, ExactlyAtTipAngleCounts)
{
  TiltDetector det = standard();
  double now = 0.0;
  EXPECT_EQ(feed(det, 45.0, now, 1.1), 1);
  EXPECT_TRUE(det.tipped());
}

TEST(TiltDetector, VibrationSpikesNeverTip)
{
  TiltDetector det = standard();
  double now = 0.0;
  for (int i = 0; i < 36 * 600; ++i) {
    now += kDt;
    const double tilt = (i % 7 == 0) ? 52.0 : 1.0;
    EXPECT_FALSE(det.update(tilt, now));
  }
  EXPECT_FALSE(det.tipped());
}

TEST(TiltDetector, OneUprightSampleRestartsTheCount)
{
  TiltDetector det = standard();
  double now = 0.0;
  EXPECT_EQ(feed(det, 80.0, now, 0.8), 0);
  now += kDt;
  EXPECT_FALSE(det.update(10.0, now));
  EXPECT_EQ(feed(det, 80.0, now, 0.8), 0);
  EXPECT_FALSE(det.tipped());
  EXPECT_EQ(feed(det, 80.0, now, 0.3), 1);
}

TEST(TiltDetector, StaysTippedUntilUprightForOneSecond)
{
  TiltDetector det = standard();
  double now = 0.0;
  feed(det, 90.0, now, 1.5);
  ASSERT_TRUE(det.tipped());
  EXPECT_EQ(feed(det, 2.0, now, 0.9), 0);
  EXPECT_TRUE(det.tipped());
  EXPECT_EQ(feed(det, 2.0, now, 0.2), 1);
  EXPECT_FALSE(det.tipped());
}

TEST(TiltDetector, BetweenReleaseAndTipKeepsState)
{
  TiltDetector det = standard();
  double now = 0.0;
  EXPECT_EQ(feed(det, 40.0, now, 5.0), 0);
  EXPECT_FALSE(det.tipped());
  feed(det, 90.0, now, 1.5);
  ASSERT_TRUE(det.tipped());
  EXPECT_EQ(feed(det, 40.0, now, 5.0), 0);
  EXPECT_TRUE(det.tipped());
}

TEST(TiltDetector, NaNSampleRestartsTheCount)
{
  TiltDetector det = standard();
  double now = 0.0;
  EXPECT_EQ(feed(det, 90.0, now, 0.8), 0);
  now += kDt;
  EXPECT_FALSE(det.update(NAN, now));
  EXPECT_EQ(feed(det, 90.0, now, 0.8), 0);
  EXPECT_FALSE(det.tipped());
}

TEST(TiltDetector, NaNNeverReleases)
{
  TiltDetector det = standard();
  double now = 0.0;
  feed(det, 90.0, now, 1.5);
  ASSERT_TRUE(det.tipped());
  EXPECT_EQ(feed(det, NAN, now, 5.0), 0);
  EXPECT_TRUE(det.tipped());
}

TEST(TiltDetector, ClockGoingBackwardsRestartsTheCount)
{
  TiltDetector det = standard();
  double now = 10.0;
  EXPECT_EQ(feed(det, 90.0, now, 0.8), 0);
  now -= 5.0;
  EXPECT_FALSE(det.update(90.0, now));
  EXPECT_EQ(feed(det, 90.0, now, 0.8), 0);
  EXPECT_FALSE(det.tipped());
}

TEST(TiltDetector, ReleaseAboveTipIsClamped)
{
  const TiltDetector det(45.0, 60.0, 1.0);
  EXPECT_DOUBLE_EQ(det.release_angle_deg(), 45.0);
}

TEST(TiltDetector, NegativeHoldIsZero)
{
  TiltDetector det(45.0, 30.0, -1.0);
  EXPECT_DOUBLE_EQ(det.hold_sec(), 0.0);
  EXPECT_TRUE(det.update(90.0, 0.0));
  EXPECT_TRUE(det.tipped());
}

TEST(TiltDetector, ReportsEachChangeOnce)
{
  TiltDetector det = standard();
  double now = 0.0;
  EXPECT_EQ(feed(det, 90.0, now, 10.0), 1);
  EXPECT_EQ(feed(det, 0.0, now, 10.0), 1);
  EXPECT_EQ(feed(det, 90.0, now, 10.0), 1);
}
