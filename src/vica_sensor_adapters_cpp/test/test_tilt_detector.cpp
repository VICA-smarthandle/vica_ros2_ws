// 몸통 넘어짐 판정의 계약(2026-10-15). 이 판정이 틀리면 서 있는 로봇이 비상정지되거나,
// 넘어진 로봇이 관리자 없이 풀린다. 그래서 '유지 시간'과 '판정 보류'를 따로 본다.

#include <gtest/gtest.h>

#include <cmath>

#include "vica_sensor_adapters_cpp/tilt_detector.hpp"

using vica_sensor_adapters_cpp::TiltDetector;

namespace
{

constexpr double kG = 9.67;   // run90~95 녹화의 |a| 중앙값
constexpr double kDt = 1.0 / 36.0;   // 실제 발행 주기(36 Hz)

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

}  // namespace

// ---- 기울기 계산 -----------------------------------------------------------

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
  // 스무더 가속 상한 2.5 m/s² 만으로는 45° 근처에도 못 간다(헤더 주석의 근거).
  const double tilt = TiltDetector::tilt_deg(2.5, 0.0, kG);
  EXPECT_GT(tilt, 13.0);
  EXPECT_LT(tilt, 16.0);
}

TEST(TiltAngle, TooSmallOrInvalidIsNaN)
{
  EXPECT_TRUE(std::isnan(TiltDetector::tilt_deg(0.0, 0.0, 0.0)));
  EXPECT_TRUE(std::isnan(TiltDetector::tilt_deg(0.5, 0.5, 0.5)));   // 자유 낙하
  EXPECT_TRUE(std::isnan(TiltDetector::tilt_deg(NAN, 0.0, kG)));
  EXPECT_TRUE(std::isnan(TiltDetector::tilt_deg(INFINITY, 0.0, kG)));
}

// ---- 넘어짐 판정 -----------------------------------------------------------

TEST(TiltDetector, StartsUnknownAndNotTipped)
{
  const TiltDetector det = standard();
  EXPECT_FALSE(det.known());
  EXPECT_FALSE(det.tipped());
}

TEST(TiltDetector, FirstUprightStateIsReportedOnceAfterHold)
{
  // 켤 때 바로 "서 있음"을 내지 않는다 — 1초 판정 뒤 한 번.
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
  // 넘어진 채 노드를 다시 켰다. 첫 상태가 곧바로 넘어짐이어야 비상정지 원인이 안 풀린다.
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
  // 노드가 1초 넘게 멈췄다가 다시 돌았다. 멈추기 전·후 진동 표본 두 개로 1초를 채우면 안 된다.
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
  now += 0.3;   // 0.25 s 넘게 빔
  EXPECT_EQ(feed(det, 90.0, now, 0.9), 0);   // 틈 뒤부터 다시 1초
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
  // 녹화에서 본 모양: 평소 1° 안팎에 가끔 52° 까지 튀는 한두 표본.
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
  EXPECT_EQ(feed(det, 80.0, now, 0.8), 0);   // 다시 처음부터 1초
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
  // 30~45° 는 지금 상태를 그대로 둔다 — 기대어 세운 로봇이 오가며 멘트를 반복하지 않게.
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
