#include <gtest/gtest.h>
#include <cmath>
#include "vica_vcc_controller/core/ultrasonic.hpp"

using namespace vica_vcc_controller::core;

namespace
{
RangeReading r(double range, double t) {return {range, 0.02, 1.5, 1.047, t, {}};}
}

TEST(Ultrasonic, TwoConsistentFreshReadingsAreConfirmed)
{
  UltrasonicChannel ch;
  ch.push(r(0.80, 10.0));
  ch.push(r(0.85, 10.46));   // 2.19 Hz 간격
  const auto c = ch.confirmed(10.5, 1.0, 2, 0.15);
  ASSERT_TRUE(c.has_value());
  EXPECT_NEAR(c->range, 0.85, 1e-9);
}

TEST(Ultrasonic, SingleSpikeIsIgnored)
{
  UltrasonicChannel ch;
  ch.push(r(1.5, 10.0));      // 에코 없음(max_range)
  ch.push(r(0.40, 10.46));    // 한 번 튄 값
  EXPECT_FALSE(ch.confirmed(10.5, 1.0, 2, 0.15).has_value());
  UltrasonicChannel ch2;
  ch2.push(r(0.40, 10.0));
  ch2.push(r(0.80, 10.46));   // 0.4 m 차이
  EXPECT_FALSE(ch2.confirmed(10.5, 1.0, 2, 0.15).has_value());
}

TEST(Ultrasonic, StaleReadingsAreDropped)
{
  // run43: 지워지지 않는 표시가 정지 9회의 원인. 1 s 지난 값은 쓰지 않는다.
  UltrasonicChannel ch;
  ch.push(r(0.80, 10.0));
  ch.push(r(0.80, 10.46));
  EXPECT_FALSE(ch.confirmed(11.2, 1.0, 2, 0.15).has_value());
}

TEST(Ultrasonic, NoEchoIsNotAnObstacle)
{
  UltrasonicChannel ch;
  ch.push(r(1.5, 10.0));
  ch.push(r(1.5, 10.46));
  EXPECT_FALSE(ch.confirmed(10.5, 1.0, 2, 0.15).has_value());
}

TEST(Ultrasonic, ArcPointsSpanFieldOfView)
{
  const auto pts = rangeToArcPoints(1.0, 60.0 * M_PI / 180.0, 5);
  ASSERT_EQ(pts.size(), 5u);
  for (const auto & p : pts) {EXPECT_NEAR(std::hypot(p.x, p.y), 1.0, 1e-9);}
  EXPECT_NEAR(std::atan2(pts.front().y, pts.front().x), -M_PI / 6, 1e-9);
  EXPECT_NEAR(std::atan2(pts.back().y, pts.back().x), M_PI / 6, 1e-9);
}

namespace
{
RangeReading at(double range, double t, Pose2D sensor)
{
  RangeReading x = r(range, t);
  x.sensor = sensor;
  return x;
}
}  // namespace

TEST(Ultrasonic, ApproachingStaticObjectIsConfirmed)
{
  // 0.5 m/s 로 정면 접근: 0.42 s 사이 센서가 0.21 m 나아가고 거리도 0.21 줄어든다. 물체 자리는 같다.
  UltrasonicChannel ch;
  ch.push(at(1.00, 10.0, {0.0, 0.0, 0.0}));
  ch.push(at(0.79, 10.42, {0.21, 0.0, 0.0}));
  const auto c = ch.confirmed(10.45, 1.0, 2, 0.15);
  ASSERT_TRUE(c.has_value());
  EXPECT_NEAR(c->range, 0.79, 1e-9);
}

TEST(Ultrasonic, SingleSpikeWhileMovingIsIgnored)
{
  UltrasonicChannel ch;
  ch.push(at(1.5, 10.0, {0.0, 0.0, 0.0}));     // 에코 없음
  ch.push(at(0.40, 10.42, {0.21, 0.0, 0.0}));  // 한 번 튄 값
  EXPECT_FALSE(ch.confirmed(10.45, 1.0, 2, 0.15).has_value());
}

TEST(Ultrasonic, InconsistentPairIsRejectedEvenWithSameRange)
{
  // 같은 거리가 두 번 나와도 센서가 0.21 m 나아갔다면 물체 자리가 0.21 m 다르다(0.15 초과).
  UltrasonicChannel ch;
  ch.push(at(0.80, 10.0, {0.0, 0.0, 0.0}));
  ch.push(at(0.80, 10.42, {0.21, 0.0, 0.0}));
  EXPECT_FALSE(ch.confirmed(10.45, 1.0, 2, 0.15).has_value());
  // 중심점 0.4 m 차이
  UltrasonicChannel ch2;
  ch2.push(at(0.60, 10.0, {0.0, 0.0, 0.0}));
  ch2.push(at(0.80, 10.42, {0.20, 0.0, 0.0}));
  EXPECT_FALSE(ch2.confirmed(10.45, 1.0, 2, 0.15).has_value());
}

TEST(Ultrasonic, ArcPointsUseStoredSensorPose)
{
  RangeReading x = at(1.0, 10.0, {2.0, 1.0, M_PI / 2});
  x.fov = 0.0;
  const auto pts = readingToArcPoints(x, 3);
  ASSERT_EQ(pts.size(), 3u);
  for (const auto & p : pts) {
    EXPECT_NEAR(p.x, 2.0, 1e-9);
    EXPECT_NEAR(p.y, 2.0, 1e-9);
  }
}

TEST(Ultrasonic, FreshMeansLatestReadingWithinMaxAge)
{
  UltrasonicChannel ch;
  EXPECT_FALSE(ch.fresh(10.0, 1.0));
  ch.push(r(1.5, 10.0));      // 에코 없음이어도 받은 것은 받은 것
  EXPECT_TRUE(ch.fresh(10.9, 1.0));
  EXPECT_FALSE(ch.fresh(11.1, 1.0));
}

namespace
{
// 옆 채널: 센서가 +y(왼쪽)를 본다. fov 60°.
RangeReading side(double range, double t, double x)
{
  RangeReading s = at(range, t, {x, 0.0, M_PI / 2});
  s.fov = 60.0 * M_PI / 180.0;
  return s;
}
}  // namespace

TEST(Ultrasonic, SideChannelAlongFlatWallIsConfirmedAtCruise)
{
  // 0.8 m 옆 벽을 0.5 m/s 로 나란히 지난다. 0.42 s 사이 센서가 0.21 m 나아가 호 중심이 벽의 다른 점을
  // 가리킨다(중심끼리 0.21 m). 새 중심은 이전 호 위(0.03 m 안)에 있으니 같은 면으로 인정한다.
  UltrasonicChannel ch;
  ch.push(side(0.80, 10.0, 0.0));
  ch.push(side(0.80, 10.42, 0.21));
  const auto c = ch.confirmed(10.45, 1.0, 2, 0.15);
  ASSERT_TRUE(c.has_value());
  EXPECT_NEAR(c->range, 0.80, 1e-9);
}

TEST(Ultrasonic, SideChannelAlongFlatWallIsConfirmedAtSlowSpeed)
{
  UltrasonicChannel ch;
  ch.push(side(0.80, 10.0, 0.0));
  ch.push(side(0.80, 10.42, 0.126));   // 0.3 m/s
  EXPECT_TRUE(ch.confirmed(10.45, 1.0, 2, 0.15).has_value());
}

TEST(Ultrasonic, SideChannelSingleSpikeIsRejected)
{
  UltrasonicChannel ch;
  ch.push(side(1.5, 10.0, 0.0));       // 에코 없음(max_range)
  ch.push(side(0.40, 10.42, 0.21));    // 한 번 튄 값
  EXPECT_FALSE(ch.confirmed(10.45, 1.0, 2, 0.15).has_value());
}

TEST(Ultrasonic, SideChannelNewestFarFromPreviousArcIsRejected)
{
  // 같은 방향에서 0.8 → 0.4: 새 중심이 이전 호에서 0.4 m 떨어져 있다.
  UltrasonicChannel ch;
  ch.push(side(0.80, 10.0, 0.0));
  ch.push(side(0.40, 10.42, 0.0));
  EXPECT_FALSE(ch.confirmed(10.45, 1.0, 2, 0.15).has_value());
}

// ── run48 F4: 노란 콘 충돌 — front_left 가 콘을 4번 봤지만 코드가 전부 버렸다 ──

TEST(Ultrasonic, ContactReadingIsObstacleAtMinRange)
{
  // F4a: min_range(0.02) 이하 값(0.01)은 무효가 아니라 맞닿은 물체다 — min_range 에 둔다.
  UltrasonicChannel ch;
  ch.push(r(0.01, 10.0));
  const auto obs = ch.obstacles(10.05);
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_NEAR(obs[0].range, 0.02, 1e-9);
  const auto pts = readingToArcPoints(obs[0], 3);
  EXPECT_NEAR(pts[1].x, 0.02, 1e-9);
}

TEST(Ultrasonic, NearReadingIsConfirmedBySingleReading)
{
  // F4b: 0.40 m 안은 한 번에 인정한다(두 번째 프레임 0.42 s 를 기다리면 늦다).
  // 대가: 가까운 헛 반사 한 번도 장애물로 본다(보고서에 기록).
  UltrasonicChannel ch;
  ch.push(r(1.5, 10.0));
  ch.push(r(0.30, 10.42));
  const auto obs = ch.obstacles(10.45);
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_NEAR(obs[0].range, 0.30, 1e-9);
}

TEST(Ultrasonic, YellowConeSequenceKeepsObstacleInFront)
{
  // run48 재현: 콘(전역 x = 0.40)에 천천히 다가가며 0.37·0.33 확인 -> 먼 반사 1.18 -> 0.03·0.01·0.01.
  // 먼 반사 하나가 확인을 깨고, 0.01 은 min_range 이하라 버려져 콘을 잊고 부딪혔다.
  UltrasonicChannel ch;
  struct S {double range, t, x;};
  const std::vector<S> seq{{0.37, 10.0, 0.03}, {0.33, 10.42, 0.07}, {1.18, 10.84, 0.12},
    {0.03, 11.26, 0.36}, {0.01, 11.68, 0.39}, {0.01, 12.10, 0.40}};
  size_t k = 0;
  for (double now = 10.40; now <= 12.50 + 1e-9; now += 0.1) {
    while (k < seq.size() && seq[k].t <= now + 1e-9) {
      ch.push(at(seq[k].range, seq[k].t, {seq[k].x, 0.0, 0.0}));
      ++k;
    }
    if (now < 10.42) {continue;}   // 두 번째 값부터(0.33 확인)
    const auto obs = ch.obstacles(now);
    ASSERT_FALSE(obs.empty()) << now;
    const Point2D c = toParent(obs.back().sensor, Point2D{obs.back().range, 0.0});
    EXPECT_NEAR(c.x, 0.40, 0.03) << now;   // 콘 자리
  }
}

TEST(Ultrasonic, SingleFarSpikeDoesNotEraseMemory)
{
  UltrasonicChannel ch;
  ch.push(r(0.60, 10.0));
  ch.push(r(0.60, 10.42));
  ch.push(r(1.18, 10.84));   // 먼 반사 한 번
  const auto obs = ch.obstacles(10.9);
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_NEAR(obs[0].range, 0.60, 1e-9);
}

TEST(Ultrasonic, TwoConsecutiveClearReadingsEraseMemory)
{
  UltrasonicChannel ch;
  ch.push(r(0.60, 10.0));
  ch.push(r(0.60, 10.42));
  ch.push(r(1.5, 10.84));    // 에코 없음
  ch.push(r(1.5, 11.26));
  EXPECT_TRUE(ch.obstacles(11.3).empty());
  // 기억보다 멀리 본 유효값 둘(서로 어긋나 확인은 안 됨)도 지운다
  UltrasonicChannel ch2;
  ch2.push(r(0.60, 10.0));
  ch2.push(r(0.60, 10.42));
  ch2.push(r(1.00, 10.84));
  ch2.push(r(1.30, 11.26));
  EXPECT_TRUE(ch2.obstacles(11.3).empty());
  // 사이에 가까운 값이 끼면 연속이 아니다
  UltrasonicChannel ch3;
  ch3.push(r(0.60, 10.0));
  ch3.push(r(0.60, 10.42));
  ch3.push(r(1.5, 10.84));
  ch3.push(r(0.62, 11.26));
  ch3.push(r(1.5, 11.68));
  EXPECT_FALSE(ch3.obstacles(11.7).empty());
}

TEST(Ultrasonic, MemoryExpiresAfterMemoryTime)
{
  UltrasonicChannel ch;
  ch.push(r(0.60, 10.0));
  ch.push(r(0.60, 10.42));
  EXPECT_FALSE(ch.obstacles(11.9).empty());   // 확인 나이 1.0 s 는 지났지만 기억 1.5 s 안
  EXPECT_TRUE(ch.obstacles(11.95).empty());
}

TEST(Ultrasonic, RangeCapTurnsFarReadingIntoNoEcho)
{
  // 10-08 바퀴 옆 0.40 m: 상한보다 먼 값(벽 0.72 m)은 에코 없음 — 확인도 기억도 안 된다.
  UltrasonicChannel ch;
  ch.push(capRange(r(0.72, 10.0), 0.40));
  ch.push(capRange(r(0.72, 10.42), 0.40));
  EXPECT_TRUE(ch.obstacles(10.5).empty());
  // 상한 안(0.33 m)은 그대로 — 0.40 안이라 한 번에 인정된다.
  ch.push(capRange(r(0.33, 10.84), 0.40));
  EXPECT_FALSE(ch.obstacles(10.9).empty());
  // 상한 0 이하 = 자르지 않음
  EXPECT_DOUBLE_EQ(capRange(r(0.72, 0.0), 0.0).range, 0.72);
  EXPECT_DOUBLE_EQ(capRange(r(0.72, 0.0), 0.40).range, 1.5);
}
