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
