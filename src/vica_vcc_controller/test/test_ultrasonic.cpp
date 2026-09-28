#include <gtest/gtest.h>
#include <cmath>
#include "vica_vcc_controller/core/ultrasonic.hpp"

using namespace vica_vcc_controller::core;

namespace
{
RangeReading r(double range, double t) {return {range, 0.02, 1.5, 1.047, t};}
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
