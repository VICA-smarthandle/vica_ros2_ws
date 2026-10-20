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
  ch.push(r(0.85, 10.46));
  const auto c = ch.confirmed(10.5, 1.0, 2, 0.15);
  ASSERT_TRUE(c.has_value());
  EXPECT_NEAR(c->range, 0.85, 1e-9);
}

TEST(Ultrasonic, SingleSpikeIsIgnored)
{
  UltrasonicChannel ch;
  ch.push(r(1.5, 10.0));
  ch.push(r(0.40, 10.46));
  EXPECT_FALSE(ch.confirmed(10.5, 1.0, 2, 0.15).has_value());
  UltrasonicChannel ch2;
  ch2.push(r(0.40, 10.0));
  ch2.push(r(0.80, 10.46));
  EXPECT_FALSE(ch2.confirmed(10.5, 1.0, 2, 0.15).has_value());
}

TEST(Ultrasonic, StaleReadingsAreDropped)
{
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
}

TEST(Ultrasonic, ApproachingStaticObjectIsConfirmed)
{
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
  ch.push(at(1.5, 10.0, {0.0, 0.0, 0.0}));
  ch.push(at(0.40, 10.42, {0.21, 0.0, 0.0}));
  EXPECT_FALSE(ch.confirmed(10.45, 1.0, 2, 0.15).has_value());
}

TEST(Ultrasonic, InconsistentPairIsRejectedEvenWithSameRange)
{
  UltrasonicChannel ch;
  ch.push(at(0.80, 10.0, {0.0, 0.0, 0.0}));
  ch.push(at(0.80, 10.42, {0.21, 0.0, 0.0}));
  EXPECT_FALSE(ch.confirmed(10.45, 1.0, 2, 0.15).has_value());
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
  ch.push(r(1.5, 10.0));
  EXPECT_TRUE(ch.fresh(10.9, 1.0));
  EXPECT_FALSE(ch.fresh(11.1, 1.0));
}

namespace
{
RangeReading side(double range, double t, double x)
{
  RangeReading s = at(range, t, {x, 0.0, M_PI / 2});
  s.fov = 60.0 * M_PI / 180.0;
  return s;
}
}

TEST(Ultrasonic, SideChannelAlongFlatWallIsConfirmedAtCruise)
{
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
  ch.push(side(0.80, 10.42, 0.126));
  EXPECT_TRUE(ch.confirmed(10.45, 1.0, 2, 0.15).has_value());
}

TEST(Ultrasonic, SideChannelSingleSpikeIsRejected)
{
  UltrasonicChannel ch;
  ch.push(side(1.5, 10.0, 0.0));
  ch.push(side(0.40, 10.42, 0.21));
  EXPECT_FALSE(ch.confirmed(10.45, 1.0, 2, 0.15).has_value());
}

TEST(Ultrasonic, SideChannelNewestFarFromPreviousArcIsRejected)
{
  UltrasonicChannel ch;
  ch.push(side(0.80, 10.0, 0.0));
  ch.push(side(0.40, 10.42, 0.0));
  EXPECT_FALSE(ch.confirmed(10.45, 1.0, 2, 0.15).has_value());
}

TEST(Ultrasonic, ContactReadingIsObstacleAtMinRange)
{
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
  UltrasonicChannel ch;
  ch.push(r(1.5, 10.0));
  ch.push(r(0.30, 10.42));
  const auto obs = ch.obstacles(10.45);
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_NEAR(obs[0].range, 0.30, 1e-9);
}

TEST(Ultrasonic, YellowConeSequenceKeepsObstacleInFront)
{
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
    if (now < 10.42) {continue;}
    const auto obs = ch.obstacles(now);
    ASSERT_FALSE(obs.empty()) << now;
    const Point2D c = toParent(obs.back().sensor, Point2D{obs.back().range, 0.0});
    EXPECT_NEAR(c.x, 0.40, 0.03) << now;
  }
}

TEST(Ultrasonic, SingleFarSpikeDoesNotEraseMemory)
{
  UltrasonicChannel ch;
  ch.push(r(0.60, 10.0));
  ch.push(r(0.60, 10.42));
  ch.push(r(1.18, 10.84));
  const auto obs = ch.obstacles(10.9);
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_NEAR(obs[0].range, 0.60, 1e-9);
}

TEST(Ultrasonic, TwoConsecutiveClearReadingsEraseMemory)
{
  UltrasonicChannel ch;
  ch.push(r(0.60, 10.0));
  ch.push(r(0.60, 10.42));
  ch.push(r(1.5, 10.84));
  ch.push(r(1.5, 11.26));
  EXPECT_TRUE(ch.obstacles(11.3).empty());
  UltrasonicChannel ch2;
  ch2.push(r(0.60, 10.0));
  ch2.push(r(0.60, 10.42));
  ch2.push(r(1.00, 10.84));
  ch2.push(r(1.30, 11.26));
  EXPECT_TRUE(ch2.obstacles(11.3).empty());
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
  EXPECT_FALSE(ch.obstacles(11.9).empty());
  EXPECT_TRUE(ch.obstacles(11.95).empty());
}

TEST(Ultrasonic, RangeCapTurnsFarReadingIntoNoEcho)
{
  UltrasonicChannel ch;
  ch.push(capRange(r(0.72, 10.0), 0.40));
  ch.push(capRange(r(0.72, 10.42), 0.40));
  EXPECT_TRUE(ch.obstacles(10.5).empty());
  ch.push(capRange(r(0.33, 10.84), 0.40));
  EXPECT_FALSE(ch.obstacles(10.9).empty());
  EXPECT_DOUBLE_EQ(capRange(r(0.72, 0.0), 0.0).range, 0.72);
  EXPECT_DOUBLE_EQ(capRange(r(0.72, 0.0), 0.40).range, 1.5);
}

namespace
{
UltrasonicParams passParams()
{
  UltrasonicParams p;
  p.pass_memory = true;
  p.memory_time = 30.0;
  return p;
}
UltrasonicChannel seenCone()
{
  UltrasonicChannel ch(passParams());
  ch.push(r(0.60, 10.0));
  ch.push(r(0.60, 10.42));
  return ch;
}
}

TEST(UltrasonicPass, ConeLeavingTheBeamByTurningIsRemembered)
{
  UltrasonicChannel ch = seenCone();
  ch.push(at(1.5, 10.84, {0.0, 0.0, 0.9}));
  ch.push(at(1.5, 11.26, {0.0, 0.0, 0.9}));
  ch.push(at(1.5, 11.68, {0.0, 0.0, 0.9}));
  const auto obs = ch.obstacles(11.7);
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_NEAR(obs[0].range, 0.60, 1e-9);
}

TEST(UltrasonicPass, SeeingThroughTheSpotTwiceErasesIt)
{
  UltrasonicChannel ch = seenCone();
  ch.push(r(1.5, 10.84));
  EXPECT_FALSE(ch.obstacles(10.9).empty());
  ch.push(r(1.5, 11.26));
  EXPECT_TRUE(ch.obstacles(11.3).empty());
}

TEST(UltrasonicPass, StandingStillDoesNotExpireTheMemory)
{
  UltrasonicChannel ch = seenCone();
  ch.forget({0.0, 0.0, 0.0}, -0.62);
  EXPECT_FALSE(ch.obstacles(30.0).empty());
}

TEST(UltrasonicPass, PassedOrFarMemoryIsForgotten)
{
  UltrasonicChannel ch = seenCone();
  ch.forget({1.30, 0.0, 0.0}, -0.62);
  EXPECT_FALSE(ch.obstacles(12.0).empty());
  ch.forget({1.50, 0.0, 0.0}, -0.62);
  EXPECT_TRUE(ch.obstacles(12.0).empty());

  UltrasonicChannel far = seenCone();
  far.forget({0.6, 2.5, -M_PI / 2.0}, -0.62);
  EXPECT_TRUE(far.obstacles(12.0).empty());
}

TEST(UltrasonicPass, SafetyCapAndSlots)
{
  UltrasonicChannel ch = seenCone();
  EXPECT_FALSE(ch.obstacles(39.0).empty());
  EXPECT_TRUE(ch.obstacles(41.0).empty());
  UltrasonicChannel many(passParams());
  for (int k = 0; k < 6; ++k) {
    const double y = 0.5 * k;
    many.push(at(0.30, 10.0 + k, {0.0, y, 0.0}));
  }
  EXPECT_EQ(many.memoryCount(), 4u);
}
