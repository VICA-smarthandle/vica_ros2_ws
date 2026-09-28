#include <gtest/gtest.h>
#include <cmath>
#include "vica_vcc_controller/core/geometry.hpp"

using namespace vica_vcc_controller::core;

namespace
{
// nav2_params.yaml local_costmap.footprint (2026-09-15 base_link 구동륜 축 이동 뒤 값)
const Polygon kFootprint{{0.151, 0.225}, {0.151, -0.225}, {-0.459, -0.225},
  {-0.569, -0.035}, {-0.569, 0.035}, {-0.459, 0.225}};
}  // namespace

TEST(Types, ToParentAndToChildAreInverse)
{
  const Pose2D frame{1.0, 2.0, 0.7};
  const Point2D p{0.3, -0.4};
  const Point2D back = toChild(frame, toParent(frame, p));
  EXPECT_NEAR(back.x, p.x, 1e-12);
  EXPECT_NEAR(back.y, p.y, 1e-12);
  EXPECT_NEAR(normalizeAngle(3.0 * M_PI), M_PI, 1e-12);
}

TEST(Geometry, PaddingFollowsNav2SignRule)
{
  const Polygon p = padFootprint(kFootprint, 0.05);
  EXPECT_NEAR(p[0].x, 0.201, 1e-9);
  EXPECT_NEAR(p[0].y, 0.275, 1e-9);
  EXPECT_NEAR(p[3].x, -0.619, 1e-9);
  EXPECT_NEAR(p[3].y, -0.085, 1e-9);
}

TEST(Geometry, CircumscribedRadiusMatchesInPlaceWidth)
{
  // 설계서 3.3: 제자리 회전 필요폭 1.250 m = 2 x 0.6248
  EXPECT_NEAR(circumscribedRadius(padFootprint(kFootprint, 0.05)), 0.6248, 0.001);
}

TEST(Geometry, UturnWidthTableOfSpec)
{
  // 설계서 3.3 표를 그대로 재현한다(몸 윤곽을 0.5도 간격으로 쓸어 계산)
  const Polygon fp = padFootprint(kFootprint, 0.05);
  EXPECT_NEAR(uturnSweptWidth(fp, 0.0, M_PI, 360), 0.965, 0.01);
  EXPECT_NEAR(uturnSweptWidth(fp, 0.1, M_PI, 360), 1.072, 0.01);
  EXPECT_NEAR(uturnSweptWidth(fp, 0.2, M_PI, 360), 1.212, 0.01);
  EXPECT_NEAR(uturnSweptWidth(fp, 0.5, M_PI, 360), 1.728, 0.01);
}

TEST(Geometry, SignedDistanceIsNegativeInside)
{
  const Polygon sq{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
  EXPECT_NEAR(signedDistanceToPolygon({0.0, 0.0}, sq), -1.0, 1e-9);
  EXPECT_NEAR(signedDistanceToPolygon({2.0, 0.0}, sq), 1.0, 1e-9);
  EXPECT_TRUE(pointInPolygon({0.5, 0.5}, sq));
  EXPECT_FALSE(pointInPolygon({1.5, 0.5}, sq));
}
