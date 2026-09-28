#pragma once
#include <utility>
#include "vica_vcc_controller/core/types.hpp"

namespace vica_vcc_controller::core
{
// nav2_costmap_2d::padFootprint 와 같은 규칙: 각 꼭짓점을 부호 방향으로 padding 만큼 민다.
Polygon padFootprint(const Polygon & fp, double padding);
double circumscribedRadius(const Polygon & fp);
// 윤곽을 step 간격 점으로 바꾼다(꼭짓점 포함).
Polygon densifyOutline(const Polygon & fp, double step);
bool pointInPolygon(const Point2D & p, const Polygon & poly);
double pointToSegmentDistance(const Point2D & p, const Point2D & a, const Point2D & b);
// 다각형 윤곽까지 거리. 안쪽이면 음수.
double signedDistanceToPolygon(const Point2D & p, const Polygon & poly);
Polygon transformPolygon(const Pose2D & pose, const Polygon & poly);
// 왼쪽으로 반지름 radius(0 = 제자리) 호를 angle 만큼 돌 때 몸이 쓰는 옆(y) 범위 [lo, hi].
std::pair<double, double> sweptLateralExtent(
  const Polygon & fp, double radius, double angle, int steps);
double uturnSweptWidth(const Polygon & fp, double radius, double angle, int steps);
}  // namespace vica_vcc_controller::core
