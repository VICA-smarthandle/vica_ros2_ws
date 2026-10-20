#pragma once
#include <utility>
#include "vica_vcc_controller/core/types.hpp"

namespace vica_vcc_controller::core
{
Polygon padFootprint(const Polygon & fp, double padding);
double circumscribedRadius(const Polygon & fp);
Polygon densifyOutline(const Polygon & fp, double step);
bool pointInPolygon(const Point2D & p, const Polygon & poly);
double pointToSegmentDistance(const Point2D & p, const Point2D & a, const Point2D & b);
double signedDistanceToPolygon(const Point2D & p, const Polygon & poly);
Polygon transformPolygon(const Pose2D & pose, const Polygon & poly);
std::pair<double, double> sweptLateralExtent(
  const Polygon & fp, double radius, double angle, int steps);
double uturnSweptWidth(const Polygon & fp, double radius, double angle, int steps);
}
