#include "vica_vcc_controller/core/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vica_vcc_controller::core
{
namespace
{
double sign0(double v) { return v > 0.0 ? 1.0 : (v < 0.0 ? -1.0 : 0.0); }
}  // namespace

Polygon padFootprint(const Polygon & fp, double padding)
{
  Polygon out;
  out.reserve(fp.size());
  for (const auto & p : fp) {
    out.push_back({p.x + sign0(p.x) * padding, p.y + sign0(p.y) * padding});
  }
  return out;
}

double circumscribedRadius(const Polygon & fp)
{
  double r = 0.0;
  for (const auto & p : fp) {r = std::max(r, std::hypot(p.x, p.y));}
  return r;
}

Polygon densifyOutline(const Polygon & fp, double step)
{
  Polygon out;
  const size_t n = fp.size();
  for (size_t i = 0; i < n; ++i) {
    const Point2D & a = fp[i];
    const Point2D & b = fp[(i + 1) % n];
    const double len = std::hypot(b.x - a.x, b.y - a.y);
    const int k = std::max(1, static_cast<int>(std::ceil(len / step)));
    for (int j = 0; j < k; ++j) {
      const double t = static_cast<double>(j) / k;
      out.push_back({a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t});
    }
  }
  return out;
}

bool pointInPolygon(const Point2D & p, const Polygon & poly)
{
  bool inside = false;
  const size_t n = poly.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const Point2D & a = poly[i];
    const Point2D & b = poly[j];
    if (((a.y > p.y) != (b.y > p.y)) &&
      (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x))
    {
      inside = !inside;
    }
  }
  return inside;
}

double pointToSegmentDistance(const Point2D & p, const Point2D & a, const Point2D & b)
{
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double l2 = dx * dx + dy * dy;
  double t = l2 > 0.0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2 : 0.0;
  t = std::clamp(t, 0.0, 1.0);
  return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

double signedDistanceToPolygon(const Point2D & p, const Polygon & poly)
{
  double d = std::numeric_limits<double>::infinity();
  const size_t n = poly.size();
  for (size_t i = 0; i < n; ++i) {
    d = std::min(d, pointToSegmentDistance(p, poly[i], poly[(i + 1) % n]));
  }
  return pointInPolygon(p, poly) ? -d : d;
}

Polygon transformPolygon(const Pose2D & pose, const Polygon & poly)
{
  Polygon out;
  out.reserve(poly.size());
  for (const auto & p : poly) {out.push_back(toParent(pose, p));}
  return out;
}

std::pair<double, double> sweptLateralExtent(
  const Polygon & fp, double radius, double angle, int steps)
{
  const Polygon outline = densifyOutline(fp, 0.01);
  double lo = std::numeric_limits<double>::infinity();
  double hi = -lo;
  for (int i = 0; i <= steps; ++i) {
    const double th = angle * i / steps;
    const Pose2D pose{radius * std::sin(th), radius - radius * std::cos(th), th};
    for (const auto & q : outline) {
      const Point2D g = toParent(pose, q);
      lo = std::min(lo, g.y);
      hi = std::max(hi, g.y);
    }
  }
  return {lo, hi};
}

double uturnSweptWidth(const Polygon & fp, double radius, double angle, int steps)
{
  const auto [lo, hi] = sweptLateralExtent(fp, radius, angle, steps);
  return hi - lo;
}
}  // namespace vica_vcc_controller::core
