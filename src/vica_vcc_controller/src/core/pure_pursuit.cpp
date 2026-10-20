#include "vica_vcc_controller/core/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>

namespace vica_vcc_controller::core
{
namespace
{
Point2D circleSegmentIntersection(const Point2D & p1, const Point2D & p2, double r)
{
  const double x1 = p1.x, x2 = p2.x, y1 = p1.y, y2 = p2.y;
  const double dx = x2 - x1, dy = y2 - y1;
  const double dr2 = dx * dx + dy * dy;
  const double D = x1 * y2 - x2 * y1;
  const double d1 = x1 * x1 + y1 * y1;
  const double d2 = x2 * x2 + y2 * y2;
  const double dd = d2 - d1;
  const double sqrt_term = std::sqrt(std::max(0.0, r * r * dr2 - D * D));
  return {(D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2,
    (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2};
}

size_t carrotIndex(const Path & path, double L)
{
  for (size_t i = 0; i < path.size(); ++i) {
    if (std::hypot(path[i].x, path[i].y) >= L) {return i;}
  }
  return path.size();
}
}

double lookaheadDistance(double v, const LookaheadParams & p)
{
  return std::clamp(std::abs(v) * p.time, p.min_dist, p.max_dist);
}

double pathLength(const Path & path)
{
  double s = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    s += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return s;
}

Path pathPrefix(const Path & path, double length)
{
  Path out;
  double s = 0.0;
  for (size_t i = 0; i < path.size(); ++i) {
    if (i > 0) {s += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);}
    out.push_back(path[i]);
    if (s >= length) {break;}
  }
  return out;
}

Point2D carrotOnPath(const Path & path, double L)
{
  const size_t i = carrotIndex(path, L);
  if (i == path.size()) {return {path.back().x, path.back().y};}
  if (i == 0) {return {path[0].x, path[0].y};}
  return circleSegmentIntersection({path[i - 1].x, path[i - 1].y}, {path[i].x, path[i].y}, L);
}

Point2D carrotWithEndExtension(
  const Path & path, double L, double max_lateral, double min_length, bool * extended)
{
  if (extended) {*extended = false;}
  if (path.empty()) {return {0.0, 0.0};}
  if (carrotIndex(path, L) < path.size() || path.size() < 2 || pathLength(path) < min_length) {
    return carrotOnPath(path, L);
  }
  const Pose2D & e = path.back();
  if (e.x <= 0.0) {return {e.x, e.y};}
  constexpr double kGeomBack = 0.3;
  std::size_t j = path.size() - 1;
  while (j > 0 && std::hypot(path[j].x - e.x, path[j].y - e.y) < kGeomBack) {--j;}
  double ux = e.x - path[j].x, uy = e.y - path[j].y;
  const double n = std::hypot(ux, uy);
  if (n < 1e-6) {return {e.x, e.y};}
  ux /= n; uy /= n;
  const double lateral = std::abs(e.x * uy - e.y * ux);
  if (lateral > max_lateral) {return {e.x, e.y};}
  const double b = e.x * ux + e.y * uy;
  const double c = e.x * e.x + e.y * e.y - L * L;
  const double disc = b * b - c;
  if (disc < 0.0) {return {e.x, e.y};}
  const double t = -b + std::sqrt(disc);
  if (t < 0.0) {return {e.x, e.y};}
  if (extended) {*extended = true;}
  return {e.x + t * ux, e.y + t * uy};
}

double carrotTangent(const Path & path, double L)
{
  if (path.size() < 2) {return path.empty() ? 0.0 : path.front().yaw;}
  size_t i = carrotIndex(path, L);
  if (i == path.size()) {i = path.size() - 1;}
  if (i == 0) {i = 1;}
  return std::atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x);
}

double curvatureTo(const Point2D & c)
{
  const double d2 = c.x * c.x + c.y * c.y;
  return d2 > 1e-9 ? 2.0 * c.y / d2 : 0.0;
}

Path offsetPath(const Path & path, double d_start, double d_end, double transition_len)
{
  Path out;
  out.reserve(path.size());
  double s = 0.0;
  for (size_t i = 0; i < path.size(); ++i) {
    if (i > 0) {s += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);}
    const size_t a = i == 0 ? 0 : i - 1;
    const size_t b = std::min(i + 1, path.size() - 1);
    double tx = path[b].x - path[a].x, ty = path[b].y - path[a].y;
    double tn = std::hypot(tx, ty);
    if (tn < 1e-9) {tx = std::cos(path[i].yaw); ty = std::sin(path[i].yaw); tn = 1.0;}
    tx /= tn; ty /= tn;
    const double u = transition_len > 1e-9 ? std::clamp(s / transition_len, 0.0, 1.0) : 1.0;
    const double d = d_start + (d_end - d_start) * u;
    out.push_back({path[i].x - ty * d, path[i].y + tx * d, std::atan2(ty, tx)});
  }
  return out;
}
}
