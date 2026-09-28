#include "vica_vcc_controller/core/speed_profile.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace vica_vcc_controller::core
{
double curveLimitAt(double k, const SpeedParams & p)
{
  const double r = 1.0 / std::max(std::abs(k), 1e-6);
  if (r >= p.curve_min_radius) {return p.desired;}
  return std::max(p.min_speed, p.desired * r / p.curve_min_radius);
}

double curvePreviewLimit(const Path & path, const SpeedParams & p)
{
  if (path.size() < 3) {return p.desired;}
  std::vector<Pose2D> pts{path.front()};
  std::vector<double> s{0.0};
  double acc = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    acc += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    if (acc - s.back() >= p.curve_sample) {pts.push_back(path[i]); s.push_back(acc);}
    if (acc > p.preview_dist) {break;}
  }
  double limit = p.desired;
  for (size_t j = 1; j + 1 < pts.size(); ++j) {
    const double ax = pts[j].x - pts[j - 1].x, ay = pts[j].y - pts[j - 1].y;
    const double bx = pts[j + 1].x - pts[j].x, by = pts[j + 1].y - pts[j].y;
    const double cx = pts[j + 1].x - pts[j - 1].x, cy = pts[j + 1].y - pts[j - 1].y;
    const double la = std::hypot(ax, ay), lb = std::hypot(bx, by), lc = std::hypot(cx, cy);
    if (la * lb * lc < 1e-12) {continue;}
    const double k = 2.0 * std::abs(ax * by - ay * bx) / (la * lb * lc);   // Menger 곡률
    const double vl = curveLimitAt(k, p);
    // 곡률을 본 세 점 중 가장 가까운 점(s[j-1])까지 거리로 계산한다 — 가운데 점 기준이면 한 칸(0.2 m) 늦게 줄인다.
    limit = std::min(limit, std::sqrt(vl * vl + 2.0 * p.curve_decel * s[j - 1]));
  }
  return limit;
}

double clearanceLimit(double c, const SpeedParams & p)
{
  if (c >= p.slow_clearance) {return p.desired;}
  return std::max(p.min_speed, p.desired * std::max(c, 0.0) / p.slow_clearance);
}

double approachLimit(double dist, double v, const SpeedParams & p)
{
  if (dist >= p.approach_dist) {return v;}
  return std::min(v, std::max(v * dist / p.approach_dist, p.approach_min));
}
}  // namespace vica_vcc_controller::core
