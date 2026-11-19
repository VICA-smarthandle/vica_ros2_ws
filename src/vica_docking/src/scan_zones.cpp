#include "vica_docking/scan_zones.hpp"

#include <cmath>

namespace vica_docking
{

int count_bumper_zone(
  const RobotGeometry & g, const std::vector<float> & ranges, double amin, double inc,
  double range_min, const std::optional<VDetection> & det, double depth, bool skip_behind_apex,
  double v_margin, double behind_margin)
{
  int near = 0;
  for (size_t i = 0; i < ranges.size(); ++i) {
    const double r = static_cast<double>(ranges[i]);
    if (!std::isfinite(r) || r < range_min) {
      continue;
    }
    const Vec2 b = beam_to_base(g, r, amin + static_cast<double>(i) * inc);
    if (!(b.x > g.bumper_x && b.x < g.bumper_x + depth && std::abs(b.y) < g.half_w)) {
      continue;
    }
    if (det) {
      const Vec2 pz = b - det->apex;
      if (skip_behind_apex && dot(pz, det->u) < behind_margin) {
        continue;
      }
      bool on_v = false;
      const Vec2 dirs[2] = {det->da, det->db};
      const double lens[2] = {det->la, det->lb};
      for (int k = 0; k < 2; ++k) {
        const double along = dot(pz, dirs[k]);
        const double perp = std::abs(dot(pz, left_of(dirs[k])));
        if (perp < v_margin && along > -0.02 && along < lens[k] + 0.03) {
          on_v = true;
        }
      }
      if (on_v) {
        continue;
      }
    }
    ++near;
  }
  return near;
}

SweepCount count_sweep(
  const RobotGeometry & g, const std::vector<float> & ranges, double amin, double inc,
  double sweep_r, double slow_r)
{
  constexpr double kLim = 120.0 * 3.14159265358979323846 / 180.0;
  SweepCount c;
  for (size_t i = 0; i < ranges.size(); ++i) {
    const double r = static_cast<double>(ranges[i]);
    const double a = amin + static_cast<double>(i) * inc;
    if (!std::isfinite(r) || !(r > 0.25) || !(std::abs(a) < kLim)) {
      continue;
    }
    // 실기 도구는 회전 감시에서 laser_y 를 더하지 않았다(값이 0 이라 같다)
    const Vec2 b = beam_to_base(g, r, a);
    const double rad = std::hypot(b.x, b.y);
    if (rad < sweep_r) {
      ++c.stop_hits;
    }
    if (rad < slow_r) {
      ++c.slow_hits;
    }
  }
  return c;
}

}  // namespace vica_docking
