#include "vica_vcc_controller/core/turn_planner.hpp"

#include <algorithm>
#include <limits>
#include <utility>

namespace vica_vcc_controller::core
{
double simulateTurnClearance(
  double angle, double radius, int dir, double run_in, const ClearanceFn & f, double sample_angle)
{
  double c = std::numeric_limits<double>::infinity();
  for (double s = 0.0; s < run_in; s += 0.05) {c = std::min(c, f({s, 0.0, 0.0}));}
  const int n = std::max(1, static_cast<int>(std::ceil(angle / sample_angle)));
  for (int i = 0; i <= n; ++i) {
    const double th = angle * i / n;
    c = std::min(c, f({run_in + radius * std::sin(th), dir * radius * (1.0 - std::cos(th)), dir * th}));
  }
  return c;
}

TurnPlan planTurn(
  double heading, double v_now, bool pivot_first, const ClearanceFn & f, const TurnParams & p)
{
  const int pref = heading >= 0.0 ? 1 : -1;
  const double a = std::abs(heading);
  std::vector<std::pair<int, double>> dirs{{pref, a}};
  if (a >= p.both_sides_angle) {dirs.push_back({-pref, 2.0 * M_PI - a});}

  std::vector<double> order;
  if (pivot_first) {order.push_back(0.0);}
  for (double r : p.radii) {order.push_back(r);}
  if (!pivot_first) {order.push_back(0.0);}

  for (double R : order) {
    TurnPlan best;
    bool found = false;
    for (const auto & [dir, ang] : dirs) {
      const double w = R > 0.0 ? p.arc_w : p.pivot_w;
      const double v_turn = w * R;
      const double run_in = std::max(0.0, v_now * v_now - v_turn * v_turn) / (2.0 * p.run_in_decel);
      const double c = simulateTurnClearance(ang, R, dir, run_in, f, p.sample_angle);
      if (c >= p.clearance && (!found || c > best.min_clearance)) {
        best = {R > 0.0 ? TurnMode::Arc : TurnMode::Pivot, R, dir, c};
        found = true;
      }
    }
    if (found) {return best;}
  }
  return TurnPlan{};
}

Twist2D turnCommand(const TurnPlan & t, const TurnParams & p)
{
  switch (t.mode) {
    case TurnMode::Arc: return {p.arc_w * t.radius, t.direction * p.arc_w};
    case TurnMode::Pivot: return {0.0, t.direction * p.pivot_w};
    default: return {0.0, 0.0};
  }
}
}  // namespace vica_vcc_controller::core
