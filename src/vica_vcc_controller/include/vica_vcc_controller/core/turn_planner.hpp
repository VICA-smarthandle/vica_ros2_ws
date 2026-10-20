#pragma once
#include <cmath>
#include <vector>
#include "vica_vcc_controller/core/clearance.hpp"
#include "vica_vcc_controller/core/types.hpp"

namespace vica_vcc_controller::core
{
enum class TurnMode { Arc, Pivot, Blocked };

struct TurnPlan
{
  TurnMode mode{TurnMode::Blocked};
  double radius{0.0};
  int direction{1};
  double min_clearance{0.0};
};

struct TurnParams
{
  std::vector<double> radii{0.2, 0.1};
  double clearance{0.05};
  double keep_clearance{-1.0};
  double arc_w{0.45};
  double pivot_w{0.35};
  double run_in_decel{0.3};
  double sample_angle{0.05};
  double both_sides_angle{170.0 * M_PI / 180.0};
};

double simulateTurnClearance(
  double angle, double radius, int dir, double run_in, const ClearanceFn & f, double sample_angle);
TurnPlan planTurn(
  double heading_error, double v_now, bool pivot_first, const ClearanceFn & f, const TurnParams & p,
  int only_dir = 0, const TurnPlan * keep = nullptr);
inline double keepClearance(const TurnParams & p) {return p.keep_clearance > 0.0 ? p.keep_clearance : p.clearance;}
Twist2D turnCommand(const TurnPlan & t, const TurnParams & p);
}
