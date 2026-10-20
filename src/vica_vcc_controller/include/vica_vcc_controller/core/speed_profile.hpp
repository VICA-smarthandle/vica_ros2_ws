#pragma once
#include "vica_vcc_controller/core/types.hpp"

namespace vica_vcc_controller::core
{
struct SpeedParams
{
  double desired{0.5};
  double min_speed{0.12};
  double curve_min_radius{1.2};
  double curve_decel{0.3};
  double preview_dist{1.5};
  double curve_sample{0.2};
  double slow_clearance{0.35};
  double approach_dist{0.6};
  double approach_min{0.05};
};

double curveLimitAt(double curvature, const SpeedParams & p);
double curvePreviewLimit(const Path & path, const SpeedParams & p);
double clearanceLimit(double clearance, const SpeedParams & p);
double approachLimit(double dist_to_end, double v, const SpeedParams & p);
}
