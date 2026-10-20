#pragma once
#include "vica_vcc_controller/core/types.hpp"

namespace vica_vcc_controller::core
{
struct LookaheadParams
{
  double time{2.5};
  double min_dist{0.6};
  double max_dist{1.2};
};

double lookaheadDistance(double v, const LookaheadParams & p);
double pathLength(const Path & path);
Path pathPrefix(const Path & path, double length);
Point2D carrotOnPath(const Path & path, double L);
Point2D carrotWithEndExtension(
  const Path & path, double L, double max_lateral, double min_length, bool * extended = nullptr);
double carrotTangent(const Path & path, double L);
double curvatureTo(const Point2D & carrot);
Path offsetPath(const Path & path, double d_start, double d_end, double transition_len);
}
