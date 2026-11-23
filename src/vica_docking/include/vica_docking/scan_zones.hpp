#pragma once

#include <optional>
#include <vector>

#include "vica_docking/geometry.hpp"
#include "vica_docking/v_detector.hpp"

namespace vica_docking
{

int count_bumper_zone(
  const RobotGeometry & g, const std::vector<float> & ranges, double amin, double inc,
  double range_min, const std::optional<VDetection> & det, double depth = 0.03,
  bool skip_behind_apex = false, double v_margin = 0.015, double behind_margin = 0.02);

struct SweepCount
{
  int stop_hits{0};
  int slow_hits{0};
};

SweepCount count_sweep(
  const RobotGeometry & g, const std::vector<float> & ranges, double amin, double inc,
  double sweep_r = 0.64, double slow_r = 0.80);

}
