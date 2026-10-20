#pragma once
#include <algorithm>
#include <cmath>
#include "vica_vcc_controller/core/types.hpp"

namespace vica_vcc_controller::core
{
struct OutputParams
{
  double max_v{0.5};
  double max_w{0.5};
  double max_ang_accel{1.2};
  double ramp_v1{0.25};
  double ramp_a1{0.5};
  double accel{0.143};
  double max_decel{1.25};
  double planned_decel{1.25};
  double resync_margin{0.15};
};

struct Desired
{
  Twist2D cmd;
  double curvature{std::nan("")};
  double radius{-1.0};
  bool planned{false};
};

class OutputStage
{
public:
  explicit OutputStage(OutputParams p = {}) : p_(p) {}
  void reset(const Twist2D & measured = {})
  {
    last_ = {std::clamp(measured.v, 0.0, p_.max_v), std::clamp(measured.w, -p_.max_w, p_.max_w)};
  }
  Twist2D apply(const Desired & d, double measured_v, double dt);
  const Twist2D & last() const {return last_;}

private:
  OutputParams p_;
  Twist2D last_;
};
}
