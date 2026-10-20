#include "vica_vcc_controller/core/output_stage.hpp"

#include <algorithm>

namespace vica_vcc_controller::core
{
Twist2D OutputStage::apply(const Desired & d, double measured_v, double dt)
{
  last_.v = std::min(last_.v, std::max(0.0, measured_v) + p_.resync_margin);

  double v = std::clamp(d.cmd.v, 0.0, p_.max_v);
  if (v > last_.v) {
    const double a = last_.v < p_.ramp_v1 ? p_.ramp_a1 : p_.accel;
    v = std::min({v, last_.v + a * dt, last_.v < p_.ramp_v1 ? std::max(p_.ramp_v1, last_.v) : v});
  } else {
    const double decel = d.planned ? std::min(p_.planned_decel, p_.max_decel) : p_.max_decel;
    v = std::max(v, last_.v - decel * dt);
  }

  if (!std::isnan(d.curvature) && std::abs(v * d.curvature) > p_.max_w) {
    v = p_.max_w / std::abs(d.curvature);
  }

  double w_target = std::isnan(d.curvature) ? d.cmd.w : v * d.curvature;
  w_target = std::clamp(w_target, -p_.max_w, p_.max_w);
  const double w = std::clamp(
    w_target, last_.w - p_.max_ang_accel * dt, last_.w + p_.max_ang_accel * dt);

  if (d.radius > 0.0) {v = std::min(v, std::abs(w) * d.radius);}
  last_ = {v, w};
  return last_;
}
}
