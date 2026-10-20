#include "vica_user_guidance_cpp/turn_detector.hpp"

#include <cmath>

#include "vica_user_guidance_cpp/guidance.hpp"
#include "vica_user_guidance_cpp/pymath.hpp"

namespace vica_guidance
{

double normalize_angle(double radians)
{
  return py_atan2(py_sin(radians), py_cos(radians));
}

double yaw_from_quaternion(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return py_atan2(siny_cosp, cosy_cosp);
}

TurnDetector::TurnDetector(
  int64_t window_ns, double enter_threshold_rad, double exit_threshold_rad,
  int64_t min_duration_ns, int64_t odom_timeout_ns)
: window_ns_(window_ns), enter_threshold_rad_(enter_threshold_rad),
  exit_threshold_rad_(exit_threshold_rad), min_duration_ns_(min_duration_ns),
  odom_timeout_ns_(odom_timeout_ns)
{
}

void TurnDetector::add_odom(double yaw_rad, int64_t now_ns)
{
  const double delta = last_yaw_rad_ ? normalize_angle(yaw_rad - *last_yaw_rad_) : 0.0;
  last_yaw_rad_ = yaw_rad;
  last_odom_ns_ = now_ns;
  unwrapped_rad_ += delta;
  samples_.emplace_back(now_ns, unwrapped_rad_);
  prune(now_ns);
}

TurnDecision TurnDetector::evaluate(int64_t now_ns)
{
  if (!is_fresh_ns(last_odom_ns_, now_ns, odom_timeout_ns_)) {return handle_stale();}

  prune(now_ns);
  const double accum_rad = window_accumulation();
  const int sign = accum_rad > 0 ? 1 : (accum_rad < 0 ? -1 : 0);
  const double magnitude = std::fabs(accum_rad);
  const int phase = transition(sign, magnitude);

  TurnDecision d;
  d.direction = active_direction_;
  d.phase = phase;
  d.turn_angle_deg = py_degrees(accum_rad);
  d.sequence_id = sequence_id_;
  d.source_stale = false;
  return d;
}

void TurnDetector::prune(int64_t now_ns)
{
  while (samples_.size() > 1 &&
    static_cast<__int128>(now_ns) - samples_.front().first > window_ns_)
  {
    samples_.pop_front();
  }
}

double TurnDetector::window_accumulation() const
{
  if (samples_.size() < 2) {return 0.0;}
  return samples_.back().second - samples_.front().second;
}

int64_t TurnDetector::held_ns(int sign) const
{
  if (sign == 0 || samples_.size() < 2) {return 0;}
  const auto & newest = samples_.back();
  int64_t boundary_ns = newest.first;
  for (auto it = samples_.rbegin(); it != samples_.rend(); ++it) {
    const double diff = newest.second - it->second;
    const int s = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
    if (s == sign || s == 0) {boundary_ns = it->first;} else {break;}
  }
  return newest.first - boundary_ns;
}

int TurnDetector::transition(int sign, double magnitude)
{
  if (active_direction_ == DIRECTION_NONE) {
    const bool entered = magnitude >= enter_threshold_rad_ &&
      held_ns(sign) >= min_duration_ns_ && sign != 0;
    if (entered) {
      active_direction_ = sign > 0 ? DIRECTION_LEFT : DIRECTION_RIGHT;
      sequence_id_ += 1;
      return PHASE_NOW;
    }
    return PHASE_IDLE;
  }
  if (magnitude <= exit_threshold_rad_) {
    active_direction_ = DIRECTION_NONE;
    return PHASE_COMPLETE;
  }
  const int current = sign > 0 ? DIRECTION_LEFT : DIRECTION_RIGHT;
  if (sign != 0 && current != active_direction_) {
    active_direction_ = DIRECTION_NONE;
    return PHASE_COMPLETE;
  }
  return PHASE_NOW;
}

TurnDecision TurnDetector::handle_stale()
{
  samples_.clear();
  last_yaw_rad_.reset();
  unwrapped_rad_ = 0.0;
  active_direction_ = DIRECTION_NONE;
  TurnDecision d;
  d.direction = DIRECTION_NONE;
  d.phase = PHASE_IDLE;
  d.turn_angle_deg = NAN;
  d.sequence_id = sequence_id_;
  d.source_stale = true;
  return d;
}

}
