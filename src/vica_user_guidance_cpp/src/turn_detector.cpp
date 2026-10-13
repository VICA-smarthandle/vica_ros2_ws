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
  // 연속 샘플 차이를 정규화해 누적한다 — 창 시작과 끝을 바로 빼면 180° 넘는 회전에서 방향이 뒤집힌다.
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
  // 윈도우 밖 샘플을 버린다. 기준점이 필요하므로 최소 1개는 남긴다.
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
  // 같은 부호 회전이 얼마나 이어졌는지 샘플에서 직접 잰다(호출 빈도에 기대지 않게).
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
  // 유지 중. 최소 지속시간은 진입에만 적용한다.
  if (magnitude <= exit_threshold_rad_) {
    active_direction_ = DIRECTION_NONE;
    return PHASE_COMPLETE;
  }
  const int current = sign > 0 ? DIRECTION_LEFT : DIRECTION_RIGHT;
  if (sign != 0 && current != active_direction_) {
    // S자 코너: 반대 방향으로 넘어갔다. 먼저 닫고 다음 tick 에 새로 진입한다.
    active_direction_ = DIRECTION_NONE;
    return PHASE_COMPLETE;
  }
  return PHASE_NOW;
}

TurnDecision TurnDetector::handle_stale()
{
  // stale 이면 누적을 버린다 — 끊긴 사이 돈 만큼 복구 첫 델타가 튀어 오탐하지 않게.
  samples_.clear();
  last_yaw_rad_.reset();
  unwrapped_rad_ = 0.0;
  active_direction_ = DIRECTION_NONE;
  TurnDecision d;
  d.direction = DIRECTION_NONE;
  d.phase = PHASE_IDLE;   // stale 에서는 COMPLETE 를 만들지 않는다
  d.turn_angle_deg = NAN;
  d.sequence_id = sequence_id_;
  d.source_stale = true;
  return d;
}

}  // namespace vica_guidance
