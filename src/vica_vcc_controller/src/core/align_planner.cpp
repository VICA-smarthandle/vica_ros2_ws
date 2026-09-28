#include "vica_vcc_controller/core/align_planner.hpp"

#include <algorithm>
#include <cmath>

namespace vica_vcc_controller::core
{
void AlignPlanner::reset()
{
  phase_ = AlignPhase::Idle;
  lag_ = p_.motor_lag;
  last_w_ = cut_err_ = cut_w_ = 0.0;
  settle_since_ = -1.0;
  attempts_ = 0;
}

void AlignPlanner::begin(double err)
{
  ++attempts_;
  dir_ = err >= 0.0 ? 1.0 : -1.0;
  last_w_ = 0.0;
  phase_ = AlignPhase::Rotating;
}

double AlignPlanner::update(double err, double measured_w, double tol, double now, double dt)
{
  switch (phase_) {
    case AlignPhase::Idle:
      if (std::abs(err) <= tol) {phase_ = AlignPhase::Done; return 0.0;}
      begin(err);
      [[fallthrough]];
    case AlignPhase::Rotating: {
        const double rem = dir_ * err;
        // 모터 지연만큼 일찍 끊는다 — run41 넘침 ±16~21° 의 원인이 지연 0.3~0.5 s 였다.
        if (rem <= 0.0 || rem <= std::abs(last_w_) * lag_) {
          cut_err_ = err;
          cut_w_ = last_w_;
          last_w_ = 0.0;
          settle_since_ = -1.0;
          phase_ = AlignPhase::Settling;
          return 0.0;
        }
        const double w = std::min(
          {p_.w_max, std::sqrt(2.0 * p_.alpha * rem), std::abs(last_w_) + p_.alpha * dt});
        last_w_ = dir_ * w;
        return last_w_;
      }
    case AlignPhase::Settling:
      if (std::abs(measured_w) >= p_.stopped_w) {settle_since_ = -1.0; return 0.0;}
      if (settle_since_ < 0.0) {settle_since_ = now;}
      if (now - settle_since_ < p_.settle - 1e-9) {return 0.0;}
      if (std::abs(cut_w_) > 0.05) {
        const double extra = dir_ * (cut_err_ - err);   // 끊은 뒤 더 돈 각
        lag_ = std::clamp(extra / std::abs(cut_w_), p_.lag_min, p_.lag_max);
      }
      if (std::abs(err) <= tol) {phase_ = AlignPhase::Done; return 0.0;}
      if (attempts_ >= p_.max_attempts) {phase_ = AlignPhase::Failed; return 0.0;}
      begin(err);
      return 0.0;
    case AlignPhase::Done:
      // 정렬 뒤 AMCL 재정렬 등으로 다시 벗어나면 남은 횟수 안에서만 다시 돈다.
      if (std::abs(err) > tol) {
        if (attempts_ >= p_.max_attempts) {phase_ = AlignPhase::Failed; return 0.0;}
        begin(err);
      }
      return 0.0;
    case AlignPhase::Failed:
    default:
      return 0.0;
  }
}
}  // namespace vica_vcc_controller::core
