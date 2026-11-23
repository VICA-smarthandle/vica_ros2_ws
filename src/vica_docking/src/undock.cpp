#include "vica_docking/undock.hpp"

#include <algorithm>
#include <cmath>

#include "vica_docking/scan_zones.hpp"

namespace vica_docking
{

const char * phase_name(UndockPhase p)
{
  switch (p) {
    case UndockPhase::kWait: return "WAIT";
    case UndockPhase::kPrecheck: return "PRECHECK";
    case UndockPhase::kBack: return "BACK";
    case UndockPhase::kSettleBack: return "SETTLE_BACK";
    case UndockPhase::kTurn: return "TURN";
    case UndockPhase::kSettleTurn: return "SETTLE_TURN";
    case UndockPhase::kDone: return "DONE";
    case UndockPhase::kAbort: return "ABORT";
  }
  return "?";
}

UndockController::UndockController(UndockParams p, RobotGeometry g, VDetectorParams vp)
: p_(p), g_(g), det_(vp, g), turn_target_deg_(p.turn_deg), turn_sign_(p.turn_dir) {}

void UndockController::start(double now)
{
  t0_ = now;
}

void UndockController::start_turn_only(double now, double remaining_deg)
{
  t0_ = now;
  turn_target_deg_ = std::abs(remaining_deg);
  turn_sign_ = remaining_deg >= 0 ? p_.turn_dir : -p_.turn_dir;
  yaw_turn0_ = yaw_;
  hold_total_ = 0.0;
  if (turn_target_deg_ <= p_.turn_done_deg) {
    result_.success = true;
    phase_ = UndockPhase::kDone;
    return;
  }
  phase_ = UndockPhase::kTurn;
}

double UndockController::turn_remaining_deg() const
{
  if (abort_phase_ == UndockPhase::kSettleBack) {
    return p_.turn_deg;
  }
  if (abort_phase_ == UndockPhase::kTurn || abort_phase_ == UndockPhase::kSettleTurn) {
    const double left = turn_target_deg_ - deg(std::abs(yaw_ - yaw_turn0_));
    return turn_sign_ == p_.turn_dir ? left : -left;
  }
  return 0.0;
}

void UndockController::abort(Fail f, const std::string & why)
{
  if (finished()) {
    return;
  }
  result_.fail = f;
  result_.why = why;
  result_.success = false;
  abort_phase_ = phase_;
  phase_ = UndockPhase::kAbort;
}

void UndockController::on_ultrasonic(int index, double now, double range, double max_range)
{
  if (index < 0 || index > 3) {
    return;
  }
  const bool clear = !std::isfinite(range) || range >= max_range - 0.005;
  us_has_[index] = true;
  us_r_[index] = clear ? std::numeric_limits<double>::infinity() : range;
  us_t_[index] = now;
}

void UndockController::on_gyro(double now, double wz)
{
  if (has_imu_t_) {
    const double dt = now - imu_t_;
    if (dt > 0 && dt < 0.2) {
      yaw_ += wz * dt;
    }
  }
  imu_t_ = now;
  has_imu_t_ = true;
}

UndockController::UsMin UndockController::us_min(double now, bool all4) const
{
  UsMin m;
  m.fresh = true;
  const int n = all4 ? 4 : 2;
  for (int i = 0; i < n; ++i) {
    if (!us_has_[i] || now - us_t_[i] > p_.us_fresh_s) {
      m.fresh = false;
    } else {
      m.d = m.d ? std::min(*m.d, us_r_[i]) : us_r_[i];
    }
  }
  return m;
}

void UndockController::on_scan(
  double now, const std::vector<float> & ranges, double amin, double inc, double range_min)
{
  if (finished()) {
    return;
  }
  ++scans_;
  const auto det = det_.detect(ranges, amin, inc, range_min);
  if (det) {
    const DockPose pz = pose_in_dock(*det);
    x_ = pz.x;
    th_ = pz.th;
    x_t_ = now;
    has_x_ = true;
    if (phase_ == UndockPhase::kWait) {
      const bool ok = has_streak_ref_ && std::abs(pz.x - streak_ref_) < 0.01;
      streak_ = ok ? streak_ + 1 : 1;
      streak_ref_ = pz.x;
      has_streak_ref_ = true;
    }
  }
  const SweepCount sc = count_sweep(g_, ranges, amin, inc, p_.sweep_r, p_.slow_r);
  sweep_hits_ = sc.stop_hits;
  slow_hits_ = sc.slow_hits;
}

bool UndockController::set_hold(double now, const std::string & why, bool clear_ok)
{
  if (!why.empty()) {
    has_clear_since_ = false;
    if (!has_hold_) {
      has_hold_ = true;
      hold_since_ = now;
    }
    hold_reason_ = why;
  } else if (!has_hold_) {
    return false;
  } else if (!clear_ok) {
    has_clear_since_ = false;
  } else {
    if (!has_clear_since_) {
      has_clear_since_ = true;
      clear_since_ = now;
    }
    if (now - clear_since_ >= p_.release_s) {
      hold_total_ += now - hold_since_;
      has_hold_ = false;
      hold_reason_.clear();
      has_clear_since_ = false;
      return false;
    }
  }
  if (now - hold_since_ > p_.wait_timeout) {
    abort(Fail::kBlocked, "기다려도 안 풀림: " + hold_reason_);
  }
  return true;
}

Cmd UndockController::on_timer(double now)
{
  if (finished()) {
    return {};
  }
  if (now - t0_ > p_.max_time) {
    abort(Fail::kTimeout, "시간 초과");
    return {};
  }
  const UsMin rear = us_min(now, false);
  double v = 0.0, w = 0.0;
  if (phase_ == UndockPhase::kWait) {
    if (streak_ >= 5 && rear.fresh) {
      x_start_ = x_;
      result_.start_x = x_;
      phase_ = UndockPhase::kPrecheck;
    } else if (now - t0_ > p_.find_timeout) {
      abort(
        streak_ >= 5 ? Fail::kSensor : Fail::kNotFound,
        streak_ >= 5 ? "뒤 초음파 값이 안 들어옴" : "V자를 5회 연속으로 찾지 못함");
      return {};
    }
  } else if (phase_ == UndockPhase::kPrecheck) {
    std::string why;
    if (!rear.fresh) {
      why = "뒤 초음파 값이 안 들어옴";
    } else if (rear.d && *rear.d < p_.clear_pre) {
      why = "출발 전 뒤에 물체";
    }
    const bool ok = rear.fresh && (!rear.d || *rear.d >= p_.clear_pre);
    if (!set_hold(now, why, ok) && !finished()) {
      phase_ = UndockPhase::kBack;
      x_start_ = x_;
      back_ = std::max(0.0, g_.bumper_x + p_.until_cm / 100.0 - x_);
    }
  } else if (phase_ == UndockPhase::kBack) {
    const double traveled = has_x_ ? x_ - x_start_ : 0.0;
    const double remain = back_ - traveled;
    if (remain <= p_.lead_back) {
      phase_ = UndockPhase::kSettleBack;
      back_stop_x_ = x_;
      settle_until_ = now + 1.0;
      settle_x_.clear();
    } else {
      std::string why;
      const bool seen = now - x_t_ <= 0.4;
      if (!seen) {
        why = "V자(거리 기준)를 못 봄";
      } else if (!rear.fresh) {
        why = "뒤 초음파 값이 안 들어옴";
      } else if (rear.d && *rear.d < remain + p_.stop_margin) {
        why = "뒤에 물체";
      }
      const bool ok = seen && rear.fresh &&
        (!rear.d || *rear.d >= remain + p_.stop_margin + p_.release_gap);
      if (!set_hold(now, why, ok) && !finished()) {
        v = -p_.v_back;
        w = std::abs(th_) < rad(0.5) ? 0.0 : std::clamp(-1.0 * th_, -0.15, 0.15);
      }
    }
  } else if (phase_ == UndockPhase::kSettleBack) {
    if (has_x_ && now - x_t_ < 0.2) {
      settle_x_.push_back(x_);
    }
    if (now >= settle_until_ && !settle_x_.empty()) {
      double s = 0.0;
      for (double xv : settle_x_) {
        s += xv;
      }
      const double fx = s / static_cast<double>(settle_x_.size());
      result_.back_cm = (fx - x_start_) * 100.0;
      result_.back_overshoot_cm = (fx - back_stop_x_) * 100.0;
      if (p_.turn_deg <= 0) {
        result_.hold_s = hold_total_;
        result_.success = true;
        phase_ = UndockPhase::kDone;
        return {};
      }
      phase_ = UndockPhase::kTurn;
      yaw_turn0_ = yaw_;
      hold_total_ = 0.0;
    }
  } else if (phase_ == UndockPhase::kTurn) {
    const double turned = std::abs(yaw_ - yaw_turn0_);
    const double lead = p_.lead_turn_deg * (w_now_ > 0 ? w_now_ / p_.w_turn : 1.0);
    if (turned >= rad(turn_target_deg_ - lead)) {
      phase_ = UndockPhase::kSettleTurn;
      turn_stop_deg_ = deg(turned);
      settle_until_ = now + 1.0;
    } else {
      const UsMin all = us_min(now, true);
      const bool gyro_ok = now - (has_imu_t_ ? imu_t_ : 0.0) <= 0.3;
      std::string why;
      if (!gyro_ok) {
        why = "자이로 값이 안 들어옴";
      } else if (!all.fresh) {
        why = "초음파(뒤·옆뒤) 값이 안 들어옴";
      } else if (all.d && *all.d < p_.turn_stop) {
        why = "회전 중 초음파에 가까운 물체";
      } else if (sweep_hits_ >= 3) {
        why = "회전 반경 안에 라이다 점";
      }
      const bool ok = gyro_ok && all.fresh && (!all.d || *all.d >= p_.turn_resume) &&
        sweep_hits_ < 3;
      if (!set_hold(now, why, ok) && !finished()) {
        const bool slow = (all.d && *all.d < p_.slow_dist) || slow_hits_ >= 3;
        w_now_ = slow ? p_.w_turn_slow : p_.w_turn;
        w = static_cast<double>(turn_sign_) * w_now_;
      } else {
        w_now_ = 0.0;
      }
    }
  } else if (phase_ == UndockPhase::kSettleTurn) {
    if (now >= settle_until_) {
      result_.turned_deg = deg(std::abs(yaw_ - yaw_turn0_));
      result_.hold_s = hold_total_;
      result_.success = true;
      phase_ = UndockPhase::kDone;
      return {};
    }
  }
  if (finished()) {
    return {};
  }
  const bool moving_cmd = std::abs(v) > 1e-3 || std::abs(w) > 1e-3;
  if (moving_cmd) {
    if (!has_move_ref_) {
      has_move_ref_ = true;
      move_t_ = now;
      move_x_ = x_;
      move_yaw_ = yaw_;
    } else if (now - move_t_ > 2.0) {
      const double dx = std::abs(v) > 1e-3 ? std::abs(x_ - move_x_) : 1.0;
      const double dyaw = (std::abs(w) > 1e-3 && std::abs(v) <= 1e-3) ?
        std::abs(yaw_ - move_yaw_) : 1.0;
      if ((dx < 0.01 && dyaw > 0.5) || (dyaw < rad(2.0) && dx > 0.5)) {
        abort(Fail::kNoMotion, "명령을 2 s 냈는데 로봇이 안 움직임");
        return {};
      }
      move_t_ = now;
      move_x_ = x_;
      move_yaw_ = yaw_;
    }
  } else {
    has_move_ref_ = false;
  }
  return {v, w};
}

}
