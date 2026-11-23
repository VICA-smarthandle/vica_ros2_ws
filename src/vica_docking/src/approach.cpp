#include "vica_docking/approach.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "vica_docking/scan_zones.hpp"

namespace vica_docking
{

const char * phase_name(ApproachPhase p)
{
  switch (p) {
    case ApproachPhase::kWait: return "WAIT";
    case ApproachPhase::kRotateToStage: return "ROTATE_TO_STAGE";
    case ApproachPhase::kDriveToStage: return "DRIVE_TO_STAGE";
    case ApproachPhase::kAlign: return "ALIGN";
    case ApproachPhase::kApproach: return "APPROACH";
    case ApproachPhase::kStopped: return "STOPPED";
    case ApproachPhase::kDone: return "DONE";
    case ApproachPhase::kAbort: return "ABORT";
  }
  return "?";
}

ApproachController::ApproachController(ApproachParams p, RobotGeometry g, VDetectorParams vp)
: p_(p), g_(g), det_(vp, g),
  d_target_(g.bumper_x + p.target_cm / 100.0),
  d_stage_(g.bumper_x + p.stage_cm / 100.0) {}

bool ApproachController::active() const
{
  return phase_ == ApproachPhase::kRotateToStage || phase_ == ApproachPhase::kDriveToStage ||
         phase_ == ApproachPhase::kAlign || phase_ == ApproachPhase::kApproach;
}

void ApproachController::start(double now)
{
  t0_ = now;
}

void ApproachController::set_cmd(double v, double w)
{
  cmd_.v = std::clamp(v, 0.0, p_.vmax);
  cmd_.w = std::clamp(w, -p_.wmax, p_.wmax);
}

void ApproachController::abort(Fail f, const std::string & why)
{
  if (finished()) {
    return;
  }
  result_.fail = f;
  result_.why = why;
  result_.success = false;
  cmd_ = {};
  phase_ = ApproachPhase::kAbort;
}

double ApproachController::carrot_bearing(const VDetection & det, double x) const
{
  const Vec2 T = det.apex + det.u * (x - 0.4);
  return std::atan2(T.y, T.x);
}

Cmd ApproachController::on_timer(double now)
{
  if (finished()) {
    return {};
  }
  const double gap = has_det_t_ ? now - last_det_t_ : std::numeric_limits<double>::infinity();
  const bool act = active();
  if (act && (front_hits_ >= p_.front_min_pts || path_hits_ >= p_.front_min_pts)) {
    has_front_clear_ = false;
    if (!front_hold_) {
      front_hold_ = true;
      front_since_ = now;
      ++result_.front_holds;
    }
  } else if (front_hold_) {
    if (!has_front_clear_) {
      has_front_clear_ = true;
      front_clear_since_ = now;
    }
    if (now - front_clear_since_ >= p_.front_release_s) {
      front_hold_ = false;
      has_front_clear_ = false;
    }
  }
  if (front_hold_ && now - front_since_ > p_.front_wait_timeout) {
    abort(Fail::kBlocked, "앞이 30 s 넘게 막힘");
    return {};
  }
  if (act && gap > p_.lost_timeout_s && !front_hold_) {
    char why[64];
    std::snprintf(why, sizeof(why), "V자를 %.1f s 넘게 못 봄", p_.lost_timeout_s);
    abort(Fail::kLost, why);
    return {};
  }
  if (now - t0_ > p_.max_time) {
    abort(Fail::kTimeout, "시간 초과");
    return {};
  }
  if (phase_ == ApproachPhase::kWait && now - t0_ > p_.find_timeout) {
    abort(Fail::kNotFound, "출발 전 V자를 5회 연속으로 찾지 못함");
    return {};
  }
  const bool hold = act && (gap > 0.25 || front_hold_);
  if (hold) {
    ++result_.holds;
  }
  const Cmd out = (act && !hold) ? cmd_ : Cmd{};
  const bool moving = std::abs(out.v) > 1e-3 || std::abs(out.w) > 1e-3;
  const bool forward = std::abs(out.v) > 1e-3;
  if (moving && last_pose_) {
    if (!has_move_ref_ || forward != move_forward_) {
      has_move_ref_ = true;
      move_forward_ = forward;
      move_t_ = now;
      move_pose_ = *last_pose_;
    } else if (now - move_t_ > p_.no_motion_s) {
      const bool moved = forward ?
        std::abs(last_pose_->x - move_pose_.x) >= 0.01 :
        std::abs(last_pose_->th - move_pose_.th) >= rad(2.0);
      if (!moved) {
        abort(Fail::kNoMotion, "명령을 2 s 냈는데 로봇이 안 움직임");
        return {};
      }
      move_t_ = now;
      move_pose_ = *last_pose_;
    }
  } else {
    has_move_ref_ = false;
  }
  return out;
}

void ApproachController::on_scan(
  double now, const std::vector<float> & ranges, double amin, double inc, double range_min)
{
  if (finished()) {
    return;
  }
  ++scans_;
  std::optional<VDetection> det = det_.detect(ranges, amin, inc, range_min);
  if (det && good_ && now - good_t_ < 1.0) {
    const double jump = norm(det->apex - good_->apex);
    const double turn = deg(std::acos(std::clamp(dot(det->u, good_->u), -1.0, 1.0)));
    if (jump > 0.10 || turn > 15.0) {
      ++result_.rejected;
      det.reset();
    }
  }
  if (det || !good_) {
    front_hits_ = count_bumper_zone(g_, ranges, amin, inc, range_min, det, p_.front_zone, true, 0.03);
  } else {
    front_hits_ = count_bumper_zone(g_, ranges, amin, inc, range_min, good_, p_.front_zone, true, 0.03, 0.16);
  }
  path_hits_ = 0;
  if (!det && good_) {
    const double depth = std::max(p_.front_zone, good_->apex.x - g_.bumper_x);
    path_hits_ = count_bumper_zone(g_, ranges, amin, inc, range_min, good_, depth, true, 0.03, 0.16);
  }
  if (!det) {
    return;
  }
  good_ = det;
  good_t_ = now;
  has_det_t_ = true;
  last_det_t_ = now;
  const DockPose pz = pose_in_dock(*det);
  last_pose_ = pz;
  if (phase_ == ApproachPhase::kWait) {
    const bool same = has_streak_ref_ && std::abs(pz.x - streak_ref_.x) < 0.02 &&
      std::abs(pz.y - streak_ref_.y) < 0.02;
    streak_ = same ? streak_ + 1 : 1;
    streak_ref_ = pz;
    has_streak_ref_ = true;
    if (streak_ < 5) {
      return;
    }
  }
  step(now, *det, pz);
}

void ApproachController::step(double now, const VDetection & det, const DockPose & pz)
{
  const double x = pz.x;
  if (phase_ == ApproachPhase::kWait) {
    result_.start = pz;
    result_.has_start = true;
    if (std::abs(pz.y) > p_.start_max_lateral || std::abs(pz.th) > rad(p_.start_max_heading_deg)) {
      abort(Fail::kOutOfRange, "출발 위치가 도크 정면에서 벗어남");
      return;
    }
    if (x > d_stage_ + 0.15) {
      phase_ = ApproachPhase::kRotateToStage;
    } else if (x - 0.10 >= p_.rot_clear) {
      phase_ = ApproachPhase::kAlign;
    } else if (std::abs(pz.y) < 0.03 && std::abs(pz.th) < rad(3.0)) {
      phase_ = ApproachPhase::kApproach;
    } else {
      abort(Fail::kTooClose, "준비 위치보다 가깝고 줄이 안 맞음(제자리 회전 여유 없음)");
      return;
    }
  }
  const Vec2 S = det.apex + det.u * d_stage_;
  const double dist = norm(S);
  const double bearing = std::atan2(S.y, S.x);
  if (phase_ == ApproachPhase::kRotateToStage) {
    if (dist < 0.15 || x < d_stage_ + 0.15) {
      phase_ = ApproachPhase::kAlign;
    } else if (std::abs(bearing) < rad(4.0)) {
      phase_ = ApproachPhase::kDriveToStage;
    } else {
      set_cmd(0.0, sgn(bearing) * std::max(0.12, std::min(p_.wmax, 1.2 * std::abs(bearing))));
    }
  }
  if (phase_ == ApproachPhase::kDriveToStage) {
    if (dist < 0.04 || x < d_stage_ + 0.01) {
      phase_ = ApproachPhase::kAlign;
    } else if (std::abs(bearing) > rad(25.0)) {
      phase_ = ApproachPhase::kRotateToStage;
    } else {
      set_cmd(std::max(0.04, std::min(p_.vmax, 0.6 * dist)), 1.5 * bearing);
    }
  }
  if (phase_ == ApproachPhase::kAlign) {
    const double b = carrot_bearing(det, x);
    if (std::abs(b) < rad(p_.align_tol_deg)) {
      phase_ = ApproachPhase::kApproach;
      set_cmd(0.0, 0.0);
    } else {
      set_cmd(0.0, sgn(b) * std::max(0.08, std::min(0.25, 1.0 * std::abs(b))));
    }
  }
  if (phase_ == ApproachPhase::kApproach) {
    const double remain = x - d_target_;
    if (remain <= p_.lead) {
      set_cmd(0.0, 0.0);
      phase_ = ApproachPhase::kStopped;
      result_.stop = pz;
      result_.has_stop = true;
      settle_t_ = now;
      return;
    }
    const double v = remain > 0.10 ? p_.vmax : p_.vfinal;
    const double b = carrot_bearing(det, x);
    double w = 0.0;
    if (!(remain < 0.05 || std::abs(b) < rad(p_.deadband_deg))) {
      w = p_.k_b * b;
    }
    set_cmd(v, w);
  }
  if (phase_ == ApproachPhase::kStopped) {
    if (now - settle_t_ >= 1.5) {
      settle_.push_back(pz);
    }
    if (settle_.size() >= 30) {
      finish();
    }
  }
}

void ApproachController::finish()
{
  double sx = 0.0, sy = 0.0, sth = 0.0;
  for (const auto & s : settle_) {
    sx += s.x;
    sy += s.y;
    sth += s.th;
  }
  const double n = static_cast<double>(settle_.size());
  result_.final_pose = {sx / n, sy / n, sth / n};
  result_.has_final = true;
  result_.err_x_cm = (result_.final_pose.x - d_target_) * 100.0;
  result_.err_y_cm = result_.final_pose.y * 100.0;
  result_.err_th_deg = deg(result_.final_pose.th);
  result_.success = std::abs(result_.err_x_cm) <= p_.ok_x_cm &&
    std::abs(result_.err_y_cm) <= p_.ok_y_cm && std::abs(result_.err_th_deg) <= p_.ok_th_deg;
  if (!result_.success) {
    result_.fail = Fail::kMisaligned;
    result_.why = "도착했지만 오차가 기준 밖";
  }
  cmd_ = {};
  phase_ = ApproachPhase::kDone;
}

}
