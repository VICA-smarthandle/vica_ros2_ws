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
  cmd_.v = std::clamp(v, 0.0, p_.vmax);    // 후진 없음
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
  // 축 위, 로봇보다 항상 0.4 m 앞 지점의 방위각(v2.1: 목표점에 묶지 않는다)
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
  // 앞 장애물: 있으면 멈추고, 비고 front_release_s 이어지면 다시 간다
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
  if (act && gap > p_.lost_timeout_s && !front_hold_) {   // 앞이 막혀 V자가 가려진 동안은 기다린다
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
  const bool hold = act && (gap > 0.25 || front_hold_);   // 끊기거나 앞이 막히면 0 으로 기다린다
  if (hold) {
    ++result_.holds;
  }
  const Cmd out = (act && !hold) ? cmd_ : Cmd{};
  // 안 움직임 검사(운영에서 더함): 명령을 2 s 냈는데 V자 거리 1 cm·방향 2° 도 안 변하면 중단.
  // 제자리 회전과 직진이 바뀌면 기준을 새로 잡는다(11-18 실기: 정렬 회전 1.6 s + 직진 0.3 s 를 거리로 재서 헛판정).
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
    if (jump > 0.10 || turn > 15.0) {   // 튀는 인식은 놓친 것으로 본다
      ++result_.rejected;
      det.reset();
    }
  }
  // 앞 30 cm 안 장애물 수.
  // 팔 끝 가장자리 점이 팔 직선에서 1.5~1.9 cm 떨어져 잡혔다(11-18 run05·06·09 녹화) → 팔 둘레 3 cm 는 V자로 본다
  if (det || !good_) {
    front_hits_ = count_bumper_zone(g_, ranges, amin, inc, range_min, det, p_.front_zone, true, 0.03);
  } else {
    // 이번에 못 봤다 — 로봇은 0.25 s 안에 멈추므로 마지막 V자 자리로 도크(꼭짓점 면 앞 16 cm)를 뺀다. 아래 통로 판정과
    // 같은 기준이다. 예전에는 1 s 뒤부터 V자 팔이 장애물로 세져, 도크 가까이서 V자만 놓치면 lost(물러나 다시)가
    // 아니라 30 s 뒤 blocked 로 끝났다(11-19 검토).
    front_hits_ = count_bumper_zone(g_, ranges, amin, inc, range_min, good_, p_.front_zone, true, 0.03, 0.16);
  }
  // V자를 못 본 스캔: 마지막으로 본 V자까지의 통로(차폭)에 물체가 있으면 '가려짐 = 막힘'이다(11-19 실기:
  // 상자가 범퍼 30 cm 밖에서 V자를 가리자 lost 로 끝났다). V자를 놓치면 0.25 s 뒤 멈추므로 마지막 자리를 그대로 쓴다.
  path_hits_ = 0;
  if (!det && good_) {
    const double depth = std::max(p_.front_zone, good_->apex.x - g_.bumper_x);
    // 꼭짓점 면 앞 16 cm 까지는 도크 자체로 본다: 팔 끝이 10 cm 앞이고, V자를 놓친 뒤 멈출 때까지 로봇이 5~6 cm
    // 더 가서(0.25 s + 미끄러짐) 마지막 V자 자리가 그만큼 낡는다. 이보다 얕으면 팔·뒤 벽이 막힘으로 세진다.
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
    if (streak_ < 5) {   // 출발 전 5회 연속 일치
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
    // 운영에서 더함: 도크 정면에서 너무 벗어나면 움직이지 않고 실패(미션이 준비 위치로 다시 보낸다)
    if (std::abs(pz.y) > p_.start_max_lateral || std::abs(pz.th) > rad(p_.start_max_heading_deg)) {
      abort(Fail::kOutOfRange, "출발 위치가 도크 정면에서 벗어남");
      return;
    }
    if (x > d_stage_ + 0.15) {
      phase_ = ApproachPhase::kRotateToStage;
    } else if (x - 0.10 >= p_.rot_clear) {   // V자 팔 끝(꼭짓점 앞 0.10 m)까지 회전 여유
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
    if (remain <= p_.lead) {   // 넘침만큼 일찍 정지 명령
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
      w = p_.k_b * b;   // 기본 직진. 마지막 5 cm 도 직진만
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

}  // namespace vica_docking
