#include "vica_vcc_controller/core/vcc_core.hpp"

#include <algorithm>
#include <cmath>

namespace vica_vcc_controller::core
{
bool motionCollides(
  const Twist2D & cmd, const ClearanceFn & f, double max_decel, double max_ang_accel,
  double stop_latency)
{
  const double v = std::max(0.0, cmd.v), w = cmd.w;
  if (v < 1e-6 && std::abs(w) < 1e-6) {return false;}
  const double t = std::max(v / max_decel, std::abs(w) / max_ang_accel) + stop_latency;
  const int n = std::max(
    1, static_cast<int>(std::ceil(std::max(v * t, std::abs(w) * t) / 0.05)));
  for (int i = 1; i <= n; ++i) {
    const double tau = t * i / n;
    const double th = w * tau;
    Pose2D p{v * tau, 0.0, th};
    if (std::abs(w) > 1e-6) {p = {v / w * std::sin(th), v / w * (1.0 - std::cos(th)), th};}
    if (f(p) < 0.0) {return true;}
  }
  return false;
}

namespace
{
double lateralError(const Path & path)
{
  const Pose2D & p0 = path.front();
  double a = p0.yaw;
  for (size_t i = 1; i < path.size(); ++i) {
    const double dx = path[i].x - p0.x, dy = path[i].y - p0.y;
    if (std::hypot(dx, dy) > 1e-6) {a = std::atan2(dy, dx); break;}
  }
  return p0.x * std::sin(a) - p0.y * std::cos(a);
}
}

void VccCore::configure(const CoreParams & p)
{
  p_ = p;
  lanes_ = LaneSelector(p.lane);
  output_ = OutputStage(p.output);
  align_ = AlignPlanner(p.align);
  sm_ = StateMachine(p.state);
  reset();
}

void VccCore::reset(const Twist2D & measured)
{
  lanes_.reset();
  output_.reset(measured);
  align_.reset();
  sm_.reset();
  turn_ = TurnPlan{};
  turn_dir_ = 0;
  stopped_since_ = -1.0;
  resync_count_ = 0;
  resync_primed_ = false;
  align_failed_since_ = -1.0;
  need_since_ = -1.0;
  turn_rot_ = 0.0;
  have_last_yaw_ = false;
  retarget_since_ = -1.0;
  path_fresh_ = false;
  deferred_last_ = false;
}

void VccCore::onNewGoal()
{
  align_.reset();
  align_failed_since_ = -1.0;
  if (sm_.state() == State::Align || sm_.state() == State::Hold) {sm_.reset();}
}

CoreOutput VccCore::step(const CoreInputs & in)
{
  CoreOutput out;
  const double v = in.measured.v;

  if (std::abs(v) < p_.stationary_speed && std::abs(output_.last().v) < p_.stationary_speed) {
    if (stopped_since_ < 0.0) {stopped_since_ = in.now;}
  } else {
    stopped_since_ = -1.0;
  }
  const bool stationary = stopped_since_ >= 0.0 && in.now - stopped_since_ >= p_.stationary_time;

  if (align_.phase() == AlignPhase::Failed) {
    if (align_failed_since_ < 0.0) {
      align_failed_since_ = in.now;
    } else if (in.now - align_failed_since_ >= p_.align_rearm_time - 1e-9) {
      align_.reset();
      align_failed_since_ = -1.0;
    }
  } else {
    align_failed_since_ = -1.0;
  }
  const double stop_r_base = std::max(0.5 * in.xy_tol, in.xy_tol - p_.arrive_margin);
  const bool passed_end = p_.pass_arrival && in.goal.x < 0.0;
  const double stop_r = passed_end ?
    std::max(stop_r_base, in.xy_tol - 0.5 * p_.arrive_margin) : stop_r_base;

  if (!in.path.empty()) {
    const double e = lateralError(in.path);
    if (std::abs(e - lanes_.offset()) <= p_.resync_offset) {
      resync_count_ = 0;
    } else if (!resync_primed_ || ++resync_count_ >= 2) {
      lanes_.syncOffset(e);
      resync_count_ = 0;
    }
    resync_primed_ = true;
  }

  const double v_des = std::min(p_.speed.desired, in.speed_cap);
  lanes_.update(in.path, v, v_des, in.now, in.dt, in.clearance);
  const Path lane_path = lanes_.lanePath(in.path, v);
  const double L = lookaheadDistance(v, p_.lookahead);
  bool extended = false;
  const Point2D carrot = p_.end_extend ?
    carrotWithEndExtension(
    lane_path, L, p_.end_extend_max_lateral, p_.end_extend_min_length, &extended) :
    carrotOnPath(lane_path, L);
  const double heading = std::atan2(carrot.y, carrot.x);
  const double path_heading = carrotTangent(lane_path, L);
  const double dist_end = std::hypot(in.goal.x, in.goal.y);
  const double yaw_err = normalizeAngle(in.goal.yaw);

  bool imminent = false;
  if (sm_.state() == State::Track && v > p_.stationary_speed) {
    const double s_stop = v * v / (2.0 * p_.output.max_decel) + v * p_.stop_latency + p_.stop_margin;
    for (const auto & ps : pathPrefix(lane_path, s_stop)) {
      if (in.clearance(ps) < 0.0) {imminent = true; break;}
    }
  }
  const bool fresh_path = path_fresh_;
  path_fresh_ = false;
  bool deferred = false;
  if (imminent && fresh_path && p_.new_path_confirm && !deferred_last_) {
    imminent = false;
    deferred = true;
  }
  deferred_last_ = deferred;

  StateInputs si;
  si.now = in.now;
  si.stationary = stationary;
  si.heading_error = heading;
  si.path_heading_error = path_heading;
  si.dist_to_end = dist_end;
  si.yaw_error_end = yaw_err;
  si.xy_tol = stop_r;
  si.yaw_tol = in.yaw_tol;
  si.lanes_blocked = lanes_.blocked();
  si.collision_imminent = imminent;
  si.align_failed = align_.phase() == AlignPhase::Failed;

  const State s0 = sm_.state();
  if (std::abs(yaw_err) > in.yaw_tol && (s0 == State::Align || dist_end < stop_r)) {
    si.align_blocked = simulateTurnClearance(
      std::abs(yaw_err), 0.0, yaw_err >= 0.0 ? 1 : -1, 0.0, in.clearance, p_.turn.sample_angle) <
      keepClearance(p_.turn);
  }

  const bool need = turnNeeded(si, p_.state);

  if (have_last_yaw_ && s0 == State::Turn) {
    turn_rot_ += std::abs(normalizeAngle(in.robot_yaw - last_yaw_));
  }
  last_yaw_ = in.robot_yaw;
  have_last_yaw_ = true;
  double turn_heading = heading;
  bool retarget = false;
  if (s0 == State::Turn && p_.turn_lock_target) {
    const double aim_world = in.robot_yaw + heading;
    if (std::abs(normalizeAngle(aim_world - turn_target_)) > p_.turn_retarget_angle) {
      if (retarget_since_ < 0.0) {retarget_since_ = in.now;}
      if (in.now - retarget_since_ >= p_.turn_retarget_persist - 1e-9) {
        turn_target_ = aim_world;
        retarget = true;
        retarget_since_ = -1.0;
      }
    } else {
      retarget_since_ = -1.0;
    }
    turn_heading = normalizeAngle(turn_target_ - in.robot_yaw);
    si.heading_error = turn_heading;
  }
  if (!need) {
    need_since_ = -1.0;
  } else if (s0 != State::Turn && need_since_ < 0.0) {
    need_since_ = in.now;
  }
  si.turn_ready = p_.turn_enter_persist <= 0.0 ||
    (need_since_ >= 0.0 && in.now - need_since_ >= p_.turn_enter_persist - 1e-9);

  if (s0 == State::Turn || (need && (s0 == State::Track || s0 == State::Hold))) {
    const bool pivot_first = s0 == State::Turn ? turn_.mode == TurnMode::Pivot :
      (stationary && std::abs(heading) <= p_.state.turn_enter_angle);
    const bool locked = s0 == State::Turn && !retarget;
    turn_ = planTurn(
      s0 == State::Turn ? turn_heading : heading, v, pivot_first, in.clearance, p_.turn,
      locked ? turn_dir_ : 0, locked ? &turn_ : nullptr);
    if (s0 == State::Turn && retarget) {turn_dir_ = turn_.direction;}
    si.turn_blocked = turn_.mode == TurnMode::Blocked;
  }
  const bool overrun = p_.turn_max_rotation > 0.0 && s0 == State::Turn &&
    turn_rot_ > p_.turn_max_rotation;

  State s = sm_.update(si);
  if (overrun && s == State::Turn) {
    sm_.forceHold(in.now, "turn_overrun");
    s = State::Hold;
  }
  if (s == State::Turn && s0 != State::Turn) {
    turn_dir_ = turn_.direction;
    turn_target_ = in.robot_yaw + heading;
    turn_rot_ = 0.0;
    retarget_since_ = -1.0;
    need_since_ = -1.0;
  }
  if (s != State::Turn) {turn_dir_ = 0;}
  if (s0 == State::Turn && s == State::Track && !in.path.empty()) {
    const double e = lateralError(in.path);
    lanes_.syncOffset(e);
    lanes_.setTargetNearest(e);
    resync_count_ = 0;
  }

  Desired d;
  bool end_straight = false;
  switch (s) {
    case State::Track: {
        SpeedParams sp = p_.speed;
        sp.desired = std::min(p_.speed.desired, in.speed_cap);
        double vdes = sp.desired;
        vdes = std::min(vdes, curvePreviewLimit(lane_path, sp));
        vdes = std::min(vdes, clearanceLimit(lanes_.currentClearance(), sp));
        vdes = std::min(vdes, lanes_.speedCap());
        vdes = approachLimit(dist_end, vdes, sp);
        if (dist_end < stop_r) {vdes = 0.0;}
        const bool position_only = p_.position_only_yaw_tol > 0.0 &&
          in.yaw_tol >= p_.position_only_yaw_tol;
        const bool position_stop = position_only && dist_end < in.xy_tol;
        if (position_stop) {vdes = 0.0;}
        if (need) {vdes = 0.0;}
        end_straight = p_.end_straight_lateral > 0.0 && !position_only &&
          pathLength(lane_path) < p_.end_extend_min_length &&
          in.goal.x > 0.0 && std::abs(in.goal.y) <= p_.end_straight_lateral;
        d.cmd = {vdes, 0.0};
        d.curvature = (position_stop || end_straight) ? 0.0 : curvatureTo(carrot);
        d.planned = !need;
        break;
      }
    case State::Turn:
      d.cmd = turnCommand(turn_, p_.turn);
      if (turn_.mode == TurnMode::Arc) {d.radius = turn_.radius;}
      break;
    case State::Align:
      align_.setStoppedVelocity(in.rot_stopped);
      d.cmd = {0.0, align_.update(yaw_err, in.measured.w, in.yaw_tol, in.now, in.dt)};
      break;
    case State::Hold:
      d.cmd = {0.0, 0.0};
      break;
  }

  out.cmd = output_.apply(d, v, in.dt);

  if (!imminent && s != State::Hold &&
    motionCollides(out.cmd, in.clearance, p_.output.max_decel, p_.output.max_ang_accel,
    p_.stop_latency))
  {
    imminent = true;
    sm_.forceHold(in.now, "motion_collision");
  }
  out.state = sm_.state();
  out.offset = lanes_.offset();
  out.target = lanes_.target();
  out.lanes_blocked = lanes_.blocked();
  out.turn_mode = turn_.mode;
  out.align_attempts = align_.attempts();
  out.reason = sm_.reason();
  out.turn_rotated = out.state == State::Turn ? turn_rot_ : 0.0;
  out.end_extended = extended;
  out.collision_deferred = deferred;
  out.end_straight = end_straight;
  out.lane_path = lane_path;

  if (imminent) {
    out.failure = Failure::CollisionAhead;
    out.cmd = {0.0, 0.0};
    output_.reset(in.measured);
  } else if (align_.phase() == AlignPhase::Failed) {
    out.failure = Failure::AlignFailed;
  } else if (s == State::Hold && std::abs(v) < p_.stationary_speed) {
    out.failure = Failure::Blocked;
  }
  return out;
}
}
