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
  // 직진 v/감속, 제자리 |w|/각감속 — 둘 다 있으면 긴 쪽(보수적). 거리 0.05 m·각 0.05 rad 마다 본다.
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
  stopped_since_ = -1.0;
}

void VccCore::onNewGoal()
{
  align_.reset();
  if (sm_.state() == State::Align || sm_.state() == State::Hold) {sm_.reset();}
}

CoreOutput VccCore::step(const CoreInputs & in)
{
  CoreOutput out;
  const double v = in.measured.v;

  // 정지 판정(0.5 s 이상 거의 멈춤)
  if (std::abs(v) < p_.stationary_speed && std::abs(output_.last().v) < p_.stationary_speed) {
    if (stopped_since_ < 0.0) {stopped_since_ = in.now;}
  } else {
    stopped_since_ = -1.0;
  }
  const bool stationary = stopped_since_ >= 0.0 && in.now - stopped_since_ >= p_.stationary_time;

  // 차선 d 를 실제 옆 위치에 다시 맞춘다: 경로 첫 점의 접선 기준 로봇의 옆 거리(왼쪽 +).
  // reset 뒤 0, 유턴 뒤 약 2R, 레일<->당근 경로 교체 뒤 어긋난 d 를 그대로 두면 차선 검사가 로봇이
  // 아닌 곳을 본다(최종 리뷰 I4).
  if (!in.path.empty()) {
    const Pose2D & p0 = in.path.front();
    const double e = p0.x * std::sin(p0.yaw) - p0.y * std::cos(p0.yaw);
    if (std::abs(e - lanes_.offset()) > p_.resync_offset) {lanes_.syncOffset(e);}
  }

  // 차선
  lanes_.update(in.path, v, in.now, in.dt, in.clearance);
  const Path lane_path = lanes_.lanePath(in.path, v);
  const double L = lookaheadDistance(v, p_.lookahead);
  const Point2D carrot = carrotOnPath(lane_path, L);
  const double heading = std::atan2(carrot.y, carrot.x);
  const double path_heading = carrotTangent(lane_path, L);
  const double dist_end = std::hypot(in.goal.x, in.goal.y);
  const double yaw_err = normalizeAngle(in.goal.yaw);

  // 정지거리 안 몸통 접촉 — 주행 중에만(회전은 유턴 계획이 매 주기 다시 검사)
  bool imminent = false;
  if (sm_.state() == State::Track && v > p_.stationary_speed) {
    const double s_stop = v * v / (2.0 * p_.output.max_decel) + v * p_.stop_latency + p_.stop_margin;
    for (const auto & ps : pathPrefix(lane_path, s_stop)) {
      if (in.clearance(ps) < 0.0) {imminent = true; break;}
    }
  }

  StateInputs si;
  si.now = in.now;
  si.stationary = stationary;
  si.heading_error = heading;
  si.path_heading_error = path_heading;
  si.dist_to_end = dist_end;
  si.yaw_error_end = yaw_err;
  si.xy_tol = in.xy_tol;
  si.yaw_tol = in.yaw_tol;
  si.lanes_blocked = lanes_.blocked();
  si.collision_imminent = imminent;
  si.align_failed = align_.phase() == AlignPhase::Failed;

  const State s0 = sm_.state();
  // 도착 정렬 회전 원: 남은 각만큼 제자리로 돌 때 몸이 쓸고 가는 자리(설계서 6.1 ③->④, 최종 리뷰 I5).
  // yaw 가 허용오차 안이면 돌 일이 없으므로 보지 않는다(도착한 채 Blocked 를 던지지 않게).
  if (std::abs(yaw_err) > in.yaw_tol && (s0 == State::Align || dist_end < in.xy_tol)) {
    si.align_blocked = simulateTurnClearance(
      std::abs(yaw_err), 0.0, yaw_err >= 0.0 ? 1 : -1, 0.0, in.clearance, p_.turn.sample_angle) <
      p_.turn.clearance;
  }

  // 유턴 계획은 필요할 때만(계산량 고정)
  const bool need = turnNeeded(si, p_.state);
  if (s0 == State::Turn || (need && (s0 == State::Track || s0 == State::Hold))) {
    const bool pivot_first = s0 == State::Turn ? turn_.mode == TurnMode::Pivot :
      (stationary && std::abs(heading) <= p_.state.turn_enter_angle);
    turn_ = planTurn(heading, v, pivot_first, in.clearance, p_.turn);
    si.turn_blocked = turn_.mode == TurnMode::Blocked;
  }

  const State s = sm_.update(si);
  if (s == State::Align && s0 != State::Align) {align_.reset();}

  Desired d;
  switch (s) {
    case State::Track: {
        SpeedParams sp = p_.speed;
        sp.desired = std::min(p_.speed.desired, in.speed_cap);
        double vdes = sp.desired;
        vdes = std::min(vdes, curvePreviewLimit(lane_path, sp));
        vdes = std::min(vdes, clearanceLimit(lanes_.currentClearance(), sp));
        vdes = std::min(vdes, lanes_.speedCap());   // 차선을 옮기는 동안의 속도
        vdes = approachLimit(dist_end, vdes, sp);
        if (dist_end < in.xy_tol) {vdes = 0.0;}   // 도착 반경 안에서는 멈춘다(RPP 도 같은 자리에서 회전으로 넘어간다)
        // 조준점이 유턴 문턱 밖(뒤쪽)이면 앞으로 가지 않는다. 최소 유지 시간 전이라도 같다 —
        // 돌 수 있으면 곧 Turn 이 제자리에서 돌고, 막혔으면 선다(최종 리뷰 I2: 1.42 m 역주행).
        if (need) {vdes = 0.0;}
        d.cmd = {vdes, 0.0};
        d.curvature = curvatureTo(carrot);
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

  // 이번에 실제로 내보낼 (v, w) 의 호를 멈출 때까지 검사한다 — 움직이는 모든 상황(Track·Turn·Align).
  // 위 차선 경로 검사는 Track 에서 그대로 둔다(최종 리뷰 I4).
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
}  // namespace vica_vcc_controller::core
