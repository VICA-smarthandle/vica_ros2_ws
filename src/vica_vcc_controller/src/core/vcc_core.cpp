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

namespace
{
// 로봇(원점)에서 경로 첫 구간(p0->p1)을 뒤로 늘인 직선까지의 옆 거리(왼쪽 +). 첫 점이 로봇보다
// 0.2~0.74 m 앞이고 그 점의 방향이 앞 구간 것(코너 노드)이어도 흔들리지 않는다(run48 F2).
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
}  // namespace

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

  // 정지 판정(0.5 s 이상 거의 멈춤)
  if (std::abs(v) < p_.stationary_speed && std::abs(output_.last().v) < p_.stationary_speed) {
    if (stopped_since_ < 0.0) {stopped_since_ = in.now;}
  } else {
    stopped_since_ = -1.0;
  }
  const bool stationary = stopped_since_ >= 0.0 && in.now - stopped_since_ >= p_.stationary_time;

  // 도착 정렬 재무장: Failed 로 align_rearm_time 이 지나면 횟수를 지우고 다시 돈다. 같은 goal 을
  // BT 가 바로 다시 보내면 onNewGoal·reset 이 안 불려 영영 Hold 에 남았다(run49 409호 17 s).
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
  // 도착으로 보고 서는 반경. checker 원보다 arrive_margin 안쪽(단 반경의 절반보다 작게는 안 한다).
  const double stop_r = std::max(0.5 * in.xy_tol, in.xy_tol - p_.arrive_margin);

  // 차선 d 를 실제 옆 위치에 다시 맞춘다: 첫 구간 연장선 기준 로봇의 옆 거리(왼쪽 +).
  // reset 뒤 0, 유턴 뒤 약 2R, 레일<->당근 경로 교체 뒤 어긋난 d 를 그대로 두면 차선 검사가 로봇이
  // 아닌 곳을 본다(최종 리뷰 I4). 한 주기만 튄 값으로는 옮기지 않는다 — 2주기 연속일 때만(run48 F2:
  // 가짜 되튐 75회, 64 % 가 새 경로 뒤 0.25 s 안). reset 뒤 첫 경로는 비교할 과거가 없어 바로 맞춘다.
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

  // 차선. 옮김 속도 후보는 차선 제한 전 목표 속도부터(run48 F1).
  const double v_des = std::min(p_.speed.desired, in.speed_cap);
  lanes_.update(in.path, v, v_des, in.now, in.dt, in.clearance);
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
  si.xy_tol = stop_r;
  si.yaw_tol = in.yaw_tol;
  si.lanes_blocked = lanes_.blocked();
  si.collision_imminent = imminent;
  si.align_failed = align_.phase() == AlignPhase::Failed;

  const State s0 = sm_.state();
  // 도착 정렬 회전 원: 남은 각만큼 제자리로 돌 때 몸이 쓸고 가는 자리(설계서 6.1 ③->④, 최종 리뷰 I5).
  // yaw 가 허용오차 안이면 돌 일이 없으므로 보지 않는다(도착한 채 Blocked 를 던지지 않게).
  if (std::abs(yaw_err) > in.yaw_tol && (s0 == State::Align || dist_end < stop_r)) {
    si.align_blocked = simulateTurnClearance(
      std::abs(yaw_err), 0.0, yaw_err >= 0.0 ? 1 : -1, 0.0, in.clearance, p_.turn.sample_angle) <
      p_.turn.clearance;
  }

  // 유턴 계획은 필요할 때만(계산량 고정)
  const bool need = turnNeeded(si, p_.state);
  if (s0 == State::Turn || (need && (s0 == State::Track || s0 == State::Hold))) {
    const bool pivot_first = s0 == State::Turn ? turn_.mode == TurnMode::Pivot :
      (stationary && std::abs(heading) <= p_.state.turn_enter_angle);
    // Turn 중 재계획은 들어갈 때 고른 방향을 지킨다(반지름·방식만 바뀐다). ±170° 근처에서 방향이
    // 주기마다 뒤집히며 w 가 0 근처를 떠는 일을 막는다(최종 리뷰 M2).
    turn_ = planTurn(heading, v, pivot_first, in.clearance, p_.turn, s0 == State::Turn ? turn_dir_ : 0);
    si.turn_blocked = turn_.mode == TurnMode::Blocked;
  }

  const State s = sm_.update(si);
  // 도착 정렬 횟수는 reset·onNewGoal 에서만 지운다. 같은 goal 안에서 Align 에 다시 들어와도 이어 센다(M3).
  // 유턴 방향은 Turn 에 들어갈 때 잠그고 나가면 푼다(M2).
  if (s == State::Turn && s0 != State::Turn) {turn_dir_ = turn_.direction;}
  if (s != State::Turn) {turn_dir_ = 0;}
  // 유턴을 마치면 d 를 지금 옆 위치로 바로 맞추고 목표도 가장 가까운 차선에 둔다. 목표가 레일에
  // 남아 있으면 나오자마자 되튄다. 레일 복귀는 보통 규칙(return_clear_time)이 맡는다(run48 F2).
  if (s0 == State::Turn && s == State::Track && !in.path.empty()) {
    const double e = lateralError(in.path);
    lanes_.syncOffset(e);
    lanes_.setTargetNearest(e);
    resync_count_ = 0;
  }

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
        if (dist_end < stop_r) {vdes = 0.0;}   // 도착 반경 안에서는 멈춘다(RPP 도 같은 자리에서 회전으로 넘어간다)
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
