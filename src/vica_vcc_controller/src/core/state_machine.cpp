#include "vica_vcc_controller/core/state_machine.hpp"

#include <cmath>

namespace vica_vcc_controller::core
{
const char * stateName(State s)
{
  switch (s) {
    case State::Track: return "TRACK";
    case State::Turn: return "TURN";
    case State::Align: return "ALIGN";
    case State::Hold: return "HOLD";
  }
  return "?";
}

bool transitionAllowed(State from, State to)
{
  if (from == to) {return false;}
  switch (from) {
    case State::Track: return true;
    case State::Turn: return true;
    case State::Align: return to == State::Track || to == State::Hold;
    case State::Hold: return to == State::Track;
  }
  return false;
}

bool turnNeeded(const StateInputs & in, const StateParams & p)
{
  const double th = in.stationary ? p.pivot_start_angle : p.turn_enter_angle;
  return std::abs(in.heading_error) > th && std::abs(in.path_heading_error) > th;
}

void StateMachine::reset()
{
  s_ = State::Track;
  entered_ = -1e9;
  reason_ = "reset";
}

void StateMachine::go(State to, double now, const char * why)
{
  if (!transitionAllowed(s_, to)) {return;}
  s_ = to;
  entered_ = now;
  reason_ = why;
}

State StateMachine::update(const StateInputs & in)
{
  const bool held = in.now - entered_ >= p_.min_state_time - 1e-9;
  const bool arrived = in.dist_to_end < in.xy_tol;
  const bool yaw_off = std::abs(in.yaw_error_end) > in.yaw_tol;

  // 우선순위 1: 안전 — 최소 유지 시간을 무시한다.
  if (in.collision_imminent) {
    if (s_ != State::Hold) {go(State::Hold, in.now, "collision_imminent");}
    return s_;
  }

  switch (s_) {
    case State::Track:
      if (in.lanes_blocked) {go(State::Hold, in.now, "lanes_blocked"); break;}
      if (arrived && yaw_off) {go(State::Align, in.now, "arrived_yaw_off"); break;}
      if (!held) {break;}
      if (!arrived && turnNeeded(in, p_)) {
        if (in.turn_blocked) {go(State::Hold, in.now, "turn_blocked");} else {
          go(State::Turn, in.now, "heading_error");
        }
      }
      break;
    case State::Turn:
      if (in.turn_blocked) {go(State::Hold, in.now, "turn_blocked"); break;}
      if (arrived) {
        go(yaw_off ? State::Align : State::Track, in.now, "arrived_during_turn");
        break;
      }
      if (!held) {break;}
      if (std::abs(in.heading_error) < p_.turn_exit_angle) {go(State::Track, in.now, "turn_done");}
      break;
    case State::Align:
      if (in.align_failed) {go(State::Hold, in.now, "align_failed"); break;}
      if (!held) {break;}
      if (in.dist_to_end > in.xy_tol + p_.align_exit_margin) {
        go(State::Track, in.now, "pushed_off_goal");
      }
      break;
    case State::Hold:
      if (!held) {break;}
      {
        // 유턴이 필요한데 막힌 채로 Track 에 나가면 조준점이 뒤인데 앞으로 기어간다(최종 리뷰 I2).
        const bool turn_stuck = !arrived && turnNeeded(in, p_) && in.turn_blocked;
        if (!in.lanes_blocked && !in.align_failed && !turn_stuck) {
          go(State::Track, in.now, "path_open");
        }
      }
      break;
  }
  return s_;
}
}  // namespace vica_vcc_controller::core
