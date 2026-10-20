#pragma once

namespace vica_vcc_controller::core
{
enum class State { Track, Turn, Align, Hold };
const char * stateName(State s);
bool transitionAllowed(State from, State to);

struct StateParams
{
  double turn_enter_angle{1.047};
  double turn_exit_angle{0.436};
  double pivot_start_angle{0.611};
  double align_exit_dist{0.5};
  double min_state_time{0.5};
};

struct StateInputs
{
  double now{0.0};
  bool stationary{false};
  double heading_error{0.0};
  double path_heading_error{0.0};
  double dist_to_end{1e9};
  double yaw_error_end{0.0};
  double xy_tol{0.25};
  double yaw_tol{0.25};
  bool lanes_blocked{false};
  bool turn_blocked{false};
  bool collision_imminent{false};
  bool align_failed{false};
  bool align_blocked{false};
  bool turn_ready{true};
};

bool turnNeeded(const StateInputs & in, const StateParams & p);

class StateMachine
{
public:
  explicit StateMachine(StateParams p = {}) : p_(p) {}
  void reset();
  State update(const StateInputs & in);
  void forceHold(double now, const char * why) {go(State::Hold, now, why);}
  State state() const {return s_;}
  const char * reason() const {return reason_;}

private:
  void go(State to, double now, const char * why);
  StateParams p_;
  State s_{State::Track};
  double entered_{-1e9};
  const char * reason_{"init"};
};
}
