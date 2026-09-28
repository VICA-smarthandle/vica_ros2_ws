#pragma once

namespace vica_vcc_controller::core
{
enum class State { Track, Turn, Align, Hold };
const char * stateName(State s);
bool transitionAllowed(State from, State to);

struct StateParams
{
  double turn_enter_angle{1.047};   // 60도 (요구 2)
  double turn_exit_angle{0.436};    // 25도 (히스테리시스)
  double pivot_start_angle{0.611};  // 35도 (정지 출발, 사용자 확인)
  double align_exit_margin{0.10};   // 도착 0.25 로 들어가고 0.35 로 나온다
  double min_state_time{0.5};
};

struct StateInputs
{
  double now{0.0};
  bool stationary{false};
  double heading_error{0.0};       // 조준점 각
  double path_heading_error{0.0};  // 조준점 구간의 레일 방향 각
  double dist_to_end{1e9};
  double yaw_error_end{0.0};
  double xy_tol{0.25};
  double yaw_tol{0.25};
  bool lanes_blocked{false};
  bool turn_blocked{false};
  bool collision_imminent{false};
  bool align_failed{false};
};

bool turnNeeded(const StateInputs & in, const StateParams & p);

class StateMachine
{
public:
  explicit StateMachine(StateParams p = {}) : p_(p) {}
  void reset();
  State update(const StateInputs & in);
  State state() const {return s_;}
  const char * reason() const {return reason_;}

private:
  void go(State to, double now, const char * why);
  StateParams p_;
  State s_{State::Track};
  double entered_{-1e9};
  const char * reason_{"init"};
};
}  // namespace vica_vcc_controller::core
