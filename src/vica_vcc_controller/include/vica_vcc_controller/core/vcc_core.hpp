#pragma once
#include "vica_vcc_controller/core/align_planner.hpp"
#include "vica_vcc_controller/core/clearance.hpp"
#include "vica_vcc_controller/core/lanes.hpp"
#include "vica_vcc_controller/core/output_stage.hpp"
#include "vica_vcc_controller/core/pure_pursuit.hpp"
#include "vica_vcc_controller/core/speed_profile.hpp"
#include "vica_vcc_controller/core/state_machine.hpp"
#include "vica_vcc_controller/core/turn_planner.hpp"

namespace vica_vcc_controller::core
{
struct CoreParams
{
  Polygon footprint;             // padding 포함, 로봇 좌표계
  LookaheadParams lookahead;
  SpeedParams speed;
  OutputParams output;
  LaneParams lane;
  TurnParams turn;
  AlignParams align;
  StateParams state;
  double stop_latency{0.3};      // backlog D2: CAN·드라이버 지연 300 ms
  double stop_margin{0.05};
  double stationary_speed{0.05};
  double stationary_time{0.5};
};

enum class Failure { None, CollisionAhead, Blocked, AlignFailed };

struct CoreInputs
{
  double now{0.0};
  double dt{0.1};
  Path path;                     // 로봇 좌표계, 가까운 점부터
  Pose2D goal;                   // 경로 마지막 점(로봇 좌표계)
  Twist2D measured;
  double xy_tol{0.25};
  double yaw_tol{0.25};
  double rot_stopped{0.05};      // goal checker rot_stopped_velocity
  double speed_cap{0.5};
  ClearanceFn clearance;         // 로봇 좌표계 자세 -> 여유
};

struct CoreOutput
{
  Twist2D cmd;
  State state{State::Track};
  double offset{0.0};
  double target{0.0};
  bool lanes_blocked{false};
  TurnMode turn_mode{TurnMode::Blocked};
  int align_attempts{0};
  Failure failure{Failure::None};
  const char * reason{""};
  Path lane_path;
};

class VccCore
{
public:
  void configure(const CoreParams & p);
  void reset();
  CoreOutput step(const CoreInputs & in);

private:
  CoreParams p_;
  LaneSelector lanes_;
  OutputStage output_;
  AlignPlanner align_;
  StateMachine sm_;
  TurnPlan turn_;
  double stopped_since_{-1.0};
};
}  // namespace vica_vcc_controller::core
