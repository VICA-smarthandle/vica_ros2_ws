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
  Polygon footprint;
  LookaheadParams lookahead;
  SpeedParams speed;
  OutputParams output;
  LaneParams lane;
  TurnParams turn;
  AlignParams align;
  StateParams state;
  double stop_latency{0.3};
  double stop_margin{0.05};
  double stationary_speed{0.05};
  double stationary_time{0.5};
  double resync_offset{0.15};
  double arrive_margin{0.03};
  double align_rearm_time{3.0};
  double turn_enter_persist{0.0};
  bool turn_lock_target{false};
  double turn_retarget_angle{M_PI / 2.0};
  double turn_retarget_persist{0.5};
  double turn_max_rotation{0.0};
  bool end_extend{false};
  double end_extend_max_lateral{0.08};
  double end_extend_min_length{0.3};
  bool pass_arrival{false};
  double position_only_yaw_tol{3.0};
  bool new_path_confirm{false};
  double end_straight_lateral{0.0};
};

enum class Failure { None, CollisionAhead, Blocked, AlignFailed };

struct CoreInputs
{
  double now{0.0};
  double dt{0.1};
  Path path;
  Pose2D goal;
  Twist2D measured;
  double xy_tol{0.25};
  double yaw_tol{0.25};
  double rot_stopped{0.05};
  double speed_cap{0.5};
  ClearanceFn clearance;
  double robot_yaw{0.0};
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
  double turn_rotated{0.0};
  bool end_extended{false};
  bool collision_deferred{false};
  bool end_straight{false};
  Path lane_path;
};

bool motionCollides(
  const Twist2D & cmd, const ClearanceFn & f, double max_decel, double max_ang_accel,
  double stop_latency);

class VccCore
{
public:
  void configure(const CoreParams & p);
  void reset(const Twist2D & measured = {});
  void onNewGoal();
  void onNewPath() {path_fresh_ = true;}
  CoreOutput step(const CoreInputs & in);

private:
  CoreParams p_;
  LaneSelector lanes_;
  OutputStage output_;
  AlignPlanner align_;
  StateMachine sm_;
  TurnPlan turn_;
  int turn_dir_{0};
  double stopped_since_{-1.0};
  int resync_count_{0};
  bool resync_primed_{false};
  double align_failed_since_{-1.0};
  double need_since_{-1.0};
  double turn_target_{0.0};
  double turn_rot_{0.0};
  double last_yaw_{0.0};
  bool have_last_yaw_{false};
  double retarget_since_{-1.0};
  bool path_fresh_{false};
  bool deferred_last_{false};
};
}
