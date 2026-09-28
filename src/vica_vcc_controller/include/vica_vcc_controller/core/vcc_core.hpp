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
  // 실제 옆 위치와 d 가 이만큼, 2주기 연속 어긋나면 d 를 다시 맞춘다(최종 리뷰 I4, run48 F2).
  // ROS 파라미터 lane_resync_threshold.
  double resync_offset{0.15};
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

// 이번 주기 명령 (v, w) 를 멈출 때까지 그대로 이어 간다고 보고, 그 호 위에서 몸통이 닿는지 본다
// (RPP isCollisionImminent 와 같은 태도, 최종 리뷰 I4). 시간 = max(v/감속, |w|/각감속) + 지연.
bool motionCollides(
  const Twist2D & cmd, const ClearanceFn & f, double max_decel, double max_ang_accel,
  double stop_latency);

class VccCore
{
public:
  void configure(const CoreParams & p);
  // 모든 내부 상태를 지운다. 출력단은 실측 속도에서 이어 간다.
  void reset(const Twist2D & measured = {});
  // 새 goal(경로 끝점이 0.5 m 넘게 이동): 도착 정렬만 초기화하고, Align·Hold 면 상황도 되돌린다.
  // 차선·출력단은 이어 간다 — 레일 BT 당근 모드가 끝점을 ~1 Hz 로 옮긴다(최종 리뷰 I3).
  void onNewGoal();
  CoreOutput step(const CoreInputs & in);

private:
  CoreParams p_;
  LaneSelector lanes_;
  OutputStage output_;
  AlignPlanner align_;
  StateMachine sm_;
  TurnPlan turn_;
  int turn_dir_{0};              // Turn 에 들어갈 때 고른 방향(나가면 0, 최종 리뷰 M2)
  double stopped_since_{-1.0};
  int resync_count_{0};          // 옆 오차가 문턱을 넘은 연속 주기 수
  bool resync_primed_{false};    // reset 뒤 첫 경로를 받았는가(첫 주기는 바로 맞춘다)
};
}  // namespace vica_vcc_controller::core
