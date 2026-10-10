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
  // 멈춤 반경 = xy_tol - arrive_margin. goal checker 원(xy_tol)보다 조금 안쪽에서 서야 checker 가
  // 로봇이 움직이는 동안 먼저 도장을 찍는다. 같은 반경이면 경계에서 VCC 만 "도착" 으로 보고 선 채
  // checker 는 영영 못 찍는 일이 남는다(run49 교착 2/8 의 한 갈래). ROS 파라미터 arrive_margin.
  double arrive_margin{0.03};
  // 도착 정렬이 횟수를 다 써서 Failed 가 된 뒤 이만큼 지나면 다시 3번 기회를 준다. 그동안
  // PlannerException 을 던지므로 controller_server failure_tolerance(10 s)보다 짧아야 한다.
  // run49: Failed 가 새 goal·1.5 s 끊김으로만 풀려 17 s 동안 예외 172회. ROS 파라미터 align_rearm_time.
  double align_rearm_time{3.0};
  // ── 유턴(Turn) 끝 정하기 (2026-10-05 해결안 C) ─────────────────────────────────────────
  // run60 사람 접근 73 s·5바퀴, run63 시작→창구 28 s·687°: Turn 은 방향을 잠그고 "조준각 < 25°" 에서 끝나는데
  // 경로가 1초마다 로봇 위치에서 다시 그려지거나 레일↔당근으로 바뀌어 조준점이 로봇과 함께 돌았다(꼬리 잡기).
  // 아래 넷 모두 기본값은 예전 동작(끔)이고 nav2_params.yaml 에서 켠다.
  // 조준각이 진입 문턱을 넘은 채 이만큼(s) 이어져야 Turn 에 들어간다. 경로가 바뀌는 순간 튄 각으로 S 자를 막는다.
  double turn_enter_persist{0.0};
  // Turn 에 들어갈 때 조준 방향을 지도 기준(robot_yaw + 조준각)으로 붙들고, 끝 판정을 그 방향으로 한다.
  bool turn_lock_target{false};
  // 새 경로의 조준 방향이 붙든 방향과 이보다 많이(rad), turn_retarget_persist(s) 이어 달라지면 다시 고른다
  // (방향도 가까운 쪽으로 새로). 장애물·경로가 정말 바뀐 경우.
  double turn_retarget_angle{M_PI / 2.0};
  double turn_retarget_persist{0.5};
  // 한 Turn 에서 돈 누적 각(rad)이 이보다 크면 Hold 로 멈추고 다시 판단한다. 0 이하 = 끔.
  double turn_max_rotation{0.0};
  // ── 경로 끝 연장(2026-10-05 해결안 가) ── 기본 끔, nav2_params.yaml 에서 켠다. pure_pursuit.hpp 근거.
  bool end_extend{false};
  double end_extend_max_lateral{0.08};   // 연장선에서 옆으로 이보다 멀면 예전처럼 끝점 조준
  double end_extend_min_length{0.3};     // 경로가 이보다 짧으면 연장하지 않는다
  // 끝점을 지나쳤어도(끝점이 로봇 뒤) goal checker 원 안(xy_tol - arrive_margin/2)이면 도착으로 본다.
  // 부드럽게 꺾으면 멈춤 반경(xy_tol - arrive_margin) 옆을 스쳐 지나칠 수 있어, 그 뒤 되돌아오는 고리를 막는다.
  bool pass_arrival{false};
  // ── 위치만 판정하는 도착(2026-10-07 대기 장소 작업) ─────────────────────────────────
  // goal checker 의 방향 허용(yaw_tol)이 이 값 이상이면 '위치만' 도착(사용자 안내, position_goal_checker
  // yaw 3.141)으로 보고, checker 원(xy_tol)에 들어오는 순간 속도 0·곡률 0 으로 곧게 선다. 끝점을
  // 조준해 마지막에 꺾는 일이 없다. 끝점 0.6 m 안에서 이미 줄인 속도(원 경계 약 0.17 m/s)를 평상 감속으로
  // 내리므로 급정지가 아니다(약 0.25 s·2 cm). 방향까지 맞추는 도착(홈·배송·대기 장소)은 그대로.
  // ROS 파라미터 position_only_yaw_tol. 0 이하 = 끔(예전 동작).
  double position_only_yaw_tol{3.0};
  // ── 새 경로 첫 주기 확인(2026-10-08 run69 헛 정지 2회) ─────────────────────────────────
  // 정지거리 검사는 '경로 선 + 차선 메모' 위에 몸을 놓고 본다. 새 경로가 온 첫 주기에는 그 둘이 아직 로봇과
  // 안 맞는다 — 205 s 는 새 선 방향으로 몸을 돌려 놓아 지나친 물체에 꼬리가 닿았고, 1225 s 는 옛 차선 메모를
  // 새 선에 얹어 몸을 벽 쪽 0.26 m 에 놓았다(차선 재동기는 일부러 2주기를 기다린다). 실제 앞은 비어 있었다.
  // 켜면 그 첫 주기의 '닿는다'는 다음 주기에 한 번 더 나와야 선다(0.1 s, 0.5 m/s 로 5 cm). 실제로 내보낼
  // (v, w) 호 검사(motionCollides)는 그 주기에도 그대로 바로 세운다. 두 주기 연속 미루는 일은 없다.
  // ROS 파라미터 new_path_collision_confirm. 기본 끔(예전 동작).
  bool new_path_confirm{false};
  // ── 마지막 구간 곧게 가기(2026-10-10 (b′), 사용자 결정) ────────────────────────────────────
  // 남은 경로가 end_extend_min_length 보다 짧으면 연장 조준이 꺼지고 끝점을 바로 조준한다. 끝점이 코앞이라
  // 옆 몇 cm 만 어긋나도 조준각이 커져 회전이 최대까지 붙고, 도착 정렬이 그 회전을 되돌린다(run65~67 도착
  // 반대 회전 10~27°, run82 사람 접근 9°). 끝점(goal)이 앞에 있고 옆으로 이 값 안이면 그 구간은 곡률 0 으로
  // 곧게 간다 — 곧게 가도 끝점을 이 거리로 스치므로 멈춤 반경(xy_tol − arrive_margin) 안에 든다.
  // 옆으로 이보다 멀면 예전처럼 끝점을 조준한다. 방향까지 맞추는 도착(사람 접근·홈·대기 장소·배송)에만 쓴다 —
  // 위치만 도착(안내, position_only_yaw_tol)의 마지막 꺾임(run71 6~17°)은 사용자 보류(10-08)라 그대로 둔다.
  // ROS 파라미터 end_straight_lateral. 0 이하 = 끔(예전 동작).
  double end_straight_lateral{0.0};
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
  double robot_yaw{0.0};         // 경로(plan) 좌표계에서 로봇 방향. Turn 목표 붙들기·누적 회전에 쓴다
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
  double turn_rotated{0.0};     // 이번 Turn 에서 돈 누적 각(rad), 진단용
  bool end_extended{false};     // 이번 주기 조준점이 경로 끝 연장선 위였나(진단용)
  bool collision_deferred{false};   // 새 경로 첫 주기라 정지거리 '닿는다'를 한 번 미뤘나(진단용)
  bool end_straight{false};     // 이번 주기 마지막 구간이라 곡률 0 으로 곧게 갔나(진단용, (b′))
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
  // 새 경로를 받았다(플러그인 setPlan). 다음 step 한 번이 '새 경로 첫 주기'다.
  void onNewPath() {path_fresh_ = true;}
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
  double align_failed_since_{-1.0};   // 도착 정렬 Failed 가 시작된 시각(재무장용)
  double need_since_{-1.0};      // 조준각이 Turn 진입 문턱을 넘기 시작한 시각(C 진입 지속)
  double turn_target_{0.0};      // Turn 에서 붙든 지도 기준 목표 방향
  double turn_rot_{0.0};         // 이번 Turn 에서 돈 누적 각
  double last_yaw_{0.0};
  bool have_last_yaw_{false};
  double retarget_since_{-1.0};
  bool path_fresh_{false};       // 이번 step 이 새 경로 첫 주기인가
  bool deferred_last_{false};    // 지난 주기에 정지거리 판정을 한 번 미뤘나(연속으로는 안 미룬다)
};
}  // namespace vica_vcc_controller::core
