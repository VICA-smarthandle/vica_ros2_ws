// 도크에서 나오기: 초음파를 보며 범퍼→꼭짓점 85 cm 까지 후진 → 제자리 180° 회전.
//
// 11-18 실기 도구 undock_test.py 의 판정을 그대로 옮겼다(후진 넘침 평균 2.36 cm, 회전 180°±1.3°,
// 사람이 뒤에 섰다 비키는 시험에서 멈춤·재출발 정상).
//   WAIT(V자 5회 일치) → PRECHECK → BACK → SETTLE_BACK → TURN → SETTLE_TURN → DONE
//   · 출발 전: 뒤 초음파 두 개가 clear_pre 안에 아무것도 없어야 한다(막히면 기다림)
//   · 후진: 뒤 거리 < 남은 거리 + 0.25 m 이면 0 으로 기다림, 여유(+0.03 m)가 1 s 이어지면 다시 감
//   · 회전: 초음파 4개(뒤 2·옆뒤 2) 중 0.25 m 안이거나 라이다 ±120°·반경 0.64 m 안 3점이면 멈춤,
//           0.30 m 이상이 1 s 이어지면 다시 돎. 0.40 m 안(라이다 0.80 m 안 3점)이면 0.12 rad/s 로 천천히
//   · 회전각은 /imu/base_link 자이로 z 적분. 정지 앞당김 5.4° 를 회전 속도에 비례
//   · 2 s 명령에도 안 움직이면 중단, 30 s 넘게 막히면 중단
// 운영에서 더한 것: 5 s 안에 V자를 못 찾으면 중단.
#pragma once

#include <array>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "vica_docking/geometry.hpp"
#include "vica_docking/types.hpp"
#include "vica_docking/v_detector.hpp"

namespace vica_docking
{

struct UndockParams
{
  double until_cm{85.0};       // 범퍼→꼭짓점 이 거리까지 후진(11-18 사용자)
  double v_back{0.08};
  double lead_back{0.024};     // 후진 정지 앞당김 m(11-18 실측 넘침)
  double clear_pre{0.80};      // 출발 전 뒤가 비어야 하는 거리 = 뒤 초음파 상한(11-18 사용자 0.8 m)
  double stop_margin{0.25};
  double wait_timeout{30.0};
  double release_s{1.0};
  double release_gap{0.03};
  double turn_stop{0.25};
  double turn_resume{0.30};
  double slow_dist{0.40};
  double w_turn_slow{0.12};
  double turn_deg{180.0};      // 0 이면 후진만(도킹 재시도용)
  double w_turn{0.30};
  int turn_dir{1};             // 1 반시계 / −1 시계
  double lead_turn_deg{5.4};
  double max_time{120.0};
  double us_fresh_s{1.0};      // 4채널이면 채널당 약 2 Hz(11-18 실측)
  double turn_done_deg{10.0};  // 남은 회전이 이 안이면 다 나온 것으로 본다(11-19 사용자 "10도 이하라면 끝났다고 간주")
  double sweep_r{0.64};
  double slow_r{0.80};
  double find_timeout{5.0};    // 운영에서 더함
};

enum class UndockPhase
{
  kWait, kPrecheck, kBack, kSettleBack, kTurn, kSettleTurn, kDone, kAbort
};
const char * phase_name(UndockPhase p);

// 초음파 칸 순서
enum UsIndex { kRearRight = 0, kRearLeft = 1, kSideRight = 2, kSideLeft = 3 };

struct UndockResult
{
  double start_x{0.0};
  double back_cm{0.0};        // 실제 후진(V자 기준)
  double back_overshoot_cm{0.0};
  double turned_deg{0.0};
  double hold_s{0.0};
  bool success{false};
  Fail fail{Fail::kNone};
  std::string why;
};

class UndockController
{
public:
  explicit UndockController(UndockParams p = {}, RobotGeometry g = {}, VDetectorParams vp = {});

  void start(double now);
  // 중간에 멈춘 나오기를 이어서(11-19 사용자): 후진은 끝났고 회전만 남았을 때. remaining_deg 는 turn_dir 방향이 +,
  // 지나쳤으면 −(반대로 돈다). turn_done_deg(10°) 안이면 움직이지 않고 끝낸다. V자는 보지 않는다(이미 돌아서 안 보인다).
  void start_turn_only(double now, double remaining_deg);
  void on_scan(
    double now, const std::vector<float> & ranges, double amin, double inc, double range_min);
  // range 가 없거나(inf) max_range 근처면 '비어 있음'
  void on_ultrasonic(int index, double now, double range, double max_range);
  void on_gyro(double now, double wz);
  Cmd on_timer(double now);
  void abort(Fail f, const std::string & why);

  UndockPhase phase() const {return phase_;}
  bool finished() const {return phase_ == UndockPhase::kDone || phase_ == UndockPhase::kAbort;}
  const UndockResult & result() const {return result_;}
  // 멈춘(abort) 단계와, 그때 남은 회전(°, turn_dir 방향 +). 후진이 덜 끝났으면 0 — 처음부터 다시 나오면 된다.
  UndockPhase abort_phase() const {return abort_phase_;}
  double turn_remaining_deg() const;
  // 진행 표시
  const std::string & hold_reason() const {return hold_reason_;}
  bool holding() const {return has_hold_;}
  // 최근 V자 축거리(못 봤으면 nullopt)
  std::optional<double> last_x() const {return has_x_ ? std::optional<double>(x_) : std::nullopt;}
  // 받은 스캔 수(V자를 못 본 것이 '스캔이 안 와서'인지 '도크가 없어서'인지 가른다)
  int scans() const {return scans_;}

private:
  struct UsMin
  {
    std::optional<double> d;   // 받은 값 중 최소(비어 있으면 inf). 받은 값이 없으면 nullopt
    bool fresh{false};
  };
  UsMin us_min(double now, bool all4) const;
  bool set_hold(double now, const std::string & why, bool clear_ok);

  UndockParams p_;
  RobotGeometry g_;
  VDetector det_;
  UndockPhase phase_{UndockPhase::kWait};
  double t0_{0.0};
  // V자 기준 위치(최근)
  bool has_x_{false};
  double x_{0.0};
  double th_{0.0};
  double x_t_{0.0};
  double x_start_{0.0};
  double back_{0.0};
  int streak_{0};
  bool has_streak_ref_{false};
  double streak_ref_{0.0};
  // 초음파: 값(inf = 비어 있음), 받은 시각
  std::array<bool, 4> us_has_{{false, false, false, false}};
  std::array<double, 4> us_r_{};
  std::array<double, 4> us_t_{};
  // 자이로 적분
  double yaw_{0.0};
  bool has_imu_t_{false};
  double imu_t_{0.0};
  double yaw_turn0_{0.0};
  double turn_target_deg_{0.0};   // 이번에 돌 각(보통 turn_deg, 이어서 할 때는 남은 각)
  int turn_sign_{1};              // 이번 회전 방향(보통 turn_dir)
  UndockPhase abort_phase_{UndockPhase::kWait};
  double turn_stop_deg_{0.0};
  double back_stop_x_{0.0};
  // 회전 감시(라이다)
  int scans_{0};
  int sweep_hits_{0};
  int slow_hits_{0};
  double w_now_{0.0};
  // 멈춤
  bool has_hold_{false};
  double hold_since_{0.0};
  std::string hold_reason_;
  bool has_clear_since_{false};
  double clear_since_{0.0};
  double hold_total_{0.0};
  // 정착
  double settle_until_{0.0};
  std::vector<double> settle_x_;
  // 안 움직임 검사
  bool has_move_ref_{false};
  double move_t_{0.0};
  double move_x_{0.0};
  double move_yaw_{0.0};
  UndockResult result_;
};

}  // namespace vica_docking
