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
  double until_cm{85.0};
  double v_back{0.08};
  double lead_back{0.024};
  double clear_pre{0.80};
  double stop_margin{0.25};
  double wait_timeout{30.0};
  double release_s{1.0};
  double release_gap{0.03};
  double turn_stop{0.25};
  double turn_resume{0.30};
  double slow_dist{0.40};
  double w_turn_slow{0.12};
  double turn_deg{180.0};
  double w_turn{0.30};
  int turn_dir{1};
  double lead_turn_deg{5.4};
  double max_time{120.0};
  double us_fresh_s{1.0};
  double turn_done_deg{10.0};
  double sweep_r{0.64};
  double slow_r{0.80};
  double find_timeout{5.0};
};

enum class UndockPhase
{
  kWait, kPrecheck, kBack, kSettleBack, kTurn, kSettleTurn, kDone, kAbort
};
const char * phase_name(UndockPhase p);

enum UsIndex { kRearRight = 0, kRearLeft = 1, kSideRight = 2, kSideLeft = 3 };

struct UndockResult
{
  double start_x{0.0};
  double back_cm{0.0};
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
  void start_turn_only(double now, double remaining_deg);
  void on_scan(
    double now, const std::vector<float> & ranges, double amin, double inc, double range_min);
  void on_ultrasonic(int index, double now, double range, double max_range);
  void on_gyro(double now, double wz);
  Cmd on_timer(double now);
  void abort(Fail f, const std::string & why);

  UndockPhase phase() const {return phase_;}
  bool finished() const {return phase_ == UndockPhase::kDone || phase_ == UndockPhase::kAbort;}
  const UndockResult & result() const {return result_;}
  UndockPhase abort_phase() const {return abort_phase_;}
  double turn_remaining_deg() const;
  const std::string & hold_reason() const {return hold_reason_;}
  bool holding() const {return has_hold_;}
  std::optional<double> last_x() const {return has_x_ ? std::optional<double>(x_) : std::nullopt;}
  int scans() const {return scans_;}

private:
  struct UsMin
  {
    std::optional<double> d;
    bool fresh{false};
  };
  UsMin us_min(double now, bool all4) const;
  bool set_hold(double now, const std::string & why, bool clear_ok);

  UndockParams p_;
  RobotGeometry g_;
  VDetector det_;
  UndockPhase phase_{UndockPhase::kWait};
  double t0_{0.0};
  bool has_x_{false};
  double x_{0.0};
  double th_{0.0};
  double x_t_{0.0};
  double x_start_{0.0};
  double back_{0.0};
  int streak_{0};
  bool has_streak_ref_{false};
  double streak_ref_{0.0};
  std::array<bool, 4> us_has_{{false, false, false, false}};
  std::array<double, 4> us_r_{};
  std::array<double, 4> us_t_{};
  double yaw_{0.0};
  bool has_imu_t_{false};
  double imu_t_{0.0};
  double yaw_turn0_{0.0};
  double turn_target_deg_{0.0};
  int turn_sign_{1};
  UndockPhase abort_phase_{UndockPhase::kWait};
  double turn_stop_deg_{0.0};
  double back_stop_x_{0.0};
  int scans_{0};
  int sweep_hits_{0};
  int slow_hits_{0};
  double w_now_{0.0};
  bool has_hold_{false};
  double hold_since_{0.0};
  std::string hold_reason_;
  bool has_clear_since_{false};
  double clear_since_{0.0};
  double hold_total_{0.0};
  double settle_until_{0.0};
  std::vector<double> settle_x_;
  bool has_move_ref_{false};
  double move_t_{0.0};
  double move_x_{0.0};
  double move_yaw_{0.0};
  UndockResult result_;
};

}
