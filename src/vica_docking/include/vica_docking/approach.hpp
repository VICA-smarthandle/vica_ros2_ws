#pragma once

#include <optional>
#include <string>
#include <vector>

#include "vica_docking/geometry.hpp"
#include "vica_docking/types.hpp"
#include "vica_docking/v_detector.hpp"

namespace vica_docking
{

struct ApproachParams
{
  double target_cm{14.0};
  double stage_cm{100.0};
  double lead{0.0136};
  double vmax{0.10};
  double vfinal{0.05};
  double wmax{0.30};
  double k_b{1.0};
  double deadband_deg{0.6};
  double align_tol_deg{1.2};
  double max_time{60.0};
  double rot_clear{0.72};
  double find_timeout{5.0};
  double start_max_lateral{0.30};
  double start_max_heading_deg{30.0};
  double no_motion_s{2.0};
  double ok_x_cm{1.5};
  double ok_y_cm{1.5};
  double ok_th_deg{5.0};
  double front_zone{0.30};
  int front_min_pts{3};
  double front_release_s{1.0};
  double front_wait_timeout{30.0};
  double lost_timeout_s{3.0};
};

enum class ApproachPhase
{
  kWait, kRotateToStage, kDriveToStage, kAlign, kApproach, kStopped, kDone, kAbort
};
const char * phase_name(ApproachPhase p);

struct ApproachResult
{
  bool has_start{false};
  DockPose start;
  bool has_stop{false};
  DockPose stop;
  bool has_final{false};
  DockPose final_pose;
  double err_x_cm{0.0};
  double err_y_cm{0.0};
  double err_th_deg{0.0};
  bool success{false};
  Fail fail{Fail::kNone};
  std::string why;
  int rejected{0};
  int holds{0};
  int front_holds{0};
};

class ApproachController
{
public:
  explicit ApproachController(ApproachParams p = {}, RobotGeometry g = {}, VDetectorParams vp = {});

  void start(double now);
  void on_scan(
    double now, const std::vector<float> & ranges, double amin, double inc, double range_min);
  Cmd on_timer(double now);
  void abort(Fail f, const std::string & why);

  ApproachPhase phase() const {return phase_;}
  bool active() const;
  bool finished() const {return phase_ == ApproachPhase::kDone || phase_ == ApproachPhase::kAbort;}
  const ApproachResult & result() const {return result_;}
  std::optional<DockPose> last_pose() const {return last_pose_;}
  Cmd planned_cmd() const {return cmd_;}
  bool front_blocked() const {return front_hold_;}
  int front_hits() const {return front_hits_;}
  double d_target() const {return d_target_;}
  int scans() const {return scans_;}

private:
  void step(double now, const VDetection & det, const DockPose & pz);
  void set_cmd(double v, double w);
  double carrot_bearing(const VDetection & det, double x) const;
  void finish();

  ApproachParams p_;
  RobotGeometry g_;
  VDetector det_;
  double d_target_;
  double d_stage_;
  ApproachPhase phase_{ApproachPhase::kWait};
  double t0_{0.0};
  Cmd cmd_;
  std::optional<VDetection> good_;
  double good_t_{0.0};
  bool has_det_t_{false};
  double last_det_t_{0.0};
  int streak_{0};
  bool has_streak_ref_{false};
  DockPose streak_ref_;
  double settle_t_{0.0};
  std::vector<DockPose> settle_;
  std::optional<DockPose> last_pose_;
  int front_hits_{0};
  int scans_{0};
  int path_hits_{0};
  bool front_hold_{false};
  double front_since_{0.0};
  bool has_front_clear_{false};
  double front_clear_since_{0.0};
  bool has_move_ref_{false};
  bool move_forward_{false};
  double move_t_{0.0};
  DockPose move_pose_;
  ApproachResult result_;
};

}
