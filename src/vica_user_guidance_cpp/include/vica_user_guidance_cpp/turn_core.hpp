#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "vica_user_guidance_cpp/rail_turn_forecast.hpp"
#include "vica_user_guidance_cpp/rclpy_compat.hpp"
#include "vica_user_guidance_cpp/turn_detector.hpp"

namespace vica_guidance
{

struct TurnParams
{
  std::string odom_topic = "/odom";
  double window_sec = 1.5;
  double enter_threshold_deg = 20.0;
  double exit_threshold_deg = 10.0;
  double min_duration_sec = 0.6;
  double odom_timeout_sec = 0.5;
  double publish_rate_hz = 20.0;
  double cue_valid_sec = 2.0;

  bool enable_rail_forecast = false;
  std::string rail_topic = "/rail_plan";
  double rail_timeout_sec = 2.5;
  std::string map_frame = "map";
  std::string base_frame = "base_footprint";
  double rail_corner_threshold_deg = 45.0;
  double rail_lead_time_sec = 2.0;
  double rail_max_offtrack_m = 0.8;
  double rail_handoff_m = 2.0;
  double rail_merge_gap_m = 1.2;
  double rail_goal_change_m = 0.5;
  double rail_behind_start_m = 0.0;
  double rail_min_prepare_dist_m = 0.0;
  double rail_rehold_end_m = 0.0;
  double controller_lookahead_time_sec = 2.5;
  double controller_lookahead_min_m = 0.6;
  double controller_lookahead_max_m = 1.2;
  double minor_turn_deg = 35.0;
  double minor_turn_min_speed_mps = 0.25;
};

struct GuideOut
{
  int64_t stamp_ns = 0;
  std::string frame_id;
  uint8_t direction = 0;
  uint8_t phase = 0;
  float distance_m = 0.f;
  float turn_angle_deg = 0.f;
  uint32_t sequence_id = 0;
  int64_t valid_until_ns = 0;
  bool source_stale = false;
};

struct TfPose
{
  double x = 0.0, y = 0.0;
  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
};

class TurnIo
{
public:
  virtual ~TurnIo() = default;
  virtual int64_t steady_now_ns() = 0;
  virtual int64_t ros_now_ns() = 0;
  virtual void log(LogLevel level, const std::string & text) = 0;
  virtual void publish_guide(const GuideOut & msg) = 0;
  virtual std::optional<TfPose> lookup_transform(
    const std::string & target, const std::string & source, std::string & error) = 0;
  virtual void trace_reason(const std::string &) {}
};

class TurnCore
{
public:
  TurnCore(const TurnParams & params, TurnIo & io);
  void log_rail_forecast_on();
  int64_t publish_period_ns() const;
  void log_started();

  void on_odom(double qx, double qy, double qz, double qw, double linear_x);
  void on_rail(const std::string & frame_id, const std::vector<Point> & xy);
  void publish_loop();
  void publish_idle_once();

  bool rail_enabled() const {return rail_enabled_;}
  const TurnParams & params() const {return p_;}

private:
  struct RobotPose
  {
    double x, y, yaw;
  };
  std::optional<RobotPose> robot_pose();
  RailCue rail_cue(const TurnDecision & decision, int64_t now_ns);
  GuideOut stamp(GuideOut msg);

  TurnParams p_;
  TurnIo & io_;
  TurnDetector detector_;
  double speed_mps_ = 0.0;
  bool rail_enabled_ = false;

  int64_t rail_timeout_ns_ = 0;
  std::optional<RailTurnArbiter> arbiter_;
  std::vector<Point> rail_xy_;
  std::vector<double> rail_cum_;
  std::vector<RailCorner> rail_corners_;
  std::optional<int64_t> rail_last_ns_;
  std::optional<Point> rail_end_;
  std::optional<double> s_hint_;
  bool tf_warned_ = false;
};

}
