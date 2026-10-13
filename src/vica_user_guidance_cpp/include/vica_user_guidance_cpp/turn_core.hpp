// 방향지시 노드의 판정·순서 — 파이썬 판 turn_guide_node.py 의 메서드를 그대로 옮겼다.
//
// [안전 경계] 이 노드는 어떤 구동 명령도 발행하지 않는다(파이썬 판과 같다). /cmd_vel_req·/cmd_vel_safe·
// Nav2 goal 에 일절 관여하지 않는 순수 판정 계층이다. 레일도 읽기만 한다.
//
// ROS 와 닿는 곳(시계·로그·발행·TF 조회)은 TurnIo 로 뺐다. 노드는 진짜 rclcpp 로, 대조 시험은 기록용
// 가짜(TF 는 진짜 tf2::BufferCore)로 채운다.
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

// 파라미터 — 이름·기본값은 파이썬 판 declare_parameter 와 같다(launch 는 YAML 이 덮어쓴다).
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

// vica_interfaces/TurnGuide 한 건(칸 검사를 마친 값).
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

// lookup_transform 결과 중 쓰는 칸.
struct TfPose
{
  double x = 0.0, y = 0.0;
  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
};

class TurnIo
{
public:
  virtual ~TurnIo() = default;
  virtual int64_t steady_now_ns() = 0;   // STEADY_TIME — 모든 신선도 판정
  virtual int64_t ros_now_ns() = 0;      // 노드 시계 — 메시지 stamp·valid_until
  virtual void log(LogLevel level, const std::string & text) = 0;
  virtual void publish_guide(const GuideOut & msg) = 0;
  // tf_buffer.lookup_transform(target, source, Time()) — 실패하면 nullopt, error 에 예외 글(str(exc)).
  virtual std::optional<TfPose> lookup_transform(
    const std::string & target, const std::string & source, std::string & error) = 0;
  // 대조 시험만 쓴다: 그 tick 의 arbiter 판정 이유. 노드는 아무것도 안 한다.
  virtual void trace_reason(const std::string &) {}
};

class TurnCore
{
public:
  // 파이썬 __init__ 의 앞부분(파라미터 계산·판정기·레일 예고 준비). 설정 오류는 PyError.
  TurnCore(const TurnParams & params, TurnIo & io);
  // 나머지는 노드가 구독·발행자를 만드는 사이사이에 파이썬과 같은 순서로 부른다:
  // [레일이면 TF·레일 구독 뒤] log_rail_forecast_on → 발행자·/odom 구독 → publish_period_ns(타이머) → log_started.
  void log_rail_forecast_on();
  int64_t publish_period_ns() const;   // create_timer(1.0 / publish_rate_hz) 와 같은 오류로 멈춘다
  void log_started();

  void on_odom(double qx, double qy, double qz, double qw, double linear_x);
  void on_rail(const std::string & frame_id, const std::vector<Point> & xy);
  void publish_loop();
  // 종료 직전 IDLE 을 1회 발행해 소비자가 회전 상태에 갇히지 않게 한다.
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

  // 레일 예고 상태(rail_enabled_ 일 때만 쓴다)
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

}  // namespace vica_guidance
