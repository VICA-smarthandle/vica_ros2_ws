// 레일(Route Server) 경로의 큰 코너를 미리 예고한다 — 파이썬 판 vica_user_guidance/rail_turn_forecast.py
// 를 그대로 옮겼다. 규칙의 이유(이어받기·코너 끝까지·알린 코너 기억, 실측 근거)는 파이썬 판 주석이 정본이다.
//
// ROS 에 기대지 않는다. 좌표는 호출자가 전부 같은 frame(map)으로 맞춰 넣는다. 읽기만 한다 — 경로를
// 바꾸거나 컨트롤러에 무엇을 보내지 않는다.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "vica_user_guidance_cpp/turn_detector.hpp"

namespace vica_guidance
{

using Point = std::pair<double, double>;

struct RailCorner
{
  int direction = 0;        // DIRECTION_LEFT / DIRECTION_RIGHT
  double angle_deg = 0.0;   // 코너 전체 꺾임. CCW(+) = LEFT
  double start_s = 0.0;
  double end_s = 0.0;
  Point start_xy;           // 코너 신원
  Point end_xy;
};

struct RailPose
{
  double s = 0.0;
  double offtrack_m = 0.0;
  double remaining_m = 0.0;
  double heading_rad = 0.0;
};

std::vector<double> cumulative_lengths(const std::vector<Point> & path_xy);

std::optional<RailPose> project_to_path(
  const std::vector<Point> & path_xy, Point robot_xy, const std::vector<double> & cum,
  std::optional<double> s_hint, double behind_start_m, double hint_back_m = 1.0,
  double hint_ahead_m = 3.0);

std::vector<RailCorner> find_corners(
  const std::vector<Point> & path_xy, double threshold_deg = 45.0, double step_m = 0.2,
  double deadband_deg = 2.0, double max_gap_m = 0.6, double merge_gap_m = 1.2);

// 이 tick 의 최종 cue(파이썬 RailCue).
struct RailCue
{
  int direction = 0;
  int phase = PHASE_IDLE;
  double distance_m = 0.0;       // PREPARE 에서만 코너까지 거리. 그 밖엔 NaN
  double turn_angle_deg = 0.0;
  int64_t sequence_id = 0;
  bool source_stale = false;
  std::string reason;
};

// 파라미터로 바꾸는 값만 생성자에서 받고, 나머지는 파이썬 dataclass 기본값 그대로다.
struct ArbiterParams
{
  double lead_time_sec = 2.0;
  double max_offtrack_m = 0.8;
  double handoff_m = 2.0;
  double still_turning_deg = 10.0;
  double controller_lookahead_time_sec = 2.5;
  double controller_lookahead_min_m = 0.6;
  double controller_lookahead_max_m = 1.2;
  double minor_turn_deg = 35.0;
  double minor_turn_min_speed_mps = 0.25;
  double min_prepare_dist_m = 0.0;
  double rehold_end_m = 0.0;
};

class RailTurnArbiter
{
public:
  explicit RailTurnArbiter(const ArbiterParams & p);

  void reset_goal();
  RailCue resolve(
    const TurnDecision & decision, const std::vector<RailCorner> & corners,
    const std::optional<RailPose> & pose, double speed_mps, bool rail_fresh,
    std::optional<double> robot_yaw_rad);

  double lead_time_sec() const {return p_.lead_time_sec;}

  // dataclass 기본값(파라미터로 바꾸지 않는다)
  static constexpr double min_speed_mps = 0.2;
  static constexpr double pass_margin_m = 0.3;
  static constexpr double same_corner_m = 0.5;
  static constexpr double consume_back_m = 1.0;
  static constexpr double consume_ahead_m = 1.5;
  static constexpr double max_heading_error_deg = 45.0;
  static constexpr double min_moving_speed_mps = 0.05;

private:
  struct Active
  {
    int phase = PHASE_IDLE;
    int direction = 0;
    std::optional<Point> corner_xy;
    std::optional<Point> corner_end_xy;
  };

  bool known(const Point & xy) const;
  void remember(const Point & xy);
  std::optional<RailCorner> find(
    const std::vector<RailCorner> & corners, const std::optional<Point> & xy) const;
  std::optional<RailCorner> refind(
    const std::vector<RailCorner> & corners, const std::optional<RailPose> & pose, int direction,
    const std::optional<Point> & end_xy) const;
  double lookahead(double speed) const;
  bool minor_swerve(const TurnDecision & decision, double speed_mps) const;
  std::optional<RailCorner> claim(
    const std::vector<RailCorner> & corners, const std::optional<RailPose> & pose, int direction);
  std::optional<RailCorner> next_prepare(
    const std::vector<RailCorner> & corners, const std::optional<RailPose> & pose,
    double speed_mps, bool on_rail, std::optional<double> robot_yaw_rad) const;
  RailCue start_prepare(const RailCorner & c, const RailPose & pose, const std::string & reason);

  ArbiterParams p_;
  int64_t seq_ = 0;
  Active active_;
  std::vector<Point> announced_;
};

}  // namespace vica_guidance
