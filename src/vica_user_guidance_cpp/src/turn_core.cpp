#include "vica_user_guidance_cpp/turn_core.hpp"

#include <cmath>

#include "vica_user_guidance_cpp/guidance.hpp"
#include "vica_user_guidance_cpp/pyfmt.hpp"
#include "vica_user_guidance_cpp/pymath.hpp"

namespace vica_guidance
{
namespace
{

// 파라미터는 도(deg)로 받고 내부는 라디안 — 변환은 여기 한 곳(파이썬 판과 같은 인자 계산 순서).
TurnDetector make_detector(const TurnParams & p)
{
  const int64_t window_ns = sec_to_ns(p.window_sec);
  const double enter = py_radians(p.enter_threshold_deg);
  const double exit = py_radians(p.exit_threshold_deg);
  const int64_t min_duration_ns = sec_to_ns(p.min_duration_sec);
  const int64_t odom_timeout_ns = sec_to_ns(p.odom_timeout_sec);
  return TurnDetector(window_ns, enter, exit, min_duration_ns, odom_timeout_ns);
}

}  // namespace

TurnCore::TurnCore(const TurnParams & params, TurnIo & io)
: p_(params), io_(io), detector_(make_detector(params))
{
  speed_mps_ = 0.0;
  rail_enabled_ = p_.enable_rail_forecast;
  if (rail_enabled_) {
    // _setup_rail(exit_threshold_deg)
    rail_timeout_ns_ = sec_to_ns(p_.rail_timeout_sec);
    ArbiterParams a;
    a.lead_time_sec = p_.rail_lead_time_sec;
    a.max_offtrack_m = p_.rail_max_offtrack_m;
    a.handoff_m = p_.rail_handoff_m;
    a.still_turning_deg = p_.exit_threshold_deg;
    a.controller_lookahead_time_sec = p_.controller_lookahead_time_sec;
    a.controller_lookahead_min_m = p_.controller_lookahead_min_m;
    a.controller_lookahead_max_m = p_.controller_lookahead_max_m;
    a.minor_turn_deg = p_.minor_turn_deg;
    a.minor_turn_min_speed_mps = p_.minor_turn_min_speed_mps;
    a.min_prepare_dist_m = p_.rail_min_prepare_dist_m;
    a.rehold_end_m = p_.rail_rehold_end_m;
    arbiter_.emplace(a);
  }
}

// 파이썬 __init__ 의 나머지 순서: (레일이면 TF 버퍼·리스너·레일 구독 → 이 로그) → 발행자·/odom 구독 →
// create_timer(1.0 / publish_rate_hz) → 세 줄 로그. 노드가 그 사이사이에 구독·발행자를 만들며 부른다 —
// 토픽 이름이 틀려 구독 만들기에서 멈추면 파이썬처럼 그 앞 로그까지만 남는다.
void TurnCore::log_rail_forecast_on()
{
  io_.log(
    LogLevel::Info, "Rail forecast ON: " + p_.rail_topic + " (코너 " +
    py_fixed(p_.rail_corner_threshold_deg, 0) + "° 이상, 몸이 돌기 " +
    py_fixed(arbiter_->lead_time_sec(), 1) + "s 전)");
}

int64_t TurnCore::publish_period_ns() const
{
  return timer_period_ns(p_.publish_rate_hz);
}

void TurnCore::log_started()
{
  io_.log(LogLevel::Info, "Subscribed: " + p_.odom_topic);
  io_.log(LogLevel::Info, "Publishing: /vica/turn_guide");
  io_.log(LogLevel::Info, "This node publishes guidance cues only; it never commands motion.");
}

// ── 레일 예고 ───────────────────────────────────────

void TurnCore::on_rail(const std::string & frame_id, const std::vector<Point> & xy)
{
  // frame 이 map 이 아니면 쓰지 않는다 — 좌표계가 다른 선을 로봇 위치와 비교하면 엉뚱한 코너를 예고한다.
  if (!frame_id.empty() && frame_id != p_.map_frame) {return;}
  if (xy.size() < 2) {return;}
  const Point end = xy.back();
  if (!rail_end_ ||
    py_dist(end.first, end.second, rail_end_->first, rail_end_->second) > p_.rail_goal_change_m)
  {
    arbiter_->reset_goal();   // 레일 끝이 옮겨졌다 = 새 목적지. 알린 코너 기억을 비운다.
  }
  rail_end_ = end;
  rail_xy_ = xy;
  rail_cum_ = cumulative_lengths(xy);
  rail_corners_ = find_corners(
    xy, p_.rail_corner_threshold_deg, 0.2, 2.0, 0.6, p_.rail_merge_gap_m);
  // 레일은 가까운 노드부터 다시 잘려 오므로 s 기준점이 매번 바뀐다. 힌트를 버린다.
  s_hint_.reset();
  rail_last_ns_ = io_.steady_now_ns();
}

std::optional<TurnCore::RobotPose> TurnCore::robot_pose()
{
  std::string error;
  const auto t = io_.lookup_transform(p_.map_frame, p_.base_frame, error);
  if (!t) {
    if (!tf_warned_) {
      io_.log(LogLevel::Warn, "레일 예고: TF 조회 실패 — 예고를 쉰다 (" + error + ")");
      tf_warned_ = true;
    }
    return std::nullopt;
  }
  tf_warned_ = false;
  return RobotPose{t->x, t->y, yaw_from_quaternion(t->qx, t->qy, t->qz, t->qw)};
}

RailCue TurnCore::rail_cue(const TurnDecision & decision, int64_t now_ns)
{
  std::optional<RailPose> pose;
  std::optional<double> yaw;
  if (rail_xy_.size() >= 2) {
    const auto robot = robot_pose();
    if (robot) {
      pose = project_to_path(
        rail_xy_, {robot->x, robot->y}, rail_cum_, s_hint_, p_.rail_behind_start_m);
      if (pose) {s_hint_ = pose->s;}
      yaw = robot->yaw;
    }
  }
  const bool fresh = is_fresh_ns(rail_last_ns_, now_ns, rail_timeout_ns_);
  return arbiter_->resolve(decision, rail_corners_, pose, std::fabs(speed_mps_), fresh, yaw);
}

// ── 공통 ─────────────────────────────────────────

void TurnCore::on_odom(double qx, double qy, double qz, double qw, double linear_x)
{
  // header.stamp(SYSTEM_TIME)가 아니라 수신 시각(STEADY_TIME)을 쓴다 — 두 시간축을 빼지 않는다.
  speed_mps_ = linear_x;
  detector_.add_odom(yaw_from_quaternion(qx, qy, qz, qw), io_.steady_now_ns());
}

GuideOut TurnCore::stamp(GuideOut msg)
{
  // header.stamp·valid_until 은 노드 시계 — 로그·rosbag·앱 표시 전용, 소비자의 stale 판정에 쓰지 않는다.
  const int64_t now = io_.ros_now_ns();
  msg.stamp_ns = time_msg_ns(now);
  msg.frame_id = "base_footprint";
  msg.valid_until_ns = time_plus_seconds(now, p_.cue_valid_sec);
  return msg;
}

void TurnCore::publish_loop()
{
  const int64_t now = io_.steady_now_ns();
  const TurnDecision decision = detector_.evaluate(now);
  GuideOut msg;
  if (rail_enabled_) {
    const RailCue cue = rail_cue(decision, now);
    io_.trace_reason(cue.reason);
    msg.direction = static_cast<uint8_t>(cue.direction);
    msg.phase = static_cast<uint8_t>(cue.phase);
    msg.distance_m = f32_field("distance_m", cue.distance_m);   // PREPARE 에서만 코너까지 거리
    msg.turn_angle_deg = f32_field("turn_angle_deg", cue.turn_angle_deg);
    msg.sequence_id = u32_field("sequence_id", cue.sequence_id);
    msg.source_stale = cue.source_stale;
  } else {
    msg.direction = static_cast<uint8_t>(decision.direction);
    msg.phase = static_cast<uint8_t>(decision.phase);
    msg.distance_m = NAN;   // 사후 판정에는 거리가 없다
    msg.turn_angle_deg = f32_field("turn_angle_deg", decision.turn_angle_deg);
    msg.sequence_id = u32_field("sequence_id", decision.sequence_id);
    msg.source_stale = decision.source_stale;
  }
  io_.publish_guide(stamp(msg));
}

void TurnCore::publish_idle_once()
{
  GuideOut msg;
  const int64_t now = io_.ros_now_ns();
  msg.stamp_ns = time_msg_ns(now);
  msg.frame_id = "base_footprint";
  msg.direction = DIRECTION_NONE;
  msg.phase = PHASE_IDLE;
  msg.distance_m = NAN;
  msg.turn_angle_deg = 0.0f;
  msg.sequence_id = u32_field("sequence_id", detector_.sequence_id());
  msg.valid_until_ns = time_msg_ns(now);
  msg.source_stale = true;
  io_.publish_guide(msg);
}

}  // namespace vica_guidance
