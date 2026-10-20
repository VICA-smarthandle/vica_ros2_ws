#include "vica_user_guidance_cpp/driver_core.hpp"

#include <cmath>
#include <cstdio>
#include <utility>

#include "vica_user_guidance_cpp/pyfmt.hpp"
#include "vica_user_guidance_cpp/rclpy_compat.hpp"

namespace vica_guidance
{
namespace
{

std::vector<double> resolve_channel_fov(
  double common, const std::vector<double> & per_channel, int channels, const std::string & name)
{
  if (static_cast<int>(per_channel.size()) != channels) {
    throw PyError(
            "ValueError", name + " 은 " + std::to_string(channels) + "칸이어야 합니다: " +
            std::to_string(per_channel.size()));
  }
  std::vector<double> out;
  for (double v : per_channel) {out.push_back(v > 0 ? v : common);}
  return out;
}

std::string rsplit_last(const std::string & s)
{
  const auto pos = s.rfind('/');
  return pos == std::string::npos ? s : s.substr(pos + 1);
}

}

DriverCore::DriverCore(const DriverParams & params, DriverIo & io, SerialFactory factory)
: p_(params), io_(io)
{
  cue_timeout_ns_ = sec_to_ns(p_.cue_timeout_sec);
  estop_timeout_ns_ = sec_to_ns(p_.estop_timeout_sec);
  arrival_hold_ns_ = sec_to_ns(p_.arrival_hold_sec);
  estop_required_ = p_.estop_required;

  warn_if_arrival_hold_too_short();

  const int64_t reconnect_ns = sec_to_ns(p_.reconnect_interval_sec);
  link_.emplace(
    p_.serial_port, p_.baudrate, p_.enable_serial, p_.write_timeout_sec, reconnect_ns,
    std::move(factory));

  send_period_ns_ = timer_period_ns(p_.send_rate_hz);
  diag_period_ns_ = timer_period_ns(p_.diag_rate_hz);

  touch_enabled_ = p_.touch_enabled;
  touch_contact_ = false;
  touch_stale_ns_ = sec_to_ns(p_.touch_stale_sec);
  touch_pub_min_ns_ = sec_to_ns(p_.touch_publish_min_interval_sec);

  us_enabled_ = p_.ultrasonic_enabled;
  if (us_enabled_) {setup_ultrasonic();}
  if (touch_enabled_) {
    io_.log(LogLevel::Info, "Touch sensor uplink enabled (D2, active-low, firmware 200ms bridge)");
  }
  if (us_enabled_ || touch_enabled_) {uplink_period_ns_ = timer_period_ns(p_.uplink_rate_hz);}

  io_.log(LogLevel::Info, "Subscribed: /vica/turn_guide, /estop_state, /vica_goal_event");
  io_.log(LogLevel::Info, "Publishing: /vica/smart_handle_state + serial");
  io_.log(LogLevel::Info, "This node never publishes /cmd_vel* nor resets the E-stop latch.");
  if (!p_.enable_serial) {
    io_.log(LogLevel::Warn, "enable_serial=false — mock mode, no serial write");
  }
  if (!estop_required_) {
    io_.log(
      LogLevel::Warn,
      "estop_required=false — stale /estop_state will NOT force ESTOP. Development only.");
  }
}

void DriverCore::warn_if_arrival_hold_too_short()
{
  const double required = protocol::firmware_arrival_duration_sec();
  const double configured = p_.arrival_hold_sec;
  if (configured < required) {
    io_.log(
      LogLevel::Error, "arrival_hold_sec=" + py_float(configured) + "s < firmware animation " +
      py_float(required) + "s. 도착 표시가 잘립니다. 값을 늘리세요.");
  }
}

void DriverCore::setup_ultrasonic()
{
  us_topics_ = p_.ultrasonic_topics;
  us_frame_ids_ = p_.ultrasonic_frame_ids;
  const auto & delays = p_.ultrasonic_measurement_delay_ms;
  if (!(us_topics_.size() == us_frame_ids_.size() && us_frame_ids_.size() == delays.size() &&
    delays.size() == static_cast<std::size_t>(protocol::US_CHANNELS)))
  {
    throw PyError(
            "ValueError",
            "ultrasonic_topics/frame_ids/measurement_delay_ms 는 모두 채널 수 " +
            std::to_string(protocol::US_CHANNELS) + "개여야 합니다: " +
            std::to_string(us_topics_.size()) + "/" + std::to_string(us_frame_ids_.size()) + "/" +
            std::to_string(delays.size()));
  }

  us_fov_ch_ = resolve_channel_fov(
    p_.ultrasonic_fov_rad, p_.ultrasonic_fov_rad_per_channel, protocol::US_CHANNELS,
    "ultrasonic_fov_rad_per_channel");
  us_min_range_ = p_.ultrasonic_min_range_m;
  us_max_range_ch_ = resolve_channel_fov(
    p_.ultrasonic_max_range_m, p_.ultrasonic_max_range_m_per_channel, protocol::US_CHANNELS,
    "ultrasonic_max_range_m_per_channel");
  us_delay_ns_.clear();
  for (int64_t ms : delays) {us_delay_ns_.push_back(static_cast<__int128>(ms) * 1000000);}
  us_stale_warn_ns_ = sec_to_ns(p_.ultrasonic_stale_warn_sec);

  us_stat_names_.clear();
  for (const auto & t : us_topics_) {us_stat_names_.push_back(rsplit_last(t));}
  us_on_ = p_.ultrasonic_channel_enabled;
  if (us_on_.size() != static_cast<std::size_t>(protocol::US_CHANNELS)) {
    throw PyError(
            "ValueError", "ultrasonic_channel_enabled 는 " + std::to_string(protocol::US_CHANNELS) +
            "칸이어야 합니다: " + std::to_string(us_on_.size()));
  }
  us_last_frame_ns_.reset();
  us_stale_warned_ = false;

  us_tf_gate_enabled_ = p_.ultrasonic_tf_gate;
  us_tf_target_ = p_.ultrasonic_tf_target_frame;

  constexpr double rad_to_deg = 180.0 / M_PI;
  std::vector<std::string> degrees;
  for (double v : us_fov_ch_) {degrees.push_back(py_round_decimal(v * rad_to_deg));}
  std::vector<std::string> on_topics;
  for (std::size_t ch = 0; ch < us_topics_.size(); ++ch) {
    if (us_on_[ch]) {on_topics.push_back(us_topics_[ch]);}
  }
  io_.log(
    LogLevel::Info, "Ultrasonic Range publishing: " + py_list_repr(us_topics_) +
    " (칠하는 폭 도 " + py_int_list_repr(degrees) + ", 최대 거리 m " +
    py_list_repr(us_max_range_ch_) + ", 켜진 채널 " + py_list_repr(on_topics) +
    ", TF 게이트 " + (us_tf_gate_enabled_ ? "켬" : "끔") + ", 기준 프레임 " + us_tf_target_ + ")");
}

void DriverCore::on_turn(int direction)
{
  turn_direction_ = direction;
  turn_last_ns_ = io_.steady_now_ns();
}

void DriverCore::on_estop(bool data)
{
  estop_active_ = data;
  estop_last_ns_ = io_.steady_now_ns();
}

void DriverCore::on_goal(const std::string & payload)
{
  const auto event = parse_goal_event(payload);
  if (!event) {
    io_.log_warn_throttled(
      5.0, "/vica_goal_event 파싱 실패 — 도착 표시가 동작하지 않습니다. "
      "JSON에 event 키가 필요합니다. payload=" + py_str_repr(py_slice(payload, 120)));
    return;
  }
  if (is_arrival_event(event)) {
    arrival_started_ns_ = io_.steady_now_ns();
    io_.log(LogLevel::Info, "도착 이벤트 수신 — Smart Handle 도착 표시 시작");
  }
}

void DriverCore::on_haptic(const std::string & data)
{
  const std::string name = py_lower_for_lookup(py_strip(data));
  int code = -1;
  if (name == "short") {
    code = protocol::HAPTIC_CMD_SHORT;
  } else if (name == "long") {
    code = protocol::HAPTIC_CMD_LONG;
  } else if (name == "tick") {
    code = protocol::HAPTIC_CMD_TICK;
  } else if (name == "locate") {
    code = protocol::HAPTIC_CMD_LOCATE;
  }
  if (code < 0) {
    io_.log(
      LogLevel::Warn, "[HAPTIC] 모르는 패턴 '" + data +
      "' — ['locate', 'long', 'short', 'tick'] 중 하나여야 합니다. 무시합니다.");
    return;
  }
  const bool ok = link_->send_raw(code, io_.steady_now_ns());
  if (ok) {
    char hex[8];
    std::snprintf(hex, sizeof(hex), "%02X", code);
    io_.log(LogLevel::Info, "[HAPTIC] " + name + " (0x" + hex + ") 전송");
  } else {
    io_.log(
      LogLevel::Warn, "[HAPTIC] " + name + " 전송 실패 — 포트 상태 fault=" +
      std::to_string(link_->fault_code()));
  }
}

void DriverCore::send_loop()
{
  const int64_t now = io_.steady_now_ns();
  link_->maybe_reconnect(now);
  GuidanceInputs in;
  in.estop_active = estop_active_;
  in.estop_last_ns = estop_last_ns_;
  in.turn_direction = turn_direction_;
  in.turn_last_ns = turn_last_ns_;
  in.arrival_started_ns = arrival_started_ns_;
  const GuidanceOutcome outcome = resolve_state_code(
    in, now, estop_timeout_ns_, cue_timeout_ns_, arrival_hold_ns_, estop_required_);
  link_->send(outcome.state_code, now);
  log_on_change(outcome);
}

void DriverCore::log_on_change(const GuidanceOutcome & outcome)
{
  if (last_reason_ && *last_reason_ == outcome.reason) {return;}
  last_reason_ = outcome.reason;
  const std::string message = std::string("[HANDLE] ") + protocol::state_name(outcome.state_code) +
    "(" + std::to_string(outcome.state_code) + ") reason=" + outcome.reason;
  io_.log(outcome.state_code == protocol::STATE_ESTOP ? LogLevel::Warn : LogLevel::Info, message);
}

void DriverCore::diag_loop()
{
  StateOut s;
  s.stamp_ns = io_.ros_now_ns();
  s.frame_id = "smart_handle";
  const bool connected = link_->connected();
  s.connected = connected;
  s.uplink_fresh = uplink_fresh(io_.steady_now_ns());
  s.user_contact = resolve_contact(touch_contact_, s.uplink_fresh);
  s.servo_ok = connected;
  s.left_led_ok = connected;
  s.right_led_ok = connected;
  s.haptic_ok = false;
  s.fault_code = link_->fault_code();
  s.last_state_code = link_->last_state_code();
  s.write_error_count = link_->write_error_count();
  io_.publish_state(s);
}

bool DriverCore::uplink_fresh(int64_t now_ns) const
{
  if (!touch_enabled_) {return false;}
  return is_fresh_ns(touch_last_frame_ns_, now_ns, touch_stale_ns_);
}

void DriverCore::uplink_loop()
{
  const int64_t now = io_.steady_now_ns();
  const Bytes data = link_->read_available(now);

  if (us_enabled_) {
    for (const auto & frame : us_acc_.feed(data)) {
      us_last_frame_ns_ = now;
      us_stale_warned_ = false;
      publish_ranges(frame);
    }
    for (const auto & sframe : us_stat_acc_.feed(data)) {
      io_.log(LogLevel::Info, format_stat_line(sframe, us_stat_names_));
    }
    for (const auto & cframe : us_cfg_acc_.feed(data)) {
      io_.log(LogLevel::Info, format_config_line(cframe, us_stat_names_));
    }
    warn_if_ultrasonic_stale(now);
  }

  if (touch_enabled_) {
    for (const auto & frame : touch_acc_.feed(data)) {
      touch_last_frame_ns_ = now;
      if (frame.touched != touch_contact_) {
        touch_contact_ = frame.touched;
        if (!is_fresh_ns(touch_pub_last_ns_, now, touch_pub_min_ns_)) {
          touch_pub_last_ns_ = now;
          diag_loop();
        }
      }
    }
  }
}

bool DriverCore::range_tf_ok(int ch, int64_t stamp_ns)
{
  if (!us_tf_gate_enabled_) {return true;}
  return io_.can_transform(us_tf_target_, us_frame_ids_[ch], stamp_ns);
}

void DriverCore::log_range_gate(int ch)
{
  const auto changed = us_gate_.take_transition(ch);
  if (!changed) {return;}
  const std::string & frame_id = us_frame_ids_[ch];
  if (*changed) {
    io_.log(
      LogLevel::Info, "[초음파] " + frame_id + " TF 복귀 — Range 발행 재개 (보류한 프레임 " +
      std::to_string(us_gate_.blocked_count(ch)) + "개)");
  } else {
    io_.log(
      LogLevel::Warn, "[초음파] " + frame_id +
      " TF 없음 — Range 발행 보류 (odom 끊김 동안 Nav2 컨트롤러를 지킨다)");
  }
}

void DriverCore::publish_ranges(const UltrasonicFrame & frame)
{
  const int64_t stamp_base = io_.ros_now_ns();
  for (int ch = 0; ch < protocol::US_CHANNELS; ++ch) {
    if (!us_on_[ch]) {continue;}
    const int mm = frame.distances_mm[ch];
    if (mm == protocol::US_DIST_INVALID) {continue;}
    const int64_t stamp = stamp_minus_delay(stamp_base, us_delay_ns_[ch]);
    const bool allowed = us_gate_.allow(ch, range_tf_ok(ch, stamp));
    log_range_gate(ch);
    if (!allowed) {continue;}
    RangeOut r;
    r.channel = ch;
    r.stamp_ns = stamp;
    r.frame_id = us_frame_ids_[ch];
    r.radiation_type = 0;
    r.field_of_view = f32_field("field_of_view", us_fov_ch_[ch]);
    r.min_range = f32_field("min_range", us_min_range_);
    const double max_range = us_max_range_ch_[ch];
    r.max_range = f32_field("max_range", max_range);
    const double value = (mm == protocol::US_CLEAR_MM || mm / 1000.0 > max_range) ?
      max_range : mm / 1000.0;
    r.range = f32_field("range", value);
    io_.publish_range(r);
  }
}

void DriverCore::warn_if_ultrasonic_stale(int64_t now_ns)
{
  if (!us_last_frame_ns_ || us_stale_warned_) {return;}
  if (now_ns - *us_last_frame_ns_ > us_stale_warn_ns_) {
    us_stale_warned_ = true;
    io_.log(LogLevel::Warn, "초음파 프레임 수신 끊김 — Range 발행 중단 (펌웨어·배선·포트 확인)");
  }
}

void DriverCore::shutdown_to_neutral()
{
  try {
    link_->send(protocol::STATE_NORMAL, io_.steady_now_ns());
  } catch (const std::exception &) {
  }
  link_->close();
}

}
