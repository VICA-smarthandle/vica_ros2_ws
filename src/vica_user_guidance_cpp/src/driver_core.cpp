#include "vica_user_guidance_cpp/driver_core.hpp"

#include <cmath>
#include <cstdio>
#include <utility>

#include "vica_user_guidance_cpp/pyfmt.hpp"

namespace vica_guidance
{
namespace
{

constexpr __int128 INT64_LIMIT = static_cast<__int128>(1) << 63;

// rclpy Node.create_timer(1.0 / rate) 가 받아들이는 주기(ns). 파이썬과 같은 오류로 멈춘다.
int64_t timer_period_ns(double rate_hz)
{
  if (rate_hz == 0.0) {throw PyError("ZeroDivisionError", "float division by zero");}
  const double x = (1.0 / rate_hz) * 1000000000.0;   // int(float(timer_period_sec) * S_TO_NS)
  if (std::isnan(x)) {throw PyError("ValueError", "cannot convert float NaN to integer");}
  if (std::isinf(x)) {throw PyError("OverflowError", "cannot convert float infinity to integer");}
  const double t = std::trunc(x);
  if (t < 0) {
    throw PyError(
            "RCLError",
            "failed to create timer: timer period must be non-negative, at ./src/rcl/timer.c:139");
  }
  if (t >= 9223372036854775807.0) {
    throw PyError("TypeError", "__init__(): incompatible constructor arguments (timer period)");
  }
  return static_cast<int64_t>(t);
}

// resolve_channel_fov: 양수 칸은 그 값, 0 이하(NaN 포함) 칸은 공통값. 길이가 다르면 ValueError.
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

// (stamp_base - Duration(nanoseconds=delay)).to_msg() 의 검사들 — rclpy 와 같은 순서·문구.
int64_t stamp_minus_delay(int64_t base_ns, __int128 delay_ns)
{
  if (delay_ns >= INT64_LIMIT || delay_ns < -INT64_LIMIT) {
    throw PyError("OverflowError", "Total nanoseconds value is too large to store in C duration.");
  }
  const __int128 r = static_cast<__int128>(base_ns) - delay_ns;
  if (r < 0) {throw PyError("ValueError", "Subtraction leads to negative time.");}
  if (r >= INT64_LIMIT) {
    throw PyError("OverflowError", "Total nanoseconds value is too large to store in C time point.");
  }
  const int64_t ns = static_cast<int64_t>(r);
  if (ns / 1000000000 > 2147483647) {
    throw PyError("AssertionError", "The 'sec' field must be an integer in [-2147483648, 2147483647]");
  }
  return ns;
}

// rosidl 파이썬 float32 칸의 범위 검사. inf 는 통과, 3.402823466e+38 을 넘는 유한값은 AssertionError.
float f32_field(const char * name, double v)
{
  if ((v < -3.402823466e+38 || v > 3.402823466e+38) && !std::isinf(v)) {
    throw PyError(
            "AssertionError", std::string("The '") + name +
            "' field must be a float in [-3.402823466e+38, 3.402823466e+38]");
  }
  return static_cast<float>(v);
}

std::string rsplit_last(const std::string & s)
{
  const auto pos = s.rfind('/');
  return pos == std::string::npos ? s : s.substr(pos + 1);
}

}  // namespace

DriverCore::DriverCore(const DriverParams & params, DriverIo & io, SerialFactory factory)
: p_(params), io_(io)
{
  cue_timeout_ns_ = sec_to_ns(p_.cue_timeout_sec);
  estop_timeout_ns_ = sec_to_ns(p_.estop_timeout_sec);
  arrival_hold_ns_ = sec_to_ns(p_.arrival_hold_sec);
  estop_required_ = p_.estop_required;

  warn_if_arrival_hold_too_short();

  // SerialLink(…, reconnect_interval_ns=sec_to_ns(…)) — 인자 계산이 열기보다 먼저다.
  const int64_t reconnect_ns = sec_to_ns(p_.reconnect_interval_sec);
  link_.emplace(
    p_.serial_port, p_.baudrate, p_.enable_serial, p_.write_timeout_sec, reconnect_ns,
    std::move(factory));

  send_period_ns_ = timer_period_ns(p_.send_rate_hz);
  diag_period_ns_ = timer_period_ns(p_.diag_rate_hz);

  // 터치 상태는 enable 여부와 무관하게 먼저 초기화한다(2026-09-04 사고: 진단이 없는 필드를 읽어 죽음).
  touch_enabled_ = p_.touch_enabled;
  touch_contact_ = false;
  touch_stale_ns_ = sec_to_ns(p_.touch_stale_sec);
  touch_pub_min_ns_ = sec_to_ns(p_.touch_publish_min_interval_sec);

  // 상향 시리얼은 한 스트림이다. 읽기는 uplink_loop 한 곳에서만 하고 누적기들에 같이 먹인다.
  us_enabled_ = p_.ultrasonic_enabled;
  if (us_enabled_) {setup_ultrasonic();}
  if (touch_enabled_) {
    io_.log(LogLevel::Info, "Touch sensor uplink enabled (D11, active-low, firmware 200ms bridge)");
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
  // 펌웨어 재생 시간(3.5 초)보다 짧으면 마지막 프레임이 잘려 "2.5회 점멸"로 보인다.
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
    // 채널 수가 어긋난 채 돌면 엉뚱한 frame_id 로 발행된다. 기동을 막는다.
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

  // f-string 은 왼쪽부터 계산된다 — round(degrees) 가 먼저 실패할 수 있다.
  constexpr double rad_to_deg = 180.0 / M_PI;   // CPython math.degrees 의 radToDeg
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

// ── 콜백 ───────────────────────────────────────────

void DriverCore::on_turn(int direction)
{
  // phase 는 보지 않고 direction 만 쓴다(COMPLETE 를 놓쳐도 NONE 으로 기본 상태에 떨어진다).
  turn_direction_ = direction;
  turn_last_ns_ = io_.steady_now_ns();
}

void DriverCore::on_estop(bool data)
{
  // 중앙 래치 결과를 구독만 한다. 여기서 reset 하지 않는다.
  estop_active_ = data;
  estop_last_ns_ = io_.steady_now_ns();
}

void DriverCore::on_goal(const std::string & payload)
{
  const auto event = parse_goal_event(payload);
  if (!event) {
    // 미션이 payload 형식을 바꾸면 도착 표시가 조용히 사라진다 — 단서를 남긴다(앞부분만, 5 초에 한 번).
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
  // 10 Hz 상태코드 사이에 끼우는 바이트라 LED·서보에는 영향이 없다. send_raw 라 last_state_code 를 안 건드린다.
  const std::string name = py_lower_for_lookup(py_strip(data));
  int code = -1;
  if (name == "short") {
    code = protocol::HAPTIC_CMD_SHORT;
  } else if (name == "long") {
    code = protocol::HAPTIC_CMD_LONG;
  } else if (name == "tick") {
    code = protocol::HAPTIC_CMD_TICK;
  } else if (name == "locate") {
    code = protocol::HAPTIC_CMD_LOCATE;   // 대기 중 호출 위치 알림(2026-10-09)
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

// ── 주기 처리 ──────────────────────────────────────

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
  // 상태가 바뀔 때만 남긴다. 10 Hz 로그는 로그를 무용지물로 만든다.
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
  // 터치는 상향이 살아 있을 때만 사실이다. stale 이면 false 로 떨어뜨린다.
  s.uplink_fresh = uplink_fresh(io_.steady_now_ns());
  s.user_contact = resolve_contact(touch_contact_, s.uplink_fresh);
  // 서보·LED 는 상향으로 관측하지 않는다. connected 와 같은 값이며 [미검증]이다.
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
        // 바뀐 순간 한 번 더 내되, 빠르게 두드려도 프로브 상한(4 Hz)을 넘지 않게 최소 간격을 둔다.
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
  // Nav2 RangeSensorLayer 가 곧 할 odom -> usonic_* 조회를 먼저 해 본다.
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
    if (!us_on_[ch]) {continue;}   // 꺼진 채널 — 토픽 없음
    const int mm = frame.distances_mm[ch];
    if (mm == protocol::US_DIST_INVALID) {continue;}   // 채널 무효 — 그 채널만 건너뛴다
    // 순차 발사라 채널마다 측정 시점이 다르다. 수신 시각에서 지연만큼 과거를 stamp 로 쓴다.
    const int64_t stamp = stamp_minus_delay(stamp_base, us_delay_ns_[ch]);
    const bool allowed = us_gate_.allow(ch, range_tf_ok(ch, stamp));
    log_range_gate(ch);
    if (!allowed) {continue;}   // TF 가 없으면 보내지 않는다
    RangeOut r;
    r.channel = ch;
    r.stamp_ns = stamp;
    r.frame_id = us_frame_ids_[ch];
    r.radiation_type = 0;   // Range.ULTRASOUND
    r.field_of_view = f32_field("field_of_view", us_fov_ch_[ch]);
    r.min_range = f32_field("min_range", us_min_range_);
    const double max_range = us_max_range_ch_[ch];
    r.max_range = f32_field("max_range", max_range);
    // 에코 없음(또는 관심 범위 밖) = 부채꼴이 뚫려 있다 → max_range 로 보내야 층이 지운다.
    const double value = (mm == protocol::US_CLEAR_MM || mm / 1000.0 > max_range) ?
      max_range : mm / 1000.0;
    r.range = f32_field("range", value);
    io_.publish_range(r);
  }
}

void DriverCore::warn_if_ultrasonic_stale(int64_t now_ns)
{
  // 프레임이 한 번이라도 온 뒤 끊기면 1회 경고한다. 발행 중단 자체는 설계된 동작이다.
  if (!us_last_frame_ns_ || us_stale_warned_) {return;}
  if (now_ns - *us_last_frame_ns_ > us_stale_warn_ns_) {
    us_stale_warned_ = true;
    io_.log(LogLevel::Warn, "초음파 프레임 수신 끊김 — Range 발행 중단 (펌웨어·배선·포트 확인)");
  }
}

void DriverCore::shutdown_to_neutral()
{
  // 보내지 않으면 마지막 표시를 유지하다 1.5 초 뒤 워치독으로 빨간불이 된다.
  try {
    link_->send(protocol::STATE_NORMAL, io_.steady_now_ns());
  } catch (const std::exception &) {
  }
  link_->close();
}

}  // namespace vica_guidance
