#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace mdrobot
{

constexpr uint8_t PID_PNT_IO_MONITOR = 0xF1;
constexpr uint8_t PID_PNT_VEL_CMD = 0xCF;
constexpr uint8_t PID_COMMAND = 0x0A;
constexpr uint8_t CMD_PNT_IO_MON_ON = 0x55;

constexpr int RET_TYPE_NONE = 0;
constexpr int RET_TYPE_ODOM = 5;

constexpr double kPi = 3.14159265358979323846;

struct CanFailure : std::runtime_error
{
  using std::runtime_error::runtime_error;
};

struct Frame
{
  uint32_t id = 0;
  uint8_t dlc = 0;
  std::array<uint8_t, 8> data{};
};

inline int64_t sec_to_ns(double seconds)
{
  const double ns = seconds * 1000000000.0;
  if (std::isnan(ns)) {
    throw std::domain_error("cannot convert float NaN to integer");
  }
  if (std::isinf(ns)) {
    throw std::overflow_error("cannot convert float infinity to integer");
  }
  if (ns >= 9.2e18) {
    return INT64_MAX;
  }
  if (ns <= -9.2e18) {
    return INT64_MIN;
  }
  return static_cast<int64_t>(ns);
}

inline bool is_fresh_ns(std::optional<int64_t> last_ns, int64_t now_ns, int64_t timeout_ns)
{
  if (!last_ns) {
    return false;
  }
  const int64_t age_ns = now_ns - *last_ns;
  return 0 <= age_ns && age_ns <= timeout_ns;
}

inline int64_t py_round(double x)
{
  if (std::isnan(x)) {
    throw std::domain_error("cannot convert float NaN to integer");
  }
  if (std::isinf(x)) {
    throw std::overflow_error("cannot convert float infinity to integer");
  }
  const double r = std::nearbyint(x);
  if (r >= 9223372036854775808.0) {
    return INT64_MAX;
  }
  if (r < -9223372036854775808.0) {
    return INT64_MIN;
  }
  return static_cast<int64_t>(r);
}

inline double clampd(double v, double lo, double hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

inline int64_t clampi(int64_t v, int64_t lo, int64_t hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

inline int64_t neg_sat(int64_t v) {return v == INT64_MIN ? INT64_MAX : -v;}

inline double sub_to_double(int64_t a, int64_t b)
{
  int64_t d = 0;
  if (!__builtin_sub_overflow(a, b, &d)) {
    return static_cast<double>(d);
  }
  return static_cast<double>(static_cast<long double>(a) - static_cast<long double>(b));
}

inline int64_t normalize_knob_pct(int64_t raw_pct, int64_t min_pct, int64_t max_pct)
{
  if (max_pct <= min_pct) {
    return raw_pct;
  }
  const int64_t scaled = py_round(
    sub_to_double(raw_pct, min_pct) * 100.0 / sub_to_double(max_pct, min_pct));
  return std::max<int64_t>(0, std::min<int64_t>(100, scaled));
}

inline double motor_speed_ratio(
  std::optional<int64_t> cmd_last_ns, std::optional<int64_t> knob_last_ns,
  int64_t knob_pct, int64_t now_ns, int64_t cmd_timeout_ns, int64_t knob_timeout_ns,
  int64_t deadzone_pct)
{
  const bool cmd_fresh = is_fresh_ns(cmd_last_ns, now_ns, cmd_timeout_ns);
  const bool knob_fresh = is_fresh_ns(knob_last_ns, now_ns, knob_timeout_ns);
  if (!cmd_fresh || !knob_fresh) {
    return 0.0;
  }
  if (knob_pct <= deadzone_pct) {
    return 0.0;
  }
  return static_cast<double>(knob_pct) / 100.0;
}

class CanLink
{
public:
  explicit CanLink(int64_t retry_interval_ns)
  : retry_interval_ns_(retry_interval_ns) {}

  void record_success()
  {
    ok_ = true;
    last_attempt_ns_.reset();
  }

  void record_error(const std::string & what, int64_t now_ns)
  {
    ok_ = false;
    last_error = what;
    last_attempt_ns_ = now_ns;
  }

  bool is_ok() const {return ok_;}

  bool should_retry(int64_t now_ns) const
  {
    if (ok_) {
      return false;
    }
    if (!last_attempt_ns_) {
      return true;
    }
    const int64_t elapsed = now_ns - *last_attempt_ns_;
    if (elapsed < 0) {
      return true;
    }
    return elapsed >= retry_interval_ns_;
  }

  void mark_retry_attempted(int64_t now_ns) {last_attempt_ns_ = now_ns;}

  std::optional<std::string> last_error;

private:
  int64_t retry_interval_ns_;
  bool ok_ = true;
  std::optional<int64_t> last_attempt_ns_;
};

struct Params
{
  std::string can_iface = "can1";
  int64_t driver_id = 0x001;
  double wheel_radius_m = 0.065;
  double wheel_base_m = 0.37;
  double max_linear_mps = 1.0;
  double max_angular_radps = 2.0;
  int64_t max_rpm = 400;
  double send_hz = 30.0;
  double resend_interval_sec = 0.05;
  int64_t deadzone_pct = 5;
  int64_t knob_min_pct = 0;
  int64_t knob_max_pct = 100;
  double knob_timeout_sec = 0.8;
  double cmd_timeout_sec = 0.5;
  bool invert_mot1 = false;
  bool invert_mot2 = false;
  int64_t min_rpm_when_moving = 0;
  double can_reconnect_interval_sec = 1.0;
};

struct Io
{
  std::function<bool()> has_bus;
  std::function<std::optional<Frame>()> recv_nowait;
  std::function<void(const Frame &)> send;
  std::function<void()> reopen;
  std::function<void(bool)> publish_can_ok;
  std::function<void(const std::string &)> log_info;
  std::function<void(const std::string &)> log_error;
};

class Core
{
public:
  Params p;
  int64_t knob_timeout_ns;
  int64_t cmd_timeout_ns;
  int64_t resend_interval_ns;
  int64_t can_reconnect_interval_ns;

  int64_t knob1 = 0;
  int64_t knob2 = 0;
  std::optional<int64_t> last_knob_ns;

  double cmd_linear_x = 0.0;
  double cmd_angular_z = 0.0;
  std::optional<int64_t> last_cmd_ns;

  std::optional<int64_t> last_print_ns;
  std::optional<int64_t> prev_rpm_mot1;
  std::optional<int64_t> prev_rpm_mot2;
  std::optional<int64_t> last_send_ns;
  std::optional<int64_t> last_can_error_log_ns;

  CanLink can_link;
  Io io;
  std::function<int64_t()> now_fn;

  explicit Core(const Params & params)
  : p(params),
    knob_timeout_ns(sec_to_ns(params.knob_timeout_sec)),
    cmd_timeout_ns(sec_to_ns(params.cmd_timeout_sec)),
    resend_interval_ns(sec_to_ns(params.resend_interval_sec)),
    can_reconnect_interval_ns(sec_to_ns(params.can_reconnect_interval_sec)),
    can_link(sec_to_ns(params.can_reconnect_interval_sec)) {}

  void log_can_error_throttled(const std::string & phase, const std::string & exc)
  {
    const int64_t now = now_fn();
    const bool due = !last_can_error_log_ns ||
      (now - *last_can_error_log_ns) >= can_reconnect_interval_ns;
    if (due) {
      io.log_error(
        "[CAN FAULT] phase=" + phase + " iface=" + p.can_iface + " error=" + exc +
        "; 출력을 0으로 유지합니다");
      last_can_error_log_ns = now;
    }
  }

  void check_arbitration_id() const
  {
    if (p.driver_id < 0) {
      throw std::invalid_argument("argument out of range");
    }
    if (p.driver_id > 0xFFFFFFFFLL) {
      throw std::invalid_argument("'I' format requires 0 <= number <= 4294967295");
    }
  }

  Frame monitor_on_frame() const
  {
    Frame f;
    f.id = static_cast<uint32_t>(p.driver_id);
    f.dlc = 8;
    f.data = {PID_COMMAND, CMD_PNT_IO_MON_ON, 0, 0, 0, 0, 0, 0};
    return f;
  }

  void send_pnt_io_monitor_on()
  {
    const Frame f = monitor_on_frame();
    check_arbitration_id();
    io.send(f);
  }

  static std::pair<uint8_t, uint8_t> le_i16_signed(int64_t value)
  {
    value = clampi(value, -32768, 32767);
    const uint32_t u = static_cast<uint32_t>(value) & 0xFFFF;
    return {static_cast<uint8_t>(u & 0xFF), static_cast<uint8_t>((u >> 8) & 0xFF)};
  }

  void send_vel_cmd(int64_t rpm1, int64_t rpm2, int ret_type)
  {
    rpm1 = clampi(rpm1, neg_sat(p.max_rpm), p.max_rpm);
    rpm2 = clampi(rpm2, neg_sat(p.max_rpm), p.max_rpm);
    const auto [lo1, hi1] = le_i16_signed(rpm1);
    const auto [lo2, hi2] = le_i16_signed(rpm2);
    Frame f;
    f.id = static_cast<uint32_t>(p.driver_id);
    f.dlc = 8;
    f.data = {PID_PNT_VEL_CMD, 1, lo1, hi1, 1, lo2, hi2, static_cast<uint8_t>(ret_type & 0xFF)};
    if (!io.has_bus()) {
      return;
    }
    check_arbitration_id();
    try {
      io.send(f);
    } catch (const CanFailure & e) {
      can_link.record_error(e.what(), now_fn());
      log_can_error_throttled("send", e.what());
    }
  }

  void drain_can_rx(int64_t now_ns)
  {
    if (!io.has_bus()) {
      return;
    }
    try {
      for (int i = 0; i < 50; ++i) {
        const auto msg = io.recv_nowait();
        if (!msg) {
          break;
        }
        const Frame & d = *msg;
        if (d.dlc == 8 && d.data[0] == PID_PNT_IO_MONITOR) {
          if (d.data[1] == 0) {
            knob1 = clampi(d.data[6], 0, 100);
            knob2 = clampi(d.data[7], 0, 100);
            last_knob_ns = now_ns;
          }
        }
      }
    } catch (const CanFailure & e) {
      can_link.record_error(e.what(), now_ns);
      log_can_error_throttled("recv", e.what());
    }
  }

  void try_reconnect_can(int64_t now_ns)
  {
    if (!can_link.should_retry(now_ns)) {
      return;
    }
    can_link.mark_retry_attempted(now_ns);
    try {
      io.reopen();
    } catch (const CanFailure & e) {
      can_link.record_error(e.what(), now_ns);
      return;
    }
    try {
      send_pnt_io_monitor_on();
    } catch (const CanFailure & e) {
      can_link.record_error(e.what(), now_ns);
      return;
    }
    can_link.record_success();
    io.log_info(
      "[CAN RECOVERED] iface=" + p.can_iface + "; 주행 재개는 관리자 reset 이후에만 가능합니다");
  }

  double mps_to_rpm(double v_mps) const
  {
    const double circumference = 2.0 * kPi * p.wheel_radius_m;
    if (circumference == 0.0) {
      throw std::domain_error("float division by zero");
    }
    return (v_mps / circumference) * 60.0;
  }

  int64_t apply_min_rpm(int64_t rpm) const
  {
    if (p.min_rpm_when_moving <= 0) {
      return rpm;
    }
    if (rpm == 0) {
      return 0;
    }
    if (rpm > -p.min_rpm_when_moving && rpm < p.min_rpm_when_moving) {
      return rpm > 0 ? p.min_rpm_when_moving : -p.min_rpm_when_moving;
    }
    return rpm;
  }

  void on_cmd(double linear_x, double angular_z)
  {
    cmd_linear_x = linear_x;
    cmd_angular_z = angular_z;
    last_cmd_ns = now_fn();
  }

  void control_loop()
  {
    const int64_t now = now_fn();

    drain_can_rx(now);

    const bool can_ok = can_link.is_ok();
    io.publish_can_ok(can_ok);

    if (!can_ok) {
      try_reconnect_can(now);
    }

    const int64_t knob_norm = normalize_knob_pct(knob1, p.knob_min_pct, p.knob_max_pct);
    double speed_ratio = motor_speed_ratio(
      last_cmd_ns, last_knob_ns, knob_norm, now, cmd_timeout_ns, knob_timeout_ns,
      p.deadzone_pct);

    if (!can_ok) {
      speed_ratio = 0.0;
    }

    const bool cmd_alive = is_fresh_ns(last_cmd_ns, now, cmd_timeout_ns);
    double raw_linear_x = 0.0;
    double raw_angular_z = 0.0;
    if (cmd_alive) {
      raw_linear_x = cmd_linear_x;
      raw_angular_z = cmd_angular_z;
    }

    const double allowed_linear = p.max_linear_mps * speed_ratio;
    const double allowed_angular = p.max_angular_radps * speed_ratio;
    const double limited_linear_x = clampd(raw_linear_x, -allowed_linear, allowed_linear);
    const double limited_angular_z = clampd(raw_angular_z, -allowed_angular, allowed_angular);

    const double v_left = limited_linear_x - limited_angular_z * p.wheel_base_m / 2.0;
    const double v_right = limited_linear_x + limited_angular_z * p.wheel_base_m / 2.0;

    double rpm_mot1_f = mps_to_rpm(v_right);
    double rpm_mot2_f = mps_to_rpm(v_left);
    if (p.invert_mot1) {
      rpm_mot1_f *= -1;
    }
    if (p.invert_mot2) {
      rpm_mot2_f *= -1;
    }
    const double max_rpm = static_cast<double>(p.max_rpm);
    int64_t rpm_mot1 = py_round(clampd(rpm_mot1_f, -max_rpm, max_rpm));
    int64_t rpm_mot2 = py_round(clampd(rpm_mot2_f, -max_rpm, max_rpm));
    rpm_mot1 = apply_min_rpm(rpm_mot1);
    rpm_mot2 = apply_min_rpm(rpm_mot2);

    const bool resend_due = !last_send_ns || (now - *last_send_ns) >= resend_interval_ns;
    if (prev_rpm_mot1 != rpm_mot1 || prev_rpm_mot2 != rpm_mot2 || resend_due) {
      send_vel_cmd(rpm_mot1, rpm_mot2, RET_TYPE_ODOM);
      prev_rpm_mot1 = rpm_mot1;
      prev_rpm_mot2 = rpm_mot2;
      last_send_ns = now;
    }

    const bool print_due = !last_print_ns || (now - *last_print_ns) > sec_to_ns(0.2);
    if (print_due) {
      const auto format = [&](char * buf, size_t size) {
          return std::snprintf(
            buf, size,
            "knob1=%3lld%%->%3lld%% limit=(%.2fm/s,%.2frad/s) cmd=(%+.2f,%+.2f) "
            "out=(%+.2f,%+.2f) rpm MOT1/R=%+4lld, MOT2/L=%+4lld",
            static_cast<long long>(knob1), static_cast<long long>(knob_norm),
            allowed_linear, allowed_angular, raw_linear_x, raw_angular_z,
            limited_linear_x, limited_angular_z,
            static_cast<long long>(rpm_mot1), static_cast<long long>(rpm_mot2));
        };
      std::string line(static_cast<size_t>(format(nullptr, 0)), '\0');
      format(line.data(), line.size() + 1);
      io.log_info(line);
      last_print_ns = now;
    }
  }

  static std::string age_text(std::optional<int64_t> last_ns, int64_t now_ns)
  {
    if (!last_ns) {
      return "never";
    }
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.3f", static_cast<double>(now_ns - *last_ns) / 1e9);
    return buf;
  }
};

}
