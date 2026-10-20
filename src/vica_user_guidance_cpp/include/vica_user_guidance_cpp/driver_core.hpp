#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "vica_user_guidance_cpp/frames.hpp"
#include "vica_user_guidance_cpp/guidance.hpp"
#include "vica_user_guidance_cpp/rclpy_compat.hpp"
#include "vica_user_guidance_cpp/serial_link.hpp"

namespace vica_guidance
{

struct DriverParams
{
  std::string serial_port = "/dev/vica_smart_handle";
  int64_t baudrate = protocol::FIRMWARE_BAUDRATE;
  double send_rate_hz = 10.0;
  double diag_rate_hz = 2.0;
  double write_timeout_sec = 0.05;
  double cue_timeout_sec = 1.0;
  double estop_timeout_sec = 1.0;
  bool estop_required = true;
  double reconnect_interval_sec = 2.0;
  double arrival_hold_sec = 4.0;
  bool enable_serial = true;

  bool ultrasonic_enabled = true;
  double ultrasonic_rate_hz = 20.0;
  double uplink_rate_hz = 20.0;
  bool touch_enabled = true;
  double touch_stale_sec = 0.5;
  double touch_publish_min_interval_sec = 0.1;
  std::vector<std::string> ultrasonic_topics{"/ultrasonic/front_left", "/ultrasonic/front_right"};
  std::vector<std::string> ultrasonic_frame_ids{"usonic_front_left", "usonic_front_right"};
  double ultrasonic_fov_rad = 0.524;
  std::vector<double> ultrasonic_fov_rad_per_channel =
    std::vector<double>(protocol::US_CHANNELS, -1.0);
  double ultrasonic_min_range_m = 0.02;
  double ultrasonic_max_range_m = 0.30;
  std::vector<double> ultrasonic_max_range_m_per_channel =
    std::vector<double>(protocol::US_CHANNELS, -1.0);
  std::vector<bool> ultrasonic_channel_enabled = std::vector<bool>(protocol::US_CHANNELS, true);
  std::vector<int64_t> ultrasonic_measurement_delay_ms{210, 105};
  double ultrasonic_stale_warn_sec = 2.0;
  bool ultrasonic_tf_gate = true;
  std::string ultrasonic_tf_target_frame = "odom";
};

struct RangeOut
{
  int channel = 0;
  int64_t stamp_ns = 0;
  std::string frame_id;
  uint8_t radiation_type = 0;
  float field_of_view = 0.f;
  float min_range = 0.f;
  float max_range = 0.f;
  float range = 0.f;
};

struct StateOut
{
  int64_t stamp_ns = 0;
  std::string frame_id;
  bool connected = false;
  bool user_contact = false;
  bool uplink_fresh = false;
  bool servo_ok = false;
  bool left_led_ok = false;
  bool right_led_ok = false;
  bool haptic_ok = false;
  int fault_code = 0;
  int last_state_code = 0;
  int64_t write_error_count = 0;
};

class DriverIo
{
public:
  virtual ~DriverIo() = default;
  virtual int64_t steady_now_ns() = 0;
  virtual int64_t ros_now_ns() = 0;
  virtual void log(LogLevel level, const std::string & text) = 0;
  virtual void log_warn_throttled(double period_sec, const std::string & text) = 0;
  virtual void publish_range(const RangeOut & range) = 0;
  virtual void publish_state(const StateOut & state) = 0;
  virtual bool can_transform(
    const std::string & target, const std::string & source, int64_t stamp_ns) = 0;
};

class DriverCore
{
public:
  DriverCore(const DriverParams & params, DriverIo & io, SerialFactory factory);

  void on_turn(int direction);
  void on_estop(bool data);
  void on_goal(const std::string & payload);
  void on_haptic(const std::string & data);

  void send_loop();
  void diag_loop();
  void uplink_loop();

  void shutdown_to_neutral();

  int64_t send_period_ns() const {return send_period_ns_;}
  int64_t diag_period_ns() const {return diag_period_ns_;}
  bool uplink_timer_needed() const {return us_enabled_ || touch_enabled_;}
  int64_t uplink_period_ns() const {return uplink_period_ns_;}
  bool ultrasonic_enabled() const {return us_enabled_;}
  bool tf_gate_enabled() const {return us_tf_gate_enabled_;}
  const std::vector<std::string> & ultrasonic_topics() const {return us_topics_;}
  const std::vector<bool> & ultrasonic_channel_on() const {return us_on_;}

private:
  void warn_if_arrival_hold_too_short();
  void setup_ultrasonic();
  void log_on_change(const GuidanceOutcome & outcome);
  bool uplink_fresh(int64_t now_ns) const;
  bool range_tf_ok(int ch, int64_t stamp_ns);
  void log_range_gate(int ch);
  void publish_ranges(const UltrasonicFrame & frame);
  void warn_if_ultrasonic_stale(int64_t now_ns);

  DriverParams p_;
  DriverIo & io_;

  int64_t cue_timeout_ns_ = 0;
  int64_t estop_timeout_ns_ = 0;
  int64_t arrival_hold_ns_ = 0;
  bool estop_required_ = true;

  std::optional<SerialLink> link_;

  bool estop_active_ = false;
  std::optional<int64_t> estop_last_ns_;
  int turn_direction_ = DIRECTION_NONE;
  std::optional<int64_t> turn_last_ns_;
  std::optional<int64_t> arrival_started_ns_;
  std::optional<std::string> last_reason_;

  int64_t send_period_ns_ = 0;
  int64_t diag_period_ns_ = 0;
  int64_t uplink_period_ns_ = 0;

  bool touch_enabled_ = true;
  bool touch_contact_ = false;
  std::optional<int64_t> touch_last_frame_ns_;
  int64_t touch_stale_ns_ = 0;
  int64_t touch_pub_min_ns_ = 0;
  std::optional<int64_t> touch_pub_last_ns_;
  TouchFrameAccumulator touch_acc_;

  bool us_enabled_ = true;
  std::vector<std::string> us_topics_;
  std::vector<std::string> us_frame_ids_;
  std::vector<double> us_fov_ch_;
  double us_min_range_ = 0.0;
  std::vector<double> us_max_range_ch_;
  std::vector<__int128> us_delay_ns_;
  int64_t us_stale_warn_ns_ = 0;
  FrameAccumulator us_acc_;
  StatFrameAccumulator us_stat_acc_;
  ConfigFrameAccumulator us_cfg_acc_;
  std::vector<std::string> us_stat_names_;
  std::vector<bool> us_on_;
  std::optional<int64_t> us_last_frame_ns_;
  bool us_stale_warned_ = false;
  bool us_tf_gate_enabled_ = true;
  std::string us_tf_target_;
  RangeTfGate us_gate_{protocol::US_CHANNELS};
};

}
