// Smart Handle 드라이버의 판정·순서 — 파이썬 판 user_guidance_driver_node.py 의 메서드를 그대로 옮겼다.
//
// [안전 경계] 이 노드가 하지 않는 것(파이썬 판과 같다):
//   - /cmd_vel_req, /cmd_vel_safe 를 발행하지 않는다
//   - Nav2 goal 을 보내거나 취소하지 않는다
//   - E-stop 래치를 소유하거나 reset 하지 않는다(/estop_state 를 구독만 한다)
//   - 상태코드 4(LINK_LOST)를 보내지 않는다(펌웨어 워치독 전용)
// 서보는 조향 장치가 아니며, 시리얼 단절은 표시·진단만 하고 주행 정지 조건으로 삼지 않는다.
//
// ROS 와 닿는 곳(시계·로그·발행·TF 조회)은 DriverIo 로 뺐다. 노드는 진짜 rclcpp 로, 대조 시험은
// 기록용 가짜로 채운다 — 같은 입력에서 파이썬 원본과 줄 단위로 같은 출력을 내는지 본다.
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

// 파라미터 — 이름·기본값은 파이썬 판 declare_parameter 와 같다(launch 는 YAML 이 덮어쓴다).
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
  double ultrasonic_rate_hz = 20.0;   // 남겨 둔 값 — 실제 읽기 주기는 uplink_rate_hz
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

// sensor_msgs/Range 한 건(float32 칸은 이미 float 로 바꾼 값).
struct RangeOut
{
  int channel = 0;
  int64_t stamp_ns = 0;
  std::string frame_id;
  uint8_t radiation_type = 0;   // Range.ULTRASOUND
  float field_of_view = 0.f;
  float min_range = 0.f;
  float max_range = 0.f;
  float range = 0.f;
};

// vica_interfaces/SmartHandleState 한 건.
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
  virtual int64_t steady_now_ns() = 0;   // STEADY_TIME — 모든 신선도 판정
  virtual int64_t ros_now_ns() = 0;      // 노드 시계(ROS_TIME) — 메시지 stamp
  virtual void log(LogLevel level, const std::string & text) = 0;
  // logger.warn(text, throttle_duration_sec=period) — 같은 자리의 경고를 period 초에 한 번만.
  virtual void log_warn_throttled(double period_sec, const std::string & text) = 0;
  virtual void publish_range(const RangeOut & range) = 0;
  virtual void publish_state(const StateOut & state) = 0;
  // tf_buffer.can_transform(target, source, Time.from_msg(stamp)). 예외는 false 로 돌려준다.
  virtual bool can_transform(
    const std::string & target, const std::string & source, int64_t stamp_ns) = 0;
};

class DriverCore
{
public:
  // 파이썬 __init__ 과 같은 순서로 검사·로그·포트 열기를 한다. 설정 오류는 PyError.
  DriverCore(const DriverParams & params, DriverIo & io, SerialFactory factory);

  // 구독 콜백
  void on_turn(int direction);
  void on_estop(bool data);
  void on_goal(const std::string & payload);
  void on_haptic(const std::string & data);

  // 주기 처리(타이머)
  void send_loop();
  void diag_loop();
  void uplink_loop();

  // 종료 시 기본 상태를 1회 보내고 포트를 닫는다.
  void shutdown_to_neutral();

  // 노드가 타이머·발행자를 만들 때 쓰는 값(파이썬 create_timer(1.0 / rate) 의 주기, ns).
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
  std::vector<__int128> us_delay_ns_;   // 파이썬 int(ms × 1e6) — 크기 제한 없는 값을 담는다
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

}  // namespace vica_guidance
