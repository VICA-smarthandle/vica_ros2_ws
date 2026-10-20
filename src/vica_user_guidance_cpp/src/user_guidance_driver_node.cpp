#include <csignal>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "vica_interfaces/msg/smart_handle_state.hpp"
#include "vica_interfaces/msg/turn_guide.hpp"

#include "vica_user_guidance_cpp/driver_core.hpp"
#include "vica_user_guidance_cpp/pyfmt.hpp"

namespace vica_guidance
{

namespace
{
builtin_interfaces::msg::Time to_msg(int64_t ns)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(ns / 1000000000);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000);
  return t;
}
}

class UserGuidanceDriverNode : public rclcpp::Node, public DriverIo
{
public:
  UserGuidanceDriverNode()
  : rclcpp::Node("user_guidance_driver_node"),
    steady_clock_(RCL_STEADY_TIME),
    throttle_clock_(RCL_SYSTEM_TIME)
  {
    DriverParams p;
    p.serial_port = declare_parameter<std::string>("serial_port", p.serial_port);
    p.baudrate = declare_parameter<int64_t>("baudrate", p.baudrate);
    p.send_rate_hz = declare_parameter<double>("send_rate_hz", p.send_rate_hz);
    p.diag_rate_hz = declare_parameter<double>("diag_rate_hz", p.diag_rate_hz);
    p.write_timeout_sec = declare_parameter<double>("write_timeout_sec", p.write_timeout_sec);
    p.cue_timeout_sec = declare_parameter<double>("cue_timeout_sec", p.cue_timeout_sec);
    p.estop_timeout_sec = declare_parameter<double>("estop_timeout_sec", p.estop_timeout_sec);
    p.estop_required = declare_parameter<bool>("estop_required", p.estop_required);
    p.reconnect_interval_sec =
      declare_parameter<double>("reconnect_interval_sec", p.reconnect_interval_sec);
    p.arrival_hold_sec = declare_parameter<double>("arrival_hold_sec", p.arrival_hold_sec);
    p.enable_serial = declare_parameter<bool>("enable_serial", p.enable_serial);

    p.ultrasonic_enabled = declare_parameter<bool>("ultrasonic_enabled", p.ultrasonic_enabled);
    p.ultrasonic_rate_hz = declare_parameter<double>("ultrasonic_rate_hz", p.ultrasonic_rate_hz);
    p.uplink_rate_hz = declare_parameter<double>("uplink_rate_hz", p.uplink_rate_hz);
    p.touch_enabled = declare_parameter<bool>("touch_enabled", p.touch_enabled);
    p.touch_stale_sec = declare_parameter<double>("touch_stale_sec", p.touch_stale_sec);
    p.touch_publish_min_interval_sec = declare_parameter<double>(
      "touch_publish_min_interval_sec", p.touch_publish_min_interval_sec);
    p.ultrasonic_topics =
      declare_parameter<std::vector<std::string>>("ultrasonic_topics", p.ultrasonic_topics);
    p.ultrasonic_frame_ids =
      declare_parameter<std::vector<std::string>>("ultrasonic_frame_ids", p.ultrasonic_frame_ids);
    p.ultrasonic_fov_rad = declare_parameter<double>("ultrasonic_fov_rad", p.ultrasonic_fov_rad);
    p.ultrasonic_fov_rad_per_channel = declare_parameter<std::vector<double>>(
      "ultrasonic_fov_rad_per_channel", p.ultrasonic_fov_rad_per_channel);
    p.ultrasonic_min_range_m =
      declare_parameter<double>("ultrasonic_min_range_m", p.ultrasonic_min_range_m);
    p.ultrasonic_max_range_m =
      declare_parameter<double>("ultrasonic_max_range_m", p.ultrasonic_max_range_m);
    p.ultrasonic_max_range_m_per_channel = declare_parameter<std::vector<double>>(
      "ultrasonic_max_range_m_per_channel", p.ultrasonic_max_range_m_per_channel);
    p.ultrasonic_channel_enabled = declare_parameter<std::vector<bool>>(
      "ultrasonic_channel_enabled", p.ultrasonic_channel_enabled);
    p.ultrasonic_measurement_delay_ms = declare_parameter<std::vector<int64_t>>(
      "ultrasonic_measurement_delay_ms", p.ultrasonic_measurement_delay_ms);
    p.ultrasonic_stale_warn_sec =
      declare_parameter<double>("ultrasonic_stale_warn_sec", p.ultrasonic_stale_warn_sec);
    p.ultrasonic_tf_gate = declare_parameter<bool>("ultrasonic_tf_gate", p.ultrasonic_tf_gate);
    p.ultrasonic_tf_target_frame = declare_parameter<std::string>(
      "ultrasonic_tf_target_frame", p.ultrasonic_tf_target_frame);

    core_ = std::make_unique<DriverCore>(p, *this, open_posix_serial);

    sub_turn_ = create_subscription<vica_interfaces::msg::TurnGuide>(
      "/vica/turn_guide", 10,
      [this](const vica_interfaces::msg::TurnGuide & m) {core_->on_turn(m.direction);});
    sub_estop_ = create_subscription<std_msgs::msg::Bool>(
      "/estop_state", 10, [this](const std_msgs::msg::Bool & m) {core_->on_estop(m.data);});
    sub_goal_ = create_subscription<std_msgs::msg::String>(
      "/vica_goal_event", 10, [this](const std_msgs::msg::String & m) {core_->on_goal(m.data);});
    sub_haptic_ = create_subscription<std_msgs::msg::String>(
      "/vica/haptic_request", 10,
      [this](const std_msgs::msg::String & m) {core_->on_haptic(m.data);});
    pub_state_ =
      create_publisher<vica_interfaces::msg::SmartHandleState>("/vica/smart_handle_state", 10);

    if (core_->ultrasonic_enabled()) {
      const auto & topics = core_->ultrasonic_topics();
      const auto & on = core_->ultrasonic_channel_on();
      for (std::size_t ch = 0; ch < topics.size(); ++ch) {
        us_pubs_.push_back(
          on[ch] ? create_publisher<sensor_msgs::msg::Range>(topics[ch], 10) : nullptr);
      }
      if (core_->tf_gate_enabled()) {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
      }
    }

    timer_send_ = create_wall_timer(
      std::chrono::nanoseconds(core_->send_period_ns()), [this] {core_->send_loop();});
    timer_diag_ = create_wall_timer(
      std::chrono::nanoseconds(core_->diag_period_ns()), [this] {core_->diag_loop();});
    if (core_->uplink_timer_needed()) {
      timer_uplink_ = create_wall_timer(
        std::chrono::nanoseconds(core_->uplink_period_ns()), [this] {core_->uplink_loop();});
    }
  }

  void shutdown_to_neutral() {core_->shutdown_to_neutral();}

  int64_t steady_now_ns() override {return steady_clock_.now().nanoseconds();}
  int64_t ros_now_ns() override {return get_clock()->now().nanoseconds();}

  void log(LogLevel level, const std::string & text) override
  {
    switch (level) {
      case LogLevel::Info: RCLCPP_INFO(get_logger(), "%s", text.c_str()); break;
      case LogLevel::Warn: RCLCPP_WARN(get_logger(), "%s", text.c_str()); break;
      case LogLevel::Error: RCLCPP_ERROR(get_logger(), "%s", text.c_str()); break;
    }
  }

  void log_warn_throttled(double period_sec, const std::string & text) override
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), throttle_clock_, static_cast<int64_t>(period_sec * 1000.0), "%s",
      text.c_str());
  }

  void publish_range(const RangeOut & r) override
  {
    sensor_msgs::msg::Range msg;
    msg.header.stamp = to_msg(r.stamp_ns);
    msg.header.frame_id = r.frame_id;
    msg.radiation_type = r.radiation_type;
    msg.field_of_view = r.field_of_view;
    msg.min_range = r.min_range;
    msg.max_range = r.max_range;
    msg.range = r.range;
    us_pubs_.at(static_cast<std::size_t>(r.channel))->publish(msg);
  }

  void publish_state(const StateOut & s) override
  {
    vica_interfaces::msg::SmartHandleState msg;
    msg.header.stamp = to_msg(s.stamp_ns);
    msg.header.frame_id = s.frame_id;
    msg.connected = s.connected;
    msg.user_contact = s.user_contact;
    msg.uplink_fresh = s.uplink_fresh;
    msg.servo_ok = s.servo_ok;
    msg.left_led_ok = s.left_led_ok;
    msg.right_led_ok = s.right_led_ok;
    msg.haptic_ok = s.haptic_ok;
    msg.fault_code = static_cast<uint8_t>(s.fault_code);
    msg.last_state_code = static_cast<uint8_t>(s.last_state_code);
    msg.write_error_count = static_cast<uint32_t>(s.write_error_count);
    pub_state_->publish(msg);
  }

  bool can_transform(
    const std::string & target, const std::string & source, int64_t stamp_ns) override
  {
    return tf_buffer_->canTransform(
      target, source, tf2::TimePoint(std::chrono::nanoseconds(stamp_ns)));
  }

private:
  rclcpp::Clock steady_clock_;
  rclcpp::Clock throttle_clock_;
  std::unique_ptr<DriverCore> core_;
  rclcpp::Subscription<vica_interfaces::msg::TurnGuide>::SharedPtr sub_turn_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_goal_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_haptic_;
  rclcpp::Publisher<vica_interfaces::msg::SmartHandleState>::SharedPtr pub_state_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr> us_pubs_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_send_, timer_diag_, timer_uplink_;
};

}

int main(int argc, char ** argv)
{
  std::signal(SIGPIPE, SIG_IGN);
  rclcpp::init(argc, argv);
  std::shared_ptr<vica_guidance::UserGuidanceDriverNode> node;
  try {
    node = std::make_shared<vica_guidance::UserGuidanceDriverNode>();
  } catch (const vica_guidance::PyError & e) {
    std::fprintf(stderr, "%s: %s\n", e.type().c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  } catch (const std::exception & e) {
    std::fprintf(stderr, "%s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  int rc = 0;
  try {
    executor.spin();
  } catch (const vica_guidance::PyError & e) {
    std::fprintf(stderr, "%s: %s\n", e.type().c_str(), e.what());
    rc = 1;
  } catch (const std::exception & e) {
    std::fprintf(stderr, "%s\n", e.what());
    rc = 1;
  }
  executor.remove_node(node);
  node->shutdown_to_neutral();
  node.reset();
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return rc;
}
