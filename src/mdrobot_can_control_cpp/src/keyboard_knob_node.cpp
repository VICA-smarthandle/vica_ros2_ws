#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <exception>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <pthread.h>
#include <signal.h>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "mdrobot_can_control_cpp/motor_core.hpp"
#include "mdrobot_can_control_cpp/socketcan.hpp"

namespace mdrobot
{

class KeyboardKnobNode : public rclcpp::Node
{
public:
  KeyboardKnobNode()
  : Node("mdrobot_can_keyboard_knob_node"),
    steady_clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME))
  {
    Params p;
    p.can_iface = declare_parameter<std::string>("can_iface", "can1");
    p.driver_id = declare_parameter<int64_t>("driver_id", 0x001);
    p.wheel_radius_m = declare_parameter<double>("wheel_radius_m", 0.065);
    p.wheel_base_m = declare_parameter<double>("wheel_base_m", 0.37);
    p.max_linear_mps = declare_parameter<double>("max_linear_mps", 1.0);
    p.max_angular_radps = declare_parameter<double>("max_angular_radps", 2.0);
    p.max_rpm = declare_parameter<int64_t>("max_rpm", 400);
    p.send_hz = declare_parameter<double>("send_hz", 30.0);
    p.resend_interval_sec = declare_parameter<double>("resend_interval_sec", 0.05);
    p.deadzone_pct = declare_parameter<int64_t>("deadzone_pct", 5);
    p.knob_min_pct = declare_parameter<int64_t>("knob_min_pct", 0);
    p.knob_max_pct = declare_parameter<int64_t>("knob_max_pct", 100);
    p.knob_timeout_sec = declare_parameter<double>("knob_timeout_sec", 0.8);
    p.cmd_timeout_sec = declare_parameter<double>("cmd_timeout_sec", 0.5);
    p.invert_mot1 = declare_parameter<bool>("invert_mot1", false);
    p.invert_mot2 = declare_parameter<bool>("invert_mot2", false);
    p.min_rpm_when_moving = declare_parameter<int64_t>("min_rpm_when_moving", 0);
    p.can_reconnect_interval_sec = declare_parameter<double>("can_reconnect_interval_sec", 1.0);

    core_ = std::make_unique<Core>(p);
    core_->now_fn = [this]() {return steady_clock_->now().nanoseconds();};

    int64_t can_flags = 0;
    try {
      can_flags = require_can_interface_up(p.can_iface);
    } catch (const std::runtime_error & e) {
      RCLCPP_FATAL(get_logger(), "%s", e.what());
      throw;
    }
    RCLCPP_INFO(
      get_logger(), "CAN preflight passed: %s IFF_UP flags=0x%s", p.can_iface.c_str(),
      format_hex_upper(can_flags).c_str());

    bus_ = std::make_unique<SocketCan>(p.can_iface);

    auto & io = core_->io;
    io.has_bus = [this]() {return static_cast<bool>(bus_);};
    io.recv_nowait = [this]() {return bus_->recv_nowait();};
    io.send = [this](const Frame & f) {bus_->send(f);};
    io.reopen = [this]() {
      std::unique_ptr<SocketCan> old_bus = std::move(bus_);
      if (old_bus) {
        old_bus->shutdown();
      }
      bus_ = std::make_unique<SocketCan>(core_->p.can_iface);
    };
    io.log_info = [this](const std::string & s) {RCLCPP_INFO(get_logger(), "%s", s.c_str());};
    io.log_error = [this](const std::string & s) {RCLCPP_ERROR(get_logger(), "%s", s.c_str());};

    RCLCPP_INFO(get_logger(), "CAN opened: %s", p.can_iface.c_str());
    RCLCPP_INFO(get_logger(), "Driver ID: 0x%s", format_hex_upper(p.driver_id, 3).c_str());

    core_->send_pnt_io_monitor_on();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    core_->send_pnt_io_monitor_on();

    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_safe", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        core_->on_cmd(msg->linear.x, msg->angular.z);
      });

    pub_can_ok_ = create_publisher<std_msgs::msg::Bool>("/motor/can_ok", 10);
    io.publish_can_ok = [this](bool ok) {
        std_msgs::msg::Bool msg;
        msg.data = ok;
        pub_can_ok_->publish(msg);
      };

    diag_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
    diag_updater_->setHardwareID(p.can_iface);
    diag_updater_->add("CAN link", this, &KeyboardKnobNode::diagnose_can_link);

    if (p.send_hz == 0.0) {
      throw std::runtime_error("float division by zero");
    }
    const double period_ns = 1.0 / p.send_hz * 1e9;
    if (!std::isfinite(period_ns)) {
      throw std::runtime_error("cannot convert float NaN to integer");
    }
    if (!(std::fabs(period_ns) < 9.2e18)) {
      throw std::overflow_error("timer period does not fit in int64 nanoseconds");
    }
    timer_ = rclcpp::create_timer(
      this, steady_clock_, rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(period_ns)),
      [this]() {core_->control_loop();});

    RCLCPP_INFO(get_logger(), "Subscribed: /cmd_vel_safe");
    RCLCPP_INFO(get_logger(), "knob1 = 최고속도 제한기");
    RCLCPP_INFO(get_logger(), "Ready.");
  }

  void diagnose_can_link(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    const int64_t now = core_->now_fn();
    if (core_->can_link.is_ok()) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "CAN link OK");
    } else {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "CAN link FAILED; motor output forced to 0");
    }
    stat.add("iface", core_->p.can_iface);
    stat.add(
      "last_error",
      "last observed (may predate recovery): " + core_->can_link.last_error.value_or("None"));
    stat.add("knob_age_sec", Core::age_text(core_->last_knob_ns, now));
    stat.add("cmd_age_sec", Core::age_text(core_->last_cmd_ns, now));
  }

  void stop_motors()
  {
    try {
      core_->send_vel_cmd(0, 0, RET_TYPE_NONE);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      core_->send_vel_cmd(0, 0, RET_TYPE_NONE);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "stop_motors failed: %s", e.what());
    }
  }

  void shutdown_hook()
  {
    RCLCPP_INFO(get_logger(), "Stopping motors...");
    stop_motors();
    if (bus_) {
      bus_->shutdown();
    }
  }

private:
  rclcpp::Clock::SharedPtr steady_clock_;
  std::unique_ptr<Core> core_;
  std::unique_ptr<SocketCan> bus_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_can_ok_;
  std::unique_ptr<diagnostic_updater::Updater> diag_updater_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}

namespace
{

struct StopRequest
{
  std::mutex mutex;
  std::promise<void> promise;
  bool fired = false;
  rclcpp::Executor * executor = nullptr;
};

StopRequest & g_stop = *new StopRequest();

void wait_for_signals(sigset_t signals)
{
  while (true) {
    int sig = 0;
    if (sigwait(&signals, &sig) != 0) {
      continue;
    }
    std::lock_guard<std::mutex> lock(g_stop.mutex);
    if (!g_stop.fired) {
      g_stop.fired = true;
      g_stop.promise.set_value();
    }
    if (g_stop.executor) {
      g_stop.executor->cancel();
    }
  }
}

}

int main(int argc, char ** argv)
{
  std::signal(SIGPIPE, SIG_IGN);

  sigset_t signals;
  sigemptyset(&signals);
  sigaddset(&signals, SIGINT);
  sigaddset(&signals, SIGTERM);
  pthread_sigmask(SIG_BLOCK, &signals, nullptr);
  std::thread(wait_for_signals, signals).detach();
  std::future<void> stop_requested = g_stop.promise.get_future();

  std::shared_ptr<mdrobot::KeyboardKnobNode> node;
  try {
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    node = std::make_shared<mdrobot::KeyboardKnobNode>();
  } catch (const std::exception & e) {
    std::fprintf(stderr, "%s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  {
    std::lock_guard<std::mutex> lock(g_stop.mutex);
    g_stop.executor = &executor;
  }
  std::exception_ptr error;
  try {
    executor.spin_until_future_complete(stop_requested);
  } catch (...) {
    error = std::current_exception();
  }
  {
    std::lock_guard<std::mutex> lock(g_stop.mutex);
    g_stop.executor = nullptr;
  }
  node->shutdown_hook();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();

  if (error) {
    try {
      std::rethrow_exception(error);
    } catch (const std::exception & e) {
      std::fprintf(stderr, "%s\n", e.what());
    } catch (...) {
      std::fprintf(stderr, "unknown exception\n");
    }
    return 1;
  }
  return 0;
}
