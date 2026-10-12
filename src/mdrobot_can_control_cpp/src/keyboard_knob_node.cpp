// MDROBOT CAN 모터 노드의 C++ 판. safety_supervisor_node 가 승인한 /cmd_vel_safe 를 받아
// CAN 으로 모터 드라이버를 구동하고, 최고속도는 드라이버가 올려 주는 knob1 로 제한한다.
//
// 파이썬 판(mdrobot_can_control/mdrobot_can_keyboard_knob_node.py)과 같은 노드 이름·
// 파라미터·토픽·QoS·진단·로그 글자를 쓴다. 판정과 계산은 motor_core.hpp 에 있다.
//
// 모터 매핑: MOT1 = 오른쪽 바퀴, MOT2 = 왼쪽 바퀴.
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
    // ========================= 사용자 파라미터 (파이썬 판과 같은 이름·기본값·순서)
    Params p;
    p.can_iface = declare_parameter<std::string>("can_iface", "can1");
    p.driver_id = declare_parameter<int64_t>("driver_id", 0x001);
    p.wheel_radius_m = declare_parameter<double>("wheel_radius_m", 0.065);  // MDH100 Ø130mm
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
    p.invert_mot1 = declare_parameter<bool>("invert_mot1", false);  // 오른쪽 바퀴
    p.invert_mot2 = declare_parameter<bool>("invert_mot2", false);  // 왼쪽 바퀴
    p.min_rpm_when_moving = declare_parameter<int64_t>("min_rpm_when_moving", 0);
    p.can_reconnect_interval_sec = declare_parameter<double>("can_reconnect_interval_sec", 1.0);

    // 최종 구동단 watchdog 은 단일 STEADY_TIME clock 과 정수 나노초를 쓴다.
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

    // ========================= CAN 초기화 — 기동 때 실패는 파이썬처럼 노드를 죽인다
    bus_ = std::make_unique<SocketCan>(p.can_iface);

    auto & io = core_->io;
    io.has_bus = [this]() {return static_cast<bool>(bus_);};
    io.recv_nowait = [this]() {return bus_->recv_nowait();};
    io.send = [this](const Frame & f) {bus_->send(f);};
    // old_bus, self.bus = self.bus, None → old_bus.shutdown() → self.bus = Bus(...)
    // 재개방이 실패하면 bus_ 는 비어 있는 채로 남는다 — 닫힌 핸들을 남기지 않는다.
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

    // 드라이버 PNT I/O monitor broadcasting ON
    core_->send_pnt_io_monitor_on();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    core_->send_pnt_io_monitor_on();

    // ========================= ROS 인터페이스
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

    // rclpy: timer_period_ns = int(float(1.0 / send_hz) * 1e9)
    if (p.send_hz == 0.0) {
      throw std::runtime_error("float division by zero");
    }
    const double period_ns = 1.0 / p.send_hz * 1e9;
    if (!std::isfinite(period_ns)) {
      throw std::runtime_error("cannot convert float NaN to integer");
    }
    if (!(std::fabs(period_ns) < 9.2e18)) {  // 파이썬도 int64 로 못 넘겨 기동 때 죽는다
      throw std::overflow_error("timer period does not fit in int64 nanoseconds");
    }
    timer_ = rclcpp::create_timer(
      this, steady_clock_, rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(period_ns)),
      [this]() {core_->control_loop();});

    RCLCPP_INFO(get_logger(), "Subscribed: /cmd_vel_safe");
    RCLCPP_INFO(get_logger(), "knob1 = 최고속도 제한기");
    RCLCPP_INFO(get_logger(), "Ready.");
  }

  // 보고 전용이다. 정지는 control_loop 이 즉시 하며 이 진단이 늦거나 실패해도 정지와 무관하다.
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
    // last_error 는 현재 오류가 아니라 "마지막으로 관측된" 오류다. 복구 후에도 남는다.
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

  // 파이썬 destroy_node(): 정지 명령 두 번(20 ms 간격) 뒤 버스를 닫는다.
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

}  // namespace mdrobot

namespace
{

// SIGINT·SIGTERM 은 이 스레드가 sigwait 로 받는다. rclcpp 기본 처리기를 쓰면 신호가 오는 순간
// 로깅까지 꺼져 "Stopping motors..." 와 정지 중 송신 실패가 ~/.ros/log 파일에 남지 않는다
// (파이썬 판은 남는다). 여기서는 멈춤 표시만 하고, 정리는 로깅이 살아 있는 main 이 한다.
// 두 번째 신호(터미널 Ctrl+C + launch 전달)도 이 스레드가 받으므로 정지 명령 두 번이 다 나간다.
struct StopRequest
{
  std::mutex mutex;
  std::promise<void> promise;
  bool fired = false;
  rclcpp::Executor * executor = nullptr;
};

// 정적 소멸 순서와 엇갈리지 않게 일부러 해제하지 않는다(종료 직전에 신호가 와도 안전).
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

}  // namespace

int main(int argc, char ** argv)
{
  // 파이썬 인터프리터는 시작할 때 SIGPIPE 를 무시한다. 그래서 파이썬 판은 화면 출력 통로가
  // 끊겨도(launch 가 먼저 죽는 등) 계속 돌며 정지 판정을 이어 간다. C++ 기본값이면 다음 로그
  // 한 줄에 정지 명령도 못 보내고 즉사한다 — 2026-10-12 격리 시험에서 실제로 그랬다.
  std::signal(SIGPIPE, SIG_IGN);

  // 다른 스레드(DDS 포함)가 생기기 전에 막아야 신호가 전부 wait_for_signals 로 간다.
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
    // 파이썬: 생성자 예외 → traceback, 종료 코드 1. 정지 명령은 보내지 않는다(노드가 없다).
    std::fprintf(stderr, "%s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // Ctrl+C(SIGINT·SIGTERM)는 spin 을 정상으로 끝낸다. 콜백 예외(NaN 명령 등)는 파이썬처럼
  // 정지 명령을 보낸 뒤 프로세스를 죽인다.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  {
    std::lock_guard<std::mutex> lock(g_stop.mutex);
    g_stop.executor = &executor;
  }
  std::exception_ptr error;
  try {
    // 신호가 cancel() 보다 먼저 와도 future 가 이미 차 있어 곧바로 빠져나온다.
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
