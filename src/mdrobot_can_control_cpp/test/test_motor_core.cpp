#include <gtest/gtest.h>

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "mdrobot_can_control_cpp/motor_core.hpp"

using mdrobot::CanFailure;
using mdrobot::Core;
using mdrobot::Frame;
using mdrobot::sec_to_ns;

namespace
{
constexpr int64_t T0 = 1000000000;
const int64_t RETRY_NS = sec_to_ns(1.0);
const int64_t CMD_TIMEOUT_NS = sec_to_ns(0.5);
const int64_t KNOB_TIMEOUT_NS = sec_to_ns(0.8);
constexpr const char * kClosed = "file descriptor cannot be a negative integer (-1)";

enum class Reopen { kFail, kDeadDriver, kHealthy };

struct Harness
{
  std::unique_ptr<Core> core;
  bool bus_present = true;
  bool bus_closed = false;
  bool dead_driver = false;
  Reopen reopen = Reopen::kHealthy;
  std::string reopen_error = "[Errno 19] No such device";
  std::deque<Frame> rx;
  std::vector<Frame> sent;
  std::vector<std::pair<std::string, std::string>> lines;

  explicit Harness(bool link_failed = true)
  {
    mdrobot::Params p;
    p.max_rpm = 1000;
    core = std::make_unique<Core>(p);
    core->now_fn = []() {return T0 + RETRY_NS;};
    if (link_failed) {
      core->can_link.record_error("gone", T0);
    }
    auto & io = core->io;
    io.has_bus = [this]() {return bus_present;};
    io.recv_nowait = [this]() -> std::optional<Frame> {
        if (bus_closed) {
          throw CanFailure(kClosed);
        }
        if (rx.empty()) {
          return std::nullopt;
        }
        Frame f = rx.front();
        rx.pop_front();
        return f;
      };
    io.send = [this](const Frame & f) {
        if (bus_closed) {
          throw CanFailure(kClosed);
        }
        if (dead_driver) {
          throw CanFailure("Network is down");
        }
        sent.push_back(f);
      };
    io.reopen = [this]() {
        bus_present = false;
        if (reopen == Reopen::kFail) {
          throw CanFailure(reopen_error);
        }
        bus_present = true;
        bus_closed = false;
        dead_driver = reopen == Reopen::kDeadDriver;
      };
    io.publish_can_ok = [](bool) {};
    io.log_info = [this](const std::string & s) {lines.emplace_back("info", s);};
    io.log_error = [this](const std::string & s) {lines.emplace_back("error", s);};
  }

  void reconnect() {core->try_reconnect_can(T0 + RETRY_NS);}
};

Frame knob_frame(uint8_t knob1, uint8_t knob2)
{
  Frame f;
  f.id = 0x701;
  f.dlc = 8;
  f.data = {0xF1, 0x00, 0x68, 0x08, 0x01, 0x03, knob1, knob2};
  return f;
}
}

TEST(CanReconnect, failed_reopen_leaves_no_bus_handle)
{
  Harness h;
  h.reopen = Reopen::kFail;
  h.reconnect();
  EXPECT_FALSE(h.bus_present);
  EXPECT_FALSE(h.core->can_link.is_ok());
}

TEST(CanReconnect, drain_survives_after_failed_reopen)
{
  Harness h;
  h.reopen = Reopen::kFail;
  h.reconnect();
  EXPECT_NO_THROW(h.core->drain_can_rx(T0 + RETRY_NS));
  EXPECT_FALSE(h.core->can_link.is_ok());
}

TEST(CanReconnect, send_survives_after_failed_reopen)
{
  Harness h;
  h.reopen = Reopen::kFail;
  h.reconnect();
  EXPECT_NO_THROW(h.core->send_vel_cmd(0, 0, mdrobot::RET_TYPE_NONE));
  EXPECT_FALSE(h.core->can_link.is_ok());
  EXPECT_TRUE(h.sent.empty());
}

TEST(CanReconnect, reopen_failing_with_valueerror_is_handled)
{
  Harness h;
  h.reopen = Reopen::kFail;
  h.reopen_error = kClosed;
  h.reconnect();
  EXPECT_FALSE(h.bus_present);
  EXPECT_FALSE(h.core->can_link.is_ok());
  EXPECT_EQ(*h.core->can_link.last_error, kClosed);
}

TEST(CanReconnect, probe_failure_keeps_link_failed)
{
  Harness h;
  h.reopen = Reopen::kDeadDriver;
  h.reconnect();
  EXPECT_FALSE(h.core->can_link.is_ok());
}

TEST(CanReconnect, successful_reopen_restores_link)
{
  Harness h;
  h.reopen = Reopen::kHealthy;
  h.reconnect();
  EXPECT_TRUE(h.core->can_link.is_ok());
  bool recovered = false;
  for (const auto & line : h.lines) {
    recovered = recovered || line.second.find("[CAN RECOVERED]") != std::string::npos;
  }
  EXPECT_TRUE(recovered);
  ASSERT_EQ(h.sent.size(), 1u);
  EXPECT_EQ(h.sent[0].data[0], mdrobot::PID_COMMAND);
  EXPECT_EQ(h.sent[0].data[1], mdrobot::CMD_PNT_IO_MON_ON);
}

TEST(CanReconnect, closed_handle_never_escapes_on_recv)
{
  Harness h;
  h.bus_closed = true;
  EXPECT_NO_THROW(h.core->drain_can_rx(T0));
  EXPECT_FALSE(h.core->can_link.is_ok());
}

TEST(CanReconnect, closed_handle_never_escapes_on_send)
{
  Harness h;
  h.bus_closed = true;
  EXPECT_NO_THROW(h.core->send_vel_cmd(0, 0, mdrobot::RET_TYPE_NONE));
  EXPECT_FALSE(h.core->can_link.is_ok());
}

TEST(CanReconnect, closed_handle_turns_healthy_link_into_failed)
{
  Harness h( false);
  ASSERT_TRUE(h.core->can_link.is_ok());
  h.bus_closed = true;
  h.core->drain_can_rx(T0);
  EXPECT_FALSE(h.core->can_link.is_ok());
  EXPECT_EQ(*h.core->can_link.last_error, kClosed);
}

TEST(KnobCycleStamp, drain_stamps_knob_with_the_injected_cycle_instant)
{
  Harness h( false);
  h.rx.push_back(knob_frame(48, 48));
  h.core->drain_can_rx(T0);
  EXPECT_EQ(h.core->knob1, 48);
  EXPECT_EQ(h.core->last_knob_ns, T0);
}

TEST(KnobCycleStamp, knob_read_in_this_cycle_is_not_judged_time_reversed)
{
  Harness h( false);
  h.rx.push_back(knob_frame(48, 48));
  const int64_t now = T0;
  h.core->drain_can_rx(now);
  const double ratio = mdrobot::motor_speed_ratio(
    now, h.core->last_knob_ns, h.core->knob1, now, CMD_TIMEOUT_NS, KNOB_TIMEOUT_NS, 5);
  EXPECT_EQ(ratio, 48 / 100.0);
}

TEST(KnobCycleStamp, drain_without_knob_frame_keeps_previous_stamp)
{
  Harness h( false);
  h.core->last_knob_ns = T0;
  h.core->drain_can_rx(T0 + sec_to_ns(1.0));
  EXPECT_EQ(h.core->last_knob_ns, T0);
}
