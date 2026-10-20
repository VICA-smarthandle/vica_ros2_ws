#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

#include "mdrobot_can_control_cpp/motor_core.hpp"

using mdrobot::CanLink;
using mdrobot::is_fresh_ns;
using mdrobot::motor_speed_ratio;
using mdrobot::normalize_knob_pct;
using mdrobot::sec_to_ns;

namespace
{
constexpr int64_t T0 = 1000000000;
const int64_t RETRY_NS = sec_to_ns(1.0);
const int64_t CMD_TIMEOUT_NS = sec_to_ns(0.5);
const int64_t KNOB_TIMEOUT_NS = sec_to_ns(0.8);
}

TEST(CanLink, new_link_is_ok)
{
  CanLink link(RETRY_NS);
  EXPECT_TRUE(link.is_ok());
  EXPECT_FALSE(link.last_error.has_value());
}

TEST(CanLink, error_marks_link_failed)
{
  CanLink link(RETRY_NS);
  link.record_error("Network is down", T0);
  EXPECT_FALSE(link.is_ok());
  EXPECT_NE(link.last_error->find("Network is down"), std::string::npos);
}

TEST(CanLink, success_restores_link)
{
  CanLink link(RETRY_NS);
  link.record_error("boom", T0);
  link.record_success();
  EXPECT_TRUE(link.is_ok());
}

TEST(CanLink, healthy_link_never_retries)
{
  CanLink link(RETRY_NS);
  EXPECT_FALSE(link.should_retry(T0));
}

TEST(CanLink, retry_waits_for_interval)
{
  CanLink link(RETRY_NS);
  link.record_error("boom", T0);
  EXPECT_FALSE(link.should_retry(T0 + RETRY_NS - 1));
  EXPECT_TRUE(link.should_retry(T0 + RETRY_NS));
}

TEST(CanLink, retry_attempt_restarts_interval)
{
  CanLink link(RETRY_NS);
  link.record_error("boom", T0);
  link.mark_retry_attempted(T0 + RETRY_NS);
  EXPECT_FALSE(link.should_retry(T0 + RETRY_NS + 1));
  EXPECT_TRUE(link.should_retry(T0 + RETRY_NS * 2));
}

TEST(CanLink, time_reversal_allows_retry)
{
  CanLink link(RETRY_NS);
  link.record_error("boom", T0);
  EXPECT_TRUE(link.should_retry(T0 - sec_to_ns(3600)));
}

TEST(MotorWatchdog, never_received_cmd_or_knob_is_stale)
{
  EXPECT_FALSE(is_fresh_ns(std::nullopt, T0, CMD_TIMEOUT_NS));
}

TEST(MotorWatchdog, cmd_just_before_timeout_is_fresh)
{
  EXPECT_TRUE(is_fresh_ns(T0, T0 + CMD_TIMEOUT_NS, CMD_TIMEOUT_NS));
}

TEST(MotorWatchdog, cmd_just_after_timeout_is_stale)
{
  EXPECT_FALSE(is_fresh_ns(T0, T0 + CMD_TIMEOUT_NS + 1, CMD_TIMEOUT_NS));
}

TEST(MotorWatchdog, time_reversal_is_stale)
{
  EXPECT_FALSE(is_fresh_ns(T0, T0 - 1, CMD_TIMEOUT_NS));
}

TEST(MotorWatchdog, both_fresh_forwards_command_scale)
{
  EXPECT_EQ(motor_speed_ratio(T0, T0, 100, T0, CMD_TIMEOUT_NS, KNOB_TIMEOUT_NS, 5), 1.0);
}

TEST(MotorWatchdog, cmd_stale_stops_even_if_knob_fresh)
{
  EXPECT_EQ(
    motor_speed_ratio(std::nullopt, T0, 100, T0, CMD_TIMEOUT_NS, KNOB_TIMEOUT_NS, 5), 0.0);
}

TEST(MotorWatchdog, knob_stale_stops_even_if_cmd_fresh)
{
  EXPECT_EQ(
    motor_speed_ratio(T0, std::nullopt, 100, T0, CMD_TIMEOUT_NS, KNOB_TIMEOUT_NS, 5), 0.0);
}

TEST(MotorWatchdog, knob_time_reversal_stops)
{
  EXPECT_EQ(
    motor_speed_ratio(
      T0, T0 + sec_to_ns(1.0), 100, T0, CMD_TIMEOUT_NS, KNOB_TIMEOUT_NS, 5), 0.0);
}

TEST(MotorWatchdog, knob_below_deadzone_stops)
{
  EXPECT_EQ(motor_speed_ratio(T0, T0, 3, T0, CMD_TIMEOUT_NS, KNOB_TIMEOUT_NS, 5), 0.0);
}

TEST(KnobRange, default_range_changes_nothing)
{
  for (const int64_t raw : {0, 5, 55, 76, 98, 100}) {
    EXPECT_EQ(normalize_knob_pct(raw, 0, 100), raw);
  }
}

TEST(KnobRange, minimum_becomes_zero) {EXPECT_EQ(normalize_knob_pct(55, 55, 98), 0);}

TEST(KnobRange, maximum_becomes_hundred) {EXPECT_EQ(normalize_knob_pct(98, 55, 98), 100);}

TEST(KnobRange, middle_becomes_about_half)
{
  EXPECT_EQ(normalize_knob_pct(76, 55, 98), 49);
  EXPECT_EQ(normalize_knob_pct(77, 55, 98), 51);
}

TEST(KnobRange, below_minimum_is_clamped_to_zero)
{
  EXPECT_EQ(normalize_knob_pct(50, 55, 98), 0);
  EXPECT_EQ(normalize_knob_pct(0, 55, 98), 0);
}

TEST(KnobRange, above_maximum_is_clamped_to_hundred)
{
  EXPECT_EQ(normalize_knob_pct(100, 55, 98), 100);
}

TEST(KnobRange, inverted_range_falls_back_to_raw)
{
  EXPECT_EQ(normalize_knob_pct(70, 98, 55), 70);
  EXPECT_EQ(normalize_knob_pct(70, 60, 60), 70);
}

TEST(KnobRange, is_monotonic)
{
  std::vector<int64_t> vals;
  for (int64_t raw = 55; raw < 99; ++raw) {
    vals.push_back(normalize_knob_pct(raw, 55, 98));
  }
  EXPECT_TRUE(std::is_sorted(vals.begin(), vals.end()));
  EXPECT_EQ(vals.front(), 0);
  EXPECT_EQ(vals.back(), 100);
}

TEST(KnobRange, real_rope_positions)
{
  const std::vector<std::pair<int64_t, int64_t>> table = {{55, 0}, {66, 26}, {87, 74}, {98, 100}};
  for (const auto & [raw, expected] : table) {
    EXPECT_EQ(normalize_knob_pct(raw, 55, 98), expected) << "raw=" << raw;
  }
}

TEST(PythonParity, round_is_half_to_even)
{
  EXPECT_EQ(mdrobot::py_round(0.5), 0);
  EXPECT_EQ(mdrobot::py_round(1.5), 2);
  EXPECT_EQ(mdrobot::py_round(2.5), 2);
  EXPECT_EQ(mdrobot::py_round(-0.5), 0);
  EXPECT_EQ(mdrobot::py_round(-1.5), -2);
  EXPECT_EQ(mdrobot::py_round(12.4999), 12);
}

TEST(PythonParity, round_of_nan_or_inf_throws_like_python)
{
  EXPECT_THROW(mdrobot::py_round(std::nan("")), std::domain_error);
  EXPECT_THROW(mdrobot::py_round(std::numeric_limits<double>::infinity()), std::overflow_error);
}

TEST(PythonParity, clamp_lets_nan_through_like_python)
{
  EXPECT_TRUE(std::isnan(mdrobot::clampd(std::nan(""), -1.0, 1.0)));
  EXPECT_FALSE(std::signbit(mdrobot::clampd(0.5, -0.0, 0.0)));
  EXPECT_TRUE(std::signbit(mdrobot::clampd(-0.5, -0.0, 0.0)));
}

TEST(PythonParity, sec_to_ns_truncates_and_rejects_nan)
{
  EXPECT_EQ(sec_to_ns(0.05), 50000000);
  EXPECT_EQ(sec_to_ns(1.0 / 30.0), 33333333);
  EXPECT_THROW(sec_to_ns(std::nan("")), std::domain_error);
  EXPECT_THROW(sec_to_ns(std::numeric_limits<double>::infinity()), std::overflow_error);
}
