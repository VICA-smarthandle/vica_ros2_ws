#include "vica_user_guidance_cpp/rclpy_compat.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <vector>

namespace vica_guidance
{
namespace
{
constexpr __int128 INT64_LIMIT = static_cast<__int128>(1) << 63;
const char * const SEC_FIELD_ERROR = "The 'sec' field must be an integer in [-2147483648, 2147483647]";
}

int64_t timer_period_ns(double rate_hz)
{
  if (rate_hz == 0.0) {throw PyError("ZeroDivisionError", "float division by zero");}
  const double x = (1.0 / rate_hz) * 1000000000.0;
  if (std::isnan(x)) {throw PyError("ValueError", "cannot convert float NaN to integer");}
  if (std::isinf(x)) {throw PyError("OverflowError", "cannot convert float infinity to integer");}
  const double t = std::trunc(x);
  if (t >= 9223372036854775807.0 || t < -9223372036854775808.0) {
    throw PyError("TypeError", "__init__(): incompatible constructor arguments (timer period)");
  }
  if (t < 0) {
    throw PyError(
            "RCLError",
            "failed to create timer: timer period must be non-negative, at ./src/rcl/timer.c:139");
  }
  return static_cast<int64_t>(t);
}

float f32_field(const char * name, double v)
{
  if ((v < -3.402823466e+38 || v > 3.402823466e+38) && !std::isinf(v)) {
    throw PyError(
            "AssertionError", std::string("The '") + name +
            "' field must be a float in [-3.402823466e+38, 3.402823466e+38]");
  }
  return static_cast<float>(v);
}

uint32_t u32_field(const char * name, int64_t v)
{
  if (v < 0 || v >= 4294967296LL) {
    throw PyError(
            "AssertionError", std::string("The '") + name +
            "' field must be an unsigned integer in [0, 4294967295]");
  }
  return static_cast<uint32_t>(v);
}

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
  return time_msg_ns(static_cast<int64_t>(r));
}

int64_t time_msg_ns(int64_t ns)
{
  if (ns / 1000000000 > 2147483647) {throw PyError("AssertionError", SEC_FIELD_ERROR);}
  return ns;
}

int64_t time_plus_seconds(int64_t now_ns, double seconds)
{
  const double x = seconds * 1000000000.0;
  if (std::isnan(x)) {throw PyError("ValueError", "cannot convert float NaN to integer");}
  if (std::isinf(x)) {throw PyError("OverflowError", "cannot convert float infinity to integer");}
  const double t = std::trunc(x);
  if (t >= 9223372036854775808.0 || t < -9223372036854775808.0) {
    throw PyError("OverflowError", "Total nanoseconds value is too large to store in C duration.");
  }
  const __int128 sum = static_cast<__int128>(now_ns) + static_cast<int64_t>(t);
  if (sum < 0) {throw PyError("ValueError", "Nanoseconds value must not be negative");}
  if (sum >= INT64_LIMIT) {throw PyError("OverflowError", "Addition leads to overflow in C storage.");}
  return time_msg_ns(static_cast<int64_t>(sum));
}

std::string py_fixed(double x, int precision)
{
  if (std::isnan(x)) {return "nan";}
  std::vector<char> buf(static_cast<std::size_t>(330 + std::max(precision, 0)));
  const auto r = std::to_chars(
    buf.data(), buf.data() + buf.size(), x, std::chars_format::fixed, precision);
  return std::string(buf.data(), r.ptr);
}

}
