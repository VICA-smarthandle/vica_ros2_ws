#pragma once

#include <cstdint>
#include <string>

#include "vica_user_guidance_cpp/pyfmt.hpp"

namespace vica_guidance
{

enum class LogLevel {Info, Warn, Error};

int64_t timer_period_ns(double rate_hz);

float f32_field(const char * name, double v);

uint32_t u32_field(const char * name, int64_t v);

int64_t stamp_minus_delay(int64_t base_ns, __int128 delay_ns);

int64_t time_msg_ns(int64_t ns);

int64_t time_plus_seconds(int64_t now_ns, double seconds);

std::string py_fixed(double x, int precision);

}
