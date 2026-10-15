// rclpy·rosidl 파이썬이 던졌을 예외를 같은 순서·같은 글자로 — 두 노드(손잡이 드라이버·방향지시)가 함께 쓴다.
//
// 파이썬 판은 설정이 이상하면 타이머 만들기·시각 계산·메시지 칸 넣기에서 예외로 멈춘다. C++ 판도 같은
// 자리에서 같은 PyError 로 멈춰야 "설정 실수가 같은 증상" 으로 보인다(대조 시험이 글자까지 비교한다).
#pragma once

#include <cstdint>
#include <string>

#include "vica_user_guidance_cpp/pyfmt.hpp"

namespace vica_guidance
{

enum class LogLevel {Info, Warn, Error};

// rclpy Node.create_timer(1.0 / rate) 가 받아들이는 주기(ns). 파이썬과 같은 오류로 멈춘다.
int64_t timer_period_ns(double rate_hz);

// rosidl 파이썬 float32 칸의 범위 검사. inf·NaN 은 통과, 3.402823466e+38 을 넘는 유한값은 AssertionError.
float f32_field(const char * name, double v);

// rosidl 파이썬 uint32 칸의 범위 검사.
uint32_t u32_field(const char * name, int64_t v);

// (stamp_base - Duration(nanoseconds=delay)).to_msg() 의 검사들.
int64_t stamp_minus_delay(int64_t base_ns, __int128 delay_ns);

// Time.to_msg() — sec 가 int32 를 넘으면 AssertionError.
int64_t time_msg_ns(int64_t ns);

// (now + Duration(seconds=sec)).to_msg() — Duration·덧셈·to_msg 의 검사를 그 순서대로.
int64_t time_plus_seconds(int64_t now_ns, double seconds);

// f"{x:.{precision}f}" — NaN 은 부호와 상관없이 "nan"(glibc printf 는 "-nan" 을 찍는다).
std::string py_fixed(double x, int precision);

}  // namespace vica_guidance
