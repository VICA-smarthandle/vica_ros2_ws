// 도킹 핵심 로직이 같이 쓰는 명령·실패 이유.
#pragma once

#include <string>

namespace vica_docking
{

struct Cmd
{
  double v{0.0};  // m/s, 앞 +
  double w{0.0};  // rad/s, 반시계 +
};

// 실패 이유. 상태 토픽 JSON 의 "reason" 값이고, 앱이 이 값으로 알림 문구를 고른다.
enum class Fail
{
  kNone,
  kNotFound,     // 출발 전 V자를 못 찾음(5회 연속 일치를 못 얻음)
  kOutOfRange,   // 출발 위치가 도크 정면에서 너무 벗어남
  kTooClose,     // 준비 위치보다 가깝고 줄이 안 맞음(제자리 회전 여유 없음)
  kLost,         // 움직이는 중 V자를 1.5 s 넘게 못 봄
  kObstacle,     // 범퍼 앞 3 cm 안에 V자가 아닌 점
  kNoMotion,     // 명령을 2 s 냈는데 안 움직임(모터 knob 등)
  kTimeout,      // 전체 제한 시간
  kBlocked,      // 해제: 30 s 넘게 막힘
  kMisaligned,   // 도착했지만 오차가 기준 밖
  kSensor,       // 센서 값이 안 들어옴
  kConflict,     // 다른 노드가 /cmd_vel_req 를 냄
  kEstop,        // 비상정지
  kCanceled,     // 사용자가 멈춤
  kNoDock,       // 도크 위치가 저장되지 않음
};

inline const char * fail_name(Fail f)
{
  switch (f) {
    case Fail::kNone: return "";
    case Fail::kNotFound: return "not_found";
    case Fail::kOutOfRange: return "out_of_range";
    case Fail::kTooClose: return "too_close";
    case Fail::kLost: return "lost";
    case Fail::kObstacle: return "obstacle";
    case Fail::kNoMotion: return "no_motion";
    case Fail::kTimeout: return "timeout";
    case Fail::kBlocked: return "blocked";
    case Fail::kMisaligned: return "misaligned";
    case Fail::kSensor: return "sensor";
    case Fail::kConflict: return "conflict";
    case Fail::kEstop: return "estop";
    case Fail::kCanceled: return "canceled";
    case Fail::kNoDock: return "no_dock";
  }
  return "unknown";
}

constexpr double kPi = 3.14159265358979323846;
inline double deg(double rad) {return rad * 180.0 / kPi;}
inline double rad(double deg) {return deg * kPi / 180.0;}
inline double sgn(double v) {return v > 0 ? 1.0 : (v < 0 ? -1.0 : 0.0);}

}  // namespace vica_docking
