#pragma once

#include <string>

namespace vica_docking
{

struct Cmd
{
  double v{0.0};
  double w{0.0};
};

enum class Fail
{
  kNone,
  kNotFound,
  kOutOfRange,
  kTooClose,
  kLost,
  kObstacle,
  kNoMotion,
  kTimeout,
  kBlocked,
  kMisaligned,
  kSensor,
  kConflict,
  kEstop,
  kCanceled,
  kNoDock,
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

}
