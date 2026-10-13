// EKF /odom yaw 변화량 기반 회전 판정 — 파이썬 판 vica_user_guidance/turn_detector.py 를 그대로 옮겼다.
//
// ROS 에 기대지 않는다. 모든 시각 인자는 호출자가 소유한 단일 STEADY_TIME 시계의 정수 나노초다.
// 이 계층은 어떤 구동 명령도 만들지 않는다.
#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <utility>

namespace vica_guidance
{

constexpr int PHASE_IDLE = 0;
constexpr int PHASE_PREPARE = 1;
constexpr int PHASE_NOW = 2;
constexpr int PHASE_COMPLETE = 3;
constexpr int PHASE_CANCELED = 4;
// DIRECTION_NONE/LEFT/RIGHT 는 guidance.hpp 와 같은 값(0/1/2)이다.

// 각도를 (-pi, pi] 로 — atan2(sin, cos).
double normalize_angle(double radians);
// quaternion 에서 yaw 만(two_d_mode 라 roll/pitch 는 0).
double yaw_from_quaternion(double x, double y, double z, double w);

// 한 tick 의 판정 결과(파이썬 TurnDecision).
struct TurnDecision
{
  int direction = 0;
  int phase = PHASE_IDLE;
  double turn_angle_deg = 0.0;   // 창 누적. CCW(+) = LEFT. stale 이면 NaN
  int64_t sequence_id = 0;
  bool source_stale = false;
};

class TurnDetector
{
public:
  TurnDetector(
    int64_t window_ns, double enter_threshold_rad, double exit_threshold_rad,
    int64_t min_duration_ns, int64_t odom_timeout_ns);

  void add_odom(double yaw_rad, int64_t now_ns);
  TurnDecision evaluate(int64_t now_ns);
  int64_t sequence_id() const {return sequence_id_;}

private:
  void prune(int64_t now_ns);
  double window_accumulation() const;
  int64_t held_ns(int sign) const;
  int transition(int sign, double magnitude);
  TurnDecision handle_stale();

  int64_t window_ns_;
  double enter_threshold_rad_;
  double exit_threshold_rad_;
  int64_t min_duration_ns_;
  int64_t odom_timeout_ns_;

  std::deque<std::pair<int64_t, double>> samples_;   // (시각_ns, 펼친_yaw_rad)
  std::optional<double> last_yaw_rad_;
  std::optional<int64_t> last_odom_ns_;
  double unwrapped_rad_ = 0.0;

  int active_direction_ = 0;
  int64_t sequence_id_ = 0;
};

}  // namespace vica_guidance
