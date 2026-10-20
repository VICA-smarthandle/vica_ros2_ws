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

double normalize_angle(double radians);
double yaw_from_quaternion(double x, double y, double z, double w);

struct TurnDecision
{
  int direction = 0;
  int phase = PHASE_IDLE;
  double turn_angle_deg = 0.0;
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

  std::deque<std::pair<int64_t, double>> samples_;
  std::optional<double> last_yaw_rad_;
  std::optional<int64_t> last_odom_ns_;
  double unwrapped_rad_ = 0.0;

  int active_direction_ = 0;
  int64_t sequence_id_ = 0;
};

}
