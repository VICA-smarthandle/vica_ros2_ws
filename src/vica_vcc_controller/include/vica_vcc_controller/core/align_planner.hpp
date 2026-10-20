#pragma once

namespace vica_vcc_controller::core
{
struct AlignParams
{
  double w_max{0.35};
  double alpha{1.2};
  double motor_lag{0.35};
  double settle{0.3};
  double stopped_w{0.05};
  int max_attempts{3};
  double lag_min{0.0};
  double lag_max{1.0};
};

enum class AlignPhase { Idle, Rotating, Settling, Done, Failed };

class AlignPlanner
{
public:
  explicit AlignPlanner(AlignParams p = {}) : p_(p), lag_(p.motor_lag) {}
  void reset();
  double update(double yaw_error, double measured_w, double tol, double now, double dt);
  AlignPhase phase() const {return phase_;}
  int attempts() const {return attempts_;}
  double lagEstimate() const {return lag_;}
  void setStoppedVelocity(double w) {if (w > 0.0) {p_.stopped_w = w;}}

private:
  void begin(double err);

  AlignParams p_;
  AlignPhase phase_{AlignPhase::Idle};
  double lag_;
  double dir_{1.0};
  double last_w_{0.0};
  double cut_err_{0.0};
  double cut_w_{0.0};
  double settle_since_{-1.0};
  int attempts_{0};
};
}
