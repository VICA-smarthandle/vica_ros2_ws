#pragma once

namespace vica_vcc_controller::core
{
struct AlignParams
{
  double w_max{0.35};      // RPP rotate_to_heading_angular_vel (run41 되돌림)
  double alpha{1.2};       // RPP max_angular_accel
  double motor_lag{0.35};  // run41 모터 지연 0.3~0.5 s 에서 고름 [추정] — 회차마다 갱신
  double settle{0.3};
  double stopped_w{0.05};  // 기본값. 실제로는 매 주기 goal checker 의 rot_stopped_velocity 로 덮는다
  int max_attempts{3};     // 사용자 결정 2026-09-24
  double lag_min{0.0};
  double lag_max{1.0};
};

enum class AlignPhase { Idle, Rotating, Settling, Done, Failed };

// 남은 각만큼 한 번에 돌고, 멈춘 뒤 확인한다(설계서 5.3).
class AlignPlanner
{
public:
  explicit AlignPlanner(AlignParams p = {}) : p_(p), lag_(p.motor_lag) {}
  void reset();
  // yaw_error = 목표 - 현재(정규화). 반환 = 회전 명령.
  double update(double yaw_error, double measured_w, double tol, double now, double dt);
  AlignPhase phase() const {return phase_;}
  int attempts() const {return attempts_;}
  double lagEstimate() const {return lag_;}
  // goal checker 의 rot_stopped_velocity 와 같은 값을 쓴다(두 곳에 따로 적지 않는다).
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
}  // namespace vica_vcc_controller::core
