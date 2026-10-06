#pragma once
#include <algorithm>
#include <cmath>
#include "vica_vcc_controller/core/types.hpp"

namespace vica_vcc_controller::core
{
struct OutputParams
{
  double max_v{0.5};          // desired_linear_vel 상한, 후진 없음
  double max_w{0.5};          // velocity_smoother max_velocity[2]
  double max_ang_accel{1.2};  // RPP max_angular_accel
  double ramp_v1{0.25};       // 요구 8: 절반 속도까지 살짝
  double ramp_a1{0.5};        //   0 -> 0.25 를 0.5 s
  double accel{0.143};        //   0.25 -> 0.5 를 1.75 s, 코너 뒤 재가속도 같다
  double max_decel{1.25};     // velocity_smoother max_decel[0]. 비상 제동(정지·Turn 진입 전 멈춤)은 완화하지 않는다
  // 2026-10-06 해결안 ①: 미리 알고 줄이는 상한(코너·차선·옆 여유·도착·속도 제한)만 이 기울기로 내린다.
  // run66 감속 25번 중 23번이 이 상한들이었는데 모두 1.25 로 내려 "감속이 확" 느껴졌다.
  // max_decel 보다 크면 max_decel 로 자른다. ROS 파라미터 planned_linear_decel(없으면 max_decel 과 같음).
  double planned_decel{1.25};
  // 실측 + 이만큼 위로만 명령을 앞세운다. 모터 지연 0.45 s x 램프 0.5 m/s^2 ≈ 0.23 보다 작으면
  // 매 주기 되감긴다(run48: 0.05 로 톱니, 0 -> 0.25 에 0.85 s). ROS 파라미터 resync_margin.
  double resync_margin{0.15};
};

struct Desired
{
  Twist2D cmd;
  double curvature{std::nan("")};
  double radius{-1.0};
  // true 면 planned_decel 로 내린다(해결안 ①). 기본은 비상 제동(max_decel).
  bool planned{false};
};

// 모든 상황이 같은 한계를 거친다(설계서 7절). 상황이 바뀌어도 직전 명령에서 이어 간다.
class OutputStage
{
public:
  explicit OutputStage(OutputParams p = {}) : p_(p) {}
  // 실측 속도에서 이어 간다(0 으로 떨어뜨렸다 다시 램프하지 않는다, 최종 리뷰 I3).
  void reset(const Twist2D & measured = {})
  {
    last_ = {std::clamp(measured.v, 0.0, p_.max_v), std::clamp(measured.w, -p_.max_w, p_.max_w)};
  }
  Twist2D apply(const Desired & d, double measured_v, double dt);
  const Twist2D & last() const {return last_;}

private:
  OutputParams p_;
  Twist2D last_;
};
}  // namespace vica_vcc_controller::core
