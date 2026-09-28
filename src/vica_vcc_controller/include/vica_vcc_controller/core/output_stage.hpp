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
  double max_decel{1.25};     // velocity_smoother max_decel[0]. 제동은 완화하지 않는다
  double resync_margin{0.05};
};

struct Desired
{
  Twist2D cmd;
  double curvature{std::nan("")};
  double radius{-1.0};
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
