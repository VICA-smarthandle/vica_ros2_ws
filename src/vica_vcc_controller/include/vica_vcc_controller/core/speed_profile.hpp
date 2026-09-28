#pragma once
#include "vica_vcc_controller/core/types.hpp"

namespace vica_vcc_controller::core
{
struct SpeedParams
{
  double desired{0.5};           // desired_linear_vel
  double min_speed{0.12};        // RPP regulated_linear_scaling_min_speed
  double curve_min_radius{1.2};  // RPP regulated_linear_scaling_min_radius
  double curve_decel{0.3};       // 코너 미리 줄이기 감속(설계서 7절)
  double preview_dist{1.5};
  double curve_sample{0.2};      // 곡률을 잴 점 간격(레일 경로 0.05 m 잡음 회피)
  double slow_clearance{0.35};   // RPP cost_scaling_dist 와 같은 값, 단 직접 잰 거리
  double approach_dist{0.6};     // RPP approach_velocity_scaling_dist
  double approach_min{0.05};     // RPP min_approach_linear_velocity
};

double curveLimitAt(double curvature, const SpeedParams & p);
// 앞 preview_dist 안 곡선마다 "그 곡선 한계에 curve_decel 로 제때 닿는 속도"의 최소.
double curvePreviewLimit(const Path & path, const SpeedParams & p);
double clearanceLimit(double clearance, const SpeedParams & p);
double approachLimit(double dist_to_end, double v, const SpeedParams & p);
}  // namespace vica_vcc_controller::core
