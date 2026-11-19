// 스캔에서 '범퍼 앞 3 cm 안 장애물'과 '제자리 회전 반경 안 점'을 센다(11-17·18 실기 도구와 같은 규칙).
#pragma once

#include <optional>
#include <vector>

#include "vica_docking/geometry.hpp"
#include "vica_docking/v_detector.hpp"

namespace vica_docking
{

// 범퍼 앞 depth 안·차체 폭 안의 점 수. V자를 알면 그 팔 위의 점과, skip_behind_apex 이면
// 꼭짓점 면보다 뒤(벽·도크 몸체, 도크 좌표 X < 0.02 m)의 점도 뺀다.
int count_bumper_zone(
  const RobotGeometry & g, const std::vector<float> & ranges, double amin, double inc,
  double range_min, const std::optional<VDetection> & det, double depth = 0.03,
  bool skip_behind_apex = false, double v_margin = 0.015, double behind_margin = 0.02);

struct SweepCount
{
  int stop_hits{0};  // 구동축 반경 sweep_r 안
  int slow_hits{0};  // 구동축 반경 slow_r 안
};

// 회전 감시: 보이는 쪽(±120°)에서 0.25 m 넘는 빔이 구동축 반경 안에 몇 개인가.
SweepCount count_sweep(
  const RobotGeometry & g, const std::vector<float> & ranges, double amin, double inc,
  double sweep_r = 0.64, double slow_r = 0.80);

}  // namespace vica_docking
