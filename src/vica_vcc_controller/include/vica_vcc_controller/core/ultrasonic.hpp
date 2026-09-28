#pragma once
#include <deque>
#include <optional>
#include <vector>
#include "vica_vcc_controller/core/types.hpp"

namespace vica_vcc_controller::core
{
struct RangeReading
{
  double range{0.0};
  double min_range{0.0};
  double max_range{0.0};
  double fov{0.0};
  double recv_time{0.0};   // 받은 시각(steady). stamp 가 아니다 — run44 stamp 지연으로 660회 무시된 교훈
};

class UltrasonicChannel
{
public:
  void push(const RangeReading & r);
  // 최근 count 개가 모두 max_age 안·유효 범위·서로 tol 안이면 최신 값.
  std::optional<RangeReading> confirmed(double now, double max_age, int count, double tol) const;

private:
  std::deque<RangeReading> hist_;
};

// 센서 좌표(x 앞)에서 거리 range, 폭 fov 의 호를 n 점으로.
std::vector<Point2D> rangeToArcPoints(double range, double fov, int n);
}  // namespace vica_vcc_controller::core
