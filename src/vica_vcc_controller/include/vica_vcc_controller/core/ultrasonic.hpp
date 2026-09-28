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
  // 받은 순간의 센서 자세(costmap 전역 좌표계 = odom). 2.4 Hz 라 나중 TF 로 옮기면 v·나이만큼 멀어 보인다.
  Pose2D sensor;
};

class UltrasonicChannel
{
public:
  void push(const RangeReading & r);
  // 최근 count 개가 모두 max_age 안·유효 범위·서로 tol 안이면 최신 값.
  std::optional<RangeReading> confirmed(double now, double max_age, int count, double tol) const;
  // 최신 측정이 max_age 안인가(/vcc/state us_fresh, 설계서 10절).
  bool fresh(double now, double max_age) const
  {
    return !hist_.empty() && now - hist_.back().recv_time <= max_age;
  }

private:
  std::deque<RangeReading> hist_;
};

// 센서 좌표(x 앞)에서 거리 range, 폭 fov 의 호를 n 점으로.
std::vector<Point2D> rangeToArcPoints(double range, double fov, int n);
// 측정값의 호 점을 받은 순간의 센서 자세로 전역 좌표에 놓는다(지금 TF 로 다시 옮기지 않는다).
std::vector<Point2D> readingToArcPoints(const RangeReading & r, int n);
}  // namespace vica_vcc_controller::core
