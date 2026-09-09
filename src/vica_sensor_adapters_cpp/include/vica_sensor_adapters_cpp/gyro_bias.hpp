// 정지 중 자이로 편향 추정 (순수 로직, ROS 비의존).
//
// 계약 정본은 파이썬 판 `vica_sensor_adapters/gyro_bias.py` 의 주석이며,
// 이 파일은 그 로직을 그대로 옮긴 것이다. 숫자가 하나라도 달라지면 EKF yaw 가
// 휘므로, 시험 19 개를 먼저 옮겨 통과시킨 뒤에 노드를 붙였다.
//
// 가장 중요한 규칙은 `aborted` 다. 보정 구간에 로봇이 움직였으면 편향을
// 적용하지 않는다 — 틀린 상수를 빼는 것은 드리프트보다 나쁘다.
#ifndef VICA_SENSOR_ADAPTERS_CPP__GYRO_BIAS_HPP_
#define VICA_SENSOR_ADAPTERS_CPP__GYRO_BIAS_HPP_

#include <array>

namespace vica_sensor_adapters_cpp
{

using Vec3 = std::array<double, 3>;

class GyroBiasEstimator
{
public:
  GyroBiasEstimator(
    int sample_count,
    double max_abs_rate,
    int refresh_sample_count = 0,
    double refresh_alpha = 0.2,
    double max_abs_dev = 0.01,
    double max_refresh_jump = 0.02);

  /// 표본을 하나 넣는다. 확정·포기 뒤에는 ZUPT 쪽으로 넘어간다.
  void add(double gx, double gy, double gz);

  /// 편향을 뺀 각속도. 확정 전이면 원값 그대로다.
  Vec3 correct(double gx, double gy, double gz) const;

  bool ready() const {return ready_;}
  bool aborted() const {return aborted_;}
  Vec3 bias() const {return bias_;}
  int collected() const {return collected_;}
  int refresh_count() const {return refresh_count_;}
  int sample_count() const {return sample_count_;}

private:
  void add_refresh(double gx, double gy, double gz);
  void reset_refresh();

  int sample_count_;
  double max_abs_rate_;
  int refresh_sample_count_;
  double refresh_alpha_;
  double max_abs_dev_;
  double max_refresh_jump_;

  Vec3 sums_{0.0, 0.0, 0.0};
  int collected_{0};
  Vec3 bias_{0.0, 0.0, 0.0};
  bool ready_{false};
  bool aborted_{false};

  // ZUPT 구간 상태
  Vec3 r_sums_{0.0, 0.0, 0.0};
  Vec3 r_first_{0.0, 0.0, 0.0};
  bool r_has_first_{false};
  int r_count_{0};
  int refresh_count_{0};
};

}  // namespace vica_sensor_adapters_cpp

#endif  // VICA_SENSOR_ADAPTERS_CPP__GYRO_BIAS_HPP_
