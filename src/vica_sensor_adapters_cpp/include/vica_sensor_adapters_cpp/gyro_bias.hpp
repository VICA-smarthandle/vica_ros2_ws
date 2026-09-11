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

  Vec3 r_sums_{0.0, 0.0, 0.0};
  Vec3 r_first_{0.0, 0.0, 0.0};
  bool r_has_first_{false};
  int r_count_{0};
  int refresh_count_{0};
};

}

#endif
