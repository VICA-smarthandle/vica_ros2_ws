#ifndef VICA_SENSOR_ADAPTERS_CPP__TILT_DETECTOR_HPP_
#define VICA_SENSOR_ADAPTERS_CPP__TILT_DETECTOR_HPP_

#include <optional>

namespace vica_sensor_adapters_cpp
{

class TiltDetector
{
public:
  /// release_angle_deg 가 tip_angle_deg 보다 크면 tip_angle_deg 로 맞춘다.
  TiltDetector(
    double tip_angle_deg, double release_angle_deg, double hold_sec,
    double max_gap_sec = kDefaultMaxGapSec);

  /// 가속도로 기울기[deg]를 낸다. 크기가 너무 작으면(자유 낙하·무효 표본) NaN.
  static double tilt_deg(double ax, double ay, double az);

  /// 표본 하나를 넣는다. 상태가 정해지거나(첫 상태) 바뀐 표본에서만 true 를 돌려준다.
  bool update(double tilt, double now_sec);

  bool known() const {return known_;}
  bool tipped() const {return tipped_;}
  double tip_angle_deg() const {return tip_angle_deg_;}
  double release_angle_deg() const {return release_angle_deg_;}
  double hold_sec() const {return hold_sec_;}

  /// 크기가 이보다 작은 가속도는 방향을 믿지 않는다(중력 9.67 의 약 1/5).
  static constexpr double kMinAccelNorm = 2.0;
  /// 표본 사이가 이보다 벌어지면 유지 시간을 처음부터 센다. 발행 주기(36 Hz,
  static constexpr double kDefaultMaxGapSec = 0.25;

private:
  double tip_angle_deg_;
  double release_angle_deg_;
  double hold_sec_;
  double max_gap_sec_;
  bool known_{false};
  bool tipped_{false};
  std::optional<double> since_;
  bool pending_tipped_{false};
  std::optional<double> last_now_;
};

}

#endif
