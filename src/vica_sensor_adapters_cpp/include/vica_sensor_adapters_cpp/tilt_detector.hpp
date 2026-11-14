// 몸통 넘어짐 판정 (순수 로직, ROS 비의존). 2026-10-15 사용자 결정.
//
// 넘어지면 중앙 비상정지 래치에 원인 `tipped` 를 건다 — 관리자가 와서 세운 뒤
// 앱에서 해제할 때까지 바퀴는 0 이다. 그래서 이 판정이 틀리면 멀쩡히 서 있는
// 로봇이 비상정지된다. 가장 중요한 규칙은 **유지 시간**이다.
//
//     run90~95 녹화(/imu/base_link, 주행 포함 54분)에서 순간 기울기는 바퀴
//     진동으로 최대 52° 까지 튀었다. 그러나 1초 내내 넘은 각은 최대 4.4° 였다.
//     순간값만 보면 주행 중에 넘어졌다고 오판한다.
//
// 기울기는 base_link 기준 가속도(중력 포함)와 몸의 위(+z) 사이의 각이다. 서 있으면
// 0°, 옆으로 누우면 90° 다. 가속·감속도 섞이지만 스무더 상한 2.5 m/s² 는 약 14° 로
// 보일 뿐이라 45° 에 닿지 않는다.
//
// 세운 뒤 풀리는 각(release)을 넘어지는 각보다 낮게 둔다. 45° 언저리에 기대 세운
// 로봇이 넘어짐·풀림을 오가며 멘트를 반복하지 않게 한다.
//
// 유지 시간은 표본이 **이어서** 들어올 때만 센다. 노드가 1초 넘게 멈췄다가 다시 돌면
// 멈추기 전·후 진동 표본 두 개만으로 1초를 채울 수 있다 — 표본 사이가 max_gap 을
// 넘으면 처음부터 다시 센다.
//
// 켠 직후의 상태는 모른다(known() = false). 서 있음·넘어짐 어느 쪽이든 1초를 채워야
// 첫 상태를 낸다. 넘어진 채 노드를 다시 켰을 때 "서 있음"을 먼저 내 비상정지 원인이
// 잠깐 풀리는 틈을 막는다.
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
  ///
  /// now_sec 은 단조 시계다. tilt 가 NaN 이면 그 표본은 어느 쪽 유지 시간에도 넣지
  /// 않고 처음부터 다시 센다(fail-safe 가 아니라 판정 보류 — 모르는 표본으로
  /// 비상정지를 걸지도, 풀지도 않는다). 시간이 거꾸로 가거나 앞 표본과 max_gap 넘게
  /// 떨어졌으면 그 표본부터 새로 센다.
  bool update(double tilt, double now_sec);

  bool known() const {return known_;}
  bool tipped() const {return tipped_;}
  double tip_angle_deg() const {return tip_angle_deg_;}
  double release_angle_deg() const {return release_angle_deg_;}
  double hold_sec() const {return hold_sec_;}

  /// 크기가 이보다 작은 가속도는 방향을 믿지 않는다(중력 9.67 의 약 1/5).
  static constexpr double kMinAccelNorm = 2.0;
  /// 표본 사이가 이보다 벌어지면 유지 시간을 처음부터 센다. 발행 주기(36 Hz,
  /// 28 ms)의 약 9 배.
  static constexpr double kDefaultMaxGapSec = 0.25;

private:
  double tip_angle_deg_;
  double release_angle_deg_;
  double hold_sec_;
  double max_gap_sec_;
  bool known_{false};
  bool tipped_{false};
  // 상태를 바꿀(정할) 조건이 이어지기 시작한 시각과 그 방향. 비어 있으면 조건 밖이다.
  std::optional<double> since_;
  bool pending_tipped_{false};
  std::optional<double> last_now_;
};

}  // namespace vica_sensor_adapters_cpp

#endif  // VICA_SENSOR_ADAPTERS_CPP__TILT_DETECTOR_HPP_
