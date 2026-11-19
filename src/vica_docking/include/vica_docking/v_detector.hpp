// 라이다 한 스캔에서 V자 도크(내각 ~120°, 팔 ~0.20 m, 벌어진 쪽이 로봇)를 찾는다.
//
// 11-17·18 실기에서 쓴 파이썬 VDetector(dock_approach_test.py v2.1)를 그대로 옮겼다.
// 같은 입력이면 같은 결과가 나와야 한다 — test/parity 가 녹화 스캔으로 대조한다.
//   ① 앞 ±75°·3 m 안 점만 쓴다
//   ② 빈 빔 수가 아니라 실제 점 간격으로 무리를 나눈다(가까이서 빔 유효율 69~84 %)
//   ③ 무리를 꺾인 곳에서 직선 조각으로 나누고(split-merge), 12° 안으로 거의 한 줄인 이웃은 합친다
//   ④ 이웃한 두 조각이 120°±15°로 만나고 팔이 0.10~0.30 m 이면 V자 후보, 점수가 가장 낮은 것
// 결과 좌표는 구동축(base_footprint) 기준.
#pragma once

#include <optional>
#include <vector>

#include "vica_docking/geometry.hpp"

namespace vica_docking
{

struct VDetection
{
  double score{0.0};
  Vec2 apex;        // 꼭짓점
  Vec2 u;           // 두 팔의 가운데 방향(단위). 꼭짓점에서 로봇 쪽(벌어진 쪽)을 향한다
  Vec2 da;          // 팔 A 방향(꼭짓점에서 바깥으로, 단위)
  Vec2 db;          // 팔 B 방향
  double inner_deg{0.0};
  double la{0.0};   // 팔 A 길이
  double lb{0.0};
  double res{0.0};  // 두 팔 직선 맞춤 잔차 중 큰 값(m)
  int na{0};        // 팔 A 점 수
  int nb{0};
};

struct VDetectorParams
{
  double inner_deg{120.0};
  double arm{0.20};
  double max_range{3.0};
  double fov_deg{75.0};
};

class VDetector
{
public:
  explicit VDetector(VDetectorParams p = {}, RobotGeometry g = {});

  // ranges 는 LaserScan.ranges 그대로(float). 찾으면 마지막 꼭짓점을 기억해 다음 점수에 쓴다.
  std::optional<VDetection> detect(
    const std::vector<float> & ranges, double angle_min, double angle_increment, double range_min);

  void reset() {has_last_ = false;}

private:
  VDetectorParams p_;
  RobotGeometry g_;
  bool has_last_{false};
  Vec2 last_;
};

// 로봇(구동축)을 V자 좌표로.
//   x  = 축거리: 꼭짓점에서 축을 따라 구동축까지(m). 범퍼→꼭짓점 = x − bumper_x
//   y  = 좌우: V자 중심선에서 로봇이 왼쪽(+)
//   th = 방향: 꼭짓점을 정면으로 보는 방향에서 반시계(+)
struct DockPose
{
  double x{0.0};
  double y{0.0};
  double th{0.0};
};
DockPose pose_in_dock(const VDetection & d);

// 직선 맞춤(주성분). c = 평균, d = 방향(단위), n = 수직, res = 수직 잔차 RMS.
struct LineFit
{
  Vec2 c;
  Vec2 d;
  Vec2 n;
  double res{0.0};
};
LineFit fit_line(const std::vector<Vec2> & P, int lo, int hi);

}  // namespace vica_docking
