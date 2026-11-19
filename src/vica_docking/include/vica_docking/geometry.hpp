// 도킹 계산에 쓰는 2차원 벡터와 로봇 치수.
// 치수는 VICA.xacro(구동축 = base_footprint 원점 기준)와 같다. 11-17·18 실기 도구
// (~/vica_data/docking_plan/tools/dock_approach_test.py v2.1)가 쓴 값 그대로다.
#pragma once

#include <cmath>

namespace vica_docking
{

struct Vec2
{
  double x{0.0};
  double y{0.0};
};

inline Vec2 operator+(Vec2 a, Vec2 b) {return {a.x + b.x, a.y + b.y};}
inline Vec2 operator-(Vec2 a, Vec2 b) {return {a.x - b.x, a.y - b.y};}
inline Vec2 operator-(Vec2 a) {return {-a.x, -a.y};}
inline Vec2 operator*(Vec2 a, double s) {return {a.x * s, a.y * s};}
inline Vec2 operator*(double s, Vec2 a) {return {a.x * s, a.y * s};}
inline double dot(Vec2 a, Vec2 b) {return a.x * b.x + a.y * b.y;}
inline double norm(Vec2 a) {return std::sqrt(a.x * a.x + a.y * a.y);}
// 왼쪽(반시계 90°) 수직
inline Vec2 left_of(Vec2 d) {return {-d.y, d.x};}

struct RobotGeometry
{
  double laser_x{0.031};      // 라이다 위치(구동축 앞)
  double laser_y{0.0};
  double laser_yaw{-0.0346};  // 장착각(11-01 실측 −1.98°)
  double bumper_x{0.151};     // 범퍼 앞면
  double half_w{0.225};       // 차체 반폭
};

// 라이다 한 빔을 구동축(base_footprint) 좌표로.
inline Vec2 beam_to_base(const RobotGeometry & g, double r, double a)
{
  const double c = std::cos(g.laser_yaw);
  const double s = std::sin(g.laser_yaw);
  const double lx = r * std::cos(a);
  const double ly = r * std::sin(a);
  return {g.laser_x + c * lx - s * ly, g.laser_y + s * lx + c * ly};
}

}  // namespace vica_docking
