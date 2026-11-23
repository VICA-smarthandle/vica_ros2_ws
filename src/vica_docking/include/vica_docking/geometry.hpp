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
inline Vec2 left_of(Vec2 d) {return {-d.y, d.x};}

struct RobotGeometry
{
  double laser_x{0.031};
  double laser_y{0.0};
  double laser_yaw{-0.0346};
  double bumper_x{0.151};
  double half_w{0.225};
};

inline Vec2 beam_to_base(const RobotGeometry & g, double r, double a)
{
  const double c = std::cos(g.laser_yaw);
  const double s = std::sin(g.laser_yaw);
  const double lx = r * std::cos(a);
  const double ly = r * std::sin(a);
  return {g.laser_x + c * lx - s * ly, g.laser_y + s * lx + c * ly};
}

}
