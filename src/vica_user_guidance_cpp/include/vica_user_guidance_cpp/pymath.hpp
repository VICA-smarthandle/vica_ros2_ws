#pragma once

namespace vica_guidance
{

constexpr double PY_PI = 3.14159265358979323846;

double py_atan2(double y, double x);
double py_sin(double x);
double py_cos(double x);
double py_sqrt(double x);
double py_hypot(double a, double b);
double py_dist(double px, double py, double qx, double qy);
inline double py_degrees(double x) {return x * (180.0 / PY_PI);}
inline double py_radians(double x) {return x * (PY_PI / 180.0);}

}
