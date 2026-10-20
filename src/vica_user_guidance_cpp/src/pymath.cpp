#include "vica_user_guidance_cpp/pymath.hpp"

#include <cmath>

#include "vica_user_guidance_cpp/pyfmt.hpp"

namespace vica_guidance
{
namespace
{

double vector_norm(int n, double * vec, double max, bool found_nan)
{
  const double T27 = 134217729.0;
  double x, scale, oldcsum, csum = 1.0, frac1 = 0.0, frac2 = 0.0, frac3 = 0.0;
  double t, hi, lo, h;
  int max_e;

  if (std::isinf(max)) {return max;}
  if (found_nan) {return NAN;}
  if (max == 0.0 || n <= 1) {return max;}
  std::frexp(max, &max_e);
  if (max_e < -1023) {
    double frac = 0.0;
    for (int i = 0; i < n; i++) {
      x = vec[i] / max;
      x = x * x;
      oldcsum = csum;
      csum += x;
      frac += (oldcsum - csum) + x;
    }
    return max * std::sqrt(csum - 1.0 + frac);
  }
  scale = std::ldexp(1.0, -max_e);
  for (int i = 0; i < n; i++) {
    x = vec[i];
    x *= scale;
    t = x * T27;
    hi = t - (t - x);
    lo = x - hi;

    x = hi * hi;
    oldcsum = csum;
    csum += x;
    frac1 += (oldcsum - csum) + x;

    x = 2.0 * hi * lo;
    oldcsum = csum;
    csum += x;
    frac2 += (oldcsum - csum) + x;

    frac3 += lo * lo;
  }
  h = std::sqrt(csum - 1.0 + (frac1 + frac2 + frac3));

  x = h;
  t = x * T27;
  hi = t - (t - x);
  lo = x - hi;

  x = -hi * hi;
  oldcsum = csum;
  csum += x;
  frac1 += (oldcsum - csum) + x;

  x = -2.0 * hi * lo;
  oldcsum = csum;
  csum += x;
  frac2 += (oldcsum - csum) + x;

  x = -lo * lo;
  oldcsum = csum;
  csum += x;
  frac3 += (oldcsum - csum) + x;

  x = csum - 1.0 + (frac1 + frac2 + frac3);
  return (h + x / (2.0 * h)) / scale;
}

double norm2(double a, double b)
{
  double v[2] = {std::fabs(a), std::fabs(b)};
  double max = 0.0;
  bool found_nan = false;
  for (double x : v) {
    found_nan |= std::isnan(x);
    if (x > max) {max = x;}
  }
  return vector_norm(2, v, max, found_nan);
}

double math_1(double x, double r)
{
  if (std::isnan(r) && !std::isnan(x)) {throw PyError("ValueError", "math domain error");}
  if (std::isinf(r) && std::isfinite(x)) {throw PyError("ValueError", "math domain error");}
  return r;
}

}

double py_atan2(double y, double x)
{
  if (std::isnan(x) || std::isnan(y)) {return NAN;}
  if (std::isinf(y)) {
    if (std::isinf(x)) {
      return std::copysign(std::copysign(1., x) == 1. ? 0.25 * PY_PI : 0.75 * PY_PI, y);
    }
    return std::copysign(0.5 * PY_PI, y);
  }
  if (std::isinf(x) || y == 0.) {
    return std::copysign(1., x) == 1. ? std::copysign(0., y) : std::copysign(PY_PI, y);
  }
  return std::atan2(y, x);
}

double py_sin(double x) {return math_1(x, std::sin(x));}
double py_cos(double x) {return math_1(x, std::cos(x));}
double py_sqrt(double x) {return math_1(x, std::sqrt(x));}

double py_hypot(double a, double b) {return norm2(a, b);}

double py_dist(double px, double py, double qx, double qy) {return norm2(px - qx, py - qy);}

}
