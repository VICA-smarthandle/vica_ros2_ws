#include "vica_docking/v_detector.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace vica_docking
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
inline double deg2rad(double d) {return d * kPi / 180.0;}
inline double rad2deg(double r) {return r * 180.0 / kPi;}

using Seg = std::pair<int, int>;

void split_merge(const std::vector<Vec2> & P, int lo, int hi, double thr, std::vector<Seg> & out)
{
  if (hi - lo < 4) {
    out.emplace_back(lo, hi);
    return;
  }
  const Vec2 a = P[lo];
  const Vec2 b = P[hi - 1];
  const Vec2 ab = b - a;
  const double L = norm(ab);
  if (L < 1e-6) {
    out.emplace_back(lo, hi);
    return;
  }
  const Vec2 n{-ab.y / L, ab.x / L};
  int k = 0;
  double kd = -1.0;
  for (int i = lo; i < hi; ++i) {
    const double dist = std::abs(dot(P[i] - a, n));
    if (dist > kd) {
      kd = dist;
      k = i - lo;
    }
  }
  if (kd > thr && 2 <= k && k <= hi - lo - 3) {
    split_merge(P, lo, lo + k + 1, thr, out);
    split_merge(P, lo + k, hi, thr, out);
  } else {
    out.emplace_back(lo, hi);
  }
}

double median_of(const std::vector<double> & v, int lo, int hi)
{
  std::vector<double> s(v.begin() + lo, v.begin() + hi);
  std::sort(s.begin(), s.end());
  const size_t m = s.size();
  return (m % 2 == 1) ? s[m / 2] : 0.5 * (s[m / 2 - 1] + s[m / 2]);
}

}

LineFit fit_line(const std::vector<Vec2> & P, int lo, int hi)
{
  const int m = hi - lo;
  Vec2 c;
  for (int i = lo; i < hi; ++i) {
    c.x += P[i].x;
    c.y += P[i].y;
  }
  c.x /= m;
  c.y /= m;
  double sxx = 0.0, sxy = 0.0, syy = 0.0;
  for (int i = lo; i < hi; ++i) {
    const double dx = P[i].x - c.x;
    const double dy = P[i].y - c.y;
    sxx += dx * dx;
    sxy += dx * dy;
    syy += dy * dy;
  }
  const double ang = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
  LineFit f;
  f.c = c;
  f.d = {std::cos(ang), std::sin(ang)};
  f.n = left_of(f.d);
  double ss = 0.0;
  for (int i = lo; i < hi; ++i) {
    const double e = dot(P[i] - c, f.n);
    ss += e * e;
  }
  f.res = std::sqrt(ss / m);
  return f;
}

VDetector::VDetector(VDetectorParams p, RobotGeometry g)
: p_(p), g_(g) {}

std::optional<VDetection> VDetector::detect(
  const std::vector<float> & ranges, double amin, double inc, double rmin)
{
  const double fov = deg2rad(p_.fov_deg);
  const double rlo = std::max(rmin, 0.16);
  std::vector<int> idx;
  std::vector<double> rr;
  std::vector<Vec2> P;
  idx.reserve(ranges.size());
  for (size_t i = 0; i < ranges.size(); ++i) {
    const double r = static_cast<double>(ranges[i]);
    const double a = amin + static_cast<double>(i) * inc;
    if (std::isfinite(r) && r >= rlo && r <= p_.max_range && std::abs(a) <= fov) {
      idx.push_back(static_cast<int>(i));
      rr.push_back(r);
      P.push_back(beam_to_base(g_, r, a));
    }
  }
  const int N = static_cast<int>(idx.size());
  if (N < 12) {
    return std::nullopt;
  }
  std::vector<Seg> clusters;
  int start = 0;
  for (int k = 1; k <= N; ++k) {
    bool cut = (k == N);
    if (!cut) {
      const int gap = idx[k] - idx[k - 1];
      cut = gap > 12 ||
        norm(P[k] - P[k - 1]) > std::max(0.05, 3.0 * rr[k] * inc * static_cast<double>(gap));
    }
    if (cut) {
      if (k - start >= 8) {
        clusters.emplace_back(start, k);
      }
      start = k;
    }
  }

  const double cos12 = std::cos(deg2rad(12.0));
  std::optional<VDetection> best;
  for (const auto & cl : clusters) {
    const int lo = cl.first;
    const int hi = cl.second;
    std::vector<Seg> segs;
    split_merge(P, lo, hi, 0.008 + 0.004 * median_of(rr, lo, hi), segs);
    bool merged = true;
    while (merged && segs.size() > 1) {
      merged = false;
      for (size_t j = 0; j + 1 < segs.size(); ++j) {
        const Seg s1 = segs[j];
        const Seg s2 = segs[j + 1];
        if (s1.second - s1.first >= 3 && s2.second - s2.first >= 3) {
          const LineFit f1 = fit_line(P, s1.first, s1.second);
          const LineFit f2 = fit_line(P, s2.first, s2.second);
          if (std::abs(dot(f1.d, f2.d)) > cos12) {
            segs[j] = {s1.first, s2.second};
            segs.erase(segs.begin() + static_cast<long>(j) + 1);
            merged = true;
            break;
          }
        }
      }
    }
    for (size_t j = 0; j + 1 < segs.size(); ++j) {
      const int a0 = segs[j].first, a1 = segs[j].second;
      const int b0 = segs[j + 1].first, b1 = segs[j + 1].second;
      if (a1 - a0 < 5 || b1 - b0 < 5) {
        continue;
      }
      const LineFit fa = (a1 - a0 > 6) ? fit_line(P, a0 + 1, a1 - 1) : fit_line(P, a0, a1);
      const LineFit fb = (b1 - b0 > 6) ? fit_line(P, b0 + 1, b1 - 1) : fit_line(P, b0, b1);
      const double det = fa.n.x * fb.n.y - fa.n.y * fb.n.x;
      if (std::abs(det) < 1e-3) {
        continue;
      }
      const double ra = dot(fa.n, fa.c);
      const double rb = dot(fb.n, fb.c);
      const Vec2 apex{(ra * fb.n.y - fa.n.y * rb) / det, (fa.n.x * rb - ra * fb.n.x) / det};
      Vec2 da = fa.d;
      Vec2 db = fb.d;
      if (!(dot(da, P[a0] - apex) > 0)) {
        da = -da;
      }
      if (!(dot(db, P[b1 - 1] - apex) > 0)) {
        db = -db;
      }
      const double inner = rad2deg(std::acos(std::clamp(dot(da, db), -1.0, 1.0)));
      const double la = dot(P[a0] - apex, da);
      const double lb = dot(P[b1 - 1] - apex, db);
      Vec2 u = da + db;
      const double un = norm(u);
      if (un < 1e-6) {
        continue;
      }
      u = u * (1.0 / un);
      if (dot(u, -apex) <= 0 || std::abs(inner - p_.inner_deg) > 15 ||
        !(0.10 <= la && la <= 0.30) || !(0.10 <= lb && lb <= 0.30))
      {
        continue;
      }
      const Vec2 mid = (P[a1 - 1] + P[b0]) * 0.5;
      if (norm(apex - mid) > 0.06) {
        continue;
      }
      double score = std::abs(inner - p_.inner_deg) +
        50.0 * (std::abs(la - p_.arm) + std::abs(lb - p_.arm)) + 200.0 * std::max(fa.res, fb.res);
      if (has_last_) {
        score += 20.0 * norm(apex - last_);
      }
      if (!best || score < best->score) {
        VDetection d;
        d.score = score;
        d.apex = apex;
        d.u = u;
        d.da = da;
        d.db = db;
        d.inner_deg = inner;
        d.la = la;
        d.lb = lb;
        d.res = std::max(fa.res, fb.res);
        d.na = a1 - a0;
        d.nb = b1 - b0;
        best = d;
      }
    }
  }
  if (best) {
    has_last_ = true;
    last_ = best->apex;
  }
  return best;
}

DockPose pose_in_dock(const VDetection & d)
{
  const Vec2 fwd = -d.u;
  const Vec2 left = left_of(fwd);
  const Vec2 p = -d.apex;
  return {dot(p, d.u), dot(p, left), -std::atan2(fwd.y, fwd.x)};
}

}
