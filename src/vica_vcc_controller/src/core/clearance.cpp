#include "vica_vcc_controller/core/clearance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include "vica_vcc_controller/core/geometry.hpp"

namespace vica_vcc_controller::core
{
namespace
{
constexpr float kFar = 1e6f;
}

void ClearanceGrid::reset(double ox, double oy, double res, int w, int h)
{
  ox_ = ox; oy_ = oy; res_ = res; w_ = w; h_ = h;
  lethal_.assign(static_cast<size_t>(w) * h, 0);
  dist_.assign(static_cast<size_t>(w) * h, kFar);
}

void ClearanceGrid::markLethal(int ix, int iy)
{
  if (ix < 0 || iy < 0 || ix >= w_ || iy >= h_) {return;}
  lethal_[idx(ix, iy)] = 1;
}

void ClearanceGrid::compute()
{
  const float a = 1.0f, b = std::sqrt(2.0f);
  for (size_t i = 0; i < dist_.size(); ++i) {dist_[i] = lethal_[i] ? 0.0f : kFar;}
  for (int y = 0; y < h_; ++y) {
    for (int x = 0; x < w_; ++x) {
      float & d = dist_[idx(x, y)];
      if (d == 0.0f) {continue;}
      if (x > 0) {d = std::min(d, dist_[idx(x - 1, y)] + a);}
      if (y > 0) {
        d = std::min(d, dist_[idx(x, y - 1)] + a);
        if (x > 0) {d = std::min(d, dist_[idx(x - 1, y - 1)] + b);}
        if (x < w_ - 1) {d = std::min(d, dist_[idx(x + 1, y - 1)] + b);}
      }
    }
  }
  for (int y = h_ - 1; y >= 0; --y) {
    for (int x = w_ - 1; x >= 0; --x) {
      float & d = dist_[idx(x, y)];
      if (d == 0.0f) {continue;}
      if (x < w_ - 1) {d = std::min(d, dist_[idx(x + 1, y)] + a);}
      if (y < h_ - 1) {
        d = std::min(d, dist_[idx(x, y + 1)] + a);
        if (x < w_ - 1) {d = std::min(d, dist_[idx(x + 1, y + 1)] + b);}
        if (x > 0) {d = std::min(d, dist_[idx(x - 1, y + 1)] + b);}
      }
    }
  }
}

bool ClearanceGrid::worldToCell(double wx, double wy, int & ix, int & iy) const
{
  ix = static_cast<int>(std::floor((wx - ox_) / res_));
  iy = static_cast<int>(std::floor((wy - oy_) / res_));
  return ix >= 0 && iy >= 0 && ix < w_ && iy < h_;
}

bool ClearanceGrid::lethalCell(int ix, int iy) const
{
  if (ix < 0 || iy < 0 || ix >= w_ || iy >= h_) {return false;}
  return lethal_[idx(ix, iy)] != 0;
}

double ClearanceGrid::distanceAt(double wx, double wy) const
{
  int ix, iy;
  if (!worldToCell(wx, wy, ix, iy)) {return kFarDistance;}
  const float d = dist_[idx(ix, iy)];
  if (d >= kFar) {return kFarDistance;}
  return static_cast<double>(d) * res_;
}

void ClearanceField::setFootprint(const Polygon & padded, double outline_step)
{
  footprint_ = padded;
  outline_ = densifyOutline(padded, outline_step);
  radius_ = circumscribedRadius(padded);
}

double ClearanceField::clearance(const Pose2D & pose) const
{
  const Polygon poly = transformPolygon(pose, footprint_);

  double minx = poly[0].x, maxx = minx, miny = poly[0].y, maxy = miny;
  for (const auto & p : poly) {
    minx = std::min(minx, p.x); maxx = std::max(maxx, p.x);
    miny = std::min(miny, p.y); maxy = std::max(maxy, p.y);
  }
  int ix0, iy0, ix1, iy1;
  grid_.worldToCell(minx, miny, ix0, iy0);
  grid_.worldToCell(maxx, maxy, ix1, iy1);
  const double res = grid_.resolution();
  const int gw = grid_.width(), gh = grid_.height();
  if (!(ix1 < 0 || iy1 < 0 || ix0 >= gw || iy0 >= gh)) {
    ix0 = std::max(ix0, 0); iy0 = std::max(iy0, 0);
    ix1 = std::min(ix1, gw - 1); iy1 = std::min(iy1, gh - 1);
    for (int ix = ix0; ix <= ix1; ++ix) {
      for (int iy = iy0; iy <= iy1; ++iy) {
        if (!grid_.lethalCell(ix, iy)) {continue;}
        const Point2D c{grid_.originX() + (ix + 0.5) * res, grid_.originY() + (iy + 0.5) * res};
        if (pointInPolygon(c, poly)) {return -1.0;}
      }
    }
  }

  double c = std::numeric_limits<double>::infinity();
  for (const auto & q : outline_) {
    const Point2D g = toParent(pose, q);
    c = std::min(c, grid_.distanceAt(g.x, g.y) - 0.5 * res);
  }

  for (const auto & p : points_) {
    if (std::hypot(p.x - pose.x, p.y - pose.y) > radius_ + 2.0) {continue;}
    c = std::min(c, signedDistanceToPolygon(p, poly));
  }
  return c;
}
}
