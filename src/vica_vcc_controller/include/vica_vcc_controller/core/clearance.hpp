#pragma once
#include <cstdint>
#include <functional>
#include <vector>
#include "vica_vcc_controller/core/types.hpp"

namespace vica_vcc_controller::core
{
class ClearanceGrid
{
public:
  static constexpr double kFarDistance = 10.0;
  void reset(double origin_x, double origin_y, double resolution, int width, int height);
  void markLethal(int ix, int iy);
  void compute();
  double distanceAt(double wx, double wy) const;
  bool worldToCell(double wx, double wy, int & ix, int & iy) const;
  bool lethalCell(int ix, int iy) const;
  double resolution() const {return res_;}
  double originX() const {return ox_;}
  double originY() const {return oy_;}
  int width() const {return w_;}
  int height() const {return h_;}

private:
  size_t idx(int x, int y) const {return static_cast<size_t>(y) * w_ + x;}
  double ox_{0.0}, oy_{0.0}, res_{0.05};
  int w_{0}, h_{0};
  std::vector<uint8_t> lethal_;
  std::vector<float> dist_;
};

using ClearanceFn = std::function<double(const Pose2D &)>;

class ClearanceField
{
public:
  void setFootprint(const Polygon & padded, double outline_step = 0.05);
  ClearanceGrid & grid() {return grid_;}
  const ClearanceGrid & grid() const {return grid_;}
  void setPoints(std::vector<Point2D> pts) {points_ = std::move(pts);}
  double clearance(const Pose2D & pose) const;

private:
  Polygon footprint_;
  Polygon outline_;
  double radius_{0.0};
  ClearanceGrid grid_;
  std::vector<Point2D> points_;
};
}
