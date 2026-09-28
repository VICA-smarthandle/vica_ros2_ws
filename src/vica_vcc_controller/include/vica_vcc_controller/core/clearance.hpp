#pragma once
#include <cstdint>
#include <functional>
#include <vector>
#include "vica_vcc_controller/core/types.hpp"

namespace vica_vcc_controller::core
{
// local costmap 의 LETHAL 칸만 모아 만든 거리장. inflation 비용은 쓰지 않는다(설계서 3.4).
class ClearanceGrid
{
public:
  static constexpr double kFarDistance = 10.0;
  void reset(double origin_x, double origin_y, double resolution, int width, int height);
  void markLethal(int ix, int iy);
  // chamfer(1, sqrt2) 2-pass 거리 변환
  void compute();
  double distanceAt(double wx, double wy) const;
  bool worldToCell(double wx, double wy, int & ix, int & iy) const;
  bool lethalCell(int ix, int iy) const;
  double resolution() const {return res_;}
  double originX() const {return ox_;}
  double originY() const {return oy_;}

private:
  size_t idx(int x, int y) const {return static_cast<size_t>(y) * w_ + x;}
  double ox_{0.0}, oy_{0.0}, res_{0.05};
  int w_{0}, h_{0};
  std::vector<uint8_t> lethal_;
  std::vector<float> dist_;
};

using ClearanceFn = std::function<double(const Pose2D &)>;

// 몸(padding 포함 footprint) 전체와 장애물 사이의 최소 여유.
class ClearanceField
{
public:
  void setFootprint(const Polygon & padded, double outline_step = 0.05);
  ClearanceGrid & grid() {return grid_;}
  const ClearanceGrid & grid() const {return grid_;}
  // 초음파 점(그리드 좌표계)
  void setPoints(std::vector<Point2D> pts) {points_ = std::move(pts);}
  // pose 는 그리드 좌표계. 음수 = 몸 안에 장애물(접촉).
  double clearance(const Pose2D & pose) const;

private:
  Polygon footprint_;
  Polygon outline_;
  double radius_{0.0};
  ClearanceGrid grid_;
  std::vector<Point2D> points_;
};
}  // namespace vica_vcc_controller::core
