#pragma once

#include <optional>
#include <vector>

#include "vica_docking/geometry.hpp"

namespace vica_docking
{

struct VDetection
{
  double score{0.0};
  Vec2 apex;
  Vec2 u;
  Vec2 da;
  Vec2 db;
  double inner_deg{0.0};
  double la{0.0};
  double lb{0.0};
  double res{0.0};
  int na{0};
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

  std::optional<VDetection> detect(
    const std::vector<float> & ranges, double angle_min, double angle_increment, double range_min);

  void reset() {has_last_ = false;}

private:
  VDetectorParams p_;
  RobotGeometry g_;
  bool has_last_{false};
  Vec2 last_;
};

struct DockPose
{
  double x{0.0};
  double y{0.0};
  double th{0.0};
};
DockPose pose_in_dock(const VDetection & d);

struct LineFit
{
  Vec2 c;
  Vec2 d;
  Vec2 n;
  double res{0.0};
};
LineFit fit_line(const std::vector<Vec2> & P, int lo, int hi);

}
