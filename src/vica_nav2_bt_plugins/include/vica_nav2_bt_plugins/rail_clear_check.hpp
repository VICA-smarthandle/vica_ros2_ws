#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "nav_msgs/msg/path.hpp"

namespace vica_nav2_bt_plugins
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};

struct CostGrid
{
  unsigned int size_x{0};
  unsigned int size_y{0};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<uint8_t> data;
};

constexpr uint8_t kLethalCost = 254;

struct RailClearVerdict
{
  bool blocked{false};
  std::size_t poses_checked{0};
  double hit_s{-1.0};
  double hit_x{0.0};
  double hit_y{0.0};
};

std::vector<Point2> parsePolygon(const std::string & text);

std::vector<Point2> padPolygon(const std::vector<Point2> & polygon, double pad);

RailClearVerdict checkRailAhead(
  const nav_msgs::msg::Path & path, double skip, const std::vector<Point2> & body,
  const CostGrid & grid);

class BlockedConfirm
{
public:
  explicit BlockedConfirm(int need = 2, double reset_gap = 2.5)
  : need_(need < 1 ? 1 : need), reset_gap_(reset_gap) {}
  bool update(bool blocked, double now)
  {
    if (last_ >= 0.0 && now - last_ > reset_gap_) {count_ = 0;}
    last_ = now;
    count_ = blocked ? count_ + 1 : 0;
    return count_ >= need_;
  }
  int count() const {return count_;}

private:
  int need_;
  double reset_gap_;
  int count_{0};
  double last_{-1.0};
};

}
