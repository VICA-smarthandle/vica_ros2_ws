#pragma once
#include <deque>
#include <optional>
#include <vector>
#include "vica_vcc_controller/core/types.hpp"

namespace vica_vcc_controller::core
{
struct RangeReading
{
  double range{0.0};
  double min_range{0.0};
  double max_range{0.0};
  double fov{0.0};
  double recv_time{0.0};
  Pose2D sensor;
};

struct UltrasonicParams
{
  double max_age{1.0};
  int confirm_count{2};
  double confirm_tol{0.15};
  double near_confirm_range{0.40};
  double memory_time{1.5};
  bool pass_memory{false};
  double pass_margin{0.2};
  double pass_max_dist{2.0};
  int memory_slots{4};
};

class UltrasonicChannel
{
public:
  explicit UltrasonicChannel(UltrasonicParams p = {}) : p_(p) {}
  void push(const RangeReading & r);
  std::optional<RangeReading> confirmed(double now, double max_age, int count, double tol) const;
  std::vector<RangeReading> obstacles(double now) const;
  void forget(const Pose2D & robot, double rear_x);
  size_t memoryCount() const {return p_.pass_memory ? memos_.size() : (memory_ ? 1u : 0u);}
  bool fresh(double now, double max_age) const
  {
    return !hist_.empty() && now - hist_.back().recv_time <= max_age;
  }

private:
  UltrasonicParams p_;
  std::deque<RangeReading> hist_;
  std::optional<RangeReading> memory_;
  int clear_count_{0};
  struct Memo
  {
    RangeReading r;
    Point2D c;
    int clear{0};
  };
  std::vector<Memo> memos_;
  void pushPass(const RangeReading & r, bool is_confirmed);
};

RangeReading capRange(RangeReading r, double cap);

std::vector<Point2D> rangeToArcPoints(double range, double fov, int n);
std::vector<Point2D> readingToArcPoints(const RangeReading & r, int n);
}
