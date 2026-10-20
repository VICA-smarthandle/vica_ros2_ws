#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include "vica_vcc_controller/core/clearance.hpp"
#include "vica_vcc_controller/core/types.hpp"

namespace vica_vcc_controller::core
{
struct LaneParams
{
  double max_offset{0.6};
  double step{0.1};
  double horizon{1.5};
  double sample_step{0.1};
  double target_clearance{0.20};
  double w_clear{10.0};
  double w_rail{1.0};
  double w_change{0.5};
  double switch_margin{0.05};
  int persist_cycles{3};
  double lane_rate{0.10};
  double return_clear_time{1.0};
  double min_transition_speed{0.1};
  std::vector<double> shift_speeds{0.3, 0.2, 0.1};
};

struct LaneScore
{
  double offset{0.0};
  double min_clearance{0.0};
  double score{0.0};
  bool valid{false};
  double speed{0.0};
};

class LaneSelector
{
public:
  explicit LaneSelector(LaneParams p = {}) : p_(p) {}
  void reset();
  void update(
    const Path & path, double v, double v_des, double now, double dt,
    const ClearanceFn & clearance);
  double offset() const {return offset_;}
  void syncOffset(double d) {offset_ = std::clamp(d, -p_.max_offset, p_.max_offset);}
  void setTargetNearest(double d)
  {
    target_ = std::clamp(std::round(d / p_.step) * p_.step, -p_.max_offset, p_.max_offset);
    pending_count_ = 0;
  }
  double target() const {return target_;}
  bool blocked() const {return blocked_;}
  double currentClearance() const {return current_clearance_;}
  double speedCap() const
  {
    return std::abs(target_ - offset_) > 0.01 && target_speed_ < v_des_ - 1e-9 ? target_speed_ : 1e9;
  }
  const std::vector<LaneScore> & scores() const {return scores_;}
  Path lanePath(const Path & path, double v) const;

private:
  Path candidatePath(const Path & path, double cand, double shift_speed) const;
  double minClearance(const Path & lane, const ClearanceFn & f) const;
  LaneScore evaluate(const Path & path, double v_des, double cand, const ClearanceFn & f) const;

  LaneParams p_;
  double offset_{0.0};
  double target_{0.0};
  double target_speed_{1e9};
  double v_des_{1e9};
  double pending_{0.0};
  int pending_count_{0};
  double pending_since_{-1.0};
  bool blocked_{false};
  double current_clearance_{1e9};
  std::vector<LaneScore> scores_;
};
}
