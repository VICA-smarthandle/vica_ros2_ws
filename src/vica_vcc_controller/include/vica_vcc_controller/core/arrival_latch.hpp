#pragma once
#include "vica_vcc_controller/core/types.hpp"

namespace vica_vcc_controller::core
{
struct ArrivalParams
{
  double xy_tol{0.15};
  double yaw_tol{0.25};
  double trans_stopped{0.03};
  double rot_stopped{0.05};
  double unlatch_dist{0.5};
};

class ArrivalLatch
{
public:
  explicit ArrivalLatch(ArrivalParams p = {}) : p_(p) {}
  bool check(const Pose2D & robot, const Pose2D & goal, double vx, double vy, double w);
  void clear() {latched_ = false;}
  bool latched() const {return latched_;}
  const ArrivalParams & params() const {return p_;}

private:
  ArrivalParams p_;
  bool latched_{false};
  double gx_{0.0};
  double gy_{0.0};
};
}
