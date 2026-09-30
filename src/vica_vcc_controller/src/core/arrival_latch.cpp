#include "vica_vcc_controller/core/arrival_latch.hpp"

#include <cmath>

namespace vica_vcc_controller::core
{
bool ArrivalLatch::check(const Pose2D & robot, const Pose2D & goal, double vx, double vy, double w)
{
  const double d = std::hypot(robot.x - goal.x, robot.y - goal.y);
  if (latched_ &&
    (std::hypot(goal.x - gx_, goal.y - gy_) > p_.unlatch_dist || d > p_.unlatch_dist))
  {
    latched_ = false;
  }
  if (!latched_) {
    if (d > p_.xy_tol) {return false;}
    latched_ = true;
    gx_ = goal.x;
    gy_ = goal.y;
  }
  if (std::abs(normalizeAngle(robot.yaw - goal.yaw)) > p_.yaw_tol) {return false;}
  // StoppedGoalChecker 와 같은 정지 조건: 회전 |w|, 병진 hypot(vx, vy).
  if (std::abs(w) > p_.rot_stopped) {return false;}
  return std::hypot(vx, vy) <= p_.trans_stopped;
}
}  // namespace vica_vcc_controller::core
