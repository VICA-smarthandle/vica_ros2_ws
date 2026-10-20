#include "vica_vcc_controller/core/path_window.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vica_vcc_controller::core
{
namespace
{
double dist(const Pose2D & a, const Pose2D & b) {return std::hypot(a.x - b.x, a.y - b.y);}

size_t firstAfterIntegratedDistance(const Path & path, size_t begin, double d)
{
  double acc = 0.0;
  for (size_t i = begin + 1; i < path.size(); ++i) {
    acc += dist(path[i - 1], path[i]);
    if (acc > d) {return i;}
  }
  return path.size();
}
}

WindowStatus PathWindow::window(const Pose2D & robot, double max_extent, Path & out)
{
  out.clear();
  if (plan_.empty()) {return WindowStatus::EmptyPlan;}
  if (p_.reject_unit_path && plan_.size() == 1) {return WindowStatus::UnitPath;}

  const double search = p_.max_robot_pose_search_dist > 0.0 ?
    p_.max_robot_pose_search_dist : std::numeric_limits<double>::max();
  const size_t upper = std::max<size_t>(1, firstAfterIntegratedDistance(plan_, 0, search));
  size_t closest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < upper && i < plan_.size(); ++i) {
    const double d = dist(robot, plan_[i]);
    if (d < best) {best = d; closest = i;}
  }
  if (plan_.size() > 1 && closest == plan_.size() - 1) {closest = plan_.size() - 2;}

  for (size_t i = closest; i < plan_.size(); ++i) {
    if (dist(robot, plan_[i]) > max_extent) {break;}
    out.push_back(plan_[i]);
  }
  plan_.erase(plan_.begin(), plan_.begin() + static_cast<long>(closest));
  return out.empty() ? WindowStatus::NoPosesInWindow : WindowStatus::Ok;
}

Pose2D toRobotFrame(const Pose2D & robot, const Pose2D & p)
{
  const Point2D q = toChild(robot, Point2D{p.x, p.y});
  return {q.x, q.y, normalizeAngle(p.yaw - robot.yaw)};
}
}
