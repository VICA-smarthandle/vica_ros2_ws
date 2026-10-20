#pragma once
#include <utility>
#include "vica_vcc_controller/core/types.hpp"

namespace vica_vcc_controller::core
{
struct PathWindowParams
{
  double max_robot_pose_search_dist{3.0};
  bool reject_unit_path{false};
};

enum class WindowStatus { Ok, EmptyPlan, UnitPath, NoPosesInWindow };

class PathWindow
{
public:
  explicit PathWindow(PathWindowParams p = {}) : p_(p) {}
  void setPlan(Path plan) {plan_ = std::move(plan);}
  bool empty() const {return plan_.empty();}
  const Path & plan() const {return plan_;}
  WindowStatus window(const Pose2D & robot, double max_extent, Path & out);

private:
  PathWindowParams p_;
  Path plan_;
};

Pose2D toRobotFrame(const Pose2D & robot, const Pose2D & p);
}
