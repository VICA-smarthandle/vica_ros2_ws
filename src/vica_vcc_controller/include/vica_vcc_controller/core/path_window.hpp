#pragma once
#include <utility>
#include "vica_vcc_controller/core/types.hpp"

namespace vica_vcc_controller::core
{
struct PathWindowParams
{
  // 가까운 점을 경로를 따라 이 거리 안에서만 찾는다(되돌아오는 구간으로 건너뛰기 방지).
  // 기본 = local costmap 6 m 의 반폭 = 지금 RPP 와 같은 값.
  double max_robot_pose_search_dist{3.0};
  // 점 1개 경로를 거부한다. 기본 꺼짐(동작 불변). run35 도착 yaw 3/8 과 연결 — 켤지는 사용자 결정.
  bool reject_unit_path{false};
};

enum class WindowStatus { Ok, EmptyPlan, UnitPath, NoPosesInWindow };

// FeasiblePathHandler(Nav2 Jazzy+) 의 경로 창 부분을 Humble 에서 쓰려고 옮긴 것. ROS 를 모른다.
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
}  // namespace vica_vcc_controller::core
