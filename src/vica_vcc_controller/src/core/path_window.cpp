// Copyright (c) 2022 Samsung Research America
// Copyright (c) 2020 Shrijit Singh
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// VICA 수정(2026-09-28): nav2_controller FeasiblePathHandler(main)와 RPP 1.1.20 transformGlobalPlan 의
// 가까운 점 찾기·지나온 길 지우기·끝 2점 유지·costmap 범위 자르기를 ROS 메시지 없이 옮겼다.
// 후진 전환점(inversion)·제자리 회전 지키기(rotation)는 VCC 에서 쓰지 않아 옮기지 않았다.

#include "vica_vcc_controller/core/path_window.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vica_vcc_controller::core
{
namespace
{
double dist(const Pose2D & a, const Pose2D & b) {return std::hypot(a.x - b.x, a.y - b.y);}

// nav2_util::geometry_utils::first_after_integrated_distance 와 같은 뜻.
size_t firstAfterIntegratedDistance(const Path & path, size_t begin, double d)
{
  double acc = 0.0;
  for (size_t i = begin + 1; i < path.size(); ++i) {
    acc += dist(path[i - 1], path[i]);
    if (acc > d) {return i;}
  }
  return path.size();
}
}  // namespace

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
  // ① 끝 방향을 계산할 2점을 남긴다(FeasiblePathHandler 와 같음).
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
}  // namespace vica_vcc_controller::core
