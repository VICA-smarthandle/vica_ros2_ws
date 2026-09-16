// 레일 경로를 controller 에 넘겨도 되는지 판정한다. ROS 노드·TF 없이 쓸 수 있다.
#pragma once

#include <cstddef>
#include <limits>
#include <string>

#include "nav_msgs/msg/path.hpp"

namespace vica_nav2_bt_plugins
{

struct RoutePathVerdict
{
  bool usable{false};
  std::size_t poses{0};
  double dist_to_path{0.0};   // 로봇에서 경로의 가장 가까운 점까지(m). 점이 없으면 inf
  double dist_to_goal{0.0};   // 로봇에서 목적지까지(m). 목적지를 안 주면 inf
  std::string reason;         // 사람이 읽는 이유 (usable 이면 비어 있다)
};

// path       route_server 가 내놓은 촘촘한 경로
// robot_x/y  경로와 같은 frame 의 로봇 위치
// min_poses  점이 이보다 적으면 못 쓴다. 2 = "선이 하나는 있어야 한다"
// max_dist   로봇이 경로에서 이보다 멀면 못 쓴다. controller 의 지역 창(3 m)보다
//            작아야 한다 — 멀면 controller 가 "0 poses" 로 실패한다
// goal_x/y   목적지. NaN 이면 아래 인계 판정을 건너뛴다
// handoff    로봇이 목적지에서 이 거리 안이면 레일을 버리고 자유주행으로 넘긴다.
//            레일 경로는 노드에서 끝나고 끝 방향이 마지막 엣지 방향이라, 목적지
//            방향이 반대면 도착 뒤 제자리 180도 회전이 남는다(2026-09-16 run6 입구:
//            40초·방향 바꿈 8회). planner 는 마지막 몇 m 를 도착 방향에 맞춰 그린다.
RoutePathVerdict checkRoutePath(
  const nav_msgs::msg::Path & path, double robot_x, double robot_y,
  std::size_t min_poses, double max_dist,
  double goal_x = std::numeric_limits<double>::quiet_NaN(),
  double goal_y = std::numeric_limits<double>::quiet_NaN(),
  double handoff = 0.0);

}  // namespace vica_nav2_bt_plugins
