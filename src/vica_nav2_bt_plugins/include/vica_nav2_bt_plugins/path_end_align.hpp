// 목적지까지 그린 경로의 마지막 점 방향을 목적지 방향으로 맞춘다. ROS 노드·TF 없이 쓸 수 있다.
#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace vica_nav2_bt_plugins
{

// 경로 끝점이 goal 에서 max_dist 안이면 끝점의 **방향만** goal 방향으로 바꾼다(위치는 그대로).
// 바꿨으면 true. 경로가 비었거나, 끝점이 멀거나, 두 frame 이 서로 다르면 손대지 않는다.
//
// 왜: run49(2026-09-28) 경로 30여 개 중 끝점이 목적지와 어긋난 것은 2개뿐이었지만(0.13~0.14 m,
// 방향 +27°·−90°), 그 −90° 하나가 409호 도착 정렬 교착(17 s) 한가운데 끼어 있었다. 로봇이 목적지
// 0.25 m 언저리에 있을 때 lattice planner 가 목적지 자세를 못 그리고 tolerance(0.2) 안 다른 자세로
// 끝낸 것으로 본다. controller 는 경로 끝점 자세로 도착을 판정하므로 그 방향이 곧 도착 방향이 된다.
bool alignPathEndToGoal(
  nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & goal, double max_dist);

}  // namespace vica_nav2_bt_plugins
