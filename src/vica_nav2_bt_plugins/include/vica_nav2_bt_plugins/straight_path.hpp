// 로봇에서 목적지까지 곧은 경로를 만든다. ROS 노드·TF 없이 쓸 수 있다.
#pragma once

#include <string>

#include "nav_msgs/msg/path.hpp"

namespace vica_nav2_bt_plugins
{

// 로봇 (rx, ry) 에서 목적지 (gx, gy) 까지 spacing(m) 간격 점으로 곧은 경로를 만든다.
// 모든 점의 방향은 선 방향(로봇 -> 목적지)이다 — 목적지에 저장된 yaw 는 쓰지 않는다.
// 시작점(로봇 자리)과 끝점(목적지 자리)은 항상 들어가 점은 최소 2개다. 두 점이 1 mm 안이면
// 선 방향이 없으므로 robot_yaw 를 쓴다. 입력이 유한하지 않거나 spacing <= 0 이면 빈 경로.
//
// 왜: 사용자 안내(위치만 도착)의 마지막 2 m 를 lattice planner 가 그리면, planner 는 목적지의
// **저장된 yaw** 로 끝나는 길만 그린다(Humble Smac Lattice, 방향 16칸). 판정기는 방향을 안 보는데
// 길 모양이 그 방향으로 휘어 도착 직전에 틀어 들어갔다. 곧은 선에는 끝 방향 조건이 없다.
nav_msgs::msg::Path makeStraightPath(
  double rx, double ry, double robot_yaw, double gx, double gy, double spacing,
  const std::string & frame);

}  // namespace vica_nav2_bt_plugins
