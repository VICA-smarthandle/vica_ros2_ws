#include "vica_nav2_bt_plugins/route_path_check.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vica_nav2_bt_plugins
{
namespace
{
// 점 (px,py) 에서 선분 a->b 까지 거리. back > 0 이면 a 앞쪽(b 반대쪽)으로 back 만큼 늘린 선분.
double segmentDistance(double px, double py, double ax, double ay, double bx, double by, double back)
{
  const double dx = bx - ax, dy = by - ay;
  const double len = std::hypot(dx, dy);
  if (len < 1e-9) {return std::hypot(px - ax, py - ay);}
  const double ux = dx / len, uy = dy / len;
  const double s = std::clamp((px - ax) * ux + (py - ay) * uy, -back, len);
  return std::hypot(px - (ax + s * ux), py - (ay + s * uy));
}
}  // namespace

RoutePathVerdict checkRoutePath(
  const nav_msgs::msg::Path & path, double robot_x, double robot_y,
  std::size_t min_poses, double max_dist,
  double goal_x, double goal_y, double handoff)
{
  RoutePathVerdict v;
  v.poses = path.poses.size();
  v.dist_to_path = std::numeric_limits<double>::infinity();
  v.dist_to_points = std::numeric_limits<double>::infinity();
  v.dist_to_goal = (std::isfinite(goal_x) && std::isfinite(goal_y)) ?
    std::hypot(goal_x - robot_x, goal_y - robot_y) : std::numeric_limits<double>::infinity();

  for (const auto & ps : path.poses) {
    // 2026-09-16 run6: route_server 코너 둥글리기가 좌표에 NaN 을 넣었다(34건).
    // NaN != NaN 이 항상 참이라 FollowPath BT 노드가 "경로가 바뀌었다"고 매 틱
    // 새 goal 을 보냈고 controller 가 "Aborting handle" 을 970회 냈다. 한 점이라도
    // 유한하지 않으면 경로 전체를 버린다.
    if (!std::isfinite(ps.pose.position.x) || !std::isfinite(ps.pose.position.y)) {
      v.reason = "non-finite pose";
      return v;
    }
    const double d = std::hypot(ps.pose.position.x - robot_x, ps.pose.position.y - robot_y);
    if (d < v.dist_to_points) {
      v.dist_to_points = d;
    }
  }
  // 레일 선까지 옆 거리. 점 하나뿐이면 점까지 거리 그대로.
  v.dist_to_path = v.dist_to_points;
  const auto & poses = path.poses;
  for (std::size_t i = 1; i < poses.size(); ++i) {
    v.dist_to_path = std::min(
      v.dist_to_path, segmentDistance(
        robot_x, robot_y, poses[i - 1].pose.position.x, poses[i - 1].pose.position.y,
        poses[i].pose.position.x, poses[i].pose.position.y, 0.0));
  }
  // 첫 구간 방향은 0.2 m 넘게 떨어진 첫 점으로 잡는다(경로 점 간격 0.05 m 의 잔떨림 회피, VCC lateralError 와 같다).
  if (poses.size() >= 2) {
    const auto & p0 = poses.front().pose.position;
    std::size_t k = 1;
    while (k + 1 < poses.size() &&
      std::hypot(poses[k].pose.position.x - p0.x, poses[k].pose.position.y - p0.y) < 0.2) {++k;}
    v.dist_to_path = std::min(
      v.dist_to_path, segmentDistance(
        robot_x, robot_y, p0.x, p0.y, poses[k].pose.position.x, poses[k].pose.position.y,
        kPrunedEdgeExtend));
  }

  // 2026-09-16 실주행: route_server 가 마지막 엣지에서 출발 노드를 잘라내면
  // 엣지 0개 -> 점 1개짜리 경로가 나온다(nav2_route path_converter.cpp).
  // 점 하나는 선이 아니다. controller 가 따라갈 것이 없다.
  if (v.poses < min_poses) {
    v.reason = "points<" + std::to_string(min_poses);
    return v;
  }
  // 경로는 로봇이 아니라 '가장 가까운 노드'에서 시작한다(로봇 위치는 경로에
  // 안 들어간다). 그 노드가 멀면 DWB 는 지역 창 밖이라며 "Resulting plan has
  // 0 poses" 로 실패한다. 그럴 땐 자유주행으로 레일까지 가는 게 맞다.
  if (v.dist_to_path > max_dist) {
    v.reason = "robot far from path";
    return v;
  }
  // 목적지 근처는 planner 가 도착 방향까지 맞춰 그리게 넘긴다(헤더 주석 참조).
  if (handoff > 0.0 && v.dist_to_goal < handoff) {
    v.reason = "near goal: freespace approach";
    return v;
  }
  v.usable = true;
  return v;
}

}  // namespace vica_nav2_bt_plugins
