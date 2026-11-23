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

// 남은 레일(로봇 가까운 점 -> 끝)의 시작 방향과 끝 방향 차이(도). 방향은 0.2 m 넘게 떨어진 점으로 잰다
// (첫 구간 방향과 같은 잔떨림 회피). 남은 길이가 모자라 방향을 못 재면 0(=직선으로 본다).
double remainingTurnDeg(const nav_msgs::msg::Path & path, double robot_x, double robot_y)
{
  const auto & ps = path.poses;
  if (ps.size() < 2) {return 0.0;}
  std::size_t near = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < ps.size(); ++i) {
    const double d = std::hypot(ps[i].pose.position.x - robot_x, ps[i].pose.position.y - robot_y);
    if (d < best) {best = d; near = i;}
  }
  const auto & a = ps[near].pose.position;
  std::size_t j = near + 1;
  while (j < ps.size() && std::hypot(ps[j].pose.position.x - a.x, ps[j].pose.position.y - a.y) < 0.2) {++j;}
  const auto & z = ps.back().pose.position;
  std::size_t k = ps.size() - 1;
  while (k > near && std::hypot(z.x - ps[k].pose.position.x, z.y - ps[k].pose.position.y) < 0.2) {--k;}
  if (j >= ps.size() || k <= near || k < j) {return 0.0;}
  const double h0 = std::atan2(ps[j].pose.position.y - a.y, ps[j].pose.position.x - a.x);
  const double h1 = std::atan2(z.y - ps[k].pose.position.y, z.x - ps[k].pose.position.x);
  return std::abs(std::remainder(h1 - h0, 2.0 * M_PI)) * 180.0 / M_PI;
}
}  // namespace

RoutePathVerdict checkRoutePath(
  const nav_msgs::msg::Path & path, double robot_x, double robot_y,
  std::size_t min_poses, double max_dist,
  double goal_x, double goal_y, double handoff,
  double keep_goal_dist, double keep_min_turn_deg)
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
    // 코너 목적지는 레일을 끝까지 쓴다(헤더 keep_goal_dist 주석). 레일 위·남은 레일이 꺾일 때만.
    if (keep_goal_dist > 0.0) {
      for (std::size_t i = 1; i < poses.size(); ++i) {
        v.goal_to_path = std::min(
          v.goal_to_path, segmentDistance(
            goal_x, goal_y, poses[i - 1].pose.position.x, poses[i - 1].pose.position.y,
            poses[i].pose.position.x, poses[i].pose.position.y, 0.0));
      }
      v.remaining_turn_deg = remainingTurnDeg(path, robot_x, robot_y);
      if (v.goal_to_path <= keep_goal_dist && v.remaining_turn_deg >= keep_min_turn_deg) {
        v.kept_for_corner = true;
        v.usable = true;
        return v;
      }
    }
    v.reason = "near goal: freespace approach";
    return v;
  }
  v.usable = true;
  return v;
}

}  // namespace vica_nav2_bt_plugins
