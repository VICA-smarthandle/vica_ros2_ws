// 레일 경로를 controller 에 넘겨도 되는지 판정한다. ROS 노드·TF 없이 쓸 수 있다.
#pragma once

#include <cmath>
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
  // 로봇에서 레일 '선'까지 옆 거리(m). 점이 없으면 inf. 2026-10-05: 점까지가 아니라 선분까지,
  // 그리고 첫 구간은 뒤로 kPrunedEdgeExtend 만큼 늘려 잰다 — 아래 checkRoutePath 주석.
  double dist_to_path{0.0};
  double dist_to_points{0.0}; // 예전 방식(가장 가까운 점까지). 로그 비교용
  double dist_to_goal{0.0};   // 로봇에서 목적지까지(m). 목적지를 안 주면 inf
  std::string reason;         // 사람이 읽는 이유 (usable 이면 비어 있다)
};

// route_server 가 지운 출발 노드 자리까지 덮는 길이(m). 레일 노드 간격은 1.0 m 이하다
// (생성기 1.0, 앱 레일 0.12~0.98). 무한히 늘리면 레일 연장선 위 멀리 뒤에 선 로봇도 '가깝다'가 된다.
constexpr double kPrunedEdgeExtend = 1.2;

// path       route_server 가 내놓은 촘촘한 경로
// robot_x/y  경로와 같은 frame 의 로봇 위치
// min_poses  점이 이보다 적으면 못 쓴다. 2 = "선이 하나는 있어야 한다"
// max_dist   로봇이 경로에서 이보다 멀면 못 쓴다. controller 의 지역 창(3 m)보다
//            작아야 한다 — 멀면 controller 가 "0 poses" 로 실패한다.
//            거리는 레일 선까지 옆 거리다. route_server 는 로봇이 첫 노드를 지나쳤으면(dot>0)
//            옆으로 얼마나 떨어졌든(max_prune_dist_from_edge 8 m) 그 노드를 지운다. 그러면 남은
//            '점'까지 거리는 옆 거리에 앞뒤 간격이 더해져 튄다(2026-10-02 run61 시작→홈: 옆 0.72 m 인데
//            1.01 m 로 재 0.8 m 문턱을 넘어 지름길로 바뀌고 반대로 77° 회전. 문턱 닫힘 14번 전부 이 착시).
//            그래서 선분까지 재고, 첫 구간은 지워진 엣지 자리까지 뒤로 늘려 잰다.
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

// 레일 거리 문턱 두 개(히스테리시스). 2026-10-01 run52~54: 로봇이 레일에서 0.75~0.9 m 에 있으면
// 안쪽 문(0.8 m)이 1 Hz 로 열렸다 닫혔다 해 레일 경로와 자유주행 경로가 번갈아 controller 에 갔다
// (robot far from path 25~32회, VCC 차선 d 0 <-> 0.6 점프 2.9~6.9회/분, 코너 급정지 1회).
// 한 번 max_dist 밖으로 나가면 rejoin_dist 안으로 들어와야 다시 통과시킨다.
class RailDistanceGate
{
public:
  // 이번 판정에 쓸 거리 문턱. rejoin_dist 가 0 이하이거나 max_dist 이상이면 문턱 하나(종전 동작).
  double threshold(double max_dist, double rejoin_dist) const
  {
    return (far_ && rejoin_dist > 0.0 && rejoin_dist < max_dist) ? rejoin_dist : max_dist;
  }
  // 판정 뒤 상태 갱신. 목적지가 0.5 m 넘게 바뀌면 새 주행으로 보고 처음부터.
  void update(double dist_to_path, double threshold, double goal_x, double goal_y)
  {
    if (std::isfinite(goal_x) && std::isfinite(goal_y)) {
      if (!std::isfinite(goal_x_) || std::hypot(goal_x - goal_x_, goal_y - goal_y_) > 0.5) {
        far_ = false;
      }
      goal_x_ = goal_x;
      goal_y_ = goal_y;
    }
    if (std::isfinite(dist_to_path)) {far_ = dist_to_path > threshold;}
  }
  bool far() const {return far_;}

private:
  bool far_{false};
  double goal_x_{std::numeric_limits<double>::quiet_NaN()};
  double goal_y_{std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace vica_nav2_bt_plugins
