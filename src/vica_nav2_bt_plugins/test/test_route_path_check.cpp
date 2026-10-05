#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "vica_nav2_bt_plugins/route_path_check.hpp"

using vica_nav2_bt_plugins::checkRoutePath;

static nav_msgs::msg::Path line(double x0, double x1, double y, int n)
{
  nav_msgs::msg::Path p;
  for (int i = 0; i < n; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = x0 + (x1 - x0) * i / (n - 1);
    ps.pose.position.y = y;
    p.poses.push_back(ps);
  }
  return p;
}

TEST(RoutePathCheck, EmptyPathIsUnusable)
{
  nav_msgs::msg::Path p;
  auto v = checkRoutePath(p, 0, 0, 2, 1.5);
  EXPECT_FALSE(v.usable);
  EXPECT_EQ(v.poses, 0u);
}

// 2026-09-16 run5 재현: 노드 12 (3.55, 5.41) 한 점, 로봇은 5 m 밖 (-1.41, 5.62).
// 기성 IsPathValid 는 이걸 통과시켰고 controller 가 "zero length" 로 섰다.
TEST(RoutePathCheck, SinglePoseIsUnusableEvenIfFree)
{
  nav_msgs::msg::Path p;
  geometry_msgs::msg::PoseStamped ps;
  ps.pose.position.x = 3.55;
  ps.pose.position.y = 5.41;
  p.poses.push_back(ps);
  auto v = checkRoutePath(p, -1.408, 5.620, 2, 1.5);
  EXPECT_FALSE(v.usable);
  EXPECT_EQ(v.poses, 1u);
  EXPECT_NE(v.reason.find("points"), std::string::npos);
}

TEST(RoutePathCheck, TwoPosesNearRobotIsUsable)
{
  auto p = line(0.0, 1.0, 0.0, 2);
  auto v = checkRoutePath(p, 0.1, 0.2, 2, 1.5);
  EXPECT_TRUE(v.usable);
  EXPECT_NEAR(v.dist_to_path, 0.2, 1e-9);      // 선분까지 옆 거리
  EXPECT_NEAR(v.dist_to_points, 0.2236, 1e-3);  // 가장 가까운 점(0,0)까지
}

// 경로는 가장 가까운 '노드'에서 시작한다. 로봇이 레일에서 멀면 DWB 지역 창(3 m)
// 밖이라 "0 poses" 로 실패한다. 그땐 자유주행이 맞다.
TEST(RoutePathCheck, RobotFarFromPathIsUnusable)
{
  auto p = line(5.0, 8.0, 0.0, 61);
  auto v = checkRoutePath(p, 0.0, 0.0, 2, 1.5);
  EXPECT_FALSE(v.usable);
  // 로봇은 레일 연장선 위 5 m 뒤다. 첫 구간을 뒤로 늘리는 것은 지워진 엣지 길이(1.2 m)까지뿐.
  EXPECT_NEAR(v.dist_to_path, 5.0 - vica_nav2_bt_plugins::kPrunedEdgeExtend, 1e-9);
  EXPECT_NEAR(v.dist_to_points, 5.0, 1e-9);
  EXPECT_NE(v.reason.find("far"), std::string::npos);
}

// 2026-10-02 run61 시작→홈 6.8 s 재현(축만 돌림): 유턴 호로 레일 옆 0.72 m 에 있는데 route_server 가
// 로봇이 지나친 첫 노드를 지워 경로가 0.8 m 앞 노드에서 시작했다. 점까지 1.08 m 라 0.8 문턱에 걸려
// 지름길로 바뀌었고 로봇은 반대로 77° 돌았다(S자). 레일 선까지 옆 거리로 재면 레일을 계속 쓴다.
TEST(RoutePathCheck, PrunedStartNodeIsMeasuredSideways)
{
  auto p = line(0.8, 3.8, 0.0, 61);
  auto v = checkRoutePath(p, 0.0, -0.72, 2, 0.8);
  EXPECT_TRUE(v.usable);
  EXPECT_NEAR(v.dist_to_path, 0.72, 1e-9);
  EXPECT_NEAR(v.dist_to_points, std::hypot(0.8, 0.72), 1e-9);
}

// 경로 점이 성겨도(노드 두 개) 그 사이 옆에 있으면 옆 거리다.
TEST(RoutePathCheck, SidewaysFromTheMiddleOfASegment)
{
  auto p = line(0.0, 2.0, 0.0, 2);
  auto v = checkRoutePath(p, 1.0, 0.3, 2, 0.8);
  EXPECT_TRUE(v.usable);
  EXPECT_NEAR(v.dist_to_path, 0.3, 1e-9);
}

// 진짜로 레일에서 멀면(옆 1.0 m) 예전처럼 못 쓴다 — 문턱은 그대로다.
TEST(RoutePathCheck, TrulyFarSidewaysIsStillUnusable)
{
  auto p = line(0.8, 3.8, 0.0, 61);
  auto v = checkRoutePath(p, 1.5, 1.0, 2, 0.8);
  EXPECT_FALSE(v.usable);
  EXPECT_NEAR(v.dist_to_path, 1.0, 1e-9);
}

// 첫 구간 방향은 0.2 m 넘게 떨어진 점으로 잡는다. 처음 몇 점이 옆으로 흔들려도 늘린 선이 휘지 않는다.
TEST(RoutePathCheck, FirstSegmentDirectionIgnoresTinyJitter)
{
  auto p = line(1.0, 4.0, 0.0, 61);
  p.poses[1].pose.position.y = 0.03;   // 0.05 m 간격 잔떨림
  auto v = checkRoutePath(p, 0.0, -0.5, 2, 0.8);
  EXPECT_TRUE(v.usable);
  EXPECT_NEAR(v.dist_to_path, 0.5, 1e-9);
}

// 레일 옆 30 cm 로 벗어나 달리는 중 — 흔한 상태. 써야 한다.
TEST(RoutePathCheck, SlightlyOffTheRailIsUsable)
{
  auto p = line(-3.0, 3.0, 0.0, 121);
  auto v = checkRoutePath(p, 0.0, 0.3, 2, 1.5);
  EXPECT_TRUE(v.usable);
  EXPECT_NEAR(v.dist_to_path, 0.3, 1e-9);
}

TEST(RoutePathCheck, MinPosesIsHonored)
{
  auto p = line(0.0, 1.0, 0.0, 3);
  EXPECT_TRUE(checkRoutePath(p, 0.0, 0.0, 3, 1.5).usable);
  EXPECT_FALSE(checkRoutePath(p, 0.0, 0.0, 4, 1.5).usable);
}

// 2026-09-16 run6: route_server 가 코너 둥글리기에서 NaN 좌표를 넣었다(안내소 구간,
// 34건). NaN 경로가 controller 로 가면 FollowPath 가 매 틱 새 goal 을 보내
// "Aborting handle" 970회. 한 점이라도 NaN 이면 버린다.
TEST(RoutePathCheck, NonFinitePoseIsUnusable)
{
  auto p = line(0.0, 3.0, 0.0, 61);
  p.poses[30].pose.position.x = std::numeric_limits<double>::quiet_NaN();
  auto v = checkRoutePath(p, 0.0, 0.1, 2, 1.5);
  EXPECT_FALSE(v.usable);
  EXPECT_NE(v.reason.find("non-finite"), std::string::npos);
}

// 목적지 2 m 안에서는 레일을 버리고 planner 가 도착 방향까지 맞춰 그리게 한다.
// run6 입구: 레일은 남향으로 들어오고 목적지는 북향(90도) -> 제자리 180도 회전 40초.
TEST(RoutePathCheck, NearGoalHandsOffToFreespace)
{
  auto p = line(0.0, 3.0, 0.0, 61);
  // 목적지 (3,0), 로봇 (1.5,0.1): 1.5 m 남음 -> 인계
  auto v = checkRoutePath(p, 1.5, 0.1, 2, 1.5, 3.0, 0.0, 2.0);
  EXPECT_FALSE(v.usable);
  EXPECT_NEAR(v.dist_to_goal, 1.5033, 1e-3);
  EXPECT_NE(v.reason.find("near goal"), std::string::npos);
  // 로봇 (0,0.1): 3 m 남음 -> 레일 유지
  EXPECT_TRUE(checkRoutePath(p, 0.0, 0.1, 2, 1.5, 3.0, 0.0, 2.0).usable);
  // 목적지를 안 주면(NaN) 인계 판정을 건너뛴다
  EXPECT_TRUE(checkRoutePath(p, 1.5, 0.1, 2, 1.5).usable);
  // handoff 0 이면 끈다
  EXPECT_TRUE(checkRoutePath(p, 1.5, 0.1, 2, 1.5, 3.0, 0.0, 0.0).usable);
}

using vica_nav2_bt_plugins::RailDistanceGate;

TEST(RailDistanceGate, OneThresholdUntilLeavingThenRejoinThreshold)
{
  // run52~54: 0.75~0.9 m 에서 문턱 하나(0.8)면 1 Hz 로 레일·자유주행이 번갈아 갔다.
  RailDistanceGate g;
  EXPECT_DOUBLE_EQ(g.threshold(0.8, 0.5), 0.8);
  g.update(0.75, g.threshold(0.8, 0.5), 10.0, 0.0);   // 안쪽 — 그대로
  EXPECT_FALSE(g.far());
  g.update(0.85, g.threshold(0.8, 0.5), 10.0, 0.0);   // 0.8 밖으로 나감
  EXPECT_TRUE(g.far());
  EXPECT_DOUBLE_EQ(g.threshold(0.8, 0.5), 0.5);
  g.update(0.75, g.threshold(0.8, 0.5), 10.0, 0.0);   // 0.75 는 아직 0.5 밖 — 계속 밖
  EXPECT_TRUE(g.far());
  g.update(0.45, g.threshold(0.8, 0.5), 10.0, 0.0);   // 0.5 안 — 돌아옴
  EXPECT_FALSE(g.far());
  EXPECT_DOUBLE_EQ(g.threshold(0.8, 0.5), 0.8);
}

TEST(RailDistanceGate, DisabledWhenRejoinNotSetOrNotSmaller)
{
  RailDistanceGate g;
  g.update(1.0, 0.8, 10.0, 0.0);
  ASSERT_TRUE(g.far());
  EXPECT_DOUBLE_EQ(g.threshold(0.8, -1.0), 0.8);
  EXPECT_DOUBLE_EQ(g.threshold(0.8, 0.9), 0.8);
}

TEST(RailDistanceGate, NewGoalStartsFresh)
{
  RailDistanceGate g;
  g.update(1.0, 0.8, 10.0, 0.0);
  ASSERT_TRUE(g.far());
  g.update(std::numeric_limits<double>::infinity(), 0.5, 20.0, 5.0);   // 다른 목적지, 거리 모름
  EXPECT_FALSE(g.far());
}
