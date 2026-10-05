#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "vica_nav2_bt_plugins/rail_clear_check.hpp"

using vica_nav2_bt_plugins::BlockedConfirm;
using vica_nav2_bt_plugins::checkRailAhead;
using vica_nav2_bt_plugins::CostGrid;
using vica_nav2_bt_plugins::padPolygon;
using vica_nav2_bt_plugins::parsePolygon;
using vica_nav2_bt_plugins::Point2;

// nav2_params.yaml global_costmap 과 같은 값. 몸통 = 손잡이 꼬리(-0.459 -> -0.569)를 뺀 사각형.
static const char * kFull =
  "[[0.151, 0.225], [0.151, -0.225], [-0.459, -0.225], [-0.569, -0.035], [-0.569, 0.035], [-0.459, 0.225]]";
static const char * kBody = "[[0.151, 0.225], [0.151, -0.225], [-0.459, -0.225], [-0.459, 0.225]]";

static nav_msgs::msg::Path straight(double x0, double x1)
{
  nav_msgs::msg::Path p;
  const int n = static_cast<int>(std::round((x1 - x0) / 0.05)) + 1;
  for (int i = 0; i < n; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = x0 + 0.05 * i;
    p.poses.push_back(ps);
  }
  return p;
}

static CostGrid emptyGrid()
{
  CostGrid g;
  g.size_x = 160;
  g.size_y = 80;
  g.resolution = 0.05;
  g.origin_x = -3.0;
  g.origin_y = -2.0;
  g.data.assign(g.size_x * g.size_y, 0);
  return g;
}

static void mark(CostGrid & g, double x, double y, uint8_t cost = 254)
{
  const int mx = static_cast<int>(std::floor((x - g.origin_x) / g.resolution));
  const int my = static_cast<int>(std::floor((y - g.origin_y) / g.resolution));
  g.data[my * g.size_x + mx] = cost;
}

static std::vector<Point2> body() {return padPolygon(parsePolygon(kBody), 0.05);}

TEST(RailClearCheck, ParsesNav2FootprintString)
{
  auto f = parsePolygon(kFull);
  ASSERT_EQ(f.size(), 6u);
  EXPECT_DOUBLE_EQ(f[3].x, -0.569);
  EXPECT_DOUBLE_EQ(f[3].y, -0.035);
  EXPECT_TRUE(parsePolygon("[[0.1, 0.2], [0.3]]").empty());   // 짝이 안 맞음
  EXPECT_TRUE(parsePolygon("").empty());
}

TEST(RailClearCheck, PaddingPushesEveryCornerOutward)
{
  auto b = body();
  ASSERT_EQ(b.size(), 4u);
  EXPECT_NEAR(b[0].x, 0.201, 1e-12);
  EXPECT_NEAR(b[0].y, 0.275, 1e-12);
  EXPECT_NEAR(b[2].x, -0.509, 1e-12);
  EXPECT_NEAR(b[2].y, -0.275, 1e-12);
}

// run61·63 의 주된 거짓 막힘: 로봇이 서 있는 자리에서 손잡이 뒤에 선 사람.
// 예전(조각 첫 점부터·꼬리 포함)은 막힘, 새 검사(0.5 m 부터·몸통만)는 비어 있음.
TEST(RailClearCheck, PersonBehindTheHandleIsNotTheRailAhead)
{
  auto g = emptyGrid();
  mark(g, -0.65, 0.0);   // 손잡이 끝(-0.619) 바로 뒤
  auto rail = straight(0.0, 3.0);
  EXPECT_TRUE(checkRailAhead(rail, 0.0, padPolygon(parsePolygon(kFull), 0.05), g).blocked);
  EXPECT_FALSE(checkRailAhead(rail, 0.5, body(), g).blocked);
}

// 로봇 몸 뒤쪽 절반 옆(지금 자리)의 물체도 앞길이 아니다.
TEST(RailClearCheck, ObstacleBesideTheRearOfTheRobotIsIgnored)
{
  auto g = emptyGrid();
  mark(g, -0.3, 0.26);
  EXPECT_FALSE(checkRailAhead(straight(0.0, 3.0), 0.5, body(), g).blocked);
}

// 레일 위 2 m 앞 사람은 막힘이다. 몸통 앞(+0.201)이 닿는 자리에서 걸린다.
TEST(RailClearCheck, ObstacleOnTheRailAheadIsBlocked)
{
  auto g = emptyGrid();
  mark(g, 2.0, 0.0);
  auto v = checkRailAhead(straight(0.0, 3.0), 0.5, body(), g);
  ASSERT_TRUE(v.blocked);
  EXPECT_NEAR(v.hit_s, 2.0 - 0.201, 0.08);
  EXPECT_NEAR(v.hit_x, 2.025, 1e-9);
}

// 바로 앞(0.3 m) 물체는 건너뛴 0.5 m 안이지만, 0.5 m 지점 몸통(-0.009 ~ +0.701)이 덮으므로 막힘이다.
TEST(RailClearCheck, ObstacleRightInFrontIsStillBlocked)
{
  auto g = emptyGrid();
  mark(g, 0.3, 0.0);
  EXPECT_TRUE(checkRailAhead(straight(0.0, 3.0), 0.5, body(), g).blocked);
}

// 옆 여유: 몸통 반폭 0.225 + 패딩 0.05 = 0.275. 칸 중심 0.325 는 비고, 0.275 는 닿는다.
TEST(RailClearCheck, SideClearanceFollowsPaddedHalfWidth)
{
  auto g1 = emptyGrid();
  mark(g1, 1.5, 0.33);
  EXPECT_FALSE(checkRailAhead(straight(0.0, 3.0), 0.5, body(), g1).blocked);
  auto g2 = emptyGrid();
  mark(g2, 1.5, 0.26);
  EXPECT_TRUE(checkRailAhead(straight(0.0, 3.0), 0.5, body(), g2).blocked);
}

// 치명(254)만 막힘이다. 내접(253)·미지(255)는 몸통이 실제로 겹친 것이 아니다.
TEST(RailClearCheck, OnlyLethalBlocks)
{
  auto g = emptyGrid();
  mark(g, 1.5, 0.0, 253);
  mark(g, 2.0, 0.0, 255);
  EXPECT_FALSE(checkRailAhead(straight(0.0, 3.0), 0.5, body(), g).blocked);
}

// 몸통은 경로 방향으로 돈다. 90° 꺾인 레일(위로)에서 꺾인 뒤 옆 물체를 경로 방향 기준으로 본다.
TEST(RailClearCheck, BodyTurnsWithThePath)
{
  nav_msgs::msg::Path p = straight(0.0, 1.0);
  for (int i = 1; i <= 40; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = 1.0;
    ps.pose.position.y = 0.05 * i;
    p.poses.push_back(ps);
  }
  auto g = emptyGrid();
  mark(g, 1.4, 1.5);   // 꺾인 뒤 진행 방향(+y)의 오른쪽 0.4 m — 반폭 밖
  EXPECT_FALSE(checkRailAhead(p, 0.5, body(), g).blocked);
  auto g2 = emptyGrid();
  mark(g2, 1.0, 1.9);  // 꺾인 뒤 정면
  EXPECT_TRUE(checkRailAhead(p, 0.5, body(), g2).blocked);
}

TEST(RailClearCheck, EmptyInputsAreNotBlocked)
{
  auto g = emptyGrid();
  EXPECT_FALSE(checkRailAhead(nav_msgs::msg::Path(), 0.5, body(), g).blocked);
  EXPECT_FALSE(checkRailAhead(straight(0.0, 3.0), 0.5, {}, g).blocked);
}

TEST(BlockedConfirm, NeedsConsecutiveBlockedTicks)
{
  BlockedConfirm c(2, 2.5);
  EXPECT_FALSE(c.update(true, 0.0));
  EXPECT_TRUE(c.update(true, 1.0));
  EXPECT_FALSE(c.update(false, 2.0));   // 한 번 비면 처음부터
  EXPECT_FALSE(c.update(true, 3.0));
  EXPECT_TRUE(c.update(true, 4.0));
}

TEST(BlockedConfirm, LongGapStartsOver)
{
  BlockedConfirm c(2, 2.5);
  EXPECT_FALSE(c.update(true, 0.0));
  EXPECT_FALSE(c.update(true, 10.0));   // 새 주행 — 앞 막힘을 잇지 않는다
  EXPECT_TRUE(c.update(true, 11.0));
}

TEST(BlockedConfirm, OneMeansImmediate)
{
  BlockedConfirm c(1, 2.5);
  EXPECT_TRUE(c.update(true, 0.0));
}
