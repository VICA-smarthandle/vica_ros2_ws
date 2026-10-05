// 레일 앞 조각을 몸통이 지나갈 수 있는지 판정한다. ROS 노드·TF 없이 쓸 수 있다.
//
// 2026-10-05 해결안 B. Nav2 1.1.20 의 IsPathValid 는 조각 첫 점부터, 손잡이 꼬리까지 포함한 몸 윤곽을
// 레일 위에 올려 본다. 그러면 로봇이 지금 서 있는 자리의 주변(손잡이 뒤에 선 사람, 곡선에서 옆으로 휘는
// 꼬리 옆 벽)을 "앞이 막혔다"로 읽는다(run61 막힘 68번 중 첫 접촉이 손잡이 꼬리 38번, run63 31번 중 20번).
// 그 판정이 레일을 버리고 지름길로 바꿔 코너를 각지게, 출발을 S자로 만들었다.
// 여기서는 조각 앞쪽 skip 만큼을 건너뛰고, 꼬리를 뺀 몸통이 치명 칸과 겹치는지만 본다.
// run61·63 재계산: 0.5~3 m·몸통만이면 68 -> 33, 31 -> 11. 남은 것은 모두 지도에 없던 물체(벽 0).
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "nav_msgs/msg/path.hpp"

namespace vica_nav2_bt_plugins
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};

// nav2_msgs/Costmap 의 값 그대로(0~255). 254 = 치명(LETHAL_OBSTACLE).
struct CostGrid
{
  unsigned int size_x{0};
  unsigned int size_y{0};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<uint8_t> data;
};

constexpr uint8_t kLethalCost = 254;

struct RailClearVerdict
{
  bool blocked{false};
  std::size_t poses_checked{0};
  double hit_s{-1.0};     // 막힌 곳: 조각 첫 점에서 레일을 따라 잰 거리(m)
  double hit_x{0.0};      // 막힌 칸의 지도 좌표
  double hit_y{0.0};
};

// "[[x, y], [x, y], ...]" (nav2 footprint 문자열 형식). 숫자가 짝이 안 맞거나 3점 미만이면 빈 목록.
std::vector<Point2> parsePolygon(const std::string & text);

// nav2_costmap_2d::padFootprint 와 같다: 각 꼭짓점을 바깥으로(x, y 부호 방향으로) pad 만큼.
std::vector<Point2> padPolygon(const std::vector<Point2> & polygon, double pad);

// path 의 첫 점에서 레일을 따라 skip 보다 먼 점마다 body(로봇 기준 좌표)를 그 점·그 방향에 올려,
// 다각형 안(경계 포함)의 칸 중 하나라도 치명이면 막힘. 지도 밖 칸과 미지(255)는 비어 있다고 본다.
RailClearVerdict checkRailAhead(
  const nav_msgs::msg::Path & path, double skip, const std::vector<Point2> & body,
  const CostGrid & grid);

// 막힘이 연속으로 need 번 나와야 막힘으로 인정한다(1 Hz 틱 기준 2번 = 약 1 s 이어짐).
// 틱 사이가 reset_gap 보다 벌어지면(새 주행·BT 정지) 처음부터 센다.
class BlockedConfirm
{
public:
  explicit BlockedConfirm(int need = 2, double reset_gap = 2.5)
  : need_(need < 1 ? 1 : need), reset_gap_(reset_gap) {}
  bool update(bool blocked, double now)
  {
    if (last_ >= 0.0 && now - last_ > reset_gap_) {count_ = 0;}
    last_ = now;
    count_ = blocked ? count_ + 1 : 0;
    return count_ >= need_;
  }
  int count() const {return count_;}

private:
  int need_;
  double reset_gap_;
  int count_{0};
  double last_{-1.0};
};

}  // namespace vica_nav2_bt_plugins
