#pragma once
#include "vica_vcc_controller/core/types.hpp"

namespace vica_vcc_controller::core
{
struct LookaheadParams
{
  double time{2.5};      // nav2_params RPP lookahead_time
  double min_dist{0.6};  // run39
  double max_dist{1.2};  // run36
};

double lookaheadDistance(double v, const LookaheadParams & p);
double pathLength(const Path & path);
// 앞에서부터 length 를 막 넘는 점까지.
Path pathPrefix(const Path & path, double length);
// 로봇 좌표계(로봇 = 원점) 경로에서 거리 L 인 조준점. 없으면 마지막 점.
Point2D carrotOnPath(const Path & path, double L);
// 조준점이 놓인 구간의 방향. 레일 자체가 로봇과 얼마나 어긋났는지 본다(유턴 판정).
double carrotTangent(const Path & path, double L);
// 원점에서 조준점을 지나는 원호의 곡률(RPP 와 같은 식).
double curvatureTo(const Point2D & carrot);
// 경로를 왼쪽(+)으로 평행이동한다. 시작은 d_start, transition_len 뒤부터 d_end, 그 사이는 일정 기울기.
// (기울기 일정 = 옆 이동 속도 일정 -> 손잡이 좌우 속도 상한을 그대로 지킨다)
Path offsetPath(const Path & path, double d_start, double d_end, double transition_len);
}  // namespace vica_vcc_controller::core
