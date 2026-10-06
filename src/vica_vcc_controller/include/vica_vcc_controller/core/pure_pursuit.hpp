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
// 경로 끝 연장(2026-10-05 해결안 가). 경로가 다 L 안에 들어오면 carrotOnPath 는 끝점을 돌려준다 — 끝점이
// 코앞(0.05 m)이면 옆 4 cm 어긋남이 조준각 39°·회전 명령 최대가 되어, 도착 직전 엉뚱한 쪽으로 꺾은 회전이
// 도착 정렬을 반대로 시작하게 했다(run62~64 도착 16번 중 9번, 1~17°). 대신 경로 마지막 모양 방향(끝에서
// geom_back 앞 점 -> 끝점)으로 곧게 이은 선 위, 원점에서 L 인 점을 조준한다. 멈추는 자리는 바뀌지 않는다
// (조준점은 꺾는 방향만 정한다). 아래 안전장치 중 하나라도 걸리면 예전처럼 끝점:
//   - 경로 길이 < min_length 또는 점 2개 미만(짧은 고리 경로는 마지막 모양이 엉뚱하다)
//   - 끝점이 로봇 뒤(x <= 0) — 지나친 뒤에 더 앞으로 가지 않게
//   - 로봇이 연장선에서 옆으로 max_lateral 넘게 떨어짐 — 부드럽게 꺾으면 끝점 옆을 지나칠 수 있다
// extended 에 연장을 썼는지 돌려준다.
Point2D carrotWithEndExtension(
  const Path & path, double L, double max_lateral, double min_length, bool * extended = nullptr);
// 조준점이 놓인 구간의 방향. 레일 자체가 로봇과 얼마나 어긋났는지 본다(유턴 판정).
double carrotTangent(const Path & path, double L);
// 원점에서 조준점을 지나는 원호의 곡률(RPP 와 같은 식).
double curvatureTo(const Point2D & carrot);
// 경로를 왼쪽(+)으로 평행이동한다. 시작은 d_start, transition_len 뒤부터 d_end, 그 사이는 일정 기울기.
// (기울기 일정 = 옆 이동 속도 일정 -> 손잡이 좌우 속도 상한을 그대로 지킨다)
Path offsetPath(const Path & path, double d_start, double d_end, double transition_len);
}  // namespace vica_vcc_controller::core
