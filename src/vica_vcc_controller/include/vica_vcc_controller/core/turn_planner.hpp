#pragma once
#include <cmath>
#include <vector>
#include "vica_vcc_controller/core/clearance.hpp"
#include "vica_vcc_controller/core/types.hpp"

namespace vica_vcc_controller::core
{
enum class TurnMode { Arc, Pivot, Blocked };

struct TurnPlan
{
  TurnMode mode{TurnMode::Blocked};
  double radius{0.0};
  int direction{1};        // +1 왼쪽(반시계), -1 오른쪽
  double min_clearance{0.0};
};

struct TurnParams
{
  std::vector<double> radii{0.2, 0.1};   // 설계서 3.3: DWB 최소 ≈ 0.2, 0630 통로 1.3 m
  double clearance{0.05};                // 처음 고를 때: 호 전체를 미리 돌려 본 최소 여유가 이 이상인 첫 반지름
  // 도는 중: 지금 반지름의 남은 호 여유가 이 이상이면 바꾸지 않는다(0 이하 = clearance 와 같음).
  // 2026-10-01 run53: 매 주기 처음 기준(0.30)으로 다시 고르니 벽에 다가가며 0.4 -> 0.3 -> 제자리로
  // 떨어지고, 제자리는 그 U턴 끝까지 고정됐다. 도착 정렬 회전 원 검사도 이 값을 쓴다.
  double keep_clearance{-1.0};
  double arc_w{0.45};                    // DWB U턴 실측 0.42~0.47(devlog 09-17 §5.4)
  double pivot_w{0.35};                  // RPP rotate_to_heading_angular_vel
  double run_in_decel{0.3};
  double sample_angle{0.05};
  double both_sides_angle{170.0 * M_PI / 180.0};
};

// 로봇 좌표계에서 run_in 만큼 직진한 뒤 dir 쪽으로 반지름 radius(0 = 제자리) 호를 angle 만큼.
double simulateTurnClearance(
  double angle, double radius, int dir, double run_in, const ClearanceFn & f, double sample_angle);
// only_dir != 0 이면 그 방향으로만 계획한다(Turn 중 방향 고정, 최종 리뷰 M2).
// keep 이 있으면(도는 중) 그 방식·반지름을 keep_clearance 기준으로 먼저 검사해, 통과하면 그대로 둔다.
TurnPlan planTurn(
  double heading_error, double v_now, bool pivot_first, const ClearanceFn & f, const TurnParams & p,
  int only_dir = 0, const TurnPlan * keep = nullptr);
inline double keepClearance(const TurnParams & p) {return p.keep_clearance > 0.0 ? p.keep_clearance : p.clearance;}
Twist2D turnCommand(const TurnPlan & t, const TurnParams & p);
}  // namespace vica_vcc_controller::core
