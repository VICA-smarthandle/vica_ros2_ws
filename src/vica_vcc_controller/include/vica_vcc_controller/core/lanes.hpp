#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include "vica_vcc_controller/core/clearance.hpp"
#include "vica_vcc_controller/core/types.hpp"

namespace vica_vcc_controller::core
{
struct LaneParams
{
  double max_offset{0.6};        // 요구 3: 0.5~0.6 m 이탈 허용
  double step{0.1};
  double horizon{1.5};           // 요구 1: 먼 물체 무시
  double sample_step{0.1};
  double target_clearance{0.20}; // 요구 4: 15~20 cm
  double w_clear{10.0};
  double w_rail{1.0};
  double w_change{0.5};
  double switch_margin{0.05};
  int persist_cycles{3};         // 0.3 s
  double lane_rate{0.10};        // m/s — 손잡이 0.115 m/s 탈락선(backlog §11)
  double return_clear_time{1.0};
  double min_transition_speed{0.1};
  std::vector<double> shift_speeds{0.3, 0.2, 0.1};   // 옮기는 동안 달릴 속도 후보(빠른 것부터)
};

struct LaneScore
{
  double offset{0.0};
  double min_clearance{0.0};
  double score{0.0};
  bool valid{false};
  double speed{0.0};       // 이 차선으로 옮기는 동안 달릴 속도
};

class LaneSelector
{
public:
  explicit LaneSelector(LaneParams p = {}) : p_(p) {}
  void reset();
  // v = 실측 속도(옆 간격 진행), v_des = 차선 제한 전 목표 속도 min(desired, speed_cap).
  // 옮김 속도 후보는 v_des 부터 고른다 — 실측에서 시작하면 느린 채 묶인다(run48 F1).
  void update(
    const Path & path, double v, double v_des, double now, double dt,
    const ClearanceFn & clearance);
  double offset() const {return offset_;}
  // 실제 옆 위치로 d 를 다시 맞춘다(유턴 뒤·경로 교체 뒤). 목표는 그대로 — 복귀 규칙이 되돌린다.
  void syncOffset(double d) {offset_ = std::clamp(d, -p_.max_offset, p_.max_offset);}
  // 목표를 d 에 가장 가까운 차선으로 둔다(유턴을 마치고 나온 자리, run48 F2). 레일 복귀는 보통 규칙이 맡는다.
  void setTargetNearest(double d)
  {
    target_ = std::clamp(std::round(d / p_.step) * p_.step, -p_.max_offset, p_.max_offset);
    pending_count_ = 0;
  }
  // 새 경로가 로봇 자리에서 다시 그려져 d 를 다시 맞췄을 때(VccCore 재동기) 비켜 가던 목표도 같은 만큼
  // 옮겨 세계 기준 옆 위치를 지킨다. 안 옮기면 새 경로마다 목표만큼 또 밀린다(run106 11-14: 7 s 에 레일에서
  // 0.25 -> 0.85 m). 목표·대기 후보가 0(레일)이면 '경로를 따른다'는 뜻이라 그대로 둔다.
  void shiftTarget(double delta)
  {
    if (std::abs(target_) < 1e-9) {return;}
    target_ = snapLane(target_ + delta);
    if (pending_count_ > 0 && std::abs(pending_) > 1e-9) {pending_ = snapLane(pending_ + delta);}
  }
  double target() const {return target_;}
  bool blocked() const {return blocked_;}
  double currentClearance() const {return current_clearance_;}
  // 옮기는 중이고 고른 옮김 속도가 목표 속도보다 낮을 때만 묶는다(run48 F1).
  double speedCap() const
  {
    return std::abs(target_ - offset_) > 0.01 && target_speed_ < v_des_ - 1e-9 ? target_speed_ : 1e9;
  }
  const std::vector<LaneScore> & scores() const {return scores_;}
  Path lanePath(const Path & path, double v) const;

private:
  // 가장 가까운 차선. 딱 가운데면 레일 쪽.
  double snapLane(double d) const
  {
    const double k = std::floor(std::abs(d) / p_.step + 0.5 - 1e-9);
    return std::clamp(std::copysign(k * p_.step, d), -p_.max_offset, p_.max_offset);
  }
  Path candidatePath(const Path & path, double cand, double shift_speed) const;
  double minClearance(const Path & lane, const ClearanceFn & f) const;
  LaneScore evaluate(const Path & path, double v_des, double cand, const ClearanceFn & f) const;

  LaneParams p_;
  double offset_{0.0};
  double target_{0.0};
  double target_speed_{1e9};
  double v_des_{1e9};
  double pending_{0.0};
  int pending_count_{0};
  double pending_since_{-1.0};
  bool blocked_{false};
  double current_clearance_{1e9};
  std::vector<LaneScore> scores_;
};
}  // namespace vica_vcc_controller::core
