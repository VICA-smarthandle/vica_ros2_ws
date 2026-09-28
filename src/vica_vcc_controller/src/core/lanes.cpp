#include "vica_vcc_controller/core/lanes.hpp"

#include <algorithm>
#include <cmath>
#include "vica_vcc_controller/core/pure_pursuit.hpp"

namespace vica_vcc_controller::core
{
namespace
{
bool same(double a, double b) {return std::abs(a - b) < 1e-6;}
}

void LaneSelector::reset()
{
  offset_ = target_ = pending_ = 0.0;
  target_speed_ = 1e9;
  pending_count_ = 0;
  pending_since_ = -1.0;
  blocked_ = false;
  current_clearance_ = 1e9;
  scores_.clear();
}

Path LaneSelector::candidatePath(const Path & path, double cand, double shift_speed) const
{
  // 옆 이동 속도 = lane_rate 가 되도록 옮김 거리를 정한다: 거리 = |옮길 양| x 달릴 속도 / lane_rate
  const double trans = std::abs(cand - offset_) * shift_speed / p_.lane_rate;
  return offsetPath(path, offset_, cand, trans);
}

Path LaneSelector::lanePath(const Path & path, double v) const
{
  const double sp = std::min(std::max(std::abs(v), p_.min_transition_speed), target_speed_);
  return candidatePath(path, target_, sp);
}

double LaneSelector::minClearance(const Path & lane, const ClearanceFn & f) const
{
  const Path lp = pathPrefix(lane, p_.horizon);
  double c = 1e9, last_s = -1e9, s = 0.0;
  for (size_t i = 0; i < lp.size(); ++i) {
    if (i > 0) {s += std::hypot(lp[i].x - lp[i - 1].x, lp[i].y - lp[i - 1].y);}
    if (i + 1 < lp.size() && s - last_s < p_.sample_step) {continue;}
    last_s = s;
    c = std::min(c, f(lp[i]));
  }
  return c;
}

LaneScore LaneSelector::evaluate(
  const Path & path, double v, double cand, const ClearanceFn & f) const
{
  const double v_ref = std::max(std::abs(v), p_.min_transition_speed);
  std::vector<double> speeds{v_ref};
  for (double sp : p_.shift_speeds) {
    if (sp < v_ref - 1e-9) {speeds.push_back(sp);}
  }
  LaneScore sc;
  sc.offset = cand;
  bool have = false;
  for (double sp : speeds) {
    const double c = minClearance(candidatePath(path, cand, sp), f);
    if (!have || c > sc.min_clearance + 1e-9) {sc.min_clearance = c; sc.speed = sp; have = true;}
    // 20 cm 가 나오는 가장 빠른 속도에서 멈춘다. 옮길 게 없으면 속도와 무관하다.
    if (c >= p_.target_clearance || same(cand, offset_)) {break;}
  }
  sc.valid = sc.min_clearance >= 0.0;
  sc.score = p_.w_clear * std::max(0.0, p_.target_clearance - sc.min_clearance) +
    p_.w_rail * std::abs(cand) + p_.w_change * std::abs(cand - target_);
  return sc;
}

void LaneSelector::update(
  const Path & path, double v, double now, double dt, const ClearanceFn & f)
{
  scores_.clear();
  const int n = static_cast<int>(std::round(p_.max_offset / p_.step));
  // 동점이면 먼저 나온 쪽 — 오른쪽(음수)부터 채점해 우측 통행을 고른다.
  for (int i = -n; i <= n; ++i) {scores_.push_back(evaluate(path, v, i * p_.step, f));}

  const LaneScore * best = nullptr;
  const LaneScore * cur = nullptr;
  for (const auto & s : scores_) {
    if (same(s.offset, target_)) {cur = &s;}
    if (!s.valid) {continue;}
    if (!best || s.score < best->score - 1e-9 ||
      (std::abs(s.score - best->score) <= 1e-9 && std::abs(s.offset) < std::abs(best->offset)))
    {
      best = &s;
    }
  }

  blocked_ = best == nullptr;
  if (!blocked_) {
    if (!cur || !cur->valid) {
      // 지금 차선이 막혔다 — 기다리지 않고 바로 옮긴다(안전).
      target_ = best->offset;
      pending_count_ = 0;
    } else if (!same(best->offset, target_) && best->score < cur->score - p_.switch_margin) {
      if (same(pending_, best->offset) && pending_count_ > 0) {
        ++pending_count_;
      } else {
        pending_ = best->offset;
        pending_count_ = 1;
        pending_since_ = now;
      }
      const bool inward = std::abs(best->offset) < std::abs(target_);
      const bool ready = inward ?
        (now - pending_since_ >= p_.return_clear_time - 1e-9) :
        (pending_count_ >= p_.persist_cycles);
      if (ready) {
        target_ = best->offset;
        pending_count_ = 0;
      }
    } else {
      pending_count_ = 0;
    }
  }

  current_clearance_ = 1e9;
  for (const auto & s : scores_) {
    if (same(s.offset, target_)) {current_clearance_ = s.min_clearance; target_speed_ = s.speed;}
  }

  // 옆 간격은 달린 거리만큼 나아간다(기울기 lane_rate / 옮김 속도). 옆 이동 속도는 lane_rate 를 넘지 않는다.
  const double slope = p_.lane_rate / std::max(target_speed_, 1e-3);
  const double step = std::min(slope * std::abs(v) * dt, p_.lane_rate * dt);
  offset_ += std::clamp(target_ - offset_, -step, step);
}
}  // namespace vica_vcc_controller::core
