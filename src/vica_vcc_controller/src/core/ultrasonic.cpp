#include "vica_vcc_controller/core/ultrasonic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vica_vcc_controller::core
{
namespace
{
bool noEcho(const RangeReading & r) {return r.range >= r.max_range - 1e-3;}
bool isObstacle(const RangeReading & r)
{
  return std::isfinite(r.range) && r.range > 0.0 && r.range >= r.min_range - 1e-9 && !noEcho(r);
}
double distanceToArc(const RangeReading & r, const Point2D & p)
{
  const double dx = p.x - r.sensor.x;
  const double dy = p.y - r.sensor.y;
  const double d = std::hypot(dx, dy);
  const double half = std::max(0.0, r.fov) / 2.0;
  if (d < 1e-9) {return r.range;}
  const double bearing = std::remainder(std::atan2(dy, dx) - r.sensor.yaw, 2.0 * M_PI);
  if (std::fabs(bearing) <= half) {return std::fabs(d - r.range);}
  double best = std::numeric_limits<double>::infinity();
  for (const double a : {-half, half}) {
    const Point2D e = toParent(r.sensor, Point2D{r.range * std::cos(a), r.range * std::sin(a)});
    best = std::min(best, std::hypot(p.x - e.x, p.y - e.y));
  }
  return best;
}
}

std::optional<RangeReading> UltrasonicChannel::confirmed(
  double now, double max_age, int count, double tol) const
{
  if (!hist_.empty()) {
    const RangeReading & last = hist_.back();
    if (now - last.recv_time <= max_age && isObstacle(last) && last.range < p_.near_confirm_range) {
      return last;
    }
  }
  if (count < 1 || static_cast<int>(hist_.size()) < count) {return std::nullopt;}
  const size_t first = hist_.size() - static_cast<size_t>(count);
  for (size_t i = first; i < hist_.size(); ++i) {
    const RangeReading & r = hist_[i];
    if (now - r.recv_time > max_age) {return std::nullopt;}
    if (!isObstacle(r)) {return std::nullopt;}
  }
  for (size_t i = first + 1; i < hist_.size(); ++i) {
    const RangeReading & newer = hist_[i];
    const Point2D c = toParent(newer.sensor, Point2D{newer.range, 0.0});
    if (distanceToArc(hist_[i - 1], c) > tol) {return std::nullopt;}
  }
  return hist_.back();
}

void UltrasonicChannel::push(const RangeReading & in)
{
  RangeReading r = in;
  if (r.range > 0.0 && r.range < r.min_range) {r.range = r.min_range;}
  hist_.push_back(r);
  while (hist_.size() > 4) {hist_.pop_front();}

  if (p_.pass_memory) {
    pushPass(r, confirmed(r.recv_time, p_.max_age, p_.confirm_count, p_.confirm_tol).has_value());
    return;
  }
  if (confirmed(r.recv_time, p_.max_age, p_.confirm_count, p_.confirm_tol)) {
    memory_ = r;
    clear_count_ = 0;
    return;
  }
  if (!memory_) {return;}
  const Point2D m = toParent(memory_->sensor, Point2D{memory_->range, 0.0});
  const double d = std::hypot(m.x - r.sensor.x, m.y - r.sensor.y);
  const bool beyond = noEcho(r) || (isObstacle(r) && r.range > d + p_.confirm_tol);
  if (!beyond) {clear_count_ = 0; return;}
  if (++clear_count_ >= 2) {
    memory_.reset();
    clear_count_ = 0;
  }
}

void UltrasonicChannel::pushPass(const RangeReading & r, bool is_confirmed)
{
  const double half = std::max(0.0, r.fov) / 2.0;
  for (auto it = memos_.begin(); it != memos_.end(); ) {
    const double dx = it->c.x - r.sensor.x, dy = it->c.y - r.sensor.y;
    const double d = std::hypot(dx, dy);
    const double bearing = std::remainder(std::atan2(dy, dx) - r.sensor.yaw, 2.0 * M_PI);
    const bool in_beam = d > 1e-6 && d <= r.max_range && std::fabs(bearing) <= half;
    if (in_beam) {
      const bool beyond = noEcho(r) || (isObstacle(r) && r.range > d + p_.confirm_tol);
      if (beyond) {
        if (++it->clear >= 2) {it = memos_.erase(it); continue;}
      } else {
        it->clear = 0;
      }
    }
    ++it;
  }
  if (!is_confirmed) {return;}
  const Point2D c = toParent(r.sensor, Point2D{r.range, 0.0});
  for (auto & m : memos_) {
    if (std::hypot(m.c.x - c.x, m.c.y - c.y) <= p_.confirm_tol) {m = Memo{r, c, 0}; return;}
  }
  memos_.push_back(Memo{r, c, 0});
  while (static_cast<int>(memos_.size()) > std::max(1, p_.memory_slots)) {memos_.erase(memos_.begin());}
}

void UltrasonicChannel::forget(const Pose2D & robot, double rear_x)
{
  if (!p_.pass_memory) {return;}
  const double c = std::cos(robot.yaw), s = std::sin(robot.yaw);
  memos_.erase(
    std::remove_if(
      memos_.begin(), memos_.end(), [&](const Memo & m) {
        const double dx = m.c.x - robot.x, dy = m.c.y - robot.y;
        const double lx = c * dx + s * dy;
        return lx < rear_x - p_.pass_margin || std::hypot(dx, dy) > p_.pass_max_dist;
      }),
    memos_.end());
}

std::vector<RangeReading> UltrasonicChannel::obstacles(double now) const
{
  if (p_.pass_memory) {
    std::vector<RangeReading> out;
    const auto cf = confirmed(now, p_.max_age, p_.confirm_count, p_.confirm_tol);
    if (cf) {out.push_back(*cf);}
    for (const auto & m : memos_) {
      if (now - m.r.recv_time > p_.memory_time) {continue;}
      if (cf && cf->recv_time == m.r.recv_time) {continue;}
      out.push_back(m.r);
    }
    return out;
  }
  std::vector<RangeReading> out;
  const auto c = confirmed(now, p_.max_age, p_.confirm_count, p_.confirm_tol);
  if (c) {out.push_back(*c);}
  if (memory_ && now - memory_->recv_time <= p_.memory_time &&
    (!c || c->recv_time != memory_->recv_time))
  {
    out.push_back(*memory_);
  }
  return out;
}

RangeReading capRange(RangeReading r, double cap)
{
  if (cap > 0.0 && std::isfinite(r.range) && r.range > cap) {r.range = r.max_range;}
  return r;
}

std::vector<Point2D> rangeToArcPoints(double range, double fov, int n)
{
  std::vector<Point2D> pts;
  if (n < 1) {return pts;}
  for (int i = 0; i < n; ++i) {
    const double a = n == 1 ? 0.0 : -fov / 2.0 + fov * i / (n - 1);
    pts.push_back({range * std::cos(a), range * std::sin(a)});
  }
  return pts;
}
std::vector<Point2D> readingToArcPoints(const RangeReading & r, int n)
{
  std::vector<Point2D> pts = rangeToArcPoints(r.range, r.fov, n);
  for (auto & p : pts) {p = toParent(r.sensor, p);}
  return pts;
}
}
