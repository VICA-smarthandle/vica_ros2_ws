#include "vica_user_guidance_cpp/rail_turn_forecast.hpp"

#include <algorithm>
#include <cmath>

#include "vica_user_guidance_cpp/guidance.hpp"
#include "vica_user_guidance_cpp/pymath.hpp"

namespace vica_guidance
{
namespace
{

double dist(const Point & a, const Point & b) {return py_dist(a.first, a.second, b.first, b.second);}

// 파이썬 max(a, b) / min(a, b): 뒤 값이 엄격히 클(작을) 때만 뒤 값 — NaN 이 끼면 앞 값이 남는다.
double pymax(double a, double b) {return b > a ? b : a;}
double pymin(double a, double b) {return b < a ? b : a;}

// _resample: 경로를 step_m 간격으로 다시 찍는다. [(s, (x, y)), ...]
std::vector<std::pair<double, Point>> resample(const std::vector<Point> & path_xy, double step_m)
{
  std::vector<std::pair<double, Point>> out;
  if (path_xy.empty()) {return out;}
  out.emplace_back(0.0, path_xy[0]);
  double traveled = 0.0;
  double next_mark = step_m;
  Point prev = path_xy[0];
  for (std::size_t i = 1; i < path_xy.size(); ++i) {
    const Point & cur = path_xy[i];
    const double seg = dist(prev, cur);
    if (seg <= 0.0) {continue;}
    while (traveled + seg >= next_mark) {
      const double r = (next_mark - traveled) / seg;
      out.emplace_back(
        next_mark, Point{prev.first + (cur.first - prev.first) * r,
          prev.second + (cur.second - prev.second) * r});
      next_mark += step_m;
    }
    traveled += seg;
    prev = cur;
  }
  if (traveled - out.back().first > 1e-6) {out.emplace_back(traveled, prev);}
  return out;
}

struct Run
{
  int sign;
  double acc;
  double start_s;
  Point start_xy;
  double end_s;
  Point end_xy;
};

}  // namespace

std::vector<double> cumulative_lengths(const std::vector<Point> & path_xy)
{
  std::vector<double> out{0.0};
  for (std::size_t i = 1; i < path_xy.size(); ++i) {
    out.push_back(out.back() + dist(path_xy[i - 1], path_xy[i]));
  }
  return out;
}

std::optional<RailPose> project_to_path(
  const std::vector<Point> & path_xy, Point robot_xy, const std::vector<double> & cum,
  std::optional<double> s_hint, double behind_start_m, double hint_back_m, double hint_ahead_m)
{
  if (path_xy.size() < 2) {return std::nullopt;}
  bool have = false;
  double best_d = 0.0, best_s = 0.0, best_h = 0.0;
  for (std::size_t i = 0; i + 1 < path_xy.size(); ++i) {
    if (s_hint && (cum[i + 1] < *s_hint - hint_back_m || cum[i] > *s_hint + hint_ahead_m)) {
      continue;
    }
    const double x0 = path_xy[i].first, y0 = path_xy[i].second;
    const double x1 = path_xy[i + 1].first, y1 = path_xy[i + 1].second;
    const double dx = x1 - x0, dy = y1 - y0;
    const double seg2 = dx * dx + dy * dy;
    const double raw = seg2 <= 0.0 ? 0.0 :
      ((robot_xy.first - x0) * dx + (robot_xy.second - y0) * dy) / seg2;
    const double seg = py_sqrt(seg2);
    double d, s;
    if (cum[i] <= 0.0 && seg > 0.0 && raw < 0.0 && -raw * seg <= behind_start_m) {
      // 첫 점 뒤: 연장선까지 옆 거리, s 는 음수.
      d = std::fabs((robot_xy.first - x0) * dy - (robot_xy.second - y0) * dx) / seg;
      s = raw * seg;
    } else {
      const double t = pymax(0.0, pymin(1.0, raw));
      const double px = x0 + dx * t, py = y0 + dy * t;
      d = py_hypot(robot_xy.first - px, robot_xy.second - py);
      s = cum[i] + t * seg;
    }
    if (!have || d < best_d) {
      have = true;
      best_d = d;
      best_s = s;
      best_h = py_atan2(dy, dx);
    }
  }
  if (!have) {return std::nullopt;}
  RailPose pose;
  pose.s = best_s;
  pose.offtrack_m = best_d;
  pose.remaining_m = cum.back() - best_s;
  pose.heading_rad = best_h;
  return pose;
}

std::vector<RailCorner> find_corners(
  const std::vector<Point> & path_xy, double threshold_deg, double step_m, double deadband_deg,
  double max_gap_m, double merge_gap_m)
{
  const auto pts = resample(path_xy, step_m);
  struct Head
  {
    double s;
    Point xy;
    double h;
  };
  std::vector<Head> heads;
  for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
    const double s0 = pts[i].first;
    const double x0 = pts[i].second.first, y0 = pts[i].second.second;
    const double x1 = pts[i + 1].second.first, y1 = pts[i + 1].second.second;
    if (py_hypot(x1 - x0, y1 - y0) > 1e-9) {
      heads.push_back({s0, {x0, y0}, py_atan2(y1 - y0, x1 - x0)});
    }
  }
  const double deadband = py_radians(deadband_deg);
  const double threshold = py_radians(threshold_deg);

  std::vector<Run> runs;
  int sign = 0;
  double acc = 0.0, start_s = 0.0, end_s = 0.0;
  Point start_xy{0.0, 0.0}, end_xy{0.0, 0.0};
  std::optional<double> last_turn_s;
  auto close = [&]() {
      if (sign != 0) {runs.push_back({sign, acc, start_s, start_xy, end_s, end_xy});}
    };
  for (std::size_t i = 0; i + 1 < heads.size(); ++i) {
    const double s_b = heads[i + 1].s;
    const Point xy_b = heads[i + 1].xy;
    const double d = normalize_angle(heads[i + 1].h - heads[i].h);
    if (std::fabs(d) <= deadband) {continue;}
    const int cur = d > 0 ? 1 : -1;
    if (cur == sign && last_turn_s && s_b - *last_turn_s <= max_gap_m) {
      acc += d;
    } else {
      close();
      sign = cur;
      acc = d;
      start_s = s_b;
      start_xy = xy_b;
    }
    end_s = s_b;
    end_xy = xy_b;
    last_turn_s = s_b;
  }
  close();

  std::vector<Run> merged;
  for (const auto & r : runs) {
    if (!merged.empty() && merged.back().sign == r.sign &&
      r.start_s - merged.back().end_s <= merge_gap_m)
    {
      merged.back().acc += r.acc;
      merged.back().end_s = r.end_s;
      merged.back().end_xy = r.end_xy;
    } else {
      merged.push_back(r);
    }
  }
  std::vector<RailCorner> out;
  for (const auto & m : merged) {
    if (!(std::fabs(m.acc) >= threshold)) {continue;}
    RailCorner c;
    c.direction = m.acc > 0 ? DIRECTION_LEFT : DIRECTION_RIGHT;
    c.angle_deg = py_degrees(m.acc);
    c.start_s = m.start_s;
    c.end_s = m.end_s;
    c.start_xy = m.start_xy;
    c.end_xy = m.end_xy;
    out.push_back(c);
  }
  return out;
}

RailTurnArbiter::RailTurnArbiter(const ArbiterParams & p)
: p_(p) {}

void RailTurnArbiter::reset_goal()
{
  announced_.clear();
  active_ = Active{};
}

bool RailTurnArbiter::known(const Point & xy) const
{
  for (const auto & a : announced_) {
    if (dist(xy, a) <= same_corner_m) {return true;}
  }
  return false;
}

void RailTurnArbiter::remember(const Point & xy)
{
  if (!known(xy)) {announced_.push_back(xy);}
}

std::optional<RailCorner> RailTurnArbiter::find(
  const std::vector<RailCorner> & corners, const std::optional<Point> & xy) const
{
  if (!xy) {return std::nullopt;}
  for (const auto & c : corners) {
    if (dist(c.start_xy, *xy) <= same_corner_m) {return c;}
  }
  return std::nullopt;
}

std::optional<RailCorner> RailTurnArbiter::refind(
  const std::vector<RailCorner> & corners, const std::optional<RailPose> & pose, int direction,
  const std::optional<Point> & end_xy) const
{
  // 붙잡은 코너를 시작점으로 못 찾을 때, 끝점이 같은 같은 방향 코너(잘린 레일의 그 코너 나머지).
  if (p_.rehold_end_m <= 0.0 || !pose || !end_xy) {return std::nullopt;}
  if (pose->remaining_m <= p_.handoff_m) {return std::nullopt;}   // 목적지 2 m 안은 다시 안 잡는다
  for (const auto & c : corners) {
    if (c.direction == direction && dist(c.end_xy, *end_xy) <= p_.rehold_end_m &&
      pose->s <= c.end_s + pass_margin_m)
    {
      return c;
    }
  }
  return std::nullopt;
}

double RailTurnArbiter::lookahead(double speed) const
{
  return pymax(
    p_.controller_lookahead_min_m,
    pymin(p_.controller_lookahead_max_m, speed * p_.controller_lookahead_time_sec));
}

bool RailTurnArbiter::minor_swerve(const TurnDecision & decision, double speed_mps) const
{
  // 달리는 중의 작은 회전(차선 옮김)이라 아직 신호를 켜지 않을 것인가.
  if (p_.minor_turn_deg <= 0.0 || speed_mps < p_.minor_turn_min_speed_mps) {return false;}
  const double angle = decision.turn_angle_deg;
  return !std::isnan(angle) && std::fabs(angle) < p_.minor_turn_deg;
}

std::optional<RailCorner> RailTurnArbiter::claim(
  const std::vector<RailCorner> & corners, const std::optional<RailPose> & pose, int direction)
{
  // 예고 없이 시작된 실제 회전이 레일 코너였다면 그 코너에 묶고 '알린 것'으로 친다(규칙 ③).
  if (!pose || pose->offtrack_m > p_.max_offtrack_m) {return std::nullopt;}
  if (pose->remaining_m <= p_.handoff_m) {return std::nullopt;}   // 목적지 2 m 안 코너는 묶지 않는다
  for (const auto & c : corners) {
    if (c.direction != direction) {continue;}
    if ((c.start_s <= pose->s && pose->s <= c.end_s) ||
      (pose->s - consume_back_m <= c.start_s && c.start_s <= pose->s + consume_ahead_m))
    {
      remember(c.start_xy);
      return c;
    }
  }
  return std::nullopt;
}

RailCue RailTurnArbiter::resolve(
  const TurnDecision & decision, const std::vector<RailCorner> & corners,
  const std::optional<RailPose> & pose, double speed_mps, bool rail_fresh,
  std::optional<double> robot_yaw_rad)
{
  const double nan = NAN;
  if (decision.source_stale) {
    active_ = Active{};
    return {DIRECTION_NONE, PHASE_IDLE, nan, nan, seq_, true, "stale"};
  }

  // 켜진 신호를 붙잡을 때는 레일이 잠깐 끊겨도 마지막 레일로 코너 끝을 잰다. 새 예고만 신선한 레일을 요구한다.
  const bool near_rail = pose && pose->offtrack_m <= p_.max_offtrack_m;
  const bool on_rail = rail_fresh && near_rail;
  Active & a = active_;

  // ── 실제 회전(사실) ──
  if (decision.phase == PHASE_NOW) {
    if (a.phase != PHASE_IDLE && a.direction == decision.direction) {
      const char * reason = a.phase == PHASE_PREPARE ? "now_inherit" : "now";
      a.phase = PHASE_NOW;   // ① 이어받기: 번호 그대로
      return {decision.direction, PHASE_NOW, nan, decision.turn_angle_deg, seq_, false, reason};
    }
    const auto c = claim(corners, pose, decision.direction);
    if (c || !minor_swerve(decision, speed_mps)) {
      seq_ += 1;   // 예고 없던(또는 반대) 회전
      Active fresh;
      fresh.phase = PHASE_NOW;
      fresh.direction = decision.direction;
      if (c) {
        fresh.corner_xy = c->start_xy;
        fresh.corner_end_xy = c->end_xy;
      }
      active_ = fresh;
      return {decision.direction, PHASE_NOW, nan, decision.turn_angle_deg, seq_, false, "now_new"};
    }
    // 달리며 비키는 작은 회전: 새 신호를 켜지 않고 아래(켜진 신호 붙잡기·새 예고)로 간다.
  }

  // ── 켜진 신호 붙잡기(② 코너 끝까지 · 도는 중) ──
  if (a.phase != PHASE_IDLE) {
    std::optional<RailCorner> corner = near_rail ? find(corners, a.corner_xy) : std::nullopt;
    if (!corner && near_rail && a.corner_xy) {
      corner = refind(corners, pose, a.direction, a.corner_end_xy);
      if (corner) {   // 다시 잘린 레일의 같은 코너
        a.corner_xy = corner->start_xy;
        remember(corner->start_xy);
      }
    }
    if (corner) {a.corner_end_xy = corner->end_xy;}
    const bool in_corner = corner && pose->s <= corner->end_s + pass_margin_m;
    const int sign = a.direction == DIRECTION_LEFT ? 1 : -1;
    const bool turning = !std::isnan(decision.turn_angle_deg) &&
      decision.turn_angle_deg * sign >= p_.still_turning_deg;
    if (in_corner || turning) {
      const double d = (corner && a.phase == PHASE_PREPARE) ?
        pymax(0.0, corner->start_s - pose->s) : nan;
      return {a.direction, a.phase, d, corner ? corner->angle_deg : decision.turn_angle_deg, seq_,
        false, a.phase == PHASE_PREPARE ? "prepare_hold" : "now_hold"};
    }
    // 끝나는 바로 그 tick 에 같은 방향 다음 코너 예고가 설 차례면 끄지 않고 넘겨준다(50 ms 틈 방지).
    const auto nxt = next_prepare(corners, pose, speed_mps, on_rail, robot_yaw_rad);
    if (nxt && nxt->direction == a.direction) {return start_prepare(*nxt, *pose, "prepare_chain");}
    const bool was_prepare = a.phase == PHASE_PREPARE;
    active_ = Active{};
    if (was_prepare) {
      return {DIRECTION_NONE, PHASE_CANCELED, nan, nan, seq_, false,
        corner ? "prepare_passed" : "prepare_off_rail"};
    }
    return {DIRECTION_NONE, PHASE_COMPLETE, nan, decision.turn_angle_deg, seq_, false, "complete"};
  }

  // ── 새 예고 ──
  const auto nxt = next_prepare(corners, pose, speed_mps, on_rail, robot_yaw_rad);
  if (nxt) {return start_prepare(*nxt, *pose, "prepare");}

  return {DIRECTION_NONE, PHASE_IDLE, nan, 0.0, seq_, false, "idle"};
}

std::optional<RailCorner> RailTurnArbiter::next_prepare(
  const std::vector<RailCorner> & corners, const std::optional<RailPose> & pose, double speed_mps,
  bool on_rail, std::optional<double> robot_yaw_rad) const
{
  if (!on_rail || pose->remaining_m <= p_.handoff_m) {return std::nullopt;}
  if (speed_mps < min_moving_speed_mps) {return std::nullopt;}
  if (robot_yaw_rad && !std::isnan(pose->heading_rad) &&
    std::fabs(normalize_angle(*robot_yaw_rad - pose->heading_rad)) >
    py_radians(max_heading_error_deg))
  {
    return std::nullopt;
  }
  // 목적지 2 m 안은 BT 가 레일을 버리고 planner 로 그린다 — 그 안에 걸친 코너는 알리지 않는다.
  const double rail_end_s = pose->s + pose->remaining_m - p_.handoff_m;
  const double v = pymax(speed_mps, min_speed_mps);
  const double body_turn_ahead = lookahead(speed_mps);
  // 아직 코너 한가운데면 다음 코너를 예고하지 않는다(반대 방향 예고 방지).
  const double near_ahead = pymax(body_turn_ahead, p_.min_prepare_dist_m);
  for (const auto & c : corners) {
    if (c.start_s - near_ahead < pose->s && pose->s < c.end_s + pass_margin_m) {
      return std::nullopt;
    }
  }
  for (const auto & c : corners) {
    if (c.end_s + pass_margin_m < pose->s) {continue;}   // 이미 지난 코너
    if (known(c.start_xy)) {continue;}                   // ③ 알린 코너
    if ((c.start_s - body_turn_ahead - pose->s) / v > p_.lead_time_sec) {
      return std::nullopt;                               // 가장 가까운 새 코너가 아직 멀다
    }
    if (c.start_s - pose->s < pymax(body_turn_ahead, p_.min_prepare_dist_m)) {
      continue;   // 이미 조준거리 안 — 늦은 예고는 하지 않고 사후 판정에 맡긴다
    }
    if (c.end_s > rail_end_s) {return std::nullopt;}
    return c;
  }
  return std::nullopt;
}

RailCue RailTurnArbiter::start_prepare(
  const RailCorner & c, const RailPose & pose, const std::string & reason)
{
  seq_ += 1;
  Active fresh;
  fresh.phase = PHASE_PREPARE;
  fresh.direction = c.direction;
  fresh.corner_xy = c.start_xy;
  fresh.corner_end_xy = c.end_xy;
  active_ = fresh;
  remember(c.start_xy);
  return {c.direction, PHASE_PREPARE, pymax(0.0, c.start_s - pose.s), c.angle_deg, seq_, false,
    reason};
}

}  // namespace vica_guidance
