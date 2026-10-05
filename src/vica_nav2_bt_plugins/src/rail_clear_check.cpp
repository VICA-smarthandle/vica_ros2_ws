#include "vica_nav2_bt_plugins/rail_clear_check.hpp"

#include <algorithm>
#include <cmath>
#include <regex>

namespace vica_nav2_bt_plugins
{
std::vector<Point2> parsePolygon(const std::string & text)
{
  static const std::regex number(R"([-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?)");
  std::vector<double> v;
  for (auto it = std::sregex_iterator(text.begin(), text.end(), number);
    it != std::sregex_iterator(); ++it)
  {
    v.push_back(std::stod(it->str()));
  }
  std::vector<Point2> out;
  if (v.size() % 2 != 0 || v.size() < 6) {return out;}
  for (std::size_t i = 0; i < v.size(); i += 2) {out.push_back({v[i], v[i + 1]});}
  return out;
}

std::vector<Point2> padPolygon(const std::vector<Point2> & polygon, double pad)
{
  std::vector<Point2> out = polygon;
  for (auto & p : out) {
    p.x += std::copysign(pad, p.x);
    p.y += std::copysign(pad, p.y);
  }
  return out;
}

namespace
{
bool inside(const std::vector<Point2> & poly, double x, double y)
{
  bool in = false;
  for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const auto & a = poly[i];
    const auto & b = poly[j];
    if ((a.y > y) != (b.y > y) && x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x) {in = !in;}
  }
  return in;
}

// 점과 선분 사이 거리 — 경계 위 칸을 안쪽으로 치기 위해 쓴다(칸 중심이 경계 밖 반 칸 안이면 겹친 것).
double segDist(double px, double py, const Point2 & a, const Point2 & b)
{
  const double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
  double t = l2 > 0.0 ? ((px - a.x) * dx + (py - a.y) * dy) / l2 : 0.0;
  t = std::clamp(t, 0.0, 1.0);
  return std::hypot(px - (a.x + t * dx), py - (a.y + t * dy));
}

bool overlaps(const std::vector<Point2> & poly, double x, double y, double half_cell)
{
  if (inside(poly, x, y)) {return true;}
  for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    if (segDist(x, y, poly[j], poly[i]) <= half_cell) {return true;}
  }
  return false;
}
}  // namespace

RailClearVerdict checkRailAhead(
  const nav_msgs::msg::Path & path, double skip, const std::vector<Point2> & body,
  const CostGrid & grid)
{
  RailClearVerdict v;
  const auto & poses = path.poses;
  if (body.size() < 3 || poses.size() < 2 || grid.size_x == 0 || grid.size_y == 0 ||
    grid.data.size() < static_cast<std::size_t>(grid.size_x) * grid.size_y || grid.resolution <= 0.0)
  {
    return v;
  }
  double reach = 0.0;   // 몸통 꼭짓점 중 원점에서 가장 먼 거리 — 칸 훑을 범위
  for (const auto & p : body) {reach = std::max(reach, std::hypot(p.x, p.y));}
  const double res = grid.resolution;
  const int r = static_cast<int>(std::ceil(reach / res)) + 1;

  double s = 0.0;
  for (std::size_t i = 0; i < poses.size(); ++i) {
    const auto & p = poses[i].pose.position;
    if (i > 0) {
      const auto & q = poses[i - 1].pose.position;
      s += std::hypot(p.x - q.x, p.y - q.y);
    }
    if (s < skip) {continue;}
    // 방향: 다음 점(끝이면 앞 점)으로. 경로 점 방향(orientation)은 노드 방향이라 코너에서 어긋난다.
    const std::size_t a = i + 1 < poses.size() ? i : i - 1;
    const auto & pa = poses[a].pose.position;
    const auto & pb = poses[a + 1].pose.position;
    const double th = std::atan2(pb.y - pa.y, pb.x - pa.x);
    const double c = std::cos(th), sn = std::sin(th);
    std::vector<Point2> world;
    world.reserve(body.size());
    for (const auto & b : body) {world.push_back({p.x + c * b.x - sn * b.y, p.y + sn * b.x + c * b.y});}
    ++v.poses_checked;

    const int cx = static_cast<int>(std::floor((p.x - grid.origin_x) / res));
    const int cy = static_cast<int>(std::floor((p.y - grid.origin_y) / res));
    for (int my = cy - r; my <= cy + r; ++my) {
      if (my < 0 || my >= static_cast<int>(grid.size_y)) {continue;}
      for (int mx = cx - r; mx <= cx + r; ++mx) {
        if (mx < 0 || mx >= static_cast<int>(grid.size_x)) {continue;}
        if (grid.data[static_cast<std::size_t>(my) * grid.size_x + mx] != kLethalCost) {continue;}
        const double wx = grid.origin_x + (mx + 0.5) * res;
        const double wy = grid.origin_y + (my + 0.5) * res;
        if (overlaps(world, wx, wy, 0.5 * res)) {
          v.blocked = true;
          v.hit_s = s;
          v.hit_x = wx;
          v.hit_y = wy;
          return v;
        }
      }
    }
  }
  return v;
}
}  // namespace vica_nav2_bt_plugins
