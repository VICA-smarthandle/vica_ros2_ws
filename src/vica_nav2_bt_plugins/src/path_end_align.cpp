#include "vica_nav2_bt_plugins/path_end_align.hpp"

#include <algorithm>
#include <cmath>

namespace vica_nav2_bt_plugins
{

namespace
{
constexpr double kMinYawDiff = M_PI / 180.0;   // 1°
}  // namespace

bool alignPathEndToGoal(
  nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & goal, double max_dist)
{
  if (path.poses.empty()) {return false;}
  const std::string & pf = path.header.frame_id;
  const std::string & gf = goal.header.frame_id;
  if (!pf.empty() && !gf.empty() && pf != gf) {return false;}
  auto & end = path.poses.back().pose;
  const double d = std::hypot(
    end.position.x - goal.pose.position.x, end.position.y - goal.pose.position.y);
  if (!(d <= max_dist)) {return false;}   // NaN 도 거른다
  // 방향 차이가 1° 이하면 그대로 둔다. planner 가 낸 사원수는 목적지와 같은 방향이어도 끝자리가
  // 달라, 정확히 같음으로 비교하면 run50 화장실에서 매번 "고쳤다" 로 셌다(실제 차이 0.0°).
  const auto & q = goal.pose.orientation;
  const auto & e = end.orientation;
  const double dot = std::abs(e.x * q.x + e.y * q.y + e.z * q.z + e.w * q.w);
  const double diff = 2.0 * std::acos(std::min(1.0, dot));   // 두 방향 사이 각(rad)
  if (diff <= kMinYawDiff) {return false;}
  end.orientation = q;
  return true;
}

}  // namespace vica_nav2_bt_plugins
