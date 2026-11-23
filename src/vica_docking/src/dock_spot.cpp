#include "vica_docking/dock_spot.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>

#include "vica_docking/types.hpp"

namespace vica_docking
{

double wrap_angle(double a)
{
  return std::remainder(a, 2.0 * kPi);
}

Pose2 spot_to_map(const DockSpot & s, const DockPose & p)
{
  const double c = std::cos(s.fwd_yaw);
  const double sn = std::sin(s.fwd_yaw);
  const double ux = -c, uy = -sn;
  const double lx = -sn, ly = c;
  return {s.apex_x + p.x * ux + p.y * lx, s.apex_y + p.x * uy + p.y * ly, wrap_angle(s.fwd_yaw + p.th)};
}

DockSpot spot_from_detection(const Pose2 & r, const VDetection & det)
{
  const DockPose p = pose_in_dock(det);
  const double c = std::cos(r.yaw);
  const double sn = std::sin(r.yaw);
  DockSpot s;
  s.apex_x = r.x + c * det.apex.x - sn * det.apex.y;
  s.apex_y = r.y + sn * det.apex.x + c * det.apex.y;
  s.fwd_yaw = wrap_angle(r.yaw - p.th);
  return s;
}

DockSpot spot_from_pose(const Pose2 & r, const DockPose & p)
{
  DockSpot s;
  s.fwd_yaw = wrap_angle(r.yaw - p.th);
  const double c = std::cos(s.fwd_yaw);
  const double sn = std::sin(s.fwd_yaw);
  s.apex_x = r.x - (p.x * (-c) + p.y * (-sn));
  s.apex_y = r.y - (p.x * (-sn) + p.y * c);
  return s;
}

Pose2 staging_pose(const DockSpot & s, double bumper_x)
{
  return spot_to_map(s, {bumper_x + s.stage_cm / 100.0, 0.0, 0.0});
}

Pose2 docked_pose(const DockSpot & s, double bumper_x)
{
  return spot_to_map(s, {bumper_x + s.target_cm / 100.0, 0.0, 0.0});
}

namespace
{
std::string trim(const std::string & v)
{
  const auto b = v.find_first_not_of(" \t\r\"'");
  if (b == std::string::npos) {
    return "";
  }
  const auto e = v.find_last_not_of(" \t\r\"'");
  return v.substr(b, e - b + 1);
}
}

std::optional<DockSpot> read_dock_spot(const std::string & path)
{
  std::ifstream f(path);
  if (!f) {
    return std::nullopt;
  }
  std::map<std::string, std::string> kv;
  std::string line;
  while (std::getline(f, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) {
      line = line.substr(0, hash);
    }
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    kv[trim(line.substr(0, colon))] = trim(line.substr(colon + 1));
  }
  try {
    DockSpot s;
    s.map_id = kv.at("map_id");
    s.apex_x = std::stod(kv.at("apex_x"));
    s.apex_y = std::stod(kv.at("apex_y"));
    s.fwd_yaw = std::stod(kv.at("fwd_yaw"));
    if (kv.count("stage_cm")) {
      s.stage_cm = std::stod(kv.at("stage_cm"));
    }
    if (kv.count("target_cm")) {
      s.target_cm = std::stod(kv.at("target_cm"));
    }
    s.saved_at = kv.count("saved_at") ? kv.at("saved_at") : "";
    s.source = kv.count("source") ? kv.at("source") : "v_detect";
    s.visited_ok = kv.count("visited_ok") && kv.at("visited_ok") == "true";
    s.refined = kv.count("refined") && kv.at("refined") == "true";
    s.refined_from = kv.count("refined_from") ? kv.at("refined_from") : "";
    if (kv.count("refined_cm")) {
      s.refined_cm = std::stod(kv.at("refined_cm"));
    }
    if (kv.count("refined_deg")) {
      s.refined_deg = std::stod(kv.at("refined_deg"));
    }
    return s;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

bool write_dock_spot(const std::string & path, const DockSpot & s, std::string * err)
{
  const std::string tmp = path + ".tmp";
  {
    std::ofstream f(tmp);
    if (!f) {
      if (err) {
        *err = "쓸 수 없음: " + tmp;
      }
      return false;
    }
    char buf[128];
    f << "# 도크 위치(vica_docking). 꼭짓점과 '도크를 정면으로 보는 방향'만 저장하고,\n"
      << "# 준비 위치·도킹 위치는 여기서 계산한다. 손으로 고치지 말고 앱에서 다시 저장한다.\n";
    f << "map_id: " << s.map_id << "\n";
    std::snprintf(buf, sizeof(buf), "apex_x: %.4f\n", s.apex_x);
    f << buf;
    std::snprintf(buf, sizeof(buf), "apex_y: %.4f\n", s.apex_y);
    f << buf;
    std::snprintf(buf, sizeof(buf), "fwd_yaw: %.5f\n", s.fwd_yaw);
    f << buf;
    std::snprintf(buf, sizeof(buf), "stage_cm: %.1f\n", s.stage_cm);
    f << buf;
    std::snprintf(buf, sizeof(buf), "target_cm: %.1f\n", s.target_cm);
    f << buf;
    f << "saved_at: " << s.saved_at << "\n";
    f << "source: " << s.source << "\n";
    f << "visited_ok: " << (s.visited_ok ? "true" : "false") << "\n";
    if (s.refined) {
      f << "refined: true\n";
      f << "refined_from: " << s.refined_from << "\n";
      std::snprintf(buf, sizeof(buf), "refined_cm: %.1f\n", s.refined_cm);
      f << buf;
      std::snprintf(buf, sizeof(buf), "refined_deg: %.1f\n", s.refined_deg);
      f << buf;
    }
    if (!f) {
      if (err) {
        *err = "쓰기 실패: " + tmp;
      }
      return false;
    }
  }
  if (std::rename(tmp.c_str(), path.c_str()) != 0) {
    if (err) {
      *err = "이름 바꾸기 실패: " + path;
    }
    return false;
  }
  return true;
}

}
