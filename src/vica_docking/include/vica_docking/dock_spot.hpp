#pragma once

#include <optional>
#include <string>

#include "vica_docking/v_detector.hpp"

namespace vica_docking
{

struct Pose2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct DockSpot
{
  std::string map_id;
  double apex_x{0.0};
  double apex_y{0.0};
  double fwd_yaw{0.0};
  double stage_cm{100.0};
  double target_cm{14.0};
  std::string saved_at;
  std::string source{"v_detect"};
  bool visited_ok{false};
  bool refined{false};
  std::string refined_from;
  double refined_cm{0.0};
  double refined_deg{0.0};
};

Pose2 spot_to_map(const DockSpot & s, const DockPose & p);
DockSpot spot_from_detection(const Pose2 & robot_in_map, const VDetection & det);
DockSpot spot_from_pose(const Pose2 & robot_in_map, const DockPose & p);
Pose2 staging_pose(const DockSpot & s, double bumper_x);
Pose2 docked_pose(const DockSpot & s, double bumper_x);

std::optional<DockSpot> read_dock_spot(const std::string & path);
bool write_dock_spot(const std::string & path, const DockSpot & s, std::string * err = nullptr);

double wrap_angle(double a);

}
