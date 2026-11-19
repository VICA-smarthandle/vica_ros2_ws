// 도크 위치(지도별 dock.yaml)와 도크 기준 ↔ 지도 좌표 변환.
//
// 저장 자리: <목적지 폴더>/dock.yaml — 미션의 home.yaml 과 같은 폴더, 지도당 1개(11-18 사용자: 홈과 따로).
// 도크 좌표계: 원점 = V자 꼭짓점, fwd_yaw = 로봇이 도크를 정면으로 볼 때의 방향(지도 기준).
// 로봇의 도크 기준 위치(축거리 x, 좌우 y, 방향 th — pose_in_dock 과 같은 정의)를 지도 위치로 바꾼다.
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
  std::string source{"v_detect"};   // v_detect(라이다로 인식해 저장) · map_pick(앱 지도에서 지정, 11-19)
  bool visited_ok{false};
  // 처음 도킹에 성공했을 때(또는 준비 위치에서 V자를 봤을 때) 라이다로 잰 값으로 한 번 고친 기록(11-19 사용자)
  bool refined{false};
  std::string refined_from;         // 고치기 전 source
  double refined_cm{0.0};           // 꼭짓점이 옮겨진 거리
  double refined_deg{0.0};          // 방향이 바뀐 각(절댓값)
};

// 도크 기준 위치 → 지도 위치
Pose2 spot_to_map(const DockSpot & s, const DockPose & p);
// 지도의 로봇 위치 + 그 자리에서 본 V자 → 도크 위치
DockSpot spot_from_detection(const Pose2 & robot_in_map, const VDetection & det);
// 지도의 로봇 위치 + 그 자리의 도크 기준 위치(pose_in_dock) → 도크 위치(꼭짓점·방향만 채운다). spot_to_map 의 역.
DockSpot spot_from_pose(const Pose2 & robot_in_map, const DockPose & p);
// 준비 위치(범퍼→꼭짓점 stage_cm, 축 위, 도크를 봄)와 도킹 위치
Pose2 staging_pose(const DockSpot & s, double bumper_x);
Pose2 docked_pose(const DockSpot & s, double bumper_x);

// dock.yaml 읽기·쓰기(평평한 key: value). 실패하면 nullopt / false.
std::optional<DockSpot> read_dock_spot(const std::string & path);
bool write_dock_spot(const std::string & path, const DockSpot & s, std::string * err = nullptr);

double wrap_angle(double a);

}  // namespace vica_docking
