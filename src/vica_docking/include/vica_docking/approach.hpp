// 전진 도킹: 준비 위치(범퍼→꼭짓점 1 m)에서 정렬 → 직진 → 범퍼→꼭짓점 14 cm.
//
// 11-18 실기 도구 dock_approach_test.py v2.1 의 판정을 그대로 옮겼다(±30 cm·±30° 출발 7/7,
// 앞뒤 −0.04~−0.33 cm, 좌우 ≤1.25 cm, 방향 ≤1.6°).
//   WAIT → (멀면 ROTATE_TO_STAGE/DRIVE_TO_STAGE) → ALIGN → APPROACH → STOPPED → DONE
//   · 출발 전 5회 연속 일치(2 cm 안)
//   · 0.10 m·15° 넘게 튀는 인식은 버림, 0.25 s 끊기면 0 으로 기다리고 3 s 면 중단(통로가 가려져 있으면 막힘으로 보고 30 s 까지 기다림)
//   · (11-18 밤 사용자 승인으로 바꿈) 범퍼 앞 30 cm·차체 폭 안에 V자 팔·벽이 아닌 점 3개면 0 으로 멈춰
//     기다리고, 비고 1 s 지나면 다시 간다. 30 s 넘게 막히면 실패. 막힌 동안은 V자를 놓쳐도 30 s 까지 기다린다.
//     (실기 도구는 3 cm 안이면 바로 중단이었다)
//   · 정지 명령은 넘침(lead 1.36 cm)만큼 일찍, 멈춘 뒤 1.5 s 기다렸다 30 스캔으로 최종 위치
// 운영에서 더한 것(멈추게만 한다): 출발 범위(±30 cm·±30°), 5 s 안에 못 찾으면 중단,
// 2 s 명령에도 안 움직이면 중단, 최종 오차 판정.
// 시간(now)은 호출자가 주는 초 단위 단조 시각이다 — 녹화 재생으로 똑같이 시험한다.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "vica_docking/geometry.hpp"
#include "vica_docking/types.hpp"
#include "vica_docking/v_detector.hpp"

namespace vica_docking
{

struct ApproachParams
{
  double target_cm{14.0};       // 도착: 범퍼→꼭짓점(11-18 사용자 "1cm만 더")
  double stage_cm{100.0};       // 준비 위치: 범퍼→꼭짓점
  double lead{0.0136};          // 정지 명령 앞당김 m(11-17 넘침 평균)
  double vmax{0.10};
  double vfinal{0.05};          // 남은 거리 10 cm 안 속도
  double wmax{0.30};
  double k_b{1.0};
  double deadband_deg{0.6};
  double align_tol_deg{1.2};
  double max_time{60.0};
  double rot_clear{0.72};       // 제자리 회전 여유: 손잡이 끝 반경 0.67 + 0.05
  // 운영에서 더함
  double find_timeout{5.0};
  double start_max_lateral{0.30};
  double start_max_heading_deg{30.0};
  double no_motion_s{2.0};
  double ok_x_cm{1.5};
  double ok_y_cm{1.5};
  double ok_th_deg{5.0};
  // 앞 장애물(11-18 밤 사용자 승인)
  double front_zone{0.30};
  int front_min_pts{3};
  double front_release_s{1.0};
  double front_wait_timeout{30.0};
  double lost_timeout_s{3.0};   // 통로가 비었는데 V자를 이만큼 못 보면 실패(11-19 사용자: 1.5 → 3 s)
};

enum class ApproachPhase
{
  kWait, kRotateToStage, kDriveToStage, kAlign, kApproach, kStopped, kDone, kAbort
};
const char * phase_name(ApproachPhase p);

struct ApproachResult
{
  bool has_start{false};
  DockPose start;
  bool has_stop{false};
  DockPose stop;            // 정지 명령 순간
  bool has_final{false};
  DockPose final_pose;      // 멈춘 뒤 30 스캔 평균
  double err_x_cm{0.0};     // − : 목표보다 더 들어감
  double err_y_cm{0.0};
  double err_th_deg{0.0};
  bool success{false};
  Fail fail{Fail::kNone};
  std::string why;
  int rejected{0};
  int holds{0};
  int front_holds{0};   // 앞 장애물로 멈춘 횟수
};

class ApproachController
{
public:
  explicit ApproachController(ApproachParams p = {}, RobotGeometry g = {}, VDetectorParams vp = {});

  void start(double now);
  void on_scan(
    double now, const std::vector<float> & ranges, double amin, double inc, double range_min);
  // 20 Hz 로 부른다. 낼 명령을 돌려준다(끊김·멈춤이면 0).
  Cmd on_timer(double now);
  void abort(Fail f, const std::string & why);

  ApproachPhase phase() const {return phase_;}
  bool active() const;
  bool finished() const {return phase_ == ApproachPhase::kDone || phase_ == ApproachPhase::kAbort;}
  const ApproachResult & result() const {return result_;}
  // 진행 표시: 최근 인식 위치(없으면 nullopt)
  std::optional<DockPose> last_pose() const {return last_pose_;}
  // 마지막 판정이 정한 명령(끊김으로 0 을 내기 전 값) — 녹화 대조용
  Cmd planned_cmd() const {return cmd_;}
  bool front_blocked() const {return front_hold_;}
  int front_hits() const {return front_hits_;}
  double d_target() const {return d_target_;}
  // 받은 스캔 수(V자를 못 본 것이 '스캔이 안 와서'인지 '도크가 없어서'인지 가른다)
  int scans() const {return scans_;}

private:
  void step(double now, const VDetection & det, const DockPose & pz);
  void set_cmd(double v, double w);
  double carrot_bearing(const VDetection & det, double x) const;
  void finish();

  ApproachParams p_;
  RobotGeometry g_;
  VDetector det_;
  double d_target_;
  double d_stage_;
  ApproachPhase phase_{ApproachPhase::kWait};
  double t0_{0.0};
  Cmd cmd_;
  std::optional<VDetection> good_;
  double good_t_{0.0};
  bool has_det_t_{false};
  double last_det_t_{0.0};
  int streak_{0};
  bool has_streak_ref_{false};
  DockPose streak_ref_;
  double settle_t_{0.0};
  std::vector<DockPose> settle_;
  std::optional<DockPose> last_pose_;
  // 앞 장애물
  int front_hits_{0};
  int scans_{0};
  int path_hits_{0};   // V자를 못 본 스캔에서, 마지막으로 본 V자까지의 통로(차폭) 안 점 수
  bool front_hold_{false};
  double front_since_{0.0};
  bool has_front_clear_{false};
  double front_clear_since_{0.0};
  // 안 움직임 검사
  bool has_move_ref_{false};
  bool move_forward_{false};   // 기준을 잡을 때 직진이었나(회전과 직진은 따로 잰다)
  double move_t_{0.0};
  DockPose move_pose_;
  ApproachResult result_;
};

}  // namespace vica_docking
