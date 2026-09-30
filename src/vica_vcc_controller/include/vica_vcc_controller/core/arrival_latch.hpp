#pragma once
#include "vica_vcc_controller/core/types.hpp"

namespace vica_vcc_controller::core
{
struct ArrivalParams
{
  double xy_tol{0.15};
  double yaw_tol{0.25};
  double trans_stopped{0.03};
  double rot_stopped{0.05};
  // 도장을 푸는 거리. 목적지(경로 끝점)가 도장 찍을 때보다 이만큼 넘게 옮겨졌거나, 로봇이 끝점에서
  // 이만큼 넘게 멀어지면 푼다. VCC align_exit_dist 와 같은 값이어야 한다(계약 시험).
  double unlatch_dist{0.5};
};

// 도착 판정: 끝점 xy_tol 안에 한 번 들어오면 위치에 도장을 찍고, 그 뒤로는 방향과 정지만 본다.
//
// Humble SimpleGoalChecker 의 stateful 도 xy 통과를 기억하지만 controller_server 가 새 경로를 받을
// 때마다(setPlannerPath) goal_checker->reset() 을 불러 지운다. 레일 BT 는 1 Hz 로 경로를 다시 주므로
// 기억이 1초도 못 간다. 그래서 여기서는 reset 으로 도장을 지우지 않고, 목적지·로봇 거리로만 푼다.
class ArrivalLatch
{
public:
  explicit ArrivalLatch(ArrivalParams p = {}) : p_(p) {}
  // robot·goal 은 같은 좌표계. vx·vy·w 는 실측 속도.
  bool check(const Pose2D & robot, const Pose2D & goal, double vx, double vy, double w);
  void clear() {latched_ = false;}
  bool latched() const {return latched_;}
  const ArrivalParams & params() const {return p_;}

private:
  ArrivalParams p_;
  bool latched_{false};
  double gx_{0.0};
  double gy_{0.0};
};
}  // namespace vica_vcc_controller::core
