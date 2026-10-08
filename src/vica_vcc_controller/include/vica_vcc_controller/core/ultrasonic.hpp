#pragma once
#include <deque>
#include <optional>
#include <vector>
#include "vica_vcc_controller/core/types.hpp"

namespace vica_vcc_controller::core
{
struct RangeReading
{
  double range{0.0};
  double min_range{0.0};
  double max_range{0.0};
  double fov{0.0};
  double recv_time{0.0};   // 받은 시각(steady). stamp 가 아니다 — run44 stamp 지연으로 660회 무시된 교훈
  // 받은 순간의 센서 자세(costmap 전역 좌표계 = odom). 2.4 Hz 라 나중 TF 로 옮기면 v·나이만큼 멀어 보인다.
  Pose2D sensor;
};

struct UltrasonicParams
{
  double max_age{1.0};
  int confirm_count{2};
  double confirm_tol{0.15};
  // 이보다 가까운 값은 한 번에 인정한다(run48 F4b: 콘 0.33 을 두 번째 프레임 0.42 s 기다리다 놓쳤다).
  double near_confirm_range{0.40};
  // 확인된 점을 이만큼 기억한다. 먼 반사 하나로 지우지 않는다(run48 F4c).
  // pass_memory 가 켜져 있으면 이 값은 '아무리 꼬여도 이보다 오래는 안 남는' 안전 상한이다.
  double memory_time{1.5};
  // ── 지나갈 때까지 기억(2026-10-08 run74 뒤, 사용자 결정) ─────────────────────────────
  // 옛 규칙은 '에코 없음 2번'이면 바로 지웠다. 몸을 틀어 콘이 빔 밖(대각선 사각)으로 나가도 지워져,
  // 콘 쪽 차선으로 돌아가 스쳤다(run71 17:06:53, run74 21:12:33). 켜면:
  //   - 확인된 점을 지도(odom) 자리 그대로 여러 개(memory_slots) 들고 있는다.
  //   - 센서가 그 자리를 '다시 비추는데' 에코 없음·더 먼 값이 2번이면 지운다(빔 밖으로 나간 건 지우지 않는다).
  //   - 몸 뒤 끝보다 pass_margin 넘게 뒤로 지나갔거나 로봇에서 pass_max_dist 넘게 멀어지면 지운다(forget).
  bool pass_memory{false};
  double pass_margin{0.2};
  double pass_max_dist{2.0};
  int memory_slots{4};
};

class UltrasonicChannel
{
public:
  explicit UltrasonicChannel(UltrasonicParams p = {}) : p_(p) {}
  // 0 < range <= min_range 는 맞닿은 물체로 보고 min_range 에 둔다(run48 F4a: 0.01~0.02 를 버렸다).
  // 받을 때마다 확인·기억을 갱신한다.
  void push(const RangeReading & r);
  // 최신 측정이 max_age 안·장애물이고 near_confirm_range 보다 가까우면 그 한 값으로 확인한다.
  // 아니면 최근 count 개가 모두 max_age 안·장애물이고, 연속한 두 측정마다 새 호 중심점이
  // 이전 호에서 tol 안이면 최신 값.
  std::optional<RangeReading> confirmed(double now, double max_age, int count, double tol) const;
  // 지금 확인된 값 ∪ memory_time 안의 기억(둘 다 받은 순간 odom 자세 기준). 같은 값이면 하나.
  std::vector<RangeReading> obstacles(double now) const;
  // pass_memory: 로봇(odom 자세)이 지나간 점·멀어진 점을 지운다. rear_x = 몸 뒤 끝(로봇 좌표, 음수).
  void forget(const Pose2D & robot, double rear_x);
  size_t memoryCount() const {return p_.pass_memory ? memos_.size() : (memory_ ? 1u : 0u);}
  // 최신 측정이 max_age 안인가(/vcc/state us_fresh, 설계서 10절).
  bool fresh(double now, double max_age) const
  {
    return !hist_.empty() && now - hist_.back().recv_time <= max_age;
  }

private:
  UltrasonicParams p_;
  std::deque<RangeReading> hist_;
  std::optional<RangeReading> memory_;
  int clear_count_{0};   // 기억보다 멀리 본(또는 에코 없는) 연속 측정 수
  struct Memo
  {
    RangeReading r;
    Point2D c;       // 호 가운데 점(odom)
    int clear{0};    // 그 자리를 비추며 에코 없음·더 먼 값을 본 연속 횟수
  };
  std::vector<Memo> memos_;   // pass_memory 기억
  void pushPass(const RangeReading & r, bool is_confirmed);
};

// 채널 거리 상한(2026-10-08 run71 뒤, 사용자 결정 — 바퀴 옆 0.40 m). cap 보다 먼 값은 VCC 에겐 "에코 없음"
// (max_range)으로 바꾼다. 확인·기억 규칙은 그대로이고 들어오는 값만 자른다. cap <= 0 이면 그대로.
RangeReading capRange(RangeReading r, double cap);

// 센서 좌표(x 앞)에서 거리 range, 폭 fov 의 호를 n 점으로.
std::vector<Point2D> rangeToArcPoints(double range, double fov, int n);
// 측정값의 호 점을 받은 순간의 센서 자세로 전역 좌표에 놓는다(지금 TF 로 다시 옮기지 않는다).
std::vector<Point2D> readingToArcPoints(const RangeReading & r, int n);
}  // namespace vica_vcc_controller::core
