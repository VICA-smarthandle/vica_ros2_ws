// 우선순위 병합·순찰 표시·도착 이벤트·초음파 TF 게이트 — 파이썬 판 guidance_priority.py · range_tf_gate.py ·
// timebase.py 와 같은 판정. 이 계층은 구동 명령을 만들지 않는다. 출력은 상태코드 1개뿐이다.
#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "vica_user_guidance_cpp/protocol.hpp"

namespace vica_guidance
{

// fresh = last 가 있고 0 <= now - last <= timeout. 시간 역전(음수 나이)은 stale — 시계가 튀면
// 안전한 쪽으로 떨어진다. 정본 계약은 vica_safety/freshness.py.
inline bool is_fresh_ns(std::optional<int64_t> last_ns, int64_t now_ns, int64_t timeout_ns)
{
  if (!last_ns) {return false;}
  const int64_t age = now_ns - *last_ns;
  return 0 <= age && age <= timeout_ns;
}

constexpr int DIRECTION_NONE = 0;
constexpr int DIRECTION_LEFT = 1;
constexpr int DIRECTION_RIGHT = 2;

// ── 순찰 표시(2026-11-14) — guidance_priority.py 의 PATROL_* 와 같은 값·판정 ──
constexpr int PATROL_NONE = 0;
// /vica/robot_state 가 이만큼 끊기면 순찰 표시를 버린다(미션 발행 1 Hz 의 3배).
constexpr int64_t PATROL_STATE_TIMEOUT_NS = 3000000000;

// dialog_state → 순찰 상태코드. "patrol" 로 시작하면 순찰이고, 아니면 PATROL_NONE.
// patrolling·patrol_dwell·patrol_paused 는 8, patrol_light 는 9, patrol_alarm 은 10.
inline int patrol_state_code(const std::string & dialog_state)
{
  if (dialog_state.rfind("patrol", 0) != 0) {return PATROL_NONE;}
  if (dialog_state == "patrol_alarm") {return protocol::STATE_PATROL_ALARM;}
  if (dialog_state == "patrol_light") {return protocol::STATE_PATROL_LIGHT;}
  return protocol::STATE_PATROL;
}

// PATROL_REASONS.get(code) — 순찰 코드가 아니면 nullptr.
constexpr const char * patrol_reason(int code)
{
  switch (code) {
    case protocol::STATE_PATROL: return "patrol";
    case protocol::STATE_PATROL_LIGHT: return "patrol_light";
    case protocol::STATE_PATROL_ALARM: return "patrol_alarm";
    default: return nullptr;
  }
}

// 지금 유효한 순찰 코드. 순찰이 아니거나 robot_state 가 끊겼으면 PATROL_NONE.
inline int active_patrol_code(
  int patrol_code, std::optional<int64_t> patrol_last_ns, int64_t now_ns,
  int64_t timeout_ns = PATROL_STATE_TIMEOUT_NS)
{
  if (!patrol_reason(patrol_code)) {return PATROL_NONE;}
  if (!is_fresh_ns(patrol_last_ns, now_ns, timeout_ns)) {return PATROL_NONE;}
  return patrol_code;
}

struct GuidanceInputs
{
  bool estop_active = false;
  std::optional<int64_t> estop_last_ns;
  int turn_direction = DIRECTION_NONE;
  std::optional<int64_t> turn_last_ns;
  std::optional<int64_t> arrival_started_ns;
  int patrol_code = PATROL_NONE;            // robot_state 로 받은 순찰 코드(8·9·10)
  std::optional<int64_t> patrol_last_ns;    // 순찰 값을 마지막으로 받은 시각
};

struct GuidanceOutcome
{
  int state_code;
  const char * reason;
};

// E-stop > 순찰 > 도착 > 회전 > 기본. 좌우를 여기서 뒤집지 않는다 — 배선 보정은 펌웨어 한 곳뿐이다
// (2026-08-01 여기서 뒤집었다가 서보까지 뒤집힌 사고, guidance_priority.py 주석).
inline GuidanceOutcome resolve_state_code(
  const GuidanceInputs & in, int64_t now_ns, int64_t estop_timeout_ns, int64_t cue_timeout_ns,
  int64_t arrival_hold_ns, bool estop_required,
  int64_t patrol_timeout_ns = PATROL_STATE_TIMEOUT_NS)
{
  if (in.estop_active) {return {protocol::STATE_ESTOP, "estop_active"};}
  if (estop_required && !is_fresh_ns(in.estop_last_ns, now_ns, estop_timeout_ns)) {
    return {protocol::STATE_ESTOP, "estop_stale"};
  }
  // 순찰 표시는 도착·회전보다 앞이다 — 순찰 중에는 그 표시가 안 나간다.
  const int patrol =
    active_patrol_code(in.patrol_code, in.patrol_last_ns, now_ns, patrol_timeout_ns);
  if (patrol != PATROL_NONE) {return {patrol, patrol_reason(patrol)};}
  if (in.arrival_started_ns) {
    const int64_t age = now_ns - *in.arrival_started_ns;
    if (0 <= age && age <= arrival_hold_ns) {return {protocol::STATE_ARRIVED, "arrival"};}
  }
  if (is_fresh_ns(in.turn_last_ns, now_ns, cue_timeout_ns)) {
    if (in.turn_direction == DIRECTION_LEFT) {return {protocol::STATE_LEFT, "turn_left"};}
    if (in.turn_direction == DIRECTION_RIGHT) {return {protocol::STATE_RIGHT, "turn_right"};}
  }
  return {protocol::STATE_NORMAL, "normal"};
}

// /vica_goal_event JSON 에서 event 문자열만 꺼낸다. 파싱 실패·event 없음·문자열 아님은 nullopt.
std::optional<std::string> parse_goal_event(const std::string & payload);

// goal_succeeded 만 도착이다(goal_canceled·goal_failed·goal_sent 는 아니다).
inline bool is_arrival_event(const std::optional<std::string> & event)
{
  return event && *event == "goal_succeeded";
}

// 초음파 Range 발행 게이트. TF 가 없는 동안 Range 를 보내지 않는다 — Nav2 RangeSensorLayer 가
// 그 조회 예외를 잡지 않아 controller_server 가 죽었다(2026-09-07). 조회는 노드가 하고 여기는
// 그 결과만 받는다. 시작은 허용으로 본다(기동 직후 경고부터 내면 정상 기동이 사고처럼 보인다).
class RangeTfGate
{
public:
  explicit RangeTfGate(int channels)
  {
    if (channels <= 0) {
      throw std::invalid_argument("채널 수는 1 이상이어야 한다: " + std::to_string(channels));
    }
    allowed_.assign(channels, true);
    pending_.assign(channels, std::nullopt);
    blocked_.assign(channels, 0);
  }

  bool allow(int ch, bool tf_ok)
  {
    if (tf_ok != allowed_.at(ch)) {
      allowed_[ch] = tf_ok;
      pending_[ch] = tf_ok;
    }
    if (!tf_ok) {++blocked_[ch];}
    return tf_ok;
  }

  // 상태가 바뀌었으면 새 상태를 1회만 돌려준다.
  std::optional<bool> take_transition(int ch)
  {
    auto p = pending_.at(ch);
    pending_[ch] = std::nullopt;
    return p;
  }

  int64_t blocked_count(int ch) const {return blocked_.at(ch);}

private:
  std::vector<bool> allowed_;
  std::vector<std::optional<bool>> pending_;
  std::vector<int64_t> blocked_;
};

}  // namespace vica_guidance
