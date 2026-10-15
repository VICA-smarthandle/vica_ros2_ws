// 우선순위 병합·도착 이벤트·초음파 TF 게이트 — 파이썬 판 guidance_priority.py · range_tf_gate.py ·
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

struct GuidanceInputs
{
  bool estop_active = false;
  std::optional<int64_t> estop_last_ns;
  int turn_direction = DIRECTION_NONE;
  std::optional<int64_t> turn_last_ns;
  std::optional<int64_t> arrival_started_ns;
};

struct GuidanceOutcome
{
  int state_code;
  const char * reason;
};

// E-stop > 도착 > 회전 > 기본. 좌우를 여기서 뒤집지 않는다 — 배선 보정은 펌웨어 한 곳뿐이다
// (2026-08-01 여기서 뒤집었다가 서보까지 뒤집힌 사고, guidance_priority.py 주석).
inline GuidanceOutcome resolve_state_code(
  const GuidanceInputs & in, int64_t now_ns, int64_t estop_timeout_ns, int64_t cue_timeout_ns,
  int64_t arrival_hold_ns, bool estop_required)
{
  if (in.estop_active) {return {protocol::STATE_ESTOP, "estop_active"};}
  if (estop_required && !is_fresh_ns(in.estop_last_ns, now_ns, estop_timeout_ns)) {
    return {protocol::STATE_ESTOP, "estop_stale"};
  }
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
