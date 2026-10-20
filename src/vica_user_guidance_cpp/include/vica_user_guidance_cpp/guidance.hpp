#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "vica_user_guidance_cpp/protocol.hpp"

namespace vica_guidance
{

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

std::optional<std::string> parse_goal_event(const std::string & payload);

inline bool is_arrival_event(const std::optional<std::string> & event)
{
  return event && *event == "goal_succeeded";
}

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

}
