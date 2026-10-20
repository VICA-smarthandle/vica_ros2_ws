#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace vica_guidance
{
namespace protocol
{

constexpr int STATE_NORMAL = 0;
constexpr int STATE_LEFT = 1;
constexpr int STATE_RIGHT = 2;
constexpr int STATE_ESTOP = 3;
constexpr int STATE_LINK_LOST = 4;
constexpr int STATE_ARRIVED = 5;
constexpr int STATE_CHARGING = 6;
constexpr int STATE_CHARGED = 7;

constexpr bool is_sendable(int code)
{
  return code == STATE_NORMAL || code == STATE_LEFT || code == STATE_RIGHT ||
         code == STATE_ESTOP || code == STATE_ARRIVED;
}

constexpr const char * state_name(int code)
{
  switch (code) {
    case STATE_NORMAL: return "NORMAL";
    case STATE_LEFT: return "LEFT";
    case STATE_RIGHT: return "RIGHT";
    case STATE_ESTOP: return "ESTOP";
    case STATE_LINK_LOST: return "LINK_LOST";
    case STATE_ARRIVED: return "ARRIVED";
    case STATE_CHARGING: return "CHARGING";
    case STATE_CHARGED: return "CHARGED";
    default: return "?";
  }
}

constexpr int FIRMWARE_ARRIVE_BLINK_MS = 500;
constexpr int FIRMWARE_ARRIVE_BLINK_COUNT = 3;
constexpr int FIRMWARE_WATCHDOG_TIMEOUT_MS = 1500;
constexpr int64_t FIRMWARE_BAUDRATE = 115200;

inline double firmware_arrival_duration_sec()
{
  const int frames = 2 * FIRMWARE_ARRIVE_BLINK_COUNT + 1;
  return FIRMWARE_ARRIVE_BLINK_MS * frames / 1000.0;
}

constexpr int US_CHANNELS = 8;
constexpr uint8_t US_FRAME_HEADER[2] = {0xAA, 0x57};
constexpr std::size_t US_FRAME_LEN = 3 + 2 * US_CHANNELS + 1;
constexpr int US_DIST_INVALID = 0;
constexpr int US_DIST_MAX_MM = 3000;
constexpr int US_CLEAR_MM = 3001;

constexpr uint8_t US_STAT_FRAME_HEADER[2] = {0xAA, 0x58};
constexpr int US_STAT_KINDS = 6;
constexpr std::size_t US_STAT_FRAME_LEN = 3 + US_CHANNELS * US_STAT_KINDS + 1;
constexpr int US_STAT_FFFE_INDEX = 3;

constexpr uint8_t US_CFG_FRAME_HEADER[2] = {0xAA, 0x59};
constexpr std::size_t US_CFG_FRAME_LEN = 3 + 2 * US_CHANNELS + 1;

constexpr uint8_t TOUCH_FRAME_HEADER[2] = {0xAA, 0x56};
constexpr std::size_t TOUCH_FRAME_LEN = 5;
constexpr uint8_t TOUCH_FLAG_CONTACT = 0x01;

constexpr int HAPTIC_CMD_SHORT = 0x10;
constexpr int HAPTIC_CMD_LONG = 0x11;
constexpr int HAPTIC_CMD_TICK = 0x12;
constexpr int HAPTIC_CMD_LOCATE = 0x13;

}
}
