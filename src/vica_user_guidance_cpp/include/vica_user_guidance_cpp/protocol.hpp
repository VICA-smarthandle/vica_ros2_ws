// Smart Handle 1바이트 시리얼 프로토콜 상수 — 파이썬 판 vica_user_guidance/protocol.py 와 같은 값.
// 정본은 펌웨어(vica_user_guidance/firmware/smart_handle_firmware/smart_handle_firmware.ino)다.
// 값을 바꿀 때는 펌웨어·파이썬 판과 함께 바꾼다(대조 시험이 파이썬 판과의 어긋남을 잡는다).
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
constexpr int STATE_LINK_LOST = 4;   // 펌웨어 워치독 전용 — ROS 는 보내지 않는다
constexpr int STATE_ARRIVED = 5;
constexpr int STATE_CHARGING = 6;    // [TARGET] 펌웨어 미구현
constexpr int STATE_CHARGED = 7;     // [TARGET] 펌웨어 미구현

// ROS 가 보내도 되는 코드. 4 는 펌웨어 워치독을 무력화하므로, 6·7 은 펌웨어에 없으므로 뺀다.
constexpr bool is_sendable(int code)
{
  return code == STATE_NORMAL || code == STATE_LEFT || code == STATE_RIGHT ||
         code == STATE_ESTOP || code == STATE_ARRIVED;
}

// protocol.STATE_NAMES.get(code, "?")
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

// 펌웨어 애니메이션 타이밍(.ino ARRIVE_BLINK_MS / ARRIVE_BLINK_COUNT).
constexpr int FIRMWARE_ARRIVE_BLINK_MS = 500;
constexpr int FIRMWARE_ARRIVE_BLINK_COUNT = 3;
constexpr int FIRMWARE_WATCHDOG_TIMEOUT_MS = 1500;
constexpr int64_t FIRMWARE_BAUDRATE = 115200;

// 도착 애니메이션이 스스로 복귀할 때까지의 재생 시간(초) = (2 × count + 1) 프레임.
inline double firmware_arrival_duration_sec()
{
  const int frames = 2 * FIRMWARE_ARRIVE_BLINK_COUNT + 1;
  return FIRMWARE_ARRIVE_BLINK_MS * frames / 1000.0;
}

// ── 상향 프레임(아두이노 → 젯슨). 헤더 2바이트 + seq + 본문 + xor(seq~본문 끝). ──
constexpr int US_CHANNELS = 8;
constexpr uint8_t US_FRAME_HEADER[2] = {0xAA, 0x57};      // 거리 8채널, 20바이트
constexpr std::size_t US_FRAME_LEN = 3 + 2 * US_CHANNELS + 1;
constexpr int US_DIST_INVALID = 0;     // 3회 연속 측정 실패 — 그 채널은 발행하지 않는다
constexpr int US_DIST_MAX_MM = 3000;
constexpr int US_CLEAR_MM = 3001;      // 범위 내 에코 없음 — max_range 로 발행해 부채꼴을 지운다

constexpr uint8_t US_STAT_FRAME_HEADER[2] = {0xAA, 0x58};  // 측정 결과 통계, 52바이트
constexpr int US_STAT_KINDS = 6;       // ok clr ffff fffe oth i2c
constexpr std::size_t US_STAT_FRAME_LEN = 3 + US_CHANNELS * US_STAT_KINDS + 1;
constexpr int US_STAT_FFFE_INDEX = 3;

constexpr uint8_t US_CFG_FRAME_HEADER[2] = {0xAA, 0x59};   // 레지스터 되읽기, 20바이트
constexpr std::size_t US_CFG_FRAME_LEN = 3 + 2 * US_CHANNELS + 1;

constexpr uint8_t TOUCH_FRAME_HEADER[2] = {0xAA, 0x56};    // 터치, 5바이트
constexpr std::size_t TOUCH_FRAME_LEN = 5;
constexpr uint8_t TOUCH_FLAG_CONTACT = 0x01;

// ── 하향 햅틱 명령. 상태코드(0~7)와 겹치지 않는 별도 바이트 ──
constexpr int HAPTIC_CMD_SHORT = 0x10;
constexpr int HAPTIC_CMD_LONG = 0x11;
constexpr int HAPTIC_CMD_TICK = 0x12;
constexpr int HAPTIC_CMD_LOCATE = 0x13;

}  // namespace protocol
}  // namespace vica_guidance
