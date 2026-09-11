"""Smart Handle 1바이트 시리얼 프로토콜 상수."""

from typing import FrozenSet

STATE_NORMAL: int = 0
STATE_LEFT: int = 1
STATE_RIGHT: int = 2
STATE_ESTOP: int = 3
STATE_LINK_LOST: int = 4
STATE_ARRIVED: int = 5
STATE_CHARGING: int = 6
STATE_CHARGED: int = 7

SENDABLE_STATE_CODES: FrozenSet[int] = frozenset(
    {
        STATE_NORMAL,
        STATE_LEFT,
        STATE_RIGHT,
        STATE_ESTOP,
        STATE_ARRIVED,
    }
)

STATE_NAMES = {
    STATE_NORMAL: "NORMAL",
    STATE_LEFT: "LEFT",
    STATE_RIGHT: "RIGHT",
    STATE_ESTOP: "ESTOP",
    STATE_LINK_LOST: "LINK_LOST",
    STATE_ARRIVED: "ARRIVED",
    STATE_CHARGING: "CHARGING",
    STATE_CHARGED: "CHARGED",
}

FIRMWARE_ARRIVE_BLINK_MS: int = 500
FIRMWARE_ARRIVE_BLINK_COUNT: int = 3
FIRMWARE_WATCHDOG_TIMEOUT_MS: int = 1500
FIRMWARE_BAUDRATE: int = 115200

US_FRAME_HEADER: bytes = b"\xaa\x55"
US_FRAME_LEN: int = 8
US_CHANNELS: int = 2
US_DIST_INVALID: int = 0
US_DIST_MAX_MM: int = 3000
US_CLEAR_MM: int = 3001
FIRMWARE_US_CYCLE_MS: int = 210

TOUCH_FRAME_HEADER: bytes = b"\xaa\x56"
TOUCH_FRAME_LEN: int = 5
TOUCH_FLAG_CONTACT: int = 0x01
FIRMWARE_TOUCH_PERIOD_MS: int = 50
FIRMWARE_TOUCH_HOLD_MS: int = 200

HAPTIC_CMD_SHORT: int = 0x10
HAPTIC_CMD_LONG: int = 0x11
FIRMWARE_HAPTIC_SHORT_ON_MS: int = 300
FIRMWARE_HAPTIC_SHORT_OFF_MS: int = 150
FIRMWARE_HAPTIC_SHORT_COUNT: int = 3
FIRMWARE_HAPTIC_LONG_ON_MS: int = 1200


def firmware_arrival_duration_sec() -> float:
    """도착 애니메이션이 스스로 복귀할 때까지의 실제 재생 시간(초)."""
    frames = 2 * FIRMWARE_ARRIVE_BLINK_COUNT + 1
    return FIRMWARE_ARRIVE_BLINK_MS * frames / 1000.0


def is_sendable(state_code: int) -> bool:
    """ROS가 이 상태코드를 아두이노로 전송해도 되는지 여부."""
    return state_code in SENDABLE_STATE_CODES
