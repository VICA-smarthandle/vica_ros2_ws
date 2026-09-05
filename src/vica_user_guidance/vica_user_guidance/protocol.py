"""Smart Handle 1바이트 시리얼 프로토콜 상수.

정본은 ``source_file/smart_handle_firmware/smart_handle_firmware.ino``다.
이 파일의 값을 바꿀 때는 반드시 펌웨어와 함께 바꾼다.
"""

from typing import FrozenSet

STATE_NORMAL: int = 0
STATE_LEFT: int = 1
STATE_RIGHT: int = 2
STATE_ESTOP: int = 3
STATE_LINK_LOST: int = 4
STATE_ARRIVED: int = 5
STATE_CHARGING: int = 6   # [TARGET] 펌웨어 미구현. 충전 상태 입력원 미정
STATE_CHARGED: int = 7    # [TARGET] 펌웨어 미구현

# ROS가 전송해도 되는 코드 집합.
#
# [중요] STATE_LINK_LOST(4)는 의도적으로 제외한다. 코드 4는 펌웨어 워치독이 스스로
# 발동하는 코드다. ROS가 이를 전송하면 펌웨어의 lastRxMillis가 갱신되어 실제 통신
# 단절을 영원히 감지하지 못하게 된다.
#
# 코드 6·7은 펌웨어에 아직 없으므로 제외한다. 보내도 펌웨어가 버린다.
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

# 펌웨어 애니메이션 타이밍. .ino의 ARRIVE_BLINK_MS / ARRIVE_BLINK_COUNT와 일치해야 한다.
FIRMWARE_ARRIVE_BLINK_MS: int = 500
FIRMWARE_ARRIVE_BLINK_COUNT: int = 3
FIRMWARE_WATCHDOG_TIMEOUT_MS: int = 1500
FIRMWARE_BAUDRATE: int = 115200

# ── 상향 초음파 프레임 (아두이노 → 젯슨, 2026-08-31 신설) ──────────────
# 정본은 펌웨어 usSendFrame()이다. 8바이트: AA 55 seq d0L d0H d1L d1H xor
# (거리 little-endian, xor 는 seq~d1H 5바이트). 헤더 0xAA/0x55 는 하향
# 상태코드(0~7)와 겹치지 않아 한 포트에서 양방향이 섞여도 안전하다.
# 채널 0 = front_left(I2C 0x68), 채널 1 = front_right(0x74) — 2026-08-31 실측.
US_FRAME_HEADER: bytes = b"\xaa\x55"
US_FRAME_LEN: int = 8
US_CHANNELS: int = 2
US_DIST_INVALID: int = 0     # 3회 연속 측정 실패 — 그 채널은 발행하지 않는다
US_DIST_MAX_MM: int = 3000   # 유효 실거리 상한
US_CLEAR_MM: int = 3001      # 범위 내 에코 없음 — max_range 로 발행해 부채꼴을 지운다
FIRMWARE_US_CYCLE_MS: int = 210  # GAP 5 + WAIT 100, 2채널. 프레임 약 4.8Hz

# ── 상향 터치 프레임 (아두이노 → 젯슨, 2026-09-04 신설 · 09-05 재도입) ───
# 정본은 펌웨어 touchPoll()이다. 5바이트: AA 56 seq flags xor (xor 는 seq^flags).
#
# 센서(D11)는 **idle HIGH / 터치 LOW** 인 active-low 타입이다(2026-09-05 인수인계
# 문서 실측). 펌웨어가 극성과 '터치 중 튐'을 정리해 flags.bit0 = 1 이면 잡음이다.
# 젯슨은 그 비트만 믿는다.
#
# [왜 초음파 프레임에 얹지 않았나] 둘은 주기가 다르다. 초음파 4.8Hz 는 측정
# 시간이 정하는 물리 한계인데, 손 놓음 판정 유예는 0.5초라 그사이 샘플이 2~3개
# 뿐이다. [왜 헤더를 갈랐나] 8바이트를 9바이트로 늘리면 헤더가 같아, 옛 파서가
# 체크섬 실패 -> 1바이트 밀기를 반복하며 **초음파까지 함께** 멈춘다.
TOUCH_FRAME_HEADER: bytes = b"\xaa\x56"
TOUCH_FRAME_LEN: int = 5
TOUCH_FLAG_CONTACT: int = 0x01   # bit0 = 잡고 있음. bit1~7 예약(0)
FIRMWARE_TOUCH_PERIOD_MS: int = 50   # 20Hz. 판정 유예 0.5초에 10프레임
FIRMWARE_TOUCH_HOLD_MS: int = 200    # 시간 브리지. LOW 본 뒤 이만큼은 '잡음'

# ── 하향 햅틱 명령 (젯슨 → 아두이노, 2026-09-04 신설) ──────────────────
# 상태코드(0~7)와 겹치지 않는 별도 바이트다. applyState()를 거치지 않으므로
# LED·서보는 그대로이고 D10 의 진동모터만 패턴대로 떨린다.
#
# **수동 전용이다.** 드라이버 노드는 이 바이트를 보내지 않는다(SENDABLE_STATE_CODES
# 에 없다). bench_test.py --haptic 으로만 쏜다. ESTOP·ARRIVED 진입 시 자동으로
# 울리는 것(7/28 계획서 6.2절)은 별도 결정 사항이며 아직 넣지 않았다.
#
# 패턴은 계획서 6.2절 그대로다. 사용자 요구: "비상제동 = 긴 진동 / 도착 = 짧은 3회".
HAPTIC_CMD_SHORT: int = 0x10     # 300ms on / 150ms off x 3회 (도착 패턴)
HAPTIC_CMD_LONG: int = 0x11      # 1200ms on x 1회 (비상 패턴)
FIRMWARE_HAPTIC_SHORT_ON_MS: int = 300   # 2026-09-04 150->300. 회전 올라올 시간
FIRMWARE_HAPTIC_SHORT_OFF_MS: int = 150
FIRMWARE_HAPTIC_SHORT_COUNT: int = 3
FIRMWARE_HAPTIC_LONG_ON_MS: int = 1200   # 2026-09-04 800->1200


def firmware_arrival_duration_sec() -> float:
    """도착 애니메이션이 스스로 복귀할 때까지의 실제 재생 시간(초).

    ON/OFF 각 count회 + 마지막 소등 유지 1프레임 = (2 * count + 1) 프레임이다.

    2026-07-28 프레임 추적 결과 500ms x 7 = 3.5초다. 초기 계획서의 "3.0초"는 오기이며,
    그 값으로 arrival_hold_sec를 잡으면 마지막 소등 프레임이 잘려 사용자에게
    "2.5회 점멸"로 보인다.
    """
    frames = 2 * FIRMWARE_ARRIVE_BLINK_COUNT + 1
    return FIRMWARE_ARRIVE_BLINK_MS * frames / 1000.0


def is_sendable(state_code: int) -> bool:
    """ROS가 이 상태코드를 아두이노로 전송해도 되는지 여부."""
    return state_code in SENDABLE_STATE_CODES
