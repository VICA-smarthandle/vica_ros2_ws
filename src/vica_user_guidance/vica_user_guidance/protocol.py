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
# 2026-09-23 8채널 (자리 번호 순). 정본은 펌웨어 usSendFrame8().
#   0 왼쪽 바퀴 옆 · 1 앞 왼쪽 · 2 앞 오른쪽 · 3 오른쪽 바퀴 옆
#   4 오른쪽 뒷바퀴 옆 · 5 후방 오른쪽 · 6 후방 왼쪽 · 7 왼쪽 뒷바퀴 옆
US_FRAME_HEADER: bytes = b"\xaa\x57"
US_CHANNELS: int = 8
US_FRAME_LEN: int = 3 + 2 * US_CHANNELS + 1   # 헤더 2 + seq + 거리 16 + xor = 20
# 옛 2채널 프레임(AA 55, 8B). 펌웨어가 호환용으로 함께 보내지만 이 파서는 헤더가
# 달라 그냥 지나친다 — 다른 브랜치의 옛 파서(앞 두 채널)를 위한 것이다.
US_LEGACY_FRAME_HEADER: bytes = b"\xaa\x55"
US_LEGACY_FRAME_LEN: int = 8
US_LEGACY_CHANNELS: int = 2
US_DIST_INVALID: int = 0     # 3회 연속 측정 실패 — 그 채널은 발행하지 않는다
US_DIST_MAX_MM: int = 3000   # 유효 실거리 상한
US_CLEAR_MM: int = 3001      # 범위 내 에코 없음 — max_range 로 발행해 부채꼴을 지운다
FIRMWARE_US_CYCLE_MS: int = 420  # 4라운드 × (GAP 5 + WAIT 100). 8채널 프레임 약 2.4Hz

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
# ── 초음파 측정 결과 통계 프레임 (2026-09-24) ─────────────────────────────
# AA 58 seq [ch0: ok clr ffff fffe oth i2c] … [ch7] xor = 3 + 8×6 + 1 = 52바이트.
# 펌웨어 usSendStatFrame() 이 정본. 약 5초(12바퀴)마다 창 안의 횟수를 보내고 0 으로 되돌린다.
# 목적: 센서가 스스로 알리는 동주파수 간섭(0xFFFE)을 다른 실패와 구분해 세기.
US_STAT_FRAME_HEADER: bytes = b"\xaa\x58"
US_STAT_KIND_NAMES = ("ok", "clr", "ffff", "fffe", "oth", "i2c")
US_STAT_KINDS: int = len(US_STAT_KIND_NAMES)
US_STAT_FRAME_LEN: int = 3 + US_CHANNELS * US_STAT_KINDS + 1   # 52
FIRMWARE_US_STAT_EVERY_CYCLES: int = 12

# ── 초음파 레지스터 설정 확인 프레임·시험 명령 (2026-09-24) ─────────────────────
# AA 59 seq [ch0: angle noise] … [ch7] xor = 20바이트. 부팅 때와 시험 명령 뒤에 한 번씩.
# 값은 센서에서 되읽은 것(실패 = 0xFF). 정본은 펌웨어 usApplyConfig().
US_CFG_FRAME_HEADER: bytes = b"\xaa\x59"
US_CFG_FRAME_LEN: int = 3 + 2 * US_CHANNELS + 1   # 20
# 하향 시험 명령(드라이버는 보내지 않는다 — 벤치 스크립트 전용)
US_CMD_RESET: int = 0x30                 # 부팅 기본값으로
US_CMD_NOISE_BASE: int = 0x30            # + 1~5 → 노이즈 저감 레벨, 8채널 모두
US_CMD_SIDE_ANGLE_BASE: int = 0x40       # + 1~4 → 지향각 레벨, 바퀴 옆 두 채널만
US_NOISE_DEFAULT: int = 1

TOUCH_FRAME_HEADER: bytes = b"\xaa\x56"
TOUCH_FRAME_LEN: int = 5
TOUCH_FLAG_CONTACT: int = 0x01   # bit0 = 잡고 있음. bit1~7 예약(0)
FIRMWARE_TOUCH_PERIOD_MS: int = 50   # 20Hz. 판정 유예 0.5초에 10프레임
FIRMWARE_TOUCH_HOLD_MS: int = 200    # 시간 브리지. LOW 본 뒤 이만큼은 '잡음'

# ── 하향 햅틱 명령 (젯슨 → 아두이노, 2026-09-04 신설) ──────────────────
# 상태코드(0~7)와 겹치지 않는 별도 바이트다. applyState()를 거치지 않으므로
# LED·서보는 그대로이고 D10 의 진동모터만 패턴대로 떨린다.
#
# 드라이버 노드(user_guidance_driver_node) 자체는 이 바이트를 스스로 보내지
# 않는다(SENDABLE_STATE_CODES 에 없다) — 미션 매니저가 /vica/haptic_request 로
# 이름을 보내면 그대로 바꿔 흘린다. 미션이 쓰는 때: 손잡이 찾기·놓침(long),
# 잡음 확인(tick), 대기 중 "비카야" 위치 알림(locate, 2026-10-09). bench_test.py --haptic 은 수동으로 쏘는 경로다.
#
# ESTOP·ARRIVED 진입 진동은 이 바이트가 아니라 펌웨어 applyState() 가 스스로
# 낸다(2026-09-30, 7/28 계획서 6.2절 복원) — 상태코드가 바뀔 때만 돈다.
# 정본: docs/superpowers/specs/2026-09-28-touch-haptic-integration-final.md 3절.
HAPTIC_CMD_SHORT: int = 0x10     # 300ms on / 150ms off x 3회 (도착 패턴)
# 1200ms on x 1회 — 손잡이 찾기(힌트·놓침)와 비상정지가 같이 쓴다. 손잡이
# 찾기는 멘트와 함께 반복되고 비상정지는 한 번뿐이라 상황으로 가려진다.
HAPTIC_CMD_LONG: int = 0x11
# 300ms on x 1회 — "잡은 걸 알아챘다"(2026-09-30 신설, 설계 D5). 도착(x3)과
# 횟수로 구별한다. 진행 중인 긴 진동을 덮어써 곧바로 끊는 역할도 한다.
HAPTIC_CMD_TICK: int = 0x12
# 1000ms on / 1000ms off x 2회 — 대기(WAITING) 중 "비카야"를 들었을 때 손잡이 위치를
# 알린다(2026-10-09 사용자). 볼일을 마친 사용자가 대기 장소의 비카를 손으로 찾게 한다.
HAPTIC_CMD_LOCATE: int = 0x13
FIRMWARE_HAPTIC_SHORT_ON_MS: int = 300   # 2026-09-04 150->300. 회전 올라올 시간
FIRMWARE_HAPTIC_SHORT_OFF_MS: int = 150
FIRMWARE_HAPTIC_SHORT_COUNT: int = 3
FIRMWARE_HAPTIC_LONG_ON_MS: int = 1200   # 2026-09-04 800->1200
FIRMWARE_HAPTIC_LOCATE_ON_MS: int = 1000   # 2026-10-09 사용자: 1초씩 두 번, 쉬는 시간 1초
FIRMWARE_HAPTIC_LOCATE_OFF_MS: int = 1000
FIRMWARE_HAPTIC_LOCATE_COUNT: int = 2


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
