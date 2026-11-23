"""프로토콜 상수와 펌웨어 결합 고정 테스트."""

import re
from pathlib import Path

import pytest

from vica_user_guidance import protocol

PKG_ROOT = Path(__file__).resolve().parents[1]
FIRMWARE_INO = (
    PKG_ROOT / "firmware" / "smart_handle_firmware" / "smart_handle_firmware.ino"
)


def test_state_codes_match_firmware_defines():
    """상태코드 값은 펌웨어 #define과 일치해야 한다."""
    assert protocol.STATE_NORMAL == 0
    assert protocol.STATE_LEFT == 1
    assert protocol.STATE_RIGHT == 2
    assert protocol.STATE_ESTOP == 3
    assert protocol.STATE_LINK_LOST == 4
    assert protocol.STATE_ARRIVED == 5


def test_link_lost_is_not_sendable():
    """ROS는 코드 4를 절대 보내지 않는다."""
    assert protocol.STATE_LINK_LOST not in protocol.SENDABLE_STATE_CODES
    assert protocol.is_sendable(protocol.STATE_LINK_LOST) is False


def test_unimplemented_charging_codes_are_not_sendable():
    """코드 6·7은 펌웨어 미구현이므로 전송 대상이 아니다."""
    assert protocol.is_sendable(protocol.STATE_DOCKED) is True
    assert protocol.is_sendable(protocol.STATE_CHARGED) is False


def test_sendable_codes_are_exactly_eight():
    """전송 가능한 코드는 0,1,2,3,5 와 순찰 8,9,10 여덟 개다(2026-11-14 순찰 3개 추가)."""
    assert protocol.SENDABLE_STATE_CODES == frozenset({0, 1, 2, 3, 5, 6, 8, 9, 10})


def _ino_code() -> str:
    """주석을 뺀 펌웨어 소스."""
    return re.sub(r"//[^\n]*", "", FIRMWARE_INO.read_text(encoding="utf-8"))


def test_patrol_codes_match_firmware_defines():
    """순찰 코드 8·9·10 은 펌웨어 #define 과 같다. 6·7(충전 예약)은 그대로 비어 있다."""
    assert (protocol.STATE_PATROL, protocol.STATE_PATROL_LIGHT,
            protocol.STATE_PATROL_ALARM) == (8, 9, 10)
    assert (protocol.STATE_DOCKED, protocol.STATE_CHARGED) == (6, 7)
    assert protocol.STATE_CODE_MAX == 10
    code = _ino_code()
    assert re.search(r"#define\s+STATE_PATROL\s+8\b", code)
    assert re.search(r"#define\s+STATE_PATROL_LIGHT\s+9\b", code)
    assert re.search(r"#define\s+STATE_PATROL_ALARM\s+10\b", code)
    assert protocol.STATE_NAMES[8] == "PATROL"
    assert protocol.STATE_NAMES[9] == "PATROL_LIGHT"
    assert protocol.STATE_NAMES[10] == "PATROL_ALARM"


def test_patrol_codes_do_not_collide_with_other_downlink_bytes():
    """상태코드 0~10 은 진동(0x10~0x13)·초음파 시험 명령(0x30~)과 겹치지 않는다."""
    others = {protocol.HAPTIC_CMD_SHORT, protocol.HAPTIC_CMD_LONG, protocol.HAPTIC_CMD_TICK,
              protocol.HAPTIC_CMD_LOCATE, protocol.US_CMD_RESET}
    assert min(others) > protocol.STATE_CODE_MAX


def test_firmware_accepts_patrol_codes_but_not_reserved_6_7():
    """펌웨어 수신 범위는 0~5 와 8~10 두 구간이다. 6·7 은 버린다."""
    code = _ino_code()
    assert re.search(r"#define\s+STATE_MAX\s+STATE_ARRIVED\b", code)
    assert re.search(r"#define\s+STATE_PATROL_MIN\s+STATE_PATROL\b", code)
    assert re.search(r"#define\s+STATE_PATROL_MAX\s+STATE_PATROL_ALARM\b", code)
    assert re.search(
        r"\(b >= STATE_MIN && b <= STATE_MAX\)\s*\|\|\s*"
        r"\(b >= STATE_PATROL_MIN && b <= STATE_PATROL_MAX\)", code)


def test_firmware_patrol_colors_are_the_confirmed_ones():
    """연두·빨강은 2026-11-11 실물로 확정. 불 켜짐은 2026-11-16 진노랑으로 바꿈(사용자)."""
    code = _ino_code()
    assert re.search(r"LIME\s*=\s*Adafruit_NeoPixel::Color\(\s*110,\s*255,\s*0\s*\)", code)
    assert re.search(r"DEEP_YELLOW\s*=\s*Adafruit_NeoPixel::Color\(\s*255,\s*170,\s*0\s*\)", code)
    assert re.search(r"RED\s*=\s*Adafruit_NeoPixel::Color\(\s*255,\s*0,\s*0\s*\)", code)
    assert "#define BLINK_MS    300" in code


def _case_body(src: str, case: str) -> str:
    m = re.search(rf"case {case}:(.*?)break;", src, re.S)
    assert m, f"applyState() 에 case {case} 가 없다"
    return m.group(1)


def test_firmware_patrol_patterns():
    """연두 상시 · 진노랑 상시 · 빨강 300ms 점멸. 셋 다 서보 가운데, 진동 없음."""
    code = _ino_code()
    steady = _case_body(code, "STATE_PATROL")
    assert "setBoth(LIME)" in steady and "currentMode = NORMAL" in steady
    light = _case_body(code, "STATE_PATROL_LIGHT")
    assert "setBoth(DEEP_YELLOW)" in light and "currentMode = NORMAL" in light
    alarm = _case_body(code, "STATE_PATROL_ALARM")
    assert "currentMode = BLINK_ALARM" in alarm and "setBoth(RED)" in alarm
    for body in (steady, light, alarm):
        assert "servoMoveTo(SERVO_CENTER)" in body
        assert "hapticStart" not in body
    assert re.search(
        r"currentMode == BLINK_ALARM && now - lastBlink >= BLINK_MS\)\s*\{[^}]*"
        r"setBoth\(blinkState \? RED : OFF\);", code)


def test_firmware_estop_and_link_lost_patterns_unchanged():
    """순찰을 넣으며 비상정지(주황 점멸)·통신 끊김(빨강 상시) 표시는 건드리지 않았다."""
    code = _ino_code()
    assert re.search(
        r"currentMode == BLINK_BOTH && now - lastBlink >= BLINK_MS\)\s*\{[^}]*"
        r"setBoth\(blinkState \? ORANGE : OFF\);", code)
    lost = _case_body(code, "STATE_LINK_LOST")
    assert "setBoth(RED)" in lost and "currentMode = NORMAL" in lost
    assert "applyState(STATE_LINK_LOST);" in code


def test_firmware_arrival_duration_is_3_5_seconds():
    """도착 재생 시간은 3.0초가 아니라 3.5초다."""
    assert protocol.firmware_arrival_duration_sec() == pytest.approx(3.5)


def test_firmware_source_is_in_package():
    """펌웨어가 패키지 안에 있어야 한다."""
    assert FIRMWARE_INO.exists(), f"펌웨어를 찾을 수 없다: {FIRMWARE_INO}"


def test_firmware_constants_match_ino_source():
    """protocol.py의 펌웨어 타이밍 상수가 실제 .ino와 일치하는지 확인한다."""
    source = FIRMWARE_INO.read_text(encoding="utf-8")
    assert f"#define ARRIVE_BLINK_MS    {protocol.FIRMWARE_ARRIVE_BLINK_MS}" in source
    assert (
        f"#define ARRIVE_BLINK_COUNT   {protocol.FIRMWARE_ARRIVE_BLINK_COUNT}" in source
    )
    assert (
        f"#define WATCHDOG_TIMEOUT_MS  {protocol.FIRMWARE_WATCHDOG_TIMEOUT_MS}"
        in source
    )
    assert f"Serial.begin({protocol.FIRMWARE_BAUDRATE})" in source


def test_docked_code_matches_firmware_define():
    """2026-11-18 도킹: dialog_state "docked" → 6(연두 숨쉬기). 펌웨어 STATE_DOCKED 와 같은 값."""
    from pathlib import Path
    from vica_user_guidance.guidance_priority import patrol_state_code
    ino = (Path(__file__).resolve().parents[1] / "firmware" / "smart_handle_firmware"
           / "smart_handle_firmware.ino").read_text(encoding="utf-8")
    assert "#define STATE_DOCKED      6" in ino
    assert patrol_state_code("docked") == protocol.STATE_DOCKED == 6
    assert patrol_state_code("docking") == 0
    assert patrol_state_code("undocking") == 0
