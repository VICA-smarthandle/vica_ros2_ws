"""프로토콜 상수와 펌웨어 결합 고정 테스트."""

from pathlib import Path

import pytest

from vica_user_guidance import protocol

PKG_ROOT = Path(__file__).resolve().parents[1]
FIRMWARE_INO = (
    PKG_ROOT.parents[2]
    / "source_file"
    / "smart_handle_firmware"
    / "smart_handle_firmware.ino"
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
    """ROS는 코드 4를 절대 보내지 않는다.

    보내면 펌웨어의 lastRxMillis가 갱신되어 실제 단절을 영원히 감지하지 못한다.
    """
    assert protocol.STATE_LINK_LOST not in protocol.SENDABLE_STATE_CODES
    assert protocol.is_sendable(protocol.STATE_LINK_LOST) is False


def test_unimplemented_charging_codes_are_not_sendable():
    """코드 6·7은 펌웨어 미구현이므로 전송 대상이 아니다."""
    assert protocol.is_sendable(protocol.STATE_CHARGING) is False
    assert protocol.is_sendable(protocol.STATE_CHARGED) is False


def test_sendable_codes_are_exactly_five():
    """전송 가능한 코드는 0,1,2,3,5 다섯 개다."""
    assert protocol.SENDABLE_STATE_CODES == frozenset({0, 1, 2, 3, 5})


def test_firmware_arrival_duration_is_3_5_seconds():
    """도착 재생 시간은 3.0초가 아니라 3.5초다.

    ON/OFF 각 3회 + 마지막 소등 유지 1프레임 = 7프레임 x 500ms.
    초기 계획서의 "3.0초" 오기가 재발하면 여기서 잡힌다.
    """
    assert protocol.firmware_arrival_duration_sec() == pytest.approx(3.5)


@pytest.mark.skipif(
    not FIRMWARE_INO.exists(), reason="펌웨어 .ino가 이 경로에 없음 (이동했을 수 있음)"
)
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
