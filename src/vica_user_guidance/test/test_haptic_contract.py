"""진동 계약 (2026-09-30, 터치×진동 연동 설계 3절).

바이트 값은 네 곳에 흩어져 있다 — protocol.py · 펌웨어 · 드라이버 패턴표 · bench_test.
한 곳만 바뀌면 "보냈는데 안 떨림"이 로그 없이 난다. 여기서 한데 묶어 고정한다.
"""
import ast
import re
from pathlib import Path

from vica_user_guidance import protocol

PKG = Path(__file__).resolve().parents[1]
INO = PKG / "firmware" / "smart_handle_firmware" / "smart_handle_firmware.ino"
BENCH = PKG / "firmware" / "bench_test.py"
DRIVER = PKG / "vica_user_guidance" / "user_guidance_driver_node.py"


def _ino() -> str:
    return INO.read_text(encoding="utf-8")


def _case_body(src: str, case: str) -> str:
    """applyState() 의 `case X:` 부터 그 break 까지."""
    m = re.search(rf"case {case}:(.*?)break;", src, re.S)
    assert m, f"applyState() 에 case {case} 가 없다"
    return m.group(1)


def test_tick_code_is_new_and_distinct():
    assert protocol.HAPTIC_CMD_TICK == 0x12
    codes = {protocol.HAPTIC_CMD_SHORT, protocol.HAPTIC_CMD_LONG, protocol.HAPTIC_CMD_TICK}
    assert len(codes) == 3
    assert not (codes & set(protocol.STATE_NAMES))


def test_firmware_defines_match_protocol():
    src = _ino()
    assert f"#define HAPTIC_CMD_SHORT      0x{protocol.HAPTIC_CMD_SHORT:02X}" in src
    assert f"#define HAPTIC_CMD_LONG       0x{protocol.HAPTIC_CMD_LONG:02X}" in src
    assert f"#define HAPTIC_CMD_TICK       0x{protocol.HAPTIC_CMD_TICK:02X}" in src


def test_firmware_handles_tick_byte():
    """loop() 가 0x12 를 받아 한 번 떨린다. 없으면 '그 밖의 값은 버린다'로 조용히 사라진다."""
    src = _ino()
    assert re.search(r"b == HAPTIC_CMD_TICK\)\s*\{\s*hapticStart\(1, HAPTIC_SHORT_ON_MS, 0\);", src)


def test_firmware_state_entry_vibrations():
    """비상정지 = 길게 ×1, 도착 = 짧게 ×3 (D4·D6). applyState() 는 상태가 바뀔 때만 돈다."""
    src = _ino()
    assert "if (state == currentState) return;" in src   # edge 트리거의 근거
    assert "hapticStart(1, HAPTIC_LONG_ON_MS, 0);" in _case_body(src, "STATE_ESTOP")
    assert ("hapticStart(HAPTIC_SHORT_COUNT, HAPTIC_SHORT_ON_MS, HAPTIC_SHORT_OFF_MS);"
            in _case_body(src, "STATE_ARRIVED"))
    # 좌우회전 때는 울리지 않는다 (D6)
    for case in ("STATE_LEFT", "STATE_RIGHT", "STATE_NORMAL", "STATE_LINK_LOST"):
        assert "hapticStart" not in _case_body(src, case), case


def _driver_patterns() -> dict:
    tree = ast.parse(DRIVER.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign)
                and any(getattr(t, "id", None) == "HAPTIC_PATTERNS" for t in node.targets)):
            return {k.value: v.attr for k, v in zip(node.value.keys, node.value.values)}
    raise AssertionError("HAPTIC_PATTERNS 가 없다")


def test_driver_pattern_table():
    """미션이 보내는 이름 세 개 = 드라이버가 아는 이름 세 개."""
    assert _driver_patterns() == {
        "short": "HAPTIC_CMD_SHORT",
        "long": "HAPTIC_CMD_LONG",
        "tick": "HAPTIC_CMD_TICK",
    }


def test_bench_tool_knows_tick():
    text = BENCH.read_text(encoding="utf-8")
    assert re.search(r'"tick":\s*\(0x12,', text)
