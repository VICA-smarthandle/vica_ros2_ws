"""진동 계약 (2026-09-30, 터치×진동 연동 설계 3절)."""
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


def test_locate_code_is_new_and_distinct():
    """대기 중 "비카야" 위치 알림(2026-10-09). 상태코드·다른 진동·초음파 명령과 안 겹친다."""
    assert protocol.HAPTIC_CMD_LOCATE == 0x13
    codes = {protocol.HAPTIC_CMD_SHORT, protocol.HAPTIC_CMD_LONG,
             protocol.HAPTIC_CMD_TICK, protocol.HAPTIC_CMD_LOCATE}
    assert len(codes) == 4
    assert not (codes & set(protocol.STATE_NAMES))
    code = re.sub(r"//[^\n]*", "", _ino())
    assert re.findall(r"\b0x13\b", code) == ["0x13"]
    assert re.search(r"#define HAPTIC_CMD_LOCATE\s+0x13\b", code)


def test_firmware_defines_match_protocol():
    src = _ino()
    assert f"#define HAPTIC_CMD_SHORT      0x{protocol.HAPTIC_CMD_SHORT:02X}" in src
    assert f"#define HAPTIC_CMD_LONG       0x{protocol.HAPTIC_CMD_LONG:02X}" in src
    assert f"#define HAPTIC_CMD_TICK       0x{protocol.HAPTIC_CMD_TICK:02X}" in src
    assert f"#define HAPTIC_CMD_LOCATE     0x{protocol.HAPTIC_CMD_LOCATE:02X}" in src
    assert f"#define HAPTIC_LOCATE_ON_MS   {protocol.FIRMWARE_HAPTIC_LOCATE_ON_MS}" in src
    assert f"#define HAPTIC_LOCATE_OFF_MS  {protocol.FIRMWARE_HAPTIC_LOCATE_OFF_MS}" in src
    assert f"#define HAPTIC_LOCATE_COUNT   {protocol.FIRMWARE_HAPTIC_LOCATE_COUNT}" in src
    assert (protocol.FIRMWARE_HAPTIC_LOCATE_ON_MS, protocol.FIRMWARE_HAPTIC_LOCATE_OFF_MS,
            protocol.FIRMWARE_HAPTIC_LOCATE_COUNT) == (1000, 1000, 2)


def test_firmware_handles_tick_byte():
    """loop() 가 0x12 를 받아 한 번 떨린다. 없으면 '그 밖의 값은 버린다'로 조용히 사라진다."""
    src = _ino()
    assert re.search(r"b == HAPTIC_CMD_TICK\)\s*\{\s*hapticStart\(1, HAPTIC_SHORT_ON_MS, 0\);", src)


def test_firmware_handles_locate_byte():
    """loop() 가 0x13 을 받아 1초씩 두 번 떤다. 없으면 '그 밖의 값은 버린다'로 조용히 사라진다."""
    src = _ino()
    assert re.search(r"b == HAPTIC_CMD_LOCATE\)\s*\{\s*hapticStart\(HAPTIC_LOCATE_COUNT, "
                     r"HAPTIC_LOCATE_ON_MS, HAPTIC_LOCATE_OFF_MS\);", src)


def test_firmware_state_entry_vibrations():
    """비상정지 = 길게 ×1, 도착 = 짧게 ×3 (D4·D6). applyState() 는 상태가 바뀔 때만 돈다."""
    src = _ino()
    assert "if (state == currentState) return;" in src
    assert "hapticStart(1, HAPTIC_LONG_ON_MS, 0);" in _case_body(src, "STATE_ESTOP")
    assert ("hapticStart(HAPTIC_SHORT_COUNT, HAPTIC_SHORT_ON_MS, HAPTIC_SHORT_OFF_MS);"
            in _case_body(src, "STATE_ARRIVED"))
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
    """미션이 보내는 이름 = 드라이버가 아는 이름."""
    assert _driver_patterns() == {
        "short": "HAPTIC_CMD_SHORT",
        "long": "HAPTIC_CMD_LONG",
        "tick": "HAPTIC_CMD_TICK",
        "locate": "HAPTIC_CMD_LOCATE",
    }


def test_bench_tool_knows_tick():
    text = BENCH.read_text(encoding="utf-8")
    assert re.search(r'"tick":\s*\(0x12,', text)


def test_bench_tool_knows_locate():
    text = BENCH.read_text(encoding="utf-8")
    assert re.search(r'"locate":\s*\(0x13,', text)
