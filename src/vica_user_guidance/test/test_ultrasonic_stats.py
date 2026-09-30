"""초음파 통계 프레임(AA 58) 파서와 펌웨어 상수 일치 시험 (2026-09-24)."""
from pathlib import Path

from vica_user_guidance import protocol
from vica_user_guidance.ultrasonic_frame import FrameAccumulator
from vica_user_guidance.ultrasonic_stats import (
    StatFrameAccumulator,
    checksum,
    format_stat_line,
)

INO = (
    Path(__file__).resolve().parents[1]
    / "firmware" / "smart_handle_firmware" / "smart_handle_firmware.ino"
)


def _frame(seq, counts):
    payload = bytes(v for row in counts for v in row)
    return protocol.US_STAT_FRAME_HEADER + bytes([seq]) + payload + bytes(
        [checksum(bytes([seq]) + payload)]
    )


def _counts():
    return [[ch, 1, 2, 3 if ch == 5 else 0, 0, 4] for ch in range(protocol.US_CHANNELS)]


def test_frame_length_is_52():
    assert protocol.US_STAT_FRAME_LEN == 52
    assert len(_frame(0, _counts())) == 52


def test_parse_roundtrip_and_split_feed():
    raw = b"\x00\x13" + _frame(7, _counts()) + b"\xaa"
    acc = StatFrameAccumulator()
    got = acc.feed(raw[:20]) + acc.feed(raw[20:])
    assert len(got) == 1
    f = got[0]
    assert f.seq == 7
    assert f.counts[5][protocol.US_STAT_KIND_NAMES.index("fffe")] == 3
    assert f.counts[2] == (2, 1, 2, 0, 0, 4)


def test_bad_checksum_is_skipped_but_next_frame_survives():
    bad = bytearray(_frame(1, _counts()))
    bad[-1] ^= 0xFF
    acc = StatFrameAccumulator()
    got = acc.feed(bytes(bad) + _frame(2, _counts()))
    assert [f.seq for f in got] == [2]


def test_distance_parser_ignores_stat_frames():
    """기존 거리 누적기는 통계 프레임을 프레임으로 잡지 않는다."""
    assert FrameAccumulator().feed(_frame(3, _counts()) * 3) == []


def test_format_line_sums_fffe():
    f = StatFrameAccumulator().feed(_frame(9, _counts()))[0]
    line = format_stat_line(f, [f"s{i}" for i in range(8)])
    assert line.endswith("fffe 합 3")
    assert "s5 5/1/2/3/0/4" in line


def test_firmware_defines_match_protocol():
    src = INO.read_text(encoding="utf-8")
    assert "#define US_STAT_H2            0x58" in src
    assert f"#define US_STAT_KINDS         {protocol.US_STAT_KINDS}" in src
    assert (
        f"#define US_STAT_EVERY_CYCLES  {protocol.FIRMWARE_US_STAT_EVERY_CYCLES}" in src
    )
    # 칸 순서 = protocol.US_STAT_KIND_NAMES
    assert "enum UsStatKind { US_ST_OK, US_ST_CLEAR, US_ST_FFFF, US_ST_FFFE, US_ST_OTHER, US_ST_I2C };" in src


# ── 설정 확인 프레임(AA 59)·시험 명령 ───────────────────────────────────
from vica_user_guidance.ultrasonic_stats import (  # noqa: E402
    ConfigFrameAccumulator,
    format_config_line,
)


def _cfg(seq, levels):
    payload = bytes(v for pair in levels for v in pair)
    return protocol.US_CFG_FRAME_HEADER + bytes([seq]) + payload + bytes(
        [checksum(bytes([seq]) + payload)]
    )


def test_config_frame_roundtrip():
    lv = [(3, 1)] * 8
    lv[0] = (4, 3)
    lv[3] = (4, 0xFF)
    got = ConfigFrameAccumulator().feed(b"\x01" + _cfg(5, lv))
    assert len(got) == 1 and got[0].levels[0] == (4, 3)
    line = format_config_line(got[0], [f"s{i}" for i in range(8)])
    assert "s0 각4/잡음3" in line and "s3 각4/잡음?" in line


def test_distance_and_stat_parsers_ignore_config_frames():
    raw = _cfg(1, [(3, 1)] * 8) * 3
    assert FrameAccumulator().feed(raw) == []
    assert StatFrameAccumulator().feed(raw) == []


def test_bench_commands_do_not_collide_with_state_or_haptic_codes():
    cmds = {protocol.US_CMD_RESET}
    cmds |= {protocol.US_CMD_NOISE_BASE + i for i in range(1, 6)}
    cmds |= {protocol.US_CMD_SIDE_ANGLE_BASE + i for i in range(1, 5)}
    used = set(protocol.STATE_NAMES) | {
        protocol.HAPTIC_CMD_SHORT, protocol.HAPTIC_CMD_LONG, protocol.HAPTIC_CMD_TICK}
    assert not (cmds & used)
    assert not (cmds & set(protocol.SENDABLE_STATE_CODES))


def test_firmware_bench_defines_match_protocol():
    src = INO.read_text(encoding="utf-8")
    assert "#define US_CFG_H2             0x59" in src
    assert f"#define US_CMD_RESET          0x{protocol.US_CMD_RESET:02X}" in src
    assert f"#define US_CMD_NOISE_BASE     0x{protocol.US_CMD_NOISE_BASE:02X}" in src
    assert f"#define US_CMD_SIDE_ANGLE_BASE 0x{protocol.US_CMD_SIDE_ANGLE_BASE:02X}" in src
    assert f"#define US_NOISE_DEFAULT      {protocol.US_NOISE_DEFAULT}" in src


# ── 채널별 칠하는 폭 (2026-09-24) ─────────────────────────────────────
import pytest  # noqa: E402
import yaml  # noqa: E402

from vica_user_guidance.ultrasonic_fov import resolve_channel_fov  # noqa: E402

CFG = Path(__file__).resolve().parents[1] / "config" / "user_guidance.yaml"


def test_resolve_channel_fov():
    assert resolve_channel_fov(0.524, [1.047, -1, 0, 1.047, -1, -1, -1, -1], 8) == [
        1.047, 0.524, 0.524, 1.047, 0.524, 0.524, 0.524, 0.524]
    with pytest.raises(ValueError):
        resolve_channel_fov(0.524, [1.0] * 7, 8)


def test_side_fov_matches_firmware_side_angle_level():
    """칠하는 폭 60°(1.047) 채널 = 펌웨어 부팅 지향각 레벨 4 채널."""
    p = yaml.safe_load(CFG.read_text(encoding="utf-8"))["user_guidance_driver_node"]["ros__parameters"]
    fov = resolve_channel_fov(p["ultrasonic_fov_rad"], p["ultrasonic_fov_rad_per_channel"], 8)
    src = INO.read_text(encoding="utf-8")
    line = next(ln for ln in src.splitlines() if ln.startswith("const uint8_t US_ANGLE_LEVEL_CH"))
    levels = [int(x) for x in line.split("{")[1].split("}")[0].split(",")]
    level_to_rad = {1: 0.524, 2: 0.698, 3: 0.873, 4: 1.047}
    for ch in range(8):
        if levels[ch] == 4:
            assert fov[ch] == pytest.approx(level_to_rad[4])
