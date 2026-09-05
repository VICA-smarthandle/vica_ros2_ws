"""상향 터치 프레임 파서 테스트.

시리얼은 바이트 경계를 보장하지 않는다. 프레임이 반쪽만 오거나, 여러 개가 붙어
오거나, 초음파 프레임과 섞여 온다. 여기서 고정하는 것은 **그 모든 경우에
'잡았다/놓았다'를 잃지 않는가**다.
"""

import pytest

from vica_user_guidance import protocol
from vica_user_guidance.touch_frame import (
    TouchFrame,
    TouchFrameAccumulator,
    checksum,
)
from vica_user_guidance.ultrasonic_frame import FrameAccumulator


def touch_bytes(seq: int, touched: bool) -> bytes:
    """펌웨어가 보낼 5바이트를 만든다."""
    flags = protocol.TOUCH_FLAG_CONTACT if touched else 0x00
    body = bytes([seq, flags])
    return protocol.TOUCH_FRAME_HEADER + body + bytes([checksum(body)])


def us_bytes(seq: int, d0: int = 1000, d1: int = 1200) -> bytes:
    """비교용 초음파 8바이트."""
    body = bytes([seq, d0 & 0xFF, d0 >> 8, d1 & 0xFF, d1 >> 8])
    x = 0
    for b in body:
        x ^= b
    return protocol.US_FRAME_HEADER + body + bytes([x])


# ── 기본 ───────────────────────────────────────────────


def test_frame_length_matches_protocol():
    assert len(touch_bytes(0, False)) == protocol.TOUCH_FRAME_LEN


def test_touched_and_released():
    acc = TouchFrameAccumulator()
    assert acc.feed(touch_bytes(1, True)) == [TouchFrame(seq=1, touched=True)]
    assert acc.feed(touch_bytes(2, False)) == [TouchFrame(seq=2, touched=False)]


def test_reserved_bits_do_not_leak_into_touched():
    """bit1~7 은 예약이다. 나중에 뭘 넣어도 bit0 판정이 흔들리면 안 된다."""
    body = bytes([7, protocol.TOUCH_FLAG_CONTACT | 0xFE])
    raw = protocol.TOUCH_FRAME_HEADER + body + bytes([checksum(body)])
    assert TouchFrameAccumulator().feed(raw) == [TouchFrame(seq=7, touched=True)]


# ── 스트림이 깨질 때 ───────────────────────────────────


def test_split_across_reads():
    """한 프레임이 두 번의 read 로 쪼개져 와도 잃지 않는다."""
    acc = TouchFrameAccumulator()
    raw = touch_bytes(3, True)
    assert acc.feed(raw[:2]) == []
    assert acc.feed(raw[2:]) == [TouchFrame(seq=3, touched=True)]


def test_several_frames_in_one_read():
    acc = TouchFrameAccumulator()
    raw = touch_bytes(1, True) + touch_bytes(2, False) + touch_bytes(3, True)
    got = acc.feed(raw)
    assert [f.touched for f in got] == [True, False, True]


def test_garbage_prefix_resyncs():
    """USB 재연결 직후의 쓰레기 바이트로 시작해도 다음 프레임을 찾는다."""
    acc = TouchFrameAccumulator()
    got = acc.feed(b"\x00\xff\xaa\x12" + touch_bytes(9, True))
    assert got == [TouchFrame(seq=9, touched=True)]


def test_bad_checksum_drops_one_byte_not_the_frame():
    """체크섬이 깨진 헤더 후보 때문에 뒤따르는 진짜 프레임을 잃으면 안 된다."""
    acc = TouchFrameAccumulator()
    broken = bytearray(touch_bytes(4, True))
    broken[-1] ^= 0xFF
    got = acc.feed(bytes(broken) + touch_bytes(5, False))
    assert got == [TouchFrame(seq=5, touched=False)]


def test_buffer_does_not_grow_without_bound():
    acc = TouchFrameAccumulator()
    for _ in range(50):
        acc.feed(b"\xaa" * 64)
    assert len(acc._buf) <= 256


# ── 초음파와 한 포트에서 섞일 때 ───────────────────────
#
# 두 프레임은 헤더 두 번째 바이트만 다르다(0x55 / 0x56). 서로를 삼키면
# 한쪽이 조용히 죽는다 — 이 두 시험이 그 사고를 막는다.


def test_touch_survives_ultrasonic_traffic():
    acc = TouchFrameAccumulator()
    stream = us_bytes(1) + touch_bytes(1, True) + us_bytes(2) + touch_bytes(2, False)
    got = acc.feed(stream)
    assert [f.touched for f in got] == [True, False]


def test_ultrasonic_survives_touch_traffic():
    """반대 방향. 초음파 파서는 손대지 않았으므로 그대로여야 한다."""
    acc = FrameAccumulator()
    stream = touch_bytes(1, True) + us_bytes(1) + touch_bytes(2, False) + us_bytes(2)
    got = acc.feed(stream)
    assert [f.seq for f in got] == [1, 2]
    assert all(f.distances_mm == (1000, 1200) for f in got)


def test_interleaved_mid_frame():
    """초음파 프레임 바이트 사이에 터치 헤더 모양이 우연히 끼어도 견딘다."""
    acc = TouchFrameAccumulator()
    noise = b"\xaa\x56\x00"          # 헤더 모양이지만 뒤가 모자란 쓰레기
    got = acc.feed(noise + touch_bytes(11, True))
    assert got == [TouchFrame(seq=11, touched=True)]


# ── 상향이 끊겼을 때의 계약 ────────────────────────────


@pytest.mark.parametrize(
    "touched,fresh,expected",
    [
        (True, True, True),      # 잡고 있고 상향도 살아 있다
        (False, True, False),    # 놓았다
        (True, False, False),    # **죽은 센서가 '잡고 있다'고 말하면 안 된다**
        (False, False, False),
    ],
)
def test_contact_is_false_when_uplink_is_stale(touched, fresh, expected):
    from vica_user_guidance.touch_frame import resolve_contact

    assert resolve_contact(touched, fresh) is expected
