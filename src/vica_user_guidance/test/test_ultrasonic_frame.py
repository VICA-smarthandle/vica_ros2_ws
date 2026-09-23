"""상향 초음파 프레임 파서 테스트 (2026-09-23 8채널).

시리얼은 바이트 경계를 보장하지 않는다 — 반쪽 프레임, 붙은 프레임, 재연결 직후
쓰레기 바이트가 전부 정상 입력이다. 파서는 그 어떤 입력에도 프레임을 잃거나
예외를 던지면 안 된다.
"""

from vica_user_guidance import protocol
from vica_user_guidance.ultrasonic_frame import (
    MAX_BUFFER_BYTES,
    FrameAccumulator,
    UltrasonicFrame,
    checksum,
)

N = protocol.US_CHANNELS


def dists(*first):
    """앞 몇 채널만 주면 나머지는 0(무효)으로 채운 8채널 튜플."""
    return tuple(first) + (0,) * (N - len(first))


def build(seq, *first):
    """펌웨어 usSendFrame8()과 같은 20바이트 프레임을 만든다."""
    payload = bytes([seq & 0xFF])
    for d in dists(*first):
        payload += bytes([d & 0xFF, (d >> 8) & 0xFF])
    return protocol.US_FRAME_HEADER + payload + bytes([checksum(payload)])


def legacy(seq, d0, d1):
    """펌웨어가 호환용으로 함께 보내는 옛 2채널 프레임(AA 55, 8B)."""
    payload = bytes([seq & 0xFF, d0 & 0xFF, (d0 >> 8) & 0xFF, d1 & 0xFF, (d1 >> 8) & 0xFF])
    return protocol.US_LEGACY_FRAME_HEADER + payload + bytes([checksum(payload)])


def test_frame_length_matches_protocol():
    assert len(build(0)) == protocol.US_FRAME_LEN == 20
    assert len(legacy(0, 0, 0)) == protocol.US_LEGACY_FRAME_LEN == 8


def test_single_frame():
    acc = FrameAccumulator()
    frames = acc.feed(build(7, 682, 224))
    assert frames == [UltrasonicFrame(seq=7, distances_mm=dists(682, 224))]


def test_all_eight_channels_in_position_order():
    """채널 순서 = 자리 번호 순. 뒤섞이면 엉뚱한 frame_id 로 나간다."""
    acc = FrameAccumulator()
    values = (64, 57, 62, 66, 81, 54, 45, 43)   # 2026-09-22 10cm 장애물 실측 순서
    frames = acc.feed(build(1, *values))
    assert frames[0].distances_mm == values


def test_multiple_frames_in_one_feed():
    acc = FrameAccumulator()
    frames = acc.feed(build(1, 100, 200) + build(2, 101, 201))
    assert [f.seq for f in frames] == [1, 2]


def test_frame_split_across_feeds():
    """USB 는 프레임을 반쪽씩 줄 수 있다. 꼬리를 버리면 프레임이 샌다."""
    acc = FrameAccumulator()
    raw = build(3, 597, 213)
    assert acc.feed(raw[:11]) == []
    frames = acc.feed(raw[11:])
    assert frames == [UltrasonicFrame(seq=3, distances_mm=dists(597, 213))]


def test_garbage_prefix_resyncs():
    """재연결 직후 스트림 중간부터 읽으면 앞이 쓰레기다."""
    acc = FrameAccumulator()
    frames = acc.feed(b"\x01\x00\xaa\x03" + build(9, 250, 3001))
    assert len(frames) == 1
    assert frames[0].distances_mm[:2] == (250, 3001)


def test_bad_checksum_dropped_next_frame_survives():
    acc = FrameAccumulator()
    bad = bytearray(build(4, 100, 100))
    bad[-1] ^= 0xFF
    frames = acc.feed(bytes(bad) + build(5, 300, 400))
    assert [f.seq for f in frames] == [5]


def test_false_header_inside_data_does_not_eat_real_frame():
    """거리값에 우연히 AA 57 이 나올 수 있다(예: d0L=0xAA, d0H=0x57).

    체크섬이 깨진 헤더 후보에서 프레임 길이만큼 통째로 버리면 뒤따르는 진짜
    프레임까지 같이 잃는다 — 1바이트만 밀어야 한다.
    """
    acc = FrameAccumulator()
    fake = b"\xaa\x57" + b"\x00" * 10   # 헤더 모양이지만 체크섬이 안 맞는 쓰레기
    frames = acc.feed(fake + build(6, 682, 224))
    assert [f.seq for f in frames] == [6]


def test_legacy_two_channel_frames_are_skipped_not_misparsed():
    """펌웨어가 함께 보내는 옛 AA 55 프레임은 무시되어야 하고, 뒤의 8채널 프레임은 산다."""
    acc = FrameAccumulator()
    stream = legacy(1, 1000, 1200) + build(1, 64, 1000, 1200) + legacy(2, 1001, 1201) + build(2, 65)
    frames = acc.feed(stream)
    assert [f.seq for f in frames] == [1, 2]
    assert frames[0].distances_mm[:3] == (64, 1000, 1200)


def test_value_semantics_passthrough():
    """0(무효)·3001(clear)·실거리는 파서가 해석하지 않고 그대로 넘긴다."""
    acc = FrameAccumulator()
    frames = acc.feed(
        build(1, protocol.US_DIST_INVALID, protocol.US_CLEAR_MM) + build(2, 1, 3000)
    )
    assert frames[0].distances_mm[:2] == (0, 3001)
    assert frames[1].distances_mm[:2] == (1, 3000)


def test_seq_wraps_are_parser_agnostic():
    acc = FrameAccumulator()
    frames = acc.feed(build(255, 10, 20) + build(0, 11, 21))
    assert [f.seq for f in frames] == [255, 0]


def test_garbage_flood_does_not_grow_buffer():
    """헤더 없는 쓰레기가 계속 들어와도 메모리가 자라면 안 된다."""
    acc = FrameAccumulator()
    for _ in range(100):
        assert acc.feed(b"\x01" * 100) == []
    assert len(acc._buf) <= MAX_BUFFER_BYTES
    # 홍수 뒤에도 정상 프레임은 파싱된다
    assert acc.feed(build(1, 50, 60)) == [
        UltrasonicFrame(seq=1, distances_mm=dists(50, 60))
    ]
