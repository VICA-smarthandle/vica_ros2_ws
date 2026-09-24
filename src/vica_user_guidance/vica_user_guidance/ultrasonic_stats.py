"""상향 초음파 통계 프레임 파서 — 순수 함수·클래스 (I/O 없음).

프레임 형식의 정본은 ``firmware/smart_handle_firmware/smart_handle_firmware.ino``
의 ``usSendStatFrame()``이며, 상수는 :mod:`protocol` 에 있다.

    AA 58 seq [ch0: ok clr ffff fffe oth i2c] … [ch7: …] xor     52바이트 · 약 5초마다
                                                     └ seq ^ 48바이트

칸 뜻(창 하나 = 약 5초 동안의 횟수, 255 에서 멈춤):
    ok   1~3000 mm 실거리             clr  0xFFFD 범위 내 에코 없음
    ffff 측정 미완료                  fffe 동주파수 간섭(센서가 스스로 알린 것)
    oth  그 밖의 범위 밖 값           i2c  트리거 쓰기 또는 거리 읽기 실패

거리 프레임(:mod:`ultrasonic_frame`)·터치 프레임과 **같은 스트림을 각자 훑는다**
(touch_frame 과 같은 이유 — 기존 누적기의 반환 타입을 바꾸지 않는다).
판정·발행은 하지 않는다. 노드가 로그로 남긴다.
"""

from dataclasses import dataclass
from typing import List, Tuple

from . import protocol

MAX_BUFFER_BYTES = 8 * protocol.US_STAT_FRAME_LEN


@dataclass(frozen=True)
class UltrasonicStatFrame:
    """검증을 통과한 통계 프레임 1개. counts[ch][kind] (kind 순서 = US_STAT_KIND_NAMES)."""

    seq: int
    counts: Tuple[Tuple[int, ...], ...]


def checksum(seq_and_payload: bytes) -> int:
    x = 0
    for b in seq_and_payload:
        x ^= b
    return x


def _try_parse(chunk: bytes) -> "UltrasonicStatFrame | None":
    if checksum(chunk[2:-1]) != chunk[-1]:
        return None
    k = protocol.US_STAT_KINDS
    counts = tuple(
        tuple(chunk[3 + ch * k + i] for i in range(k))
        for ch in range(protocol.US_CHANNELS)
    )
    return UltrasonicStatFrame(seq=chunk[2], counts=counts)


class StatFrameAccumulator:
    """바이트 스트림을 먹여 완성된 통계 프레임 목록을 돌려받는다.

    체크섬이 깨진 헤더 후보는 1바이트만 버린다(다른 프레임 중간에 우연히
    0xAA 0x58 이 나와도 뒤따르는 진짜 프레임을 잃지 않게).
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> List[UltrasonicStatFrame]:
        self._buf.extend(data)
        frames: List[UltrasonicStatFrame] = []
        n = protocol.US_STAT_FRAME_LEN
        i = 0
        buf = self._buf
        while len(buf) - i >= n:
            if bytes(buf[i : i + 2]) != protocol.US_STAT_FRAME_HEADER:
                i += 1
                continue
            frame = _try_parse(bytes(buf[i : i + n]))
            if frame is None:
                i += 1
                continue
            frames.append(frame)
            i += n
        del buf[:i]
        if len(buf) > MAX_BUFFER_BYTES:
            del buf[:-MAX_BUFFER_BYTES]
        return frames


def format_stat_line(frame: UltrasonicStatFrame, channel_names) -> str:
    """로그 한 줄. 채널마다 'ok/clr/ffff/fffe/oth/i2c' 를 적는다.

    예) "[US 통계 #3] left_wheel 12/0/0/0/0/0 · front_left 3/9/0/1/0/0 …  fffe 합 1"
    """
    parts = []
    fffe = protocol.US_STAT_KIND_NAMES.index("fffe")
    total_fffe = 0
    for ch, row in enumerate(frame.counts):
        name = channel_names[ch] if ch < len(channel_names) else f"ch{ch}"
        parts.append(f"{name} " + "/".join(str(v) for v in row))
        total_fffe += row[fffe]
    return (
        f"[US 통계 #{frame.seq}] (ok/clr/ffff/fffe/oth/i2c) "
        + " · ".join(parts)
        + f"  fffe 합 {total_fffe}"
    )
