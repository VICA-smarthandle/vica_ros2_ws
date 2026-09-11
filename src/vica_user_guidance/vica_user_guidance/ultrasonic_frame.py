"""상향 초음파 프레임 파서 — 순수 함수·클래스 (I/O 없음)."""

from dataclasses import dataclass
from typing import List, Tuple

from . import protocol

MAX_BUFFER_BYTES = 256


@dataclass(frozen=True)
class UltrasonicFrame:
    """검증을 통과한 상향 프레임 1개."""

    seq: int
    distances_mm: Tuple[int, ...]


def checksum(seq_and_payload: bytes) -> int:
    """seq + 거리 4바이트(총 5바이트)의 xor."""
    x = 0
    for b in seq_and_payload:
        x ^= b
    return x


def _try_parse(chunk: bytes) -> "UltrasonicFrame | None":
    """헤더가 맞는 8바이트 덩어리를 프레임으로 해석한다. 체크섬 불일치는 None."""
    if checksum(chunk[2:7]) != chunk[7]:
        return None
    d0 = chunk[3] | (chunk[4] << 8)
    d1 = chunk[5] | (chunk[6] << 8)
    return UltrasonicFrame(seq=chunk[2], distances_mm=(d0, d1))


class FrameAccumulator:
    """바이트 스트림을 먹여 완성된 프레임 목록을 돌려받는다."""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> List[UltrasonicFrame]:
        self._buf.extend(data)
        frames: List[UltrasonicFrame] = []

        i = 0
        buf = self._buf
        while len(buf) - i >= protocol.US_FRAME_LEN:
            if bytes(buf[i : i + 2]) != protocol.US_FRAME_HEADER:
                i += 1
                continue
            frame = _try_parse(bytes(buf[i : i + protocol.US_FRAME_LEN]))
            if frame is None:
                i += 1
                continue
            frames.append(frame)
            i += protocol.US_FRAME_LEN

        del buf[:i]
        if len(buf) > MAX_BUFFER_BYTES:
            del buf[:-MAX_BUFFER_BYTES]

        return frames
