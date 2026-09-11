"""상향 터치 프레임 파서 — 순수 함수·클래스 (I/O 없음)."""

from dataclasses import dataclass
from typing import List

from . import protocol

MAX_BUFFER_BYTES = 256


@dataclass(frozen=True)
class TouchFrame:
    """검증을 통과한 상향 터치 프레임 1개."""

    seq: int
    touched: bool


def resolve_contact(touched: bool, uplink_fresh: bool) -> bool:
    """발행할 ``user_contact`` 값을 정한다."""
    return touched and uplink_fresh


def checksum(seq_and_flags: bytes) -> int:
    """seq + flags 2바이트의 xor."""
    x = 0
    for b in seq_and_flags:
        x ^= b
    return x


def _try_parse(chunk: bytes) -> "TouchFrame | None":
    """헤더가 맞는 5바이트 덩어리를 프레임으로 해석한다. 체크섬 불일치는 None."""
    if checksum(chunk[2:4]) != chunk[4]:
        return None
    return TouchFrame(
        seq=chunk[2],
        touched=bool(chunk[3] & protocol.TOUCH_FLAG_CONTACT),
    )


class TouchFrameAccumulator:
    """바이트 스트림을 먹여 완성된 터치 프레임 목록을 돌려받는다."""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> List[TouchFrame]:
        self._buf.extend(data)
        frames: List[TouchFrame] = []

        i = 0
        buf = self._buf
        while len(buf) - i >= protocol.TOUCH_FRAME_LEN:
            if bytes(buf[i : i + 2]) != protocol.TOUCH_FRAME_HEADER:
                i += 1
                continue
            frame = _try_parse(bytes(buf[i : i + protocol.TOUCH_FRAME_LEN]))
            if frame is None:
                i += 1
                continue
            frames.append(frame)
            i += protocol.TOUCH_FRAME_LEN

        del buf[:i]
        if len(buf) > MAX_BUFFER_BYTES:
            del buf[:-MAX_BUFFER_BYTES]

        return frames
