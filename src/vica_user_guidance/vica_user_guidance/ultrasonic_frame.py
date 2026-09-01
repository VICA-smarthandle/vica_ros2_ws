"""상향 초음파 프레임 파서 — 순수 함수·클래스 (I/O 없음).

프레임 형식의 정본은 ``firmware/smart_handle_firmware/smart_handle_firmware.ino``
의 ``usSendFrame()``이며, 상수는 :mod:`protocol` 에 있다.

시리얼은 바이트 경계를 보장하지 않는다 — 한 번의 read 에 프레임이 반쪽만
오거나 여러 개가 붙어 올 수 있고, USB 재연결 직후에는 쓰레기 바이트로 시작할
수 있다. 그래서 파서는 스트림 누적기(:class:`FrameAccumulator`)로 만들고,
동기가 깨지면 1바이트씩 밀며 다음 헤더를 다시 찾는다.
"""

from dataclasses import dataclass
from typing import List, Tuple

from . import protocol

# 쓰레기 바이트가 계속 들어와도 버퍼가 자라지 않게 자른다. 프레임 8바이트의
# 32배면 정상 트래픽(4.8Hz x 8B)에서는 절대 차지 않는다.
MAX_BUFFER_BYTES = 256


@dataclass(frozen=True)
class UltrasonicFrame:
    """검증을 통과한 상향 프레임 1개.

    distances_mm 는 펌웨어 원시값 그대로다:
    0 = 채널 무효, 1~3000 = 실거리 mm, 3001 = 범위 내 에코 없음(clear).
    해석(발행/미발행/clear)은 노드 쪽 책임이다.
    """

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
    """바이트 스트림을 먹여 완성된 프레임 목록을 돌려받는다.

    체크섬이 깨진 헤더 후보는 프레임으로 소비하지 않고 1바이트만 버린다 —
    진짜 프레임 중간에 우연히 0xAA 0x55 가 나온 경우, 8바이트를 통째로
    버리면 뒤따르는 진짜 프레임까지 같이 잃기 때문이다.
    """

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

        # 소비한 앞부분을 버리고, 남은 미완성 꼬리는 다음 feed 를 기다린다.
        del buf[:i]
        if len(buf) > MAX_BUFFER_BYTES:
            del buf[:-MAX_BUFFER_BYTES]

        return frames
