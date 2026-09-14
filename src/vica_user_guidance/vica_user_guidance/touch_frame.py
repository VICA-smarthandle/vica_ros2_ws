"""상향 터치 프레임 파서 — 순수 함수·클래스 (I/O 없음).

프레임 형식의 정본은 ``firmware/smart_handle_firmware/smart_handle_firmware.ino``
의 ``touchSendFrame()``이며, 상수는 :mod:`protocol` 에 있다.

    AA 56 seq flags xor        5바이트 · 20 Hz
              │     └ seq ^ flags
              └ bit0 = 잡고 있음. bit1~7 예약(0)

``ultrasonic_frame`` 과 **같은 스트림을 각자 훑는다.** 한 누적기가 두 프레임을
모두 돌려주게 만들지 않은 이유는, 그렇게 하면 ``feed()`` 의 반환 타입이 바뀌어
초음파 쪽 호출자와 시험이 전부 따라 움직이기 때문이다. 지금 초음파 경로는
실주행으로 확인된 상태라 건드리지 않는 편이 싸다.

두 번 훑는 비용은 무시할 수준이다 — 상향 트래픽이 초당 약 200바이트다.

**판정은 여기서 하지 않는다.** 3초 진입도 0.5초 놓침도 mission_manager 몫이다.
이 모듈은 "그 순간 잡고 있었나"까지만 말한다.
"""

from dataclasses import dataclass
from typing import List

from . import protocol

# 쓰레기 바이트가 계속 들어와도 버퍼가 자라지 않게 자른다. 5바이트의 51배면
# 정상 트래픽(20Hz x 5B)에서는 절대 차지 않는다.
MAX_BUFFER_BYTES = 256


@dataclass(frozen=True)
class TouchFrame:
    """검증을 통과한 상향 터치 프레임 1개."""

    seq: int
    touched: bool


def resolve_contact(touched: bool, uplink_fresh: bool) -> bool:
    """발행할 ``user_contact`` 값을 정한다.

    상향이 끊겼으면 마지막 값을 붙들지 않고 false 로 떨어뜨린다. 붙들면 죽은
    센서가 "잡고 있다"고 말하게 되고, 모드 진입과 손 놓음 판정이 그 값으로
    갈리므로 사용자는 왜 안 되는지 알 수 없다.

    한 줄짜리지만 노드 안에 묻어두지 않고 여기 두는 이유는, 이것이 상위와의
    계약이라 조용히 깨지면 안 되기 때문이다.
    """
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
    """바이트 스트림을 먹여 완성된 터치 프레임 목록을 돌려받는다.

    체크섬이 깨진 헤더 후보는 프레임으로 소비하지 않고 1바이트만 버린다 —
    초음파 프레임 중간에 우연히 0xAA 0x56 이 나온 경우, 5바이트를 통째로
    버리면 뒤따르는 진짜 프레임까지 같이 잃기 때문이다.
    """

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

        # 소비한 앞부분을 버리고, 남은 미완성 꼬리는 다음 feed 를 기다린다.
        del buf[:i]
        if len(buf) > MAX_BUFFER_BYTES:
            del buf[:-MAX_BUFFER_BYTES]

        return frames
