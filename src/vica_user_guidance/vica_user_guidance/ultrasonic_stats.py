"""상향 초음파 통계 프레임 파서 — 순수 함수·클래스 (I/O 없음)."""

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
    """바이트 스트림을 먹여 완성된 통계 프레임 목록을 돌려받는다."""

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
    """로그 한 줄. 채널마다 'ok/clr/ffff/fffe/oth/i2c' 를 적는다."""
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


@dataclass(frozen=True)
class UltrasonicConfigFrame:
    """센서에서 되읽은 채널별 (지향각 레벨, 노이즈 저감 레벨). 되읽기 실패 = 0xFF."""

    seq: int
    levels: Tuple[Tuple[int, int], ...]


class ConfigFrameAccumulator:
    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> List[UltrasonicConfigFrame]:
        self._buf.extend(data)
        frames: List[UltrasonicConfigFrame] = []
        n = protocol.US_CFG_FRAME_LEN
        i = 0
        buf = self._buf
        while len(buf) - i >= n:
            if bytes(buf[i : i + 2]) != protocol.US_CFG_FRAME_HEADER:
                i += 1
                continue
            chunk = bytes(buf[i : i + n])
            if checksum(chunk[2:-1]) != chunk[-1]:
                i += 1
                continue
            levels = tuple(
                (chunk[3 + 2 * ch], chunk[4 + 2 * ch])
                for ch in range(protocol.US_CHANNELS)
            )
            frames.append(UltrasonicConfigFrame(seq=chunk[2], levels=levels))
            i += n
        del buf[:i]
        if len(buf) > MAX_BUFFER_BYTES:
            del buf[:-MAX_BUFFER_BYTES]
        return frames


def format_config_line(frame: UltrasonicConfigFrame, channel_names) -> str:
    parts = []
    for ch, (a, nz) in enumerate(frame.levels):
        name = channel_names[ch] if ch < len(channel_names) else f"ch{ch}"
        fa = "?" if a == 0xFF else str(a)
        fn = "?" if nz == 0xFF else str(nz)
        parts.append(f"{name} 각{fa}/잡음{fn}")
    return f"[US 설정 #{frame.seq}] (센서 되읽기) " + " · ".join(parts)
