"""Turn an OccupancyGrid into a PNG the app can display."""

import struct
import zlib

OCCUPIED_THRESH = 65
FREE_THRESH = 25

GRAY_OCCUPIED = 0
GRAY_FREE = 254
GRAY_UNKNOWN = 205


def occupancy_to_gray(data, width: int, height: int) -> bytes:
    """Convert OccupancyGrid.data to top-down 8-bit grayscale pixels."""
    if width <= 0 or height <= 0:
        raise ValueError(f'격자 크기가 올바르지 않습니다: {width}x{height}')
    if len(data) != width * height:
        raise ValueError(
            f'격자 칸 수가 맞지 않습니다: data {len(data)} != {width}x{height}'
        )

    table = bytes(
        GRAY_OCCUPIED if 0 <= value <= 100 and value >= OCCUPIED_THRESH
        else GRAY_FREE if 0 <= value <= FREE_THRESH
        else GRAY_UNKNOWN
        for value in range(-128, 128)
    )

    rows = []
    for y in range(height - 1, -1, -1):
        start = y * width
        row = data[start:start + width]
        rows.append(bytes(table[value + 128] for value in row))
    return b''.join(rows)


def encode_png_gray(pixels: bytes, width: int, height: int, level: int = 6) -> bytes:
    """Encode top-down 8-bit grayscale pixels as a PNG."""
    if len(pixels) != width * height:
        raise ValueError(
            f'픽셀 수가 맞지 않습니다: {len(pixels)} != {width}x{height}'
        )

    raw = bytearray()
    for y in range(height):
        raw.append(0)
        raw += pixels[y * width:(y + 1) * width]

    def chunk(tag: bytes, payload: bytes) -> bytes:
        return (
            struct.pack('>I', len(payload))
            + tag
            + payload
            + struct.pack('>I', zlib.crc32(tag + payload) & 0xFFFFFFFF)
        )

    return (
        b'\x89PNG\r\n\x1a\n'
        + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0))
        + chunk(b'IDAT', zlib.compress(bytes(raw), level))
        + chunk(b'IEND', b'')
    )


def grid_to_png(data, width: int, height: int, level: int = 6) -> bytes:
    """Encode OccupancyGrid.data directly as PNG bytes."""
    return encode_png_gray(
        occupancy_to_gray(data, width, height), width, height, level
    )
