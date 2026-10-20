"""Turn an OccupancyGrid into a PNG the app can display."""

import math
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


POSE_MAX_AGE_SEC = 5.0


def quaternion_to_yaw_degrees(x: float, y: float, z: float, w: float) -> float:
    """Return the heading (rotation about Z) in degrees, counter-clockwise positive."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def robot_pose_fields(pose, age_sec, max_age_sec: float = POSE_MAX_AGE_SEC) -> dict:
    """Return the preview JSON fields for the robot pose, or {} when unknown."""
    if pose is None or age_sec is None or age_sec > max_age_sec:
        return {}
    x, y, yaw = pose
    return {
        'robot_x': round(float(x), 3),
        'robot_y': round(float(y), 3),
        'robot_yaw': round(float(yaw), 2),
    }


def tilt_fields(tilt_deg) -> dict:
    """Return the preview JSON field for the map tilt, or {} when unknown."""
    if tilt_deg is None:
        return {}
    return {'tilt_deg': round(float(tilt_deg), 1)}
