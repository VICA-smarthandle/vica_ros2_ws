"""Unit tests for the mapping preview encoder.

로봇도 ROS 도 없이 도는 시험이다. 이 파일이 지키는 것은 세 가지다.
  1. 저장된 지도와 같은 규약으로 색을 칠하는가 (trinary 임계값)
  2. 세로를 뒤집는가 — 안 뒤집으면 지도가 상하로 뒤집혀 보인다
  3. 진짜 PNG 가 나오는가
"""

import struct
import zlib

import pytest

from vica_cartographer.map_preview import (
    encode_png_gray,
    GRAY_FREE,
    GRAY_OCCUPIED,
    GRAY_UNKNOWN,
    grid_to_png,
    occupancy_to_gray,
)


def test_unknown_free_occupied_use_map_saver_values():
    """Trinary 값이 nav2 map_server 와 같아야 저장본과 같아 보인다."""
    pixels = occupancy_to_gray([-1, 0, 100], 3, 1)
    assert pixels == bytes([GRAY_UNKNOWN, GRAY_FREE, GRAY_OCCUPIED])


def test_threshold_boundaries_follow_the_yaml():
    """Yaml 의 occupied_thresh 0.65 / free_thresh 0.25 를 그대로 쓴다."""
    pixels = occupancy_to_gray([25, 26, 64, 65], 4, 1)
    assert pixels == bytes(
        [GRAY_FREE, GRAY_UNKNOWN, GRAY_UNKNOWN, GRAY_OCCUPIED]
    )


def test_rows_are_flipped_vertically():
    """Grid 는 아래줄이 먼저고 이미지 파일은 위줄이 먼저다."""
    pixels = occupancy_to_gray([0, 0, 100, 100], 2, 2)
    assert pixels[:2] == bytes([GRAY_OCCUPIED, GRAY_OCCUPIED]), '위줄이 먼저 와야 한다'
    assert pixels[2:] == bytes([GRAY_FREE, GRAY_FREE])


def test_size_mismatch_is_rejected():
    """칸 수가 안 맞으면 조용히 이상한 그림을 만들지 않고 멈춘다."""
    with pytest.raises(ValueError):
        occupancy_to_gray([0, 0, 0], 2, 2)
    with pytest.raises(ValueError):
        occupancy_to_gray([], 0, 0)


def test_png_has_a_valid_signature_and_chunks():
    png = grid_to_png([-1] * 6, 3, 2)
    assert png[:8] == b'\x89PNG\r\n\x1a\n'
    assert png[12:16] == b'IHDR'
    assert png[-8:-4] == b'IEND'


def test_png_header_carries_the_grid_size():
    png = grid_to_png([-1] * 12, 4, 3)
    width, height, depth, colour = struct.unpack('>IIBB', png[16:26])
    assert (width, height) == (4, 3)
    assert depth == 8, '8비트'
    assert colour == 0, '회색조'


def test_png_round_trips_back_to_the_same_pixels():
    """직접 쓴 인코더라 되읽어서 확인한다. 필터 0 이므로 줄 앞 1바이트만 떼면 된다."""
    grid = [-1, 0, 100, 100, 0, -1]
    png = grid_to_png(grid, 3, 2)

    offset = 8
    payload = b''
    while offset < len(png):
        length = struct.unpack('>I', png[offset:offset + 4])[0]
        tag = png[offset + 4:offset + 8]
        if tag == b'IDAT':
            payload = png[offset + 8:offset + 8 + length]
            break
        offset += 12 + length
    raw = zlib.decompress(payload)

    decoded = b''.join(raw[y * 4 + 1:y * 4 + 4] for y in range(2))
    assert decoded == occupancy_to_gray(grid, 3, 2)


def test_compression_actually_helps():
    """압축비가 무너지면 rosbridge 로 그냥 보내는 것과 다를 바 없어진다.

    실측 근거: vica_map_0630(572x443, 25만 칸)이 raw 253,396 -> PNG 4,533 바이트였다.
    여기서는 같은 성질(같은 값이 넓게 이어짐)을 가진 작은 격자로 방향만 잠근다.
    """
    grid = [-1] * (200 * 200)
    png = grid_to_png(grid, 200, 200)
    assert len(png) < len(grid) // 20, f'압축이 안 됐다: {len(png)} bytes'


def test_encode_rejects_pixel_count_mismatch():
    with pytest.raises(ValueError):
        encode_png_gray(b'\x00' * 3, 2, 2)
