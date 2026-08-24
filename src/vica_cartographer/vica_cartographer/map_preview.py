"""Turn an OccupancyGrid into a PNG the app can display.

rclpy 를 쓰지 않는다. 노트북에서 pytest 로 전부 검증할 수 있어야 하기 때문이다.

**왜 Pillow 를 안 쓰는가.** 젯슨에 Pillow 가 없을 수 있다 —
`scripts/vica_map_save.sh` 가 "Pillow 가 없어 픽셀 크기는 확인하지 못했다" 경로를
갖고 있다. 표준 라이브러리(zlib, struct)만으로 회색조 PNG 를 쓰는 것은 40줄이면
되고, 실측해 보니 오히려 더 작고 빨랐다(vica_map_0630, 572x443 = 25만 칸 기준).

    순수 zlib level 6   4,533 bytes   1.1 ms/장
    Pillow              4,740 bytes   1.6 ms/장
    압축하지 않은 raw   253,396 bytes

점유격자는 같은 값이 넓게 이어져 있어 zlib 이 극단적으로 잘 줄인다. 그래서
rosbridge 로 격자를 그대로 보내는 것(JSON 703 KB, cbor-raw 247 KB)보다
PNG 를 HTTP 로 내려받는 편이 150배 넘게 싸다.
"""

import struct
import zlib

# nav2 map_server 의 trinary 규약과 같은 값이다. maps/*.yaml 의
# occupied_thresh 0.65 / free_thresh 0.25 를 그대로 쓴다. 여기서 다른 값을 쓰면
# 미리보기와 저장된 지도가 다르게 보인다.
OCCUPIED_THRESH = 65
FREE_THRESH = 25

# map_saver 가 쓰는 회색조 값. 앱이 보는 저장된 지도와 같아 보이게 맞춘다.
GRAY_OCCUPIED = 0
GRAY_FREE = 254
GRAY_UNKNOWN = 205


def occupancy_to_gray(data, width: int, height: int) -> bytes:
    """Convert OccupancyGrid.data to top-down 8-bit grayscale pixels.

    **세로를 뒤집는다.** OccupancyGrid 의 첫 칸은 origin 쪽, 즉 아래쪽 줄이고
    y 가 커질수록 위로 간다. 이미지 파일은 위에서 아래로 쓴다. 뒤집지 않으면
    지도가 상하로 뒤집혀 보이고, 앱의 flipMapY 설정과 겹쳐 원인을 찾기 어려워진다.
    """
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
    """Encode top-down 8-bit grayscale pixels as a PNG.

    filter type 0(None)만 쓴다. 적응 필터를 넣으면 조금 더 줄지만 계산이 늘고,
    점유격자는 그러지 않아도 이미 50배 넘게 줄어든다.

    level 은 6 이 기본이다. 실측에서 1 은 6,686 bytes / 0.5 ms, 9 는
    3,478 bytes / 6.4 ms 였다. 6 이 크기와 시간의 균형점이다.
    """
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
