"""Measure how far a map is tilted and straighten it on save (2026-10-07).

**왜 지도가 삐뚤어지는가.** Cartographer 는 매핑을 시작하는 순간 로봇
(tracking_frame = base_footprint, config/vica_2d.lua)이 보던 방향을 map 의 +x 로
잡는다. 로봇을 복도와 몇 도 비껴 세우고 시작하면 저장된 지도도 그만큼 기운다.
저장된 6장 실측(이 코드, 반시계 양수): map_1006_150446 +5.4° · vica_map_0910 −5.4° ·
vica_map_0903_d −4.5° · vica_map_0630 +1.6° · map_1002_150946 +0.9° · vica_map_0817 −0.9°.

**yaml 의 origin yaw 로는 못 고친다.** Nav2 costmap static_layer 는 origin 의 x·y 만
읽고 yaw 는 버린다(humble processMap). 그래서 그림 자체를 돌리고 origin 을 다시 잡는다.

**어떻게 돌리는가 — 액자 비유.** 기운 사진을 액자째 돌리는 것이 아니라, 새 액자를
바르게 걸고 그 칸마다 "원래 사진의 어느 칸이 여기 오는가"를 거꾸로 찾아 그대로 옮겨
적는다(가장 가까운 칸, nearest). 색을 섞지 않으므로 벽·빈칸·모름 세 값이 그대로
남는다. 거꾸로 찾기에서 빠지는 외딴 벽칸이 없도록 원본 벽칸은 새 자리에 한 번 더
찍는다. 대신 기울어진 선을 칸에 맞추다 보면 벽에 틈이 생길 수 있어, 3×3 closing
으로 **원래 빈칸이던 자리만** 벽으로 메운다. 3×3 closing 은 두 칸(10 cm) 이하 틈을
메우고 지도 전체에 적용된다 — 10 cm 틈은 내접 반경(0.277 m) 안이라 원래도
inflation 이 막던 자리다(빈칸 감소 실측 −0.02~−0.05 %).

    벽 1칸 팽창은 쓰지 않는다 — 6장 시뮬레이션에서 벽이 전부 5~6 cm 다가오고 통과
    가능 면적이 2~7 % 줄었으며, 0903_d 는 통로 하나가 닫혔다.
    nearest + 벽칸 찍기 + closing 은 벽까지 거리 중앙값 0, 통과 면적 −0.1~0.4 %,
    구역 분리 0(6장, 10-07).
    실제 벽 이동은 이론상 최대 칸 대각선의 절반(3.5 cm)이다.

**2° 미만은 돌리지 않는다.** 한 칸 오차는 각도와 무관하게 생기지만, 막히는 면적은
각도에 비례한다(0.5° 0.08 m² · 1° 0.15 · 2° 0.27 · 5° 0.65). 조금 기운 지도는 돌려서
얻는 것보다 잃는 것이 크다. 문턱은 사용자 결정(10-07)이다.

rclpy 를 쓰지 않는다. map_preview_node 가 import 하고, scripts/vica_map_save.sh 가
``python3 -m vica_cartographer.map_align save`` 로 부르며, pytest 로 노트북에서도 돈다.
Pillow·scipy 도 쓰지 않는다 — 젯슨에 없을 수 있다(map_preview.py 머리말).
"""

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import sys

import numpy as np

# 정렬해도 이보다 작게 기울었으면 그대로 둔다. 위 머리말 '2° 미만'.
DEFAULT_MIN_DEG = 2.0

# 벽 칸이 이보다 적으면 방향을 재지 않는다. 0.05 m 칸으로 7.5 m 어치 벽이다.
# 매핑을 막 시작해 벽이 몇 조각뿐일 때 엉뚱한 각도가 나오는 것을 막는다.
MIN_WALL_CELLS = 150

# 가장 잘 맞는 각도의 점수가 전체 각도 점수 중앙값보다 이만큼은 커야 "벽 방향이
# 있다"고 본다. 둥근 방이나 잡음뿐인 지도는 어느 각도로 봐도 점수가 비슷하다.
# 저장된 지도 6장은 2.7~5.1, 벽을 무작위로 흩뿌린 점은 1.06~1.11, 원은 1.00 이었다
# (10-07 실측).
MIN_PEAK_RATIO = 1.5

# 기울기를 잴 때 쓰는 벽칸 상한. 넘으면 고르게 건너뛰어 이만큼만 쓴다. 계산 시간이
# 벽칸 수에 비례해(젯슨: 1만 칸 49 ms, 6만 칸 226 ms — 10-07 검토) 큰 지도에서 2초마다
# 도는 미리보기가 무거워진다. 방향은 일부 점만으로도 같게 나온다(시험이 6장으로 확인).
MAX_TILT_POINTS = 8000

# 각도 탐색. 1° 간격으로 훑고 가장 좋은 곳 ±1° 를 0.05° 간격으로 다시 본다.
COARSE_STEP_DEG = 1.0
FINE_STEP_DEG = 0.05

# map_saver 회색조(map_preview.py 와 같은 값). trinary 규약.
GRAY_OCCUPIED = 0
GRAY_FREE = 254
GRAY_UNKNOWN = 205
# 이 값 이하를 벽으로 본다(occupied_thresh 0.65 → 255×0.35 ≈ 89).
WALL_MAX = 100
# 이 값 이상을 빈칸으로 본다.
FREE_MIN = 250

# 회전을 한 번에 계산하는 칸 수. 칸당 약 67 B 라 25만 칸이면 17 MB 안팎이다.
ROTATE_CHUNK_CELLS = 250_000

# 돌린 지도 둘레에 남길 모름 칸. 원래 지도의 내용 끝에서 이만큼 띄운다.
MARGIN_CELLS = 10

# 원본을 숨겨 두는 곳(B안, 10-07 확정). 점으로 시작해 앱 목록(maps/*.png)·터미널
# 고르기(maps/*.yaml)에 잡히지 않고, yaml·png 를 두지 않아 Nav2 로 띄울 수도 없다.
ORIGINAL_DIRNAME = '.original'

# 저장 스크립트 출력에서 결과를 찾는 머리표. mapping_supervisor_node 가 읽는다.
RESULT_PREFIX = 'VICA_ALIGN '


# ---------------------------------------------------------------------------
# 기울기 측정
# ---------------------------------------------------------------------------

def _projection_score(xs, ys, angle_rad: float) -> float:
    """Sum of squared histogram counts along the two axes at this angle.

    벽이 이 각도와 나란하면 같은 줄에 칸이 몰려 막대가 높고 좁아진다. 막대 높이의
    제곱합은 몰릴수록 커진다. 가로·세로 두 방향을 같이 보므로 90° 마다 같은 값이다.
    """
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    score = 0.0
    for along in (xs * cos_a + ys * sin_a, -xs * sin_a + ys * cos_a):
        bins = np.floor(along - along.min()).astype(np.int64)
        counts = np.bincount(bins)
        score += float(np.dot(counts, counts))
    return score


def estimate_tilt_deg(xs, ys):
    """Return the wall direction in degrees (−45, 45], or None if unknown.

    xs·ys 는 벽 칸 중심 좌표다(칸 단위, x 오른쪽·y 위쪽 = ROS map 좌표 방향).
    돌려준 값만큼 **시계 방향**으로(−각도) 돌리면 벽이 화면과 나란해진다.
    양수 = 벽이 반시계로 기울었다.
    """
    xs = np.asarray(xs, dtype=np.float64)
    ys = np.asarray(ys, dtype=np.float64)
    if xs.size < MIN_WALL_CELLS:
        return None
    if xs.size > MAX_TILT_POINTS:
        stride = int(math.ceil(xs.size / MAX_TILT_POINTS))
        xs = xs[::stride]
        ys = ys[::stride]
    xs = xs - xs.mean()
    ys = ys - ys.mean()

    coarse = np.arange(-45.0, 45.0, COARSE_STEP_DEG)
    scores = np.array(
        [_projection_score(xs, ys, math.radians(a)) for a in coarse]
    )
    median = float(np.median(scores))
    if median <= 0 or float(scores.max()) / median < MIN_PEAK_RATIO:
        return None

    best = float(coarse[int(np.argmax(scores))])
    fine = np.arange(best - 1.0, best + 1.0 + 1e-9, FINE_STEP_DEG)
    fine_scores = np.array(
        [_projection_score(xs, ys, math.radians(a)) for a in fine]
    )
    # 짧은 벽은 ±0.4° 안에서 같은 칸 줄에 그대로 머물러 점수가 똑같다(꼭대기가
    # 평평하다). 첫 값을 고르면 반듯한 방도 −0.4° 로 읽힌다. 평평한 꼭대기의
    # 가운데를 쓴다. 칸 경계에 좌표가 딱 걸리면 몇 점이 깎이므로 0.1 % 안은 같다고 본다.
    top = np.flatnonzero(fine_scores >= fine_scores.max() * 0.999)
    if top[-1] - top[0] == len(top) - 1:
        angle = float((fine[top[0]] + fine[top[-1]]) / 2)
    else:
        angle = float(fine[top[0]])
    # −45 와 45 는 같은 방향이다. (−45, 45] 로 접는다.
    while angle <= -45.0:
        angle += 90.0
    while angle > 45.0:
        angle -= 90.0
    return angle


def wall_points_top_down(image):
    """Wall cell centres from a top-down gray image, in cells with y up."""
    rows, cols = np.nonzero(np.asarray(image) <= WALL_MAX)
    height = image.shape[0]
    return cols + 0.5, (height - 1 - rows) + 0.5


def tilt_of_image(image):
    """Tilt in degrees of a top-down gray map image, or None."""
    xs, ys = wall_points_top_down(image)
    return estimate_tilt_deg(xs, ys)


def tilt_of_occupancy(data, width: int, height: int, occupied_thresh: int = 65):
    """Tilt of an OccupancyGrid (bottom-up rows), or None. map_preview_node 용."""
    if width <= 0 or height <= 0:
        return None
    grid = np.asarray(data, dtype=np.int16)
    if grid.size != width * height:
        return None
    cells = np.nonzero(grid >= occupied_thresh)[0]
    return estimate_tilt_deg(cells % width + 0.5, cells // width + 0.5)


# ---------------------------------------------------------------------------
# 회전
# ---------------------------------------------------------------------------

def _close_free_gaps(image):
    """Fill gaps of up to two cells between walls, but only where the cell was free.

    3×3 팽창 뒤 3×3 침식(closing). 원래 벽은 그대로고, 새로 벽이 되는 것은 closing
    결과가 벽이면서 지금 빈칸(254)인 칸뿐이다. 모름(205) 칸은 건드리지 않는다.
    양쪽 벽에서 한 칸씩 자라 만나므로 두 칸(10 cm) 틈까지 닫힌다.
    """
    wall = image <= WALL_MAX
    padded = np.pad(wall, 2, constant_values=False)
    height, width = wall.shape
    dilated = np.zeros((height + 2, width + 2), dtype=bool)
    for dr in range(3):
        for dc in range(3):
            dilated |= padded[dr:dr + height + 2, dc:dc + width + 2]
    eroded = np.ones((height, width), dtype=bool)
    for dr in range(3):
        for dc in range(3):
            eroded &= dilated[dr:dr + height, dc:dc + width]
    filled = image.copy()
    filled[eroded & (image >= FREE_MIN)] = GRAY_OCCUPIED
    return filled


def rotate_map(image, resolution: float, origin_x: float, origin_y: float,
               tilt_deg: float, margin_cells: int = MARGIN_CELLS):
    """Rotate a top-down gray map by −tilt_deg. Returns (image, origin_x, origin_y).

    좌표 약속(nav2 map_server 와 같다): origin 은 그림 왼쪽 아래 칸의 왼쪽 아래
    모서리이고, 칸 (열 c, 아래에서 j 번째 줄)의 중심은
    origin + ((c + 0.5)·res, (j + 0.5)·res) 다.

    새 좌표계 q 와 옛 좌표계 p 의 관계는 q = R(−θ)·p 다. 새 칸마다 p = R(θ)·q 를
    계산해 그 자리의 옛 칸 값을 그대로 옮긴다(nearest). 그림 크기는 옛 지도의
    **내용(모름이 아닌 칸)** 을 감싸는 상자 + margin_cells 로 정한다.
    """
    image = np.asarray(image, dtype=np.uint8)
    height, width = image.shape
    theta = math.radians(tilt_deg)
    cos_t, sin_t = math.cos(theta), math.sin(theta)

    rows, cols = np.nonzero(image != GRAY_UNKNOWN)
    if rows.size == 0:
        rows = np.array([0, height - 1])
        cols = np.array([0, width - 1])
    # 내용 칸 네 귀퉁이만 돌려도 상자가 정해진다.
    px = origin_x + (np.array([cols.min(), cols.max() + 1.0]) * resolution)
    py = origin_y + (np.array([height - 1 - rows.max(), height - rows.min()],
                              dtype=np.float64) * resolution)
    corners_x = np.array([px[0], px[1], px[0], px[1]])
    corners_y = np.array([py[0], py[0], py[1], py[1]])
    qx = corners_x * cos_t + corners_y * sin_t
    qy = -corners_x * sin_t + corners_y * cos_t

    margin = margin_cells * resolution
    new_origin_x = float(qx.min() - margin)
    new_origin_y = float(qy.min() - margin)
    new_width = int(math.ceil((qx.max() + margin - new_origin_x) / resolution))
    new_height = int(math.ceil((qy.max() + margin - new_origin_y) / resolution))

    # 새 칸 중심(아래에서 j 번째 줄) → 옛 좌표. 줄 묶음으로 나눠 계산한다 — 한 번에
    # 하면 칸당 약 67 B 라 800만 칸(200 m × 100 m) 층이면 540 MB 가 되고, 이 계산은
    # Cartographer 가 아직 떠 있는 동안 돈다(10-07 검토).
    cc = (np.arange(new_width) + 0.5) * resolution + new_origin_x
    bottom_up = np.full((new_height, new_width), GRAY_UNKNOWN, dtype=np.uint8)
    rows_per_chunk = max(1, ROTATE_CHUNK_CELLS // max(new_width, 1))
    for start in range(0, new_height, rows_per_chunk):
        stop = min(new_height, start + rows_per_chunk)
        jj = (np.arange(start, stop) + 0.5) * resolution + new_origin_y
        grid_x, grid_y = np.meshgrid(cc, jj)
        old_c = np.floor(
            (grid_x * cos_t - grid_y * sin_t - origin_x) / resolution
        ).astype(np.int64)
        old_j = np.floor(
            (grid_x * sin_t + grid_y * cos_t - origin_y) / resolution
        ).astype(np.int64)
        inside = (old_c >= 0) & (old_c < width) & (old_j >= 0) & (old_j < height)
        block = bottom_up[start:stop]
        block[inside] = image[height - 1 - old_j[inside], old_c[inside]]

    # 거꾸로 찾기만 하면 옛 칸 몇 개는 아무 새 칸도 고르지 않아 빠진다. 벽이 이어진
    # 곳은 이웃이 메워 티가 안 나지만, 한 칸짜리 외딴 장애물은 통째로 사라진다
    # (0903_d 에서 94개 중 1개, 10-07). 그래서 옛 벽칸마다 새 자리를 계산해 그 칸을
    # 벽으로 찍는다. 찍는 칸과 거꾸로 찾기가 고르는 칸이 늘 같지는 않아 벽이
    # 군데군데 한 칸 두꺼워진다(벽칸 약 +1.5 %, 10-07 검토 실측). 통과 면적 영향은
    # 6장에서 −0.1~0.4 % 였다.
    wall_rows, wall_cols = np.nonzero(image <= WALL_MAX)
    if wall_rows.size:
        wx = origin_x + (wall_cols + 0.5) * resolution
        wy = origin_y + (height - 1 - wall_rows + 0.5) * resolution
        nc = np.floor((wx * cos_t + wy * sin_t - new_origin_x) / resolution)
        nj = np.floor((-wx * sin_t + wy * cos_t - new_origin_y) / resolution)
        nc = np.clip(nc.astype(np.int64), 0, new_width - 1)
        nj = np.clip(nj.astype(np.int64), 0, new_height - 1)
        bottom_up[nj, nc] = GRAY_OCCUPIED
    rotated = bottom_up[::-1].copy()
    return _close_free_gaps(rotated), new_origin_x, new_origin_y


# ---------------------------------------------------------------------------
# 파일
# ---------------------------------------------------------------------------

def read_pgm(path: Path):
    """Read a binary (P5) 8-bit PGM as written by nav2 map_saver."""
    raw = Path(path).read_bytes()
    fields = []
    index = 0
    while len(fields) < 4:
        while index < len(raw) and raw[index:index + 1].isspace():
            index += 1
        if raw[index:index + 1] == b'#':
            while index < len(raw) and raw[index:index + 1] not in (b'\n', b'\r'):
                index += 1
            continue
        start = index
        while index < len(raw) and not raw[index:index + 1].isspace():
            index += 1
        fields.append(raw[start:index])
    index += 1  # 최대값 뒤 공백 한 글자
    if fields[0] != b'P5':
        raise ValueError(f'P5 pgm 이 아닙니다: {path}')
    width, height, maxval = int(fields[1]), int(fields[2]), int(fields[3])
    if maxval > 255:
        raise ValueError(f'16비트 pgm 은 다루지 않습니다: {path}')
    pixels = np.frombuffer(raw, dtype=np.uint8, count=width * height, offset=index)
    return pixels.reshape(height, width).copy()


def write_pgm(path: Path, image, resolution: float) -> None:
    """Write a P5 PGM with the same header style as map_saver."""
    image = np.asarray(image, dtype=np.uint8)
    height, width = image.shape
    header = (
        f'P5\n# CREATOR: vica map_align {resolution:.3f} m/pix\n'
        f'{width} {height}\n255\n'
    ).encode('ascii')
    Path(path).write_bytes(header + image.tobytes())


def read_map_yaml(path: Path):
    """Return (lines, resolution, origin_x, origin_y) from a map_saver yaml."""
    lines = Path(path).read_text(encoding='utf-8').splitlines()
    resolution = None
    origin = None
    for line in lines:
        key, _, value = line.partition(':')
        if key.strip() == 'resolution':
            resolution = float(value.strip())
        elif key.strip() == 'origin':
            parts = [p.strip() for p in value.strip().strip('[]').split(',')]
            origin = [float(p) for p in parts]
    if resolution is None or origin is None or len(origin) < 2:
        raise ValueError(f'yaml 에 resolution·origin 이 없습니다: {path}')
    if len(origin) >= 3 and abs(origin[2]) > 1e-9:
        # 지금 저장 경로(Cartographer → map_saver)는 항상 0 이다. 0 이 아니면 그
        # 각도까지 계산에 넣어야 하는데 costmap 이 어차피 무시하는 값이라 돌리지 않는다.
        raise ValueError(f'origin yaw 가 0 이 아닙니다({origin[2]}): {path}')
    return lines, resolution, origin[0], origin[1]


def _format_number(value: float) -> str:
    text = f'{value:.4f}'.rstrip('0').rstrip('.')
    return '0' if text in ('-0', '') else text


def write_map_yaml(path: Path, lines, origin_x: float, origin_y: float) -> None:
    """Rewrite only the origin line, keeping every other line as it was."""
    out = []
    for line in lines:
        if line.partition(':')[0].strip() == 'origin':
            out.append(
                f'origin: [{_format_number(origin_x)}, {_format_number(origin_y)}, 0]'
            )
        else:
            out.append(line)
    Path(path).write_text('\n'.join(out) + '\n', encoding='utf-8')


def _set_orphans_aside(original_dir: Path, name: str, stamp: str) -> list:
    """Rename leftovers of an older map with the same name. 지우지 않는다.

    저장 스크립트가 같은 이름의 지도가 없음을 이미 확인했으므로, 여기 남아 있는 것은
    손으로 지운 옛 지도의 원본이다. 그대로 두면 새 지도의 원본인 척하게 된다.
    """
    moved = []
    for suffix in ('.pgm', '.json'):
        path = original_dir / f'{name}{suffix}'
        if path.exists():
            target = original_dir / f'{name}{suffix}.orphan-{stamp}'
            os.replace(path, target)
            moved.append(target.name)
    return moved


# ---------------------------------------------------------------------------
# 저장 단계
# ---------------------------------------------------------------------------

def straighten_saved_map(maps_dir: Path, name: str, align: bool,
                         min_deg: float = DEFAULT_MIN_DEG, now=None) -> dict:
    """Measure, and when asked, straighten maps/<name>.pgm·yaml in place.

    결과 dict 의 result:
        rotated        돌려서 다시 저장했다(원본은 maps/.original/)
        small          정렬을 골랐지만 기울기가 min_deg 미만이라 그대로 뒀다
        no_walls       벽 방향을 찾지 못해 그대로 뒀다
        not_requested  정렬하지 않고 저장을 골랐다(기울기는 재서 알려 준다)
        failed         돌리다 실패해 그대로 뒀다(지도 자체는 멀쩡하다)
    """
    maps_dir = Path(maps_dir)
    now = now or datetime.now()
    pgm_path = maps_dir / f'{name}.pgm'
    yaml_path = maps_dir / f'{name}.yaml'
    result = {'result': 'failed', 'tilt_deg': None, 'rotated_deg': 0.0,
              'min_deg': min_deg, 'message': ''}

    original_dir = maps_dir / ORIGINAL_DIRNAME
    stamp = now.strftime('%Y%m%d-%H%M%S')
    # 저장 스크립트가 같은 이름의 지도가 없음을 확인한 뒤라, 여기 남은 원본은 손으로
    # 지운 옛 지도의 것이다. 무엇보다 먼저(파일을 못 읽어도) 옆으로 비켜 둔다 —
    # 돌리지 않은 새 지도가 옛 원본을 제 것처럼 달고 있게 되는 일을 막는다(10-07 검토).
    try:
        moved = _set_orphans_aside(original_dir, name, stamp) if original_dir.is_dir() else []
    except OSError:
        moved = []

    try:
        image = read_pgm(pgm_path)
        yaml_raw = yaml_path.read_bytes()  # 실패 때 글자 그대로 되돌리려고
        lines, resolution, origin_x, origin_y = read_map_yaml(yaml_path)
    except (OSError, ValueError) as error:
        if not align:
            # 정렬을 고르지 않았으니 '돌리지 못했다'고 말할 일이 아니다.
            result['result'] = 'not_requested'
            result['message'] = '정렬하지 않고 그대로 저장했습니다.'
        else:
            result['message'] = f'지도 파일을 읽지 못해 그대로 둡니다: {error}'
        return result

    measured = tilt_of_image(image)
    # 화면·메시지·판정이 모두 같은 숫자를 쓰도록 소수 한 자리로 한 번만 반올림한다.
    # 앱은 이 값을 그대로 toStringAsFixed(1) 하므로 터미널과 앱 숫자가 같다.
    tilt = None if measured is None else round(measured, 1)
    result['tilt_deg'] = tilt

    if not align:
        result['result'] = 'not_requested'
        result['message'] = (
            '정렬하지 않고 그대로 저장했습니다'
            + ('.' if tilt is None else f'(기울기 {abs(tilt):.1f}°).')
        )
        return result
    if tilt is None:
        result['result'] = 'no_walls'
        result['message'] = '벽 방향을 찾지 못해 돌리지 않고 그대로 저장했습니다.'
        return result
    # 보이는 값으로 판정한다. 1.96° 를 "2.0°라 돌리지 않았다"고 말하는 일이 없다.
    if abs(tilt) < min_deg:
        result['result'] = 'small'
        result['message'] = f'기울기가 {abs(tilt):.1f}°라 돌리지 않고 그대로 저장했습니다.'
        return result

    pgm_tmp = maps_dir / f'{name}.pgm.align-tmp'
    yaml_tmp = maps_dir / f'{name}.yaml.align-tmp'
    backup_pgm = original_dir / f'{name}.pgm'
    backup_memo = original_dir / f'{name}.json'
    written = []          # 이번에 새로 만든 원본·메모. 실패하면 지운다.
    yaml_replaced = False
    try:
        # 회전은 잰 값 그대로(반올림 전)로 한다. 남는 기울기가 가장 작다.
        rotated, new_x, new_y = rotate_map(
            image, resolution, origin_x, origin_y, measured
        )
        original_dir.mkdir(exist_ok=True)
        # 원본을 먼저 안전한 곳에 둔 뒤에야 덮어쓴다.
        # 쓰다가 실패해도 반쪽 파일이 원본인 척 남지 않게, 쓰기 전에 목록에 넣는다.
        written.append(backup_pgm)
        backup_pgm.write_bytes(pgm_path.read_bytes())
        memo = {
            'map_id': name,
            'rotated_deg': round(-measured, 3),
            'tilt_deg': round(measured, 3),
            'resolution': resolution,
            'original_origin': [origin_x, origin_y, 0.0],
            'original_size': [int(image.shape[1]), int(image.shape[0])],
            'new_origin': [round(new_x, 4), round(new_y, 4), 0.0],
            'new_size': [int(rotated.shape[1]), int(rotated.shape[0])],
            'original_yaml': lines,
            'method': 'nearest + wall splat + closing 3x3 (free cells only)',
            'saved_at': now.isoformat(timespec='seconds'),
            'note': '옛 좌표 p 와 새 좌표 q 는 q = R(rotated_deg)·p 관계다.',
        }
        if moved:
            memo['orphans_set_aside'] = moved
        written.append(backup_memo)
        backup_memo.write_text(
            json.dumps(memo, ensure_ascii=False, indent=2) + '\n', encoding='utf-8'
        )
        write_pgm(pgm_tmp, rotated, resolution)
        write_map_yaml(yaml_tmp, lines, new_x, new_y)
        os.replace(yaml_tmp, yaml_path)
        yaml_replaced = True
        os.replace(pgm_tmp, pgm_path)
    except (OSError, ValueError, MemoryError) as error:
        # 지도는 돌리기 전 모습 그대로 남겨야 한다. yaml 만 바뀐 채면(돌린 origin +
        # 돌리지 않은 그림) 벽이 엉뚱한 곳에 놓이므로 원래 yaml 로 되돌린다.
        if yaml_replaced:
            try:
                yaml_path.write_bytes(yaml_raw)
            except OSError:
                pass
        # "돌렸다"는 메모와 원본 사본이 남으면 돌린 지도인 척하므로 같이 치운다.
        for path in [pgm_tmp, yaml_tmp] + written:
            try:
                path.unlink()
            except OSError:
                pass
        result['message'] = f'지도를 돌리지 못해 그대로 저장했습니다: {error}'
        return result

    residual = tilt_of_image(rotated)
    result['result'] = 'rotated'
    result['rotated_deg'] = -tilt
    result['residual_deg'] = None if residual is None else round(residual, 2)
    result['message'] = f'지도를 {abs(tilt):.1f}° 돌려 바르게 세웠습니다.'
    return result


def _cmd_save(args) -> int:
    result = straighten_saved_map(
        Path(args.maps_dir), args.name, args.align, args.min_deg
    )
    tilt = result['tilt_deg']
    print(f'  기울기: {"측정 못 함" if tilt is None else f"{tilt:+.2f}°"}')
    if result['result'] == 'rotated':
        print(f'  돌림: {result["rotated_deg"]:+.2f}° → 남은 기울기 '
              f'{result.get("residual_deg")}°')
        print(f'  원본: {Path(args.maps_dir) / ORIGINAL_DIRNAME / (args.name + ".pgm")}')
    print(f'  {result["message"]}')
    print(RESULT_PREFIX + json.dumps(result, ensure_ascii=False))
    return 0


def _cmd_measure(args) -> int:
    for name in args.pgm:
        tilt = tilt_of_image(read_pgm(Path(name)))
        print(f'{name}: {"측정 못 함" if tilt is None else f"{tilt:+.2f}°"}')
    return 0


def main(argv=None) -> int:
    """Command line: ``save MAPS_DIR NAME [--align]`` or ``measure FILE.pgm…``."""
    parser = argparse.ArgumentParser(prog='map_align', description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest='command', required=True)
    save = sub.add_parser('save', help='저장 직후 기울기를 재고, 고르면 바로 세운다')
    save.add_argument('maps_dir')
    save.add_argument('name')
    save.add_argument('--align', action='store_true')
    save.add_argument('--min-deg', type=float, default=DEFAULT_MIN_DEG)
    save.set_defaults(func=_cmd_save)
    measure = sub.add_parser('measure', help='pgm 의 기울기만 잰다')
    measure.add_argument('pgm', nargs='+')
    measure.set_defaults(func=_cmd_measure)
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
