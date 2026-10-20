"""Measure how far a map is tilted and straighten it on save (2026-10-07)."""

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import sys

import numpy as np

DEFAULT_MIN_DEG = 2.0

MIN_WALL_CELLS = 150

MIN_PEAK_RATIO = 1.5

MAX_TILT_POINTS = 8000

COARSE_STEP_DEG = 1.0
FINE_STEP_DEG = 0.05

GRAY_OCCUPIED = 0
GRAY_FREE = 254
GRAY_UNKNOWN = 205
WALL_MAX = 100
FREE_MIN = 250

ROTATE_CHUNK_CELLS = 250_000

MARGIN_CELLS = 10

ORIGINAL_DIRNAME = '.original'

RESULT_PREFIX = 'VICA_ALIGN '


def _projection_score(xs, ys, angle_rad: float) -> float:
    """Sum of squared histogram counts along the two axes at this angle."""
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    score = 0.0
    for along in (xs * cos_a + ys * sin_a, -xs * sin_a + ys * cos_a):
        bins = np.floor(along - along.min()).astype(np.int64)
        counts = np.bincount(bins)
        score += float(np.dot(counts, counts))
    return score


def estimate_tilt_deg(xs, ys):
    """Return the wall direction in degrees (−45, 45], or None if unknown."""
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
    top = np.flatnonzero(fine_scores >= fine_scores.max() * 0.999)
    if top[-1] - top[0] == len(top) - 1:
        angle = float((fine[top[0]] + fine[top[-1]]) / 2)
    else:
        angle = float(fine[top[0]])
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


def _close_free_gaps(image):
    """Fill gaps of up to two cells between walls, but only where the cell was free."""
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
    """Rotate a top-down gray map by −tilt_deg. Returns (image, origin_x, origin_y)."""
    image = np.asarray(image, dtype=np.uint8)
    height, width = image.shape
    theta = math.radians(tilt_deg)
    cos_t, sin_t = math.cos(theta), math.sin(theta)

    rows, cols = np.nonzero(image != GRAY_UNKNOWN)
    if rows.size == 0:
        rows = np.array([0, height - 1])
        cols = np.array([0, width - 1])
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
    index += 1
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
    """Rename leftovers of an older map with the same name. 지우지 않는다."""
    moved = []
    for suffix in ('.pgm', '.json'):
        path = original_dir / f'{name}{suffix}'
        if path.exists():
            target = original_dir / f'{name}{suffix}.orphan-{stamp}'
            os.replace(path, target)
            moved.append(target.name)
    return moved


def straighten_saved_map(maps_dir: Path, name: str, align: bool,
                         min_deg: float = DEFAULT_MIN_DEG, now=None) -> dict:
    """Measure, and when asked, straighten maps/<name>.pgm·yaml in place."""
    maps_dir = Path(maps_dir)
    now = now or datetime.now()
    pgm_path = maps_dir / f'{name}.pgm'
    yaml_path = maps_dir / f'{name}.yaml'
    result = {'result': 'failed', 'tilt_deg': None, 'rotated_deg': 0.0,
              'min_deg': min_deg, 'message': ''}

    original_dir = maps_dir / ORIGINAL_DIRNAME
    stamp = now.strftime('%Y%m%d-%H%M%S')
    try:
        moved = _set_orphans_aside(original_dir, name, stamp) if original_dir.is_dir() else []
    except OSError:
        moved = []

    try:
        image = read_pgm(pgm_path)
        yaml_raw = yaml_path.read_bytes()
        lines, resolution, origin_x, origin_y = read_map_yaml(yaml_path)
    except (OSError, ValueError) as error:
        if not align:
            result['result'] = 'not_requested'
            result['message'] = '정렬하지 않고 그대로 저장했습니다.'
        else:
            result['message'] = f'지도 파일을 읽지 못해 그대로 둡니다: {error}'
        return result

    measured = tilt_of_image(image)
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
    if abs(tilt) < min_deg:
        result['result'] = 'small'
        result['message'] = f'기울기가 {abs(tilt):.1f}°라 돌리지 않고 그대로 저장했습니다.'
        return result

    pgm_tmp = maps_dir / f'{name}.pgm.align-tmp'
    yaml_tmp = maps_dir / f'{name}.yaml.align-tmp'
    backup_pgm = original_dir / f'{name}.pgm'
    backup_memo = original_dir / f'{name}.json'
    written = []
    yaml_replaced = False
    try:
        rotated, new_x, new_y = rotate_map(
            image, resolution, origin_x, origin_y, measured
        )
        original_dir.mkdir(exist_ok=True)
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
        if yaml_replaced:
            try:
                yaml_path.write_bytes(yaml_raw)
            except OSError:
                pass
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
