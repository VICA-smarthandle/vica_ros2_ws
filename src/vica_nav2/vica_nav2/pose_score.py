"""Score a candidate Nav2 initial pose against the map, using only the scan."""

from dataclasses import dataclass
import math

import numpy as np

try:  # pragma: no cover - 설치 여부에 따라 갈린다
    from scipy.ndimage import distance_transform_edt as _scipy_edt
except ImportError:  # pragma: no cover
    _scipy_edt = None


SIGMA_HIT = 0.2
MAX_DIST = 2.0
MAX_BEAMS = 180

MIN_RANGE = 0.25
MAX_RANGE = 11.0

OCCUPIED_THRESHOLD = 65


COARSE_XY_RADIUS = 0.5
COARSE_XY_STEP = 0.1
COARSE_YAW_STEP = math.radians(5.0)

TOP_K = 10
PEAK_MIN_XY = 0.15
PEAK_MIN_YAW = math.radians(10.0)

MID_XY_RADIUS = 0.06
MID_XY_STEP = 0.015
MID_YAW_RADIUS = math.radians(3.0)
MID_YAW_STEP = math.radians(0.5)

FINE_XY_RADIUS = 0.015
FINE_XY_STEP = 0.003
FINE_YAW_RADIUS = math.radians(0.6)
FINE_YAW_STEP = math.radians(0.1)

YAW_WINDOW = math.radians(45.0)

YAW_EDGE_WARN = math.radians(40.0)

SUPPRESS_YAW = math.radians(30.0)

MIN_SCORE = 70.0
WARN_SCORE = 50.0
MIN_BEAMS = 60
MIN_MARGIN = 10.0


@dataclass(frozen=True)
class MapGrid:
    """nav_msgs/OccupancyGrid 에서 필요한 것만 뽑은 것."""

    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float

    @property
    def height(self) -> int:
        """Return the row count."""
        return int(self.data.shape[0])

    @property
    def width(self) -> int:
        """Return the column count."""
        return int(self.data.shape[1])


@dataclass(frozen=True)
class ScanBeams:
    """필터와 솎기를 마친 빔. 센서 좌표계다."""

    ranges: np.ndarray
    angles: np.ndarray
    total: int

    @property
    def used(self) -> int:
        """How many beams survived filtering."""
        return int(self.ranges.size)


@dataclass(frozen=True)
class MatchResult:
    """One search outcome, already judged against the thresholds."""

    ok: bool
    reason: str
    score: float
    x: float
    y: float
    yaw: float
    used_beams: int
    total_beams: int
    runner_up_score: float
    margin: float
    moved_m: float
    moved_deg: float
    at_yaw_edge: bool


def filter_beams(
    ranges,
    angle_min: float,
    angle_increment: float,
    *,
    min_range: float = MIN_RANGE,
    max_range: float = MAX_RANGE,
    max_beams: int = MAX_BEAMS,
) -> ScanBeams:
    """Drop unusable beams, then thin what is left down to ``max_beams``."""
    values = np.asarray(ranges, dtype=np.float64)
    total = int(values.size)
    if total == 0:
        empty = np.empty(0, dtype=np.float64)
        return ScanBeams(empty, empty, 0)

    angles = angle_min + np.arange(total, dtype=np.float64) * angle_increment
    good = np.isfinite(values) & (values >= min_range) & (values <= max_range)
    index = np.nonzero(good)[0]

    if max_beams > 0 and index.size > max_beams:
        pick = np.linspace(0, index.size - 1, max_beams)
        index = np.unique(index[np.rint(pick).astype(np.int64)])

    return ScanBeams(values[index], angles[index], total)


def build_likelihood_field(
    grid: MapGrid,
    *,
    occupied_threshold: int = OCCUPIED_THRESHOLD,
    max_dist: float = MAX_DIST,
) -> np.ndarray:
    """Distance in metres from every cell to the nearest occupied cell, capped."""
    occupied = np.asarray(grid.data) >= occupied_threshold
    if not occupied.any():
        return np.full(occupied.shape, float(max_dist), dtype=np.float64)

    if _scipy_edt is not None:
        cells = _scipy_edt(~occupied)
    else:  # pragma: no cover - scipy 가 없는 젯슨을 위한 대비책
        cells = _chamfer_distance(occupied)

    return np.minimum(np.asarray(cells, dtype=np.float64) * grid.resolution, max_dist)


def _chamfer_distance(occupied: np.ndarray) -> np.ndarray:
    """Two-pass chamfer distance in cells. scipy 가 없을 때만 쓴다."""
    far = float(occupied.size)
    dist = np.where(occupied, 0.0, far)
    diag = math.sqrt(2.0)
    columns = np.arange(dist.shape[1], dtype=np.float64)

    def scan_row(row: np.ndarray) -> np.ndarray:
        left = np.minimum.accumulate(row - columns) + columns
        right = np.minimum.accumulate((row + columns)[::-1])[::-1] - columns
        return np.minimum(left, right)

    for _ in range(2):
        for i in range(dist.shape[0]):
            dist[i] = scan_row(dist[i])
        for step in (1, -1):
            rows = range(1, dist.shape[0]) if step == 1 else range(dist.shape[0] - 2, -1, -1)
            for i in rows:
                prev = dist[i - step]
                cand = prev + 1.0
                cand[1:] = np.minimum(cand[1:], prev[:-1] + diag)
                cand[:-1] = np.minimum(cand[:-1], prev[1:] + diag)
                dist[i] = np.minimum(dist[i], cand)
        dist = dist[::-1].copy()
    return dist


def _beam_points(beams: ScanBeams):
    """Beam endpoints in the sensor frame."""
    return beams.ranges * np.cos(beams.angles), beams.ranges * np.sin(beams.angles)


def beam_hits(beams: ScanBeams, x: float, y: float, yaw: float, *, sensor=(0.0, 0.0, 0.0)):
    """빔 끝점을 map 좌표로 옮긴다. 앱이 지도 위에 겹쳐 그릴 점들이다."""
    if beams.used == 0:
        return np.empty(0, dtype=np.float64), np.empty(0, dtype=np.float64)

    point_x, point_y = _beam_points(beams)
    sensor_x, sensor_y, sensor_yaw = sensor
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    laser_x = cos_yaw * sensor_x - sin_yaw * sensor_y
    laser_y = sin_yaw * sensor_x + cos_yaw * sensor_y
    beam_yaw = yaw + sensor_yaw
    cos_beam, sin_beam = math.cos(beam_yaw), math.sin(beam_yaw)

    world_x = x + laser_x + cos_beam * point_x - sin_beam * point_y
    world_y = y + laser_y + sin_beam * point_x + cos_beam * point_y
    return world_x, world_y


def _lookup(field: np.ndarray, grid: MapGrid, wx: np.ndarray, wy: np.ndarray,
            max_dist: float) -> np.ndarray:
    """Read the distance field at world points. 지도 밖은 완전 빗나감으로 친다."""
    col = np.floor((wx - grid.origin_x) / grid.resolution).astype(np.int64)
    row = np.floor((wy - grid.origin_y) / grid.resolution).astype(np.int64)
    inside = (col >= 0) & (col < grid.width) & (row >= 0) & (row < grid.height)
    out = np.full(wx.shape, float(max_dist), dtype=np.float64)
    if inside.any():
        out[inside] = field[row[inside], col[inside]]
    return out


def _score_translations(
    field: np.ndarray,
    grid: MapGrid,
    point_x: np.ndarray,
    point_y: np.ndarray,
    xs: np.ndarray,
    ys: np.ndarray,
    yaw: float,
    sensor,
    sigma_hit: float,
    max_dist: float,
) -> np.ndarray:
    """Score many (x, y) at one yaw. Returns (len(xs),) in 0..1."""
    sensor_x, sensor_y, sensor_yaw = sensor
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    laser_x = cos_yaw * sensor_x - sin_yaw * sensor_y
    laser_y = sin_yaw * sensor_x + cos_yaw * sensor_y
    beam_yaw = yaw + sensor_yaw
    cos_beam, sin_beam = math.cos(beam_yaw), math.sin(beam_yaw)

    rel_x = laser_x + cos_beam * point_x - sin_beam * point_y
    rel_y = laser_y + sin_beam * point_x + cos_beam * point_y

    world_x = xs[:, None] + rel_x[None, :]
    world_y = ys[:, None] + rel_y[None, :]
    dist = _lookup(field, grid, world_x, world_y, max_dist)
    return np.exp(-(dist * dist) / (2.0 * sigma_hit * sigma_hit)).mean(axis=1)


def score_pose(
    field: np.ndarray,
    grid: MapGrid,
    beams: ScanBeams,
    x: float,
    y: float,
    yaw: float,
    *,
    sensor=(0.0, 0.0, 0.0),
    sigma_hit: float = SIGMA_HIT,
    max_dist: float = MAX_DIST,
) -> float:
    """Score one pose. Returns 0..100. 확정 뒤 AMCL 자세를 다시 잴 때 쓴다."""
    if beams.used == 0:
        return 0.0
    point_x, point_y = _beam_points(beams)
    got = _score_translations(
        field, grid, point_x, point_y,
        np.array([x]), np.array([y]), yaw, sensor, sigma_hit, max_dist,
    )
    return float(got[0] * 100.0)


def _angle_diff(a: float, b: float) -> float:
    """Shortest signed difference a - b, wrapped to [-pi, pi]."""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def _grid_1d(center: float, radius: float, step: float) -> np.ndarray:
    count = int(round(radius / step))
    return center + np.arange(-count, count + 1, dtype=np.float64) * step


def _yaw_candidates(yaw_hint):
    """Full circle when the operator did not say which way the robot faces."""
    if yaw_hint is None:
        count = int(round(2.0 * math.pi / COARSE_YAW_STEP))
        return np.arange(count, dtype=np.float64) * COARSE_YAW_STEP
    return _grid_1d(float(yaw_hint), YAW_WINDOW, COARSE_YAW_STEP)


def _pick_peaks(all_x, all_y, all_yaw, all_score, count, min_xy, min_yaw):
    """점수 높은 순으로 **서로 떨어진** 후보만 고른다."""
    order = np.argsort(all_score)[::-1]
    picks = []
    for index in order:
        x, y, yaw = all_x[index], all_y[index], all_yaw[index]
        too_close = False
        for px, py, pyaw in picks:
            near_xy = math.hypot(x - px, y - py) < min_xy
            near_yaw = abs(_angle_diff(yaw, pyaw)) < min_yaw
            if near_xy and near_yaw:
                too_close = True
                break
        if not too_close:
            picks.append((float(x), float(y), float(yaw)))
            if len(picks) >= count:
                break
    return picks


def _parabola_offset(low: float, mid: float, high: float) -> float:
    """세 점의 값으로 격자 사이 꼭짓점 위치를 구한다. -0.5 ~ +0.5 칸."""
    if not (mid >= low and mid >= high):
        return 0.0
    denominator = low - 2.0 * mid + high
    if abs(denominator) < 1e-12:
        return 0.0
    offset = 0.5 * (low - high) / denominator
    return float(offset) if -0.5 <= offset <= 0.5 else 0.0


def _refine_axis(line: np.ndarray, index: int, step: float) -> float:
    """한 축을 따라 자른 점수 열에서 꼭짓점 보정량을 구한다. 미터 또는 라디안."""
    if line.size < 3 or index <= 0 or index >= line.size - 1:
        return 0.0
    return _parabola_offset(
        float(line[index - 1]), float(line[index]), float(line[index + 1]),
    ) * step


def _sweep(field, grid, beams, xs, ys, yaws, sensor, sigma_hit, max_dist):
    """Score the full (xs x ys x yaws) block. Returns flat arrays."""
    point_x, point_y = _beam_points(beams)
    mesh_x, mesh_y = np.meshgrid(xs, ys, indexing='ij')
    flat_x, flat_y = mesh_x.ravel(), mesh_y.ravel()

    scores = np.empty((yaws.size, flat_x.size), dtype=np.float64)
    for i, yaw in enumerate(yaws):
        scores[i] = _score_translations(
            field, grid, point_x, point_y, flat_x, flat_y,
            float(yaw), sensor, sigma_hit, max_dist,
        )
    return (
        np.tile(flat_x, yaws.size),
        np.tile(flat_y, yaws.size),
        np.repeat(yaws, flat_x.size),
        scores.ravel(),
    )


def search_pose(
    field: np.ndarray,
    grid: MapGrid,
    beams: ScanBeams,
    x: float,
    y: float,
    *,
    yaw_hint=None,
    sensor=(0.0, 0.0, 0.0),
    sigma_hit: float = SIGMA_HIT,
    max_dist: float = MAX_DIST,
    min_score: float = MIN_SCORE,
    min_beams: int = MIN_BEAMS,
    min_margin: float = MIN_MARGIN,
) -> MatchResult:
    """Find the best pose near (x, y) and judge it."""
    if beams.used == 0:
        return MatchResult(
            False, 'few_beams', 0.0, x, y, float(yaw_hint or 0.0),
            beams.used, beams.total, 0.0, 0.0, 0.0, 0.0, False,
        )

    coarse_x = _grid_1d(x, COARSE_XY_RADIUS, COARSE_XY_STEP)
    coarse_y = _grid_1d(y, COARSE_XY_RADIUS, COARSE_XY_STEP)
    coarse_yaw = _yaw_candidates(yaw_hint)

    all_x, all_y, all_yaw, all_score = _sweep(
        field, grid, beams, coarse_x, coarse_y, coarse_yaw, sensor, sigma_hit, max_dist,
    )
    best = int(np.argmax(all_score))
    best_yaw = all_yaw[best]

    wrapped = np.abs((all_yaw - best_yaw + math.pi) % (2.0 * math.pi) - math.pi)
    other_peak = wrapped > SUPPRESS_YAW
    runner_up = float(all_score[other_peak].max() * 100.0) if other_peak.any() else 0.0

    peaks = _pick_peaks(
        all_x, all_y, all_yaw, all_score, TOP_K, PEAK_MIN_XY, PEAK_MIN_YAW,
    )
    mid_best = None
    for peak_x, peak_y, peak_yaw in peaks:
        got = _sweep(
            field, grid, beams,
            _grid_1d(peak_x, MID_XY_RADIUS, MID_XY_STEP),
            _grid_1d(peak_y, MID_XY_RADIUS, MID_XY_STEP),
            _grid_1d(peak_yaw, MID_YAW_RADIUS, MID_YAW_STEP),
            sensor, sigma_hit, max_dist,
        )
        top = int(np.argmax(got[3]))
        if mid_best is None or got[3][top] > mid_best[3]:
            mid_best = (float(got[0][top]), float(got[1][top]),
                        float(got[2][top]), float(got[3][top]))
    best_x, best_y, best_yaw = mid_best[0], mid_best[1], mid_best[2]

    fine_xs = _grid_1d(best_x, FINE_XY_RADIUS, FINE_XY_STEP)
    fine_ys = _grid_1d(best_y, FINE_XY_RADIUS, FINE_XY_STEP)
    fine_yaws = _grid_1d(best_yaw, FINE_YAW_RADIUS, FINE_YAW_STEP)
    fine_x, fine_y, fine_yaw, fine_score = _sweep(
        field, grid, beams, fine_xs, fine_ys, fine_yaws, sensor, sigma_hit, max_dist,
    )
    top = int(np.argmax(fine_score))
    got_x = float(fine_x[top])
    got_y = float(fine_y[top])
    got_yaw = float(fine_yaw[top])
    score = float(fine_score[top] * 100.0)

    cube = fine_score.reshape(fine_yaws.size, fine_xs.size, fine_ys.size)
    yaw_i, xy_i = divmod(top, fine_xs.size * fine_ys.size)
    x_i, y_i = divmod(xy_i, fine_ys.size)
    got_x += _refine_axis(cube[yaw_i, :, y_i], x_i, FINE_XY_STEP)
    got_y += _refine_axis(cube[yaw_i, x_i, :], y_i, FINE_XY_STEP)
    got_yaw += _refine_axis(cube[:, x_i, y_i], yaw_i, FINE_YAW_STEP)
    got_yaw = float((got_yaw + math.pi) % (2.0 * math.pi) - math.pi)

    refined = score_pose(
        field, grid, beams, got_x, got_y, got_yaw,
        sensor=sensor, sigma_hit=sigma_hit, max_dist=max_dist,
    )
    if refined >= score:
        score = refined
    else:
        got_x = float(fine_x[top])
        got_y = float(fine_y[top])
        got_yaw = float((fine_yaw[top] + math.pi) % (2.0 * math.pi) - math.pi)

    turned = 0.0 if yaw_hint is None else _angle_diff(got_yaw, float(yaw_hint))

    window_margin = score - runner_up

    if yaw_hint is not None:
        flipped = max(
            score_pose(
                field, grid, beams, got_x, got_y, got_yaw + math.pi + d,
                sensor=sensor, sigma_hit=sigma_hit, max_dist=max_dist,
            )
            for d in (-FINE_YAW_RADIUS, 0.0, FINE_YAW_RADIUS)
        )
        runner_up = max(runner_up, flipped)

    margin = score - runner_up
    judge_margin = window_margin if yaw_hint is not None else margin
    ok, reason = judge(score, beams.used, judge_margin, min_score, min_beams, min_margin)

    return MatchResult(
        ok=ok,
        reason=reason,
        score=score,
        x=got_x,
        y=got_y,
        yaw=got_yaw,
        used_beams=beams.used,
        total_beams=beams.total,
        runner_up_score=runner_up,
        margin=margin,
        moved_m=float(math.hypot(got_x - x, got_y - y)),
        moved_deg=float(math.degrees(turned)),
        at_yaw_edge=bool(yaw_hint is not None and abs(turned) >= YAW_EDGE_WARN),
    )


def judge(
    score: float,
    used_beams: int,
    margin: float,
    min_score: float = MIN_SCORE,
    min_beams: int = MIN_BEAMS,
    min_margin: float = MIN_MARGIN,
):
    """Apply the three thresholds. Returns (ok, reason)."""
    if used_beams < min_beams:
        return False, 'few_beams'
    if score < min_score:
        return False, 'low_score'
    if margin < min_margin:
        return False, 'ambiguous'
    return True, ''
