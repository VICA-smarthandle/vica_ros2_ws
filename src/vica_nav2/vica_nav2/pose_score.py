"""Score a candidate Nav2 initial pose against the map, using only the scan.

rclpy 를 쓰지 않는다. 로봇 없이 pytest 로 전부 검증할 수 있어야 하기 때문이다.
ROS 배선은 pose_bootstrap_node 가 한다.

**왜 따로 채점하는가.** AMCL 은 "지도와 얼마나 맞는가"를 알려주지 않는다.
/amcl_pose 의 공분산은 입자가 얼마나 퍼졌는지지 지도와 맞는지가 아니다. 다 같이
틀린 자리에 모여 있어도 공분산은 작다. 그리고 /initialpose 를 발행하는 순간
되돌릴 수 없어서 "확인한 뒤 확정"이 원천적으로 안 된다. 그래서 확인은 이 모듈이,
반영만 AMCL 이 한다.

**잣대는 AMCL 것을 그대로 쓴다.** nav2_params.yaml 의

    laser_model_type: "likelihood_field"
    sigma_hit: 0.2
    laser_likelihood_max_dist: 2.0
    max_beams: 180

우리 점수가 높으면 AMCL 도 그 자세를 좋게 본다가 성립해야 하는데, 다른 잣대를
쓰면 그 보장이 깨진다.

**한 군데는 일부러 다르다.** AMCL 은 빔 점수를 세제곱해서 더한다(pz*pz*pz).
이 모듈은 평균만 낸다. 사람이 읽을 % 여야 하기 때문이다. "82%"가 "빔의 82% 만큼
맞았다"로 읽혀야 한다. 세제곱한 값은 사람에게 아무 의미가 없다.
[미검증] 두 방식이 같은 자세를 1등으로 뽑는지는 실기 bag 으로 확인해야 한다.
확인법은 test_pose_score.py 의 test_cube_weighting_picks_same_winner 주석에 적었다.
"""

from dataclasses import dataclass
import math

import numpy as np

try:  # pragma: no cover - 설치 여부에 따라 갈린다
    from scipy.ndimage import distance_transform_edt as _scipy_edt
except ImportError:  # pragma: no cover
    _scipy_edt = None


# --- AMCL 과 맞춘 값 (vica_nav2/config/nav2_params.yaml) ---------------------
SIGMA_HIT = 0.2
MAX_DIST = 2.0
MAX_BEAMS = 180

# --- Cartographer 와 맞춘 값 (vica_cartographer/config/vica_2d.lua) ----------
#
# min_range 0.25: 0.15 로 두면 차체 프레임이 장애물로 들어온다. 후방 좌우 대칭으로
#   0.220~0.273 m 근접 반사가 검출률 89~98% 로 잡힌다.
# max_range 11.0: [CM-1] 라이다 신고값은 12.00 m 지만 실내 최대 반사가 9.04 m 다.
#   12.0 으로 두면 아무것도 안 맞은 빔이 유효한 것처럼 섞인다.
MIN_RANGE = 0.25
MAX_RANGE = 11.0

# --- 점유 판정. map_preview.py 의 trinary 기준과 같다 ------------------------
OCCUPIED_THRESHOLD = 65

# --- 탐색 격자 ---------------------------------------------------------------
#
# 사람 손가락은 위치를 20~50 cm, 각도를 10~20도 틀린다. 그래서 찍은 값을 그대로
# 쓰지 않고 그 주변을 훑는다. Cartographer 의 real_time_correlative_scan_matcher
# 와 같은 방식이다.
COARSE_XY_RADIUS = 0.5
COARSE_XY_STEP = 0.1
COARSE_YAW_STEP = math.radians(5.0)
FINE_XY_RADIUS = 0.1
FINE_XY_STEP = 0.025
FINE_YAW_RADIUS = math.radians(5.0)
FINE_YAW_STEP = math.radians(1.0)

# 사람이 4방향 버튼으로 방향을 골랐을 때 훑는 범위.
#
# 90도 단위 입력의 최대 오차가 +-45도라 이 창과 딱 맞물린다. 어떤 방향이든 4칸 중
# 하나로 이 안에 들어오므로 빈틈이 없다. 후보가 8712 -> 2299 로 줄어드는 것은 덤이다.
YAW_WINDOW = math.radians(45.0)

# 결과가 창 가장자리에 붙으면 진짜 답이 창 밖일 수 있다. 앱이 경고 문구를 붙인다.
YAW_EDGE_WARN = math.radians(40.0)

# --- 2등 봉우리를 고를 때 최고점 주변을 얼마나 지울지 ------------------------
#
# **각도로만 지운다.** 위치는 안 본다. 이유가 있다.
#
# 복도에서는 앞뒤로 미끄러지는 후보들이 늘 비슷한 점수를 낸다. 그것까지 2등으로
# 세면 어느 복도에서나 격차가 0 이 되어 확정 버튼이 영영 안 열린다. 그런데 그
# 미끄러짐은 탐색 창이 +-0.5 m 라 애초에 0.5 m 를 못 넘고, 방향만 맞으면 AMCL 이
# 주행하면서 스스로 좁힌다.
#
# 정말 막아야 하는 것은 **180도 뒤집힘**이다. 그건 로봇이 반대로 가 버리는 사고고
# AMCL 이 스스로 못 고친다(recovery_alpha_fast/slow 가 0.0 이라 전역 재초기화를
# 하지 않는다). 그래서 "각도가 30도 넘게 다른 후보 중 최고점"을 2등으로 본다.
SUPPRESS_YAW = math.radians(30.0)

# --- 합격선 ------------------------------------------------------------------
#
# [TARGET] 실기에서 정답 자리(저장된 장소 핀)에 로봇을 놓고 잰 값으로 확정한다.
#
# MIN_SCORE 를 90 으로 못 잡는 이유: 지도에 없는 사람·가구가 점수를 깎는다.
#   1 m 앞에 선 사람 하나는 약 28도를 가린다. 180 빔 중 14 개가 통째로 0 점이다.
#   그 빔들은 "지도에는 2 m 안에 아무것도 없는데 라이다는 뭔가 봤다"라서 완전
#   빗나감으로 채점된다. 사람 둘에 카트 하나면 20% 가 날아간다.
#   반대로 텅 빈 복도에서 70% 밖에 안 나오면 짚은 자리가 틀린 것이다.
MIN_SCORE = 70.0
WARN_SCORE = 50.0
MIN_BEAMS = 60
MIN_MARGIN = 10.0


@dataclass(frozen=True)
class MapGrid:
    """nav_msgs/OccupancyGrid 에서 필요한 것만 뽑은 것."""

    data: np.ndarray          # (height, width) int8. row 0 이 origin_y 쪽이다
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
    total: int                # 필터 전 빔 수

    @property
    def used(self) -> int:
        """How many beams survived filtering."""
        return int(self.ranges.size)


@dataclass(frozen=True)
class MatchResult:
    """One search outcome, already judged against the thresholds."""

    ok: bool
    reason: str
    score: float              # 0~100
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
    """Drop unusable beams, then thin what is left down to ``max_beams``.

    거르는 것과 근거:
      - min_range 미만: 차체 프레임 반사다. 지도에 없는 물체라 무조건 감점이 된다.
      - max_range 초과: 아무것도 안 맞은 빔이다. 끝점이 허공이라 채점 대상이 아니다.
      - NaN/inf: 드라이버가 무반사를 이렇게 내보낸다.

    180 개로 솎는 것은 AMCL 의 max_beams 와 일부러 같은 수다. 계산량이 4 분의 1 로
    주는 것은 덤이고, 본뜻은 같은 정보량으로 채점한다는 것이다.
    """
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
    """Distance in metres from every cell to the nearest occupied cell, capped.

    노드가 켜질 때 **한 번만** 만든다. 이후 채점은 배열 인덱싱이라 사실상 공짜다.

    **max_dist 로 자르는 것은 점수에 영향이 없다.** sigma_hit 이 0.2 라
    exp(-d^2/0.08) 은 d=0.8 m 에서 이미 0.0003 이다. 2 m 에서 자르든 5 m 에서
    자르든 읽히는 값이 같다. 자르는 이유는 순전히 계산 절약이다. 상한이 없으면
    벽에서 먼 칸까지 전부 채워야 한다.

    unknown(-1) 은 점유로 보지 않는다. 아직 안 가 본 곳이지 벽이 아니다.
    """
    occupied = np.asarray(grid.data) >= occupied_threshold
    if not occupied.any():
        return np.full(occupied.shape, float(max_dist), dtype=np.float64)

    if _scipy_edt is not None:
        cells = _scipy_edt(~occupied)
    else:  # pragma: no cover - scipy 가 없는 젯슨을 위한 대비책
        cells = _chamfer_distance(occupied)

    return np.minimum(np.asarray(cells, dtype=np.float64) * grid.resolution, max_dist)


def _chamfer_distance(occupied: np.ndarray) -> np.ndarray:
    """Two-pass chamfer distance in cells. scipy 가 없을 때만 쓴다.

    정확한 EDT 보다 최대 4% 정도 크게 나온다. d=0.2 m 에서 8 mm 인데 sigma_hit 이
    0.2 m 라 점수 차이가 없다. 정확도가 아니라 의존성을 줄이려는 것이다.
    """
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
    """Score many (x, y) at one yaw. Returns (len(xs),) in 0..1.

    yaw 별로 묶는 이유: 빔을 돌리는 계산은 yaw 에만 달려 있다. 한 번 돌려 놓고
    위치는 더하기만 하면 된다. 후보 8712 개에 회전을 8712 번 하는 대신 72 번 한다.
    """
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
    """Find the best pose near (x, y) and judge it.

    사람은 "이 방쯤"만 짚고 정밀한 자세는 여기서 찾는다. 지도 전체를 훑지 않으므로
    자동 위치 추정에서 문제가 됐던 "비슷한 복도끼리 헷갈림"이 애초에 안 생긴다.
    """
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
    best_x, best_y, best_yaw = all_x[best], all_y[best], all_yaw[best]

    # 2등은 각도가 크게 다른 후보 중에서 고른다. 앞뒤 미끄러짐은 2등으로 세지 않는다.
    wrapped = np.abs((all_yaw - best_yaw + math.pi) % (2.0 * math.pi) - math.pi)
    other_peak = wrapped > SUPPRESS_YAW
    runner_up = float(all_score[other_peak].max() * 100.0) if other_peak.any() else 0.0

    fine_x, fine_y, fine_yaw, fine_score = _sweep(
        field, grid, beams,
        _grid_1d(best_x, FINE_XY_RADIUS, FINE_XY_STEP),
        _grid_1d(best_y, FINE_XY_RADIUS, FINE_XY_STEP),
        _grid_1d(best_yaw, FINE_YAW_RADIUS, FINE_YAW_STEP),
        sensor, sigma_hit, max_dist,
    )
    top = int(np.argmax(fine_score))
    got_x = float(fine_x[top])
    got_y = float(fine_y[top])
    got_yaw = float((fine_yaw[top] + math.pi) % (2.0 * math.pi) - math.pi)
    score = float(fine_score[top] * 100.0)

    turned = 0.0 if yaw_hint is None else _angle_diff(got_yaw, float(yaw_hint))
    margin = score - runner_up
    ok, reason = judge(score, beams.used, margin, min_score, min_beams, min_margin)

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
    """Apply the three thresholds. Returns (ok, reason).

    순서가 있다. 빔이 모자라면 점수 자체가 못 믿을 값이라 먼저 본다. 격차는 점수가
    합격한 뒤에만 의미가 있다 -- 둘 다 낮으면 그냥 잘못 짚은 것이지 대칭이 아니다.
    """
    if used_beams < min_beams:
        return False, 'few_beams'
    if score < min_score:
        return False, 'low_score'
    if margin < min_margin:
        return False, 'ambiguous'
    return True, ''
