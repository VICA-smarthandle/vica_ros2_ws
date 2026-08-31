"""Contract tests for pose_score.

로봇 없이 돌아간다. 인공 지도를 만들고 거기서 스캔을 쏘아 만들기 때문이다.
합격선 숫자를 근거 없이 정하지 않으려고 실기보다 먼저 만든 시험이다.
"""

import math

import numpy as np
import pytest

from vica_nav2.pose_score import (
    build_likelihood_field,
    filter_beams,
    judge,
    MapGrid,
    MAX_DIST,
    _parabola_offset,
    _pick_peaks,
    score_pose,
    search_pose,
)

RESOLUTION = 0.05
LASER_OFFSET = (0.185, 0.0, 0.0)   # 라이다가 로봇 중심보다 18.5 cm 앞이다


def _blank(width_m, height_m):
    """Solid block of occupied cells to carve corridors out of."""
    cols = int(width_m / RESOLUTION)
    rows = int(height_m / RESOLUTION)
    return np.full((rows, cols), 100, dtype=np.int8)


def _carve(data, x0, y0, x1, y1):
    """Carve a free rectangle, in metres, out of the block."""
    c0, c1 = int(x0 / RESOLUTION), int(x1 / RESOLUTION)
    r0, r1 = int(y0 / RESOLUTION), int(y1 / RESOLUTION)
    data[r0:r1, c0:c1] = 0


def cross_map():
    """가로 복도 하나와 세로 복도 하나. 세로가 가운데가 아니라 좌우 비대칭이다."""
    data = _blank(20.0, 20.0)
    _carve(data, 1.0, 8.0, 19.0, 10.0)
    _carve(data, 5.0, 1.0, 7.0, 19.0)
    return MapGrid(data, RESOLUTION, 0.0, 0.0)


def straight_map():
    """40 m 곧은 복도. 양쪽 끝이 11 m 밖이라 앞뒤가 완전히 대칭이다."""
    data = _blank(40.0, 20.0)
    _carve(data, 1.0, 8.0, 39.0, 10.0)
    return MapGrid(data, RESOLUTION, 0.0, 0.0)


def simulate_scan(grid, x, y, yaw, *, sensor=LASER_OFFSET, count=240, max_range=11.0):
    """Ray cast a 360 degree scan from the sensor mounted on a robot at (x, y, yaw)."""
    sx, sy, syaw = sensor
    origin_x = x + math.cos(yaw) * sx - math.sin(yaw) * sy
    origin_y = y + math.sin(yaw) * sx + math.cos(yaw) * sy
    origin_yaw = yaw + syaw

    angle_min = -math.pi
    angle_increment = 2.0 * math.pi / count
    step = grid.resolution
    ranges = []
    for i in range(count):
        theta = origin_yaw + angle_min + i * angle_increment
        dx, dy = math.cos(theta) * step, math.sin(theta) * step
        cx, cy, travelled = origin_x, origin_y, 0.0
        hit = float('inf')
        while travelled <= max_range:
            col = int((cx - grid.origin_x) / grid.resolution)
            row = int((cy - grid.origin_y) / grid.resolution)
            if not (0 <= col < grid.width and 0 <= row < grid.height):
                break
            if grid.data[row, col] >= 65:
                hit = travelled
                break
            cx += dx
            cy += dy
            travelled += step
        ranges.append(hit)
    return np.array(ranges), angle_min, angle_increment


def beams_at(grid, x, y, yaw, **kwargs):
    """Simulate a scan and run it through the same filter the node uses."""
    ranges, angle_min, increment = simulate_scan(grid, x, y, yaw, **kwargs)
    return filter_beams(ranges, angle_min, increment)


# --- 빔 필터 ---------------------------------------------------------------

def test_filter_drops_body_reflection_and_no_return():
    """0.25 m 미만은 차체, 11 m 초과와 inf 는 무반사다. 셋 다 채점 대상이 아니다."""
    ranges = [0.22, 0.24, 3.0, 5.5, 11.5, float('inf'), float('nan')]
    beams = filter_beams(ranges, -math.pi, 0.1)
    assert beams.total == 7
    assert beams.used == 2
    assert np.allclose(sorted(beams.ranges), [3.0, 5.5])


def test_filter_thins_down_to_max_beams():
    """AMCL 의 max_beams 180 과 같은 수로 맞춘다."""
    beams = filter_beams([5.0] * 721, -math.pi, 2 * math.pi / 721)
    assert beams.total == 721
    assert beams.used == 180


def test_filter_keeps_everything_when_already_sparse():
    """솎기는 180 개를 넘을 때만 한다."""
    beams = filter_beams([5.0] * 40, -math.pi, 0.1)
    assert beams.used == 40


# --- 거리표 -----------------------------------------------------------------

def test_field_is_zero_on_walls_and_grows_toward_the_middle():
    """거리표가 벽에서 0 이고 복도 한가운데로 갈수록 커진다."""
    grid = cross_map()
    field = build_likelihood_field(grid)
    on_wall = field[int(8.0 / RESOLUTION) - 1, int(12.0 / RESOLUTION)]
    mid_corridor = field[int(9.0 / RESOLUTION), int(12.0 / RESOLUTION)]
    assert on_wall == pytest.approx(0.0)
    # 복도 폭이 2 m 라 한가운데가 벽에서 1 m 다. 상한 2 m 에는 닿지도 않는다.
    assert mid_corridor == pytest.approx(1.0, abs=0.05)
    assert field.max() < MAX_DIST


def test_field_with_no_walls_is_all_capped():
    """벽이 하나도 없으면 모든 칸이 상한이다. 0 으로 나누는 사고를 막는다."""
    grid = MapGrid(np.zeros((20, 20), dtype=np.int8), RESOLUTION, 0.0, 0.0)
    assert build_likelihood_field(grid).min() == pytest.approx(MAX_DIST)


def test_unknown_cells_are_not_walls():
    """-1 은 아직 안 가 본 곳이지 벽이 아니다."""
    data = np.full((40, 40), -1, dtype=np.int8)
    field = build_likelihood_field(MapGrid(data, RESOLUTION, 0.0, 0.0))
    assert field.min() == pytest.approx(MAX_DIST)


def test_cutting_the_field_at_2m_does_not_change_the_score():
    """2 m 에서 자르든 5 m 에서 자르든 점수가 같다.

    sigma_hit 0.2 라 exp(-d^2/0.08) 은 0.8 m 에서 이미 0.0003 이다. 자르는 것은
    정확도가 아니라 계산량 때문이라는 설명의 근거다.
    """
    grid = cross_map()
    beams = beams_at(grid, 12.0, 9.0, 0.0)
    tight = build_likelihood_field(grid, max_dist=2.0)
    loose = build_likelihood_field(grid, max_dist=5.0)
    a = score_pose(tight, grid, beams, 11.4, 9.4, 0.3, sensor=LASER_OFFSET, max_dist=2.0)
    b = score_pose(loose, grid, beams, 11.4, 9.4, 0.3, sensor=LASER_OFFSET, max_dist=5.0)
    assert a == pytest.approx(b, abs=0.01)


# --- 채점 -------------------------------------------------------------------

def test_truth_scores_high_and_sideways_error_scores_low():
    """복도를 가로지르는 방향으로 어긋나면 점수가 확 떨어진다."""
    grid = cross_map()
    field = build_likelihood_field(grid)
    beams = beams_at(grid, 12.0, 9.0, 0.0)
    truth = score_pose(field, grid, beams, 12.0, 9.0, 0.0, sensor=LASER_OFFSET)
    sideways = score_pose(field, grid, beams, 12.0, 9.5, 0.0, sensor=LASER_OFFSET)
    assert truth > 95.0
    assert sideways < 60.0


def test_sliding_along_a_corridor_barely_changes_the_score():
    """앞뒤로 1 m 밀려도 점수가 거의 안 깎인다. 점수만으로는 못 잡는 오차다.

    이게 이 로봇이 2026-08-12 에 44 m 복도에서 7.01 m 밀린 것과 같은 현상이다
    (회전 오차는 0.031 로 멀쩡했다). 양옆 벽은 앞뒤에 대해 아무 말도 하지 않는다.

    그래서 설계가 이렇게 갈린다:
      - 앞뒤 오차는 **사람이 짚는 +-0.5 m 창**으로 묶는다. 점수로는 못 묶는다.
      - 2 등 판정은 위치가 아니라 **각도**로만 한다. 앞뒤 미끄러짐까지 2 등으로
        세면 어느 복도에서나 격차가 0 이 되어 확정 버튼이 영영 안 열린다.
    """
    grid = cross_map()
    field = build_likelihood_field(grid)
    beams = beams_at(grid, 12.0, 9.0, 0.0)
    truth = score_pose(field, grid, beams, 12.0, 9.0, 0.0, sensor=LASER_OFFSET)
    slid = score_pose(field, grid, beams, 13.0, 9.0, 0.0, sensor=LASER_OFFSET)
    assert truth - slid < 15.0


def test_sensor_offset_shifts_the_recovered_pose():
    """18.5 cm 를 빼먹으면 찾아낸 자세가 그만큼 밀린다. TF 를 꼭 써야 하는 근거다.

    복도에서는 이 밀림이 점수를 거의 안 깎으므로(위 시험) 점수로는 못 알아챈다.
    조용히 틀리는 종류라 더 위험하다.
    """
    grid = cross_map()
    field = build_likelihood_field(grid)
    beams = beams_at(grid, 12.0, 9.0, 0.0)
    right = search_pose(field, grid, beams, 12.0, 9.0, yaw_hint=0.0, sensor=LASER_OFFSET)
    wrong = search_pose(field, grid, beams, 12.0, 9.0, yaw_hint=0.0, sensor=(0.0, 0.0, 0.0))
    assert abs(right.x - 12.0) < 0.05
    assert wrong.x - right.x == pytest.approx(0.185, abs=0.05)


def test_points_outside_the_map_count_as_a_full_miss():
    """지도 밖 끝점은 완전 빗나감이다. 무시하면 지도 밖이 만점이 된다."""
    grid = cross_map()
    field = build_likelihood_field(grid)
    beams = beams_at(grid, 12.0, 9.0, 0.0)
    assert score_pose(field, grid, beams, 500.0, 500.0, 0.0) == pytest.approx(0.0, abs=1e-6)


# --- 탐색 -------------------------------------------------------------------

def test_search_recovers_the_pose_from_a_rough_tap():
    """사람이 30 cm·15도 틀리게 짚어도 매처가 되찾는다."""
    grid = cross_map()
    field = build_likelihood_field(grid)
    beams = beams_at(grid, 12.0, 9.0, 0.0)
    got = search_pose(
        field, grid, beams, 12.3, 8.8,
        yaw_hint=math.radians(15.0), sensor=LASER_OFFSET,
    )
    assert got.ok
    assert math.hypot(got.x - 12.0, got.y - 9.0) < 0.08
    assert abs(math.degrees(got.yaw)) < 3.0
    assert got.score > 90.0
    assert got.moved_m > 0.2


def test_search_reports_how_far_it_moved_the_tap():
    """얼마나 옮겼는지를 앱이 그대로 보여줄 수 있어야 한다."""
    grid = cross_map()
    field = build_likelihood_field(grid)
    beams = beams_at(grid, 12.0, 9.0, 0.0)
    got = search_pose(field, grid, beams, 12.3, 9.0, yaw_hint=0.0, sensor=LASER_OFFSET)
    assert got.moved_m == pytest.approx(math.hypot(got.x - 12.3, got.y - 9.0), abs=1e-9)
    assert abs(got.moved_deg) < 5.0
    assert not got.at_yaw_edge


def test_search_without_a_hint_still_finds_the_angle():
    """방향을 안 줘도 5도 간격으로 360도를 훑으니 각도 자체는 찾는다."""
    grid = cross_map()
    field = build_likelihood_field(grid)
    beams = beams_at(grid, 12.0, 9.0, math.radians(90.0))
    got = search_pose(field, grid, beams, 12.0, 9.0, yaw_hint=None, sensor=LASER_OFFSET)
    assert abs(math.degrees(got.yaw) - 90.0) < 3.0


def test_symmetric_corridor_is_reported_as_ambiguous():
    """양옆 벽만 보이는 복도에서는 0도와 180도가 똑같이 맞는다. 정보가 없는 것이다."""
    grid = straight_map()
    field = build_likelihood_field(grid)
    beams = beams_at(grid, 20.0, 9.0, 0.0)
    got = search_pose(field, grid, beams, 20.0, 9.0, yaw_hint=None, sensor=LASER_OFFSET)
    assert got.score > 90.0          # 잘 맞기는 한다
    assert got.margin < 10.0         # 그런데 뒤집힌 자세도 똑같이 잘 맞는다
    assert not got.ok
    assert got.reason == 'ambiguous'


def test_direction_hint_resolves_the_flip():
    """사람이 방향을 알려주면 뒤집힌 후보가 애초에 후보에 안 들어온다."""
    grid = straight_map()
    field = build_likelihood_field(grid)
    beams = beams_at(grid, 20.0, 9.0, 0.0)
    got = search_pose(field, grid, beams, 20.0, 9.0, yaw_hint=0.0, sensor=LASER_OFFSET)
    assert got.ok
    assert abs(math.degrees(got.yaw)) < 5.0


def test_flipped_hint_passes_but_reports_a_collapsed_margin():
    """대칭 복도에서 반대 힌트도 통과는 한다 — 사람이 최종 판단하는 설계 계약.

    다만 격차(margin)에는 창 밖의 뒤집힌 자세를 채점해 넣으므로 무너진 값이
    보고되고, 앱이 '헷갈릴 방향 있음' 과 경고 문구를 띄울 재료가 된다.
    2026-08-25 실기에서 반대 방향이 77점·격차 41로 조용히 통과한 것의 수리다.
    """
    grid = straight_map()
    field = build_likelihood_field(grid)
    beams = beams_at(grid, 20.0, 9.0, 0.0)  # 실제 로봇은 0도를 본다
    got = search_pose(
        field, grid, beams, 20.0, 9.0, yaw_hint=math.pi, sensor=LASER_OFFSET,
    )
    assert got.ok                        # 확정 가능 여부는 사람 몫 그대로
    assert abs(abs(math.degrees(got.yaw)) - 180.0) < 5.0
    assert got.margin < 10.0             # 진짜 방향(창 밖)이 격차를 무너뜨린다


def test_correct_hint_in_an_asymmetric_spot_keeps_a_wide_margin():
    """비대칭한 자리에서는 올바른 힌트의 격차가 넓게 유지된다 — 경고 오발 방지.

    자리는 끝벽 2 m 앞이다. 뒤집힌 해석은 그 벽을 반대쪽(11 m 밖)에 둬야 해서
    수십 개 빔이 통째로 어긋난다. 복도 순수 중앙(대칭)과 달리 여기서는 경고가
    나면 안 된다.
    """
    grid = cross_map()
    field = build_likelihood_field(grid)
    beams = beams_at(grid, 3.0, 9.0, 0.0)
    got = search_pose(field, grid, beams, 3.0, 9.0, yaw_hint=0.0, sensor=LASER_OFFSET)
    assert got.ok
    assert got.margin >= 10.0


def test_hint_window_covers_the_worst_90_degree_input():
    """4 방향 버튼의 최대 오차 45도가 탐색 창과 딱 맞물린다는 것."""
    grid = cross_map()
    field = build_likelihood_field(grid)
    beams = beams_at(grid, 12.0, 9.0, math.radians(44.0))
    got = search_pose(field, grid, beams, 12.0, 9.0, yaw_hint=0.0, sensor=LASER_OFFSET)
    assert abs(math.degrees(got.yaw) - 44.0) < 3.0
    assert got.at_yaw_edge          # 창 가장자리라 앱이 경고를 붙여야 한다


def test_search_with_no_beams_is_blocked_not_crashed():
    """빔이 없어도 죽지 않고 막는다."""
    grid = cross_map()
    field = build_likelihood_field(grid)
    beams = filter_beams([float('inf')] * 100, -math.pi, 0.06)
    got = search_pose(field, grid, beams, 12.0, 9.0, sensor=LASER_OFFSET)
    assert not got.ok
    assert got.reason == 'few_beams'
    assert got.score == 0.0


# --- 판정 -------------------------------------------------------------------

def test_judge_checks_beams_before_score():
    """빔이 모자라면 점수 자체가 못 믿을 값이라 먼저 본다."""
    assert judge(99.0, 10, 50.0) == (False, 'few_beams')


def test_judge_checks_score_before_margin():
    """둘 다 낮으면 대칭이 아니라 그냥 잘못 짚은 것이다."""
    assert judge(30.0, 180, 1.0) == (False, 'low_score')


def test_judge_passes_only_when_all_three_pass():
    """점수·빔·격차 셋을 다 넘어야 확정 버튼이 열린다."""
    assert judge(75.0, 180, 20.0) == (True, '')
    assert judge(75.0, 180, 5.0) == (False, 'ambiguous')


# --- AMCL 과의 차이 ---------------------------------------------------------

def test_cube_weighting_picks_the_same_winner():
    """AMCL 은 빔 점수를 세제곱해서 더하고 우리는 평균만 낸다.

    사람이 읽을 % 여야 해서 일부러 다르게 뒀다. 그 대가로 1 등이 갈릴 수 있는데,
    인공 지도에서는 갈리지 않는다는 것을 여기서 못박는다.

    [미검증] 실기 bag 에서도 같은지는 확인해야 한다. 확인법은 같다 --
    같은 스캔으로 두 잣대의 1 등을 각각 구해 격자 간격(2.5 cm · 1도) 안인지 본다.
    """
    grid = cross_map()
    field = build_likelihood_field(grid)
    beams = beams_at(grid, 12.0, 9.0, 0.0)

    z_hit, z_rand_term = 0.5, 0.5 / 12.0
    best_mean, best_cube = None, None
    for dx in np.arange(-0.3, 0.31, 0.05):
        for dy in np.arange(-0.3, 0.31, 0.05):
            for deg in range(-10, 11, 2):
                pose = (12.0 + dx, 9.0 + dy, math.radians(deg))
                mean = score_pose(field, grid, beams, *pose, sensor=LASER_OFFSET)
                per_beam = z_hit * (mean / 100.0) + z_rand_term
                cube = per_beam ** 3
                if best_mean is None or mean > best_mean[0]:
                    best_mean = (mean, pose)
                if best_cube is None or cube > best_cube[0]:
                    best_cube = (cube, pose)

    assert best_mean[1] == best_cube[1]


# --- scipy 없는 젯슨 대비책 -------------------------------------------------

def test_chamfer_fallback_is_close_enough_to_scipy():
    """Scipy 가 없어도 쓸 수 있어야 한다. 오차가 sigma_hit 앞에서 무의미한지 본다."""
    from vica_nav2 import pose_score

    grid = cross_map()
    exact = build_likelihood_field(grid)
    saved = pose_score._scipy_edt
    try:
        pose_score._scipy_edt = None
        approx = build_likelihood_field(grid)
    finally:
        pose_score._scipy_edt = saved

    assert np.abs(approx - exact).max() < 0.1

    beams = beams_at(grid, 12.0, 9.0, 0.0)
    a = score_pose(exact, grid, beams, 12.0, 9.0, 0.0, sensor=LASER_OFFSET)
    b = score_pose(approx, grid, beams, 12.0, 9.0, 0.0, sensor=LASER_OFFSET)
    assert a == pytest.approx(b, abs=1.0)


# --- 3단계 탐색 (2026-08-31) -------------------------------------------------
#
# 종전 2단계는 정밀 격자가 2.5 cm 라 같은 자리를 여러 번 짚으면 68 점과 72 점이
# 오갔다. 합격선이 70 이라 손가락 1~2 cm 차이로 당락이 갈렸다.


def test_repeated_taps_land_on_the_same_answer():
    """이 파일에서 가장 중요한 시험이다.

    관리자가 같은 자리 주변을 여러 번 눌러도 **같은 답**이 나와야 한다.
    실기에서 "될 때도 있고 안 될 때도 있다"고 보고된 증상이 이것이었다.
    """
    grid = cross_map()
    field = build_likelihood_field(grid)
    truth = (12.0, 9.0, 0.3)
    beams = beams_at(grid, *truth)

    found = []
    for dx, dy in [(-0.25, -0.2), (-0.1, 0.15), (0.0, 0.0),
                   (0.12, -0.08), (0.25, 0.22), (0.18, 0.05)]:
        result = search_pose(
            field, grid, beams,
            truth[0] + dx, truth[1] + dy,
            yaw_hint=truth[2] + math.radians(15),
            sensor=LASER_OFFSET,
        )
        found.append(result)

    # 되찾은 자세들이 서로 1 cm 안에 모여야 한다.
    xs = [r.x for r in found]
    ys = [r.y for r in found]
    assert max(xs) - min(xs) < 0.01
    assert max(ys) - min(ys) < 0.01

    # 점수도 흔들리지 않아야 한다. 합격선 근처에서 오르내리면 당락이 갈린다.
    scores = [r.score for r in found]
    assert max(scores) - min(scores) < 1.0


def test_search_beats_the_old_grid_resolution():
    """격자(3 mm)보다 정밀해야 한다. 포물선 보간이 하는 일이다."""
    grid = cross_map()
    field = build_likelihood_field(grid)
    truth = (12.0, 9.0, 0.0)
    beams = beams_at(grid, *truth)

    result = search_pose(
        field, grid, beams,
        truth[0] + 0.17, truth[1] - 0.13,
        yaw_hint=truth[2], sensor=LASER_OFFSET,
    )
    assert math.hypot(result.x - truth[0], result.y - truth[1]) < 0.01


# --- 포물선 보간 -------------------------------------------------------------


def test_parabola_finds_the_peak_between_samples():
    """오른쪽이 높으면 꼭짓점도 오른쪽에 있다."""
    assert _parabola_offset(0.5, 1.0, 0.9) > 0
    assert _parabola_offset(0.9, 1.0, 0.5) < 0
    assert _parabola_offset(0.5, 1.0, 0.5) == pytest.approx(0.0)


def test_parabola_refuses_when_the_middle_is_not_the_peak():
    """가운데가 최대가 아니면 포물선 가정이 깨진다. 그때는 손대지 않는다."""
    assert _parabola_offset(1.0, 0.5, 0.9) == 0.0
    assert _parabola_offset(1.0, 1.0, 1.0) == 0.0


# --- 봉우리 고르기 -----------------------------------------------------------


def test_peaks_skip_neighbours_of_the_same_hill():
    """상위 N 개를 그냥 뽑으면 한 봉우리의 이웃 칸이 자리를 다 차지한다."""
    xs = np.array([0.0, 0.02, 1.0, 1.02])
    ys = np.zeros(4)
    yaws = np.zeros(4)
    scores = np.array([0.9, 0.89, 0.8, 0.79])

    picks = _pick_peaks(xs, ys, yaws, scores, 4, 0.15, math.radians(10))

    assert len(picks) == 2
    assert picks[0][0] == pytest.approx(0.0)
    assert picks[1][0] == pytest.approx(1.0)


def test_peaks_keep_a_different_angle_at_the_same_spot():
    """같은 자리라도 각도가 다르면 다른 봉우리다 - 180도 뒤집힘이 그렇다."""
    xs = np.zeros(2)
    ys = np.zeros(2)
    yaws = np.array([0.0, math.pi])
    scores = np.array([0.9, 0.85])

    picks = _pick_peaks(xs, ys, yaws, scores, 2, 0.15, math.radians(10))

    assert len(picks) == 2


# --- 라이다 점 좌표 (2026-08-31) ---------------------------------------------
#
# RViz 는 초기 위치를 잡은 뒤 그 자세 기준으로 라이다 점을 지도에 겹쳐 보여준다.
# 앱에도 같은 그림을 주려고 채점에 쓴 좌표를 서비스 응답에 실어 보낸다.


def test_hits_land_on_the_walls_at_the_true_pose():
    """정답 자세에서는 점이 벽 위에 놓인다. 이 그림이 '맞았다'의 뜻이다."""
    from vica_nav2.pose_score import _lookup, beam_hits

    grid = cross_map()
    field = build_likelihood_field(grid)
    truth = (12.0, 9.0, 0.4)
    beams = beams_at(grid, *truth)

    hit_x, hit_y = beam_hits(beams, *truth, sensor=LASER_OFFSET)
    distances = _lookup(field, grid, hit_x, hit_y, MAX_DIST)

    assert hit_x.size == beams.used
    assert float(np.median(distances)) < 0.05


def test_hits_move_with_the_pose():
    """자세를 옮기면 점도 그만큼 옮겨진다 - 틀린 자세는 그림으로 드러난다."""
    from vica_nav2.pose_score import beam_hits

    grid = cross_map()
    beams = beams_at(grid, 12.0, 9.0, 0.0)

    base_x, _ = beam_hits(beams, 12.0, 9.0, 0.0, sensor=LASER_OFFSET)
    moved_x, _ = beam_hits(beams, 12.3, 9.0, 0.0, sensor=LASER_OFFSET)

    assert float(np.mean(moved_x - base_x)) == pytest.approx(0.3)


def test_hits_include_the_laser_offset():
    """라이다는 로봇 중심보다 18.5 cm 앞이다. 그림도 그 사실을 따라야 한다."""
    from vica_nav2.pose_score import beam_hits

    grid = cross_map()
    beams = beams_at(grid, 12.0, 9.0, 0.0)

    with_offset, _ = beam_hits(beams, 12.0, 9.0, 0.0, sensor=LASER_OFFSET)
    without, _ = beam_hits(beams, 12.0, 9.0, 0.0, sensor=(0.0, 0.0, 0.0))

    assert float(np.mean(with_offset - without)) == pytest.approx(LASER_OFFSET[0])


def test_hits_with_no_beams_is_empty_not_crashed():
    """라이다가 안 들어와도 화면이 죽으면 안 된다."""
    from vica_nav2.pose_score import beam_hits

    empty = filter_beams([], 0.0, 0.1)
    hit_x, hit_y = beam_hits(empty, 1.0, 2.0, 0.0)

    assert hit_x.size == 0
    assert hit_y.size == 0
