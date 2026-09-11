"""global planner가 실제 차체로 충돌을 검사하고, 실측 최협 코너를 돌 수 있는지 감시한다."""
import json
import math
from pathlib import Path

import pytest
import yaml


MEASURED_NARROWEST_CORRIDOR = 1.10

REPLAN_PERIOD_SEC = 1.0

BT_PLANNER_ID = 'GridBased'

FOOTPRINT_AWARE_PLUGINS = {
    'nav2_smac_planner/SmacPlannerHybrid',
    'nav2_smac_planner/SmacPlannerLattice',
}

ALTERNATIVE_KEY = 'GridBasedAlt'


def _load_params():
    config_path = Path(__file__).parents[1] / 'config' / 'nav2_params.yaml'
    return yaml.safe_load(config_path.read_text(encoding='utf-8'))


def _planner_blocks(params):
    """(활성 블록, 대안 블록)을 돌려준다. 둘 다 같은 기준으로 검사한다."""
    planner = params['planner_server']['ros__parameters']
    active_name = planner['planner_plugins'][0]
    return planner[active_name], planner[ALTERNATIVE_KEY]


def _padded_footprint(params, costmap='local_costmap'):
    cm = params[costmap][costmap]['ros__parameters']
    points = yaml.safe_load(cm['footprint'])
    pad = cm['footprint_padding']
    return [
        (x + (pad if x >= 0 else -pad), y + (pad if y >= 0 else -pad))
        for x, y in points
    ]


def _fits_corner(padded_footprint, radius, width, step_deg=0.5):
    """폭 width의 L자 통로에서 반경 radius의 90° 좌회전이 벽을 넘지 않는지."""
    cx, cy = width / 2 - radius, width / 2 + radius
    steps = int(round(90.0 / step_deg))
    for i in range(steps + 1):
        th = math.radians(i * step_deg)
        cos_t, sin_t = math.cos(th), math.sin(th)
        px, py = cx + radius * sin_t, cy - radius * cos_t
        for fx, fy in padded_footprint:
            x = px + fx * cos_t - fy * sin_t
            y = py + fx * sin_t + fy * cos_t
            in_horizontal = (0.0 <= y <= width) and (x <= width)
            in_vertical = (0.0 <= x <= width) and (y >= 0.0)
            if not (in_horizontal or in_vertical):
                return False
    return True


def _min_corner_width(padded_footprint, radius):
    width = 0.60
    while width <= 2.50:
        if _fits_corner(padded_footprint, radius, width):
            return width
        width += 0.01
    return None


def _lattice_metadata(block):
    path = Path(block['lattice_filepath'])
    return json.loads(path.read_text(encoding='utf-8'))['lattice_metadata']


def _turning_radius(block):
    """블록이 실제로 쓰는 최소 회전 반경."""
    if 'minimum_turning_radius' in block:
        return block['minimum_turning_radius']
    return _lattice_metadata(block)['turning_radius']


def test_exactly_one_planner_is_active_and_matches_the_bt_planner_id():
    planner = _load_params()['planner_server']['ros__parameters']
    plugins = planner['planner_plugins']

    assert plugins == [BT_PLANNER_ID], (
        f'planner_plugins {plugins}는 기본 BT의 planner_id'
        f' "{BT_PLANNER_ID}" 하나여야 한다'
    )
    assert ALTERNATIVE_KEY in planner, (
        f'대안 블록 {ALTERNATIVE_KEY}가 없다. 두 planner를 한 줄로'
        ' 갈아타며 비교하려면 두 블록이 모두 있어야 한다'
    )
    assert ALTERNATIVE_KEY not in plugins, (
        f'{ALTERNATIVE_KEY}가 planner_plugins에 있으면 Nav2가 실제로 로드한다'
    )


def test_planner_and_controller_use_the_same_collision_model():
    """planner와 DWB critic이 같은 자로 재야 한다. 이것이 진짜 계약이다."""
    params = _load_params()
    active, _alternative = _planner_blocks(params)
    critics = params['controller_server']['ros__parameters']['FollowPath']['critics']

    planner_sees_footprint = active['plugin'] in FOOTPRINT_AWARE_PLUGINS
    controller_sees_footprint = 'ObstacleFootprint' in critics
    controller_sees_point = 'BaseObstacle' in critics

    assert controller_sees_footprint != controller_sees_point, (
        f'DWB critics {critics}에 장애물 critic이 없거나 둘 다 있다.'
        ' BaseObstacle(점)이나 ObstacleFootprint(면) 중 하나만 둔다'
    )
    assert planner_sees_footprint == controller_sees_footprint, (
        f'planner {active["plugin"]}와 DWB critic이 로봇 형태를 다르게 본다.'
        f' planner footprint={planner_sees_footprint},'
        f' controller footprint={controller_sees_footprint}.'
        ' 한쪽이 통과 가능으로 만든 경로를 다른 쪽이 거부해'
        ' 2026-07-28 통로 갇힘이 재발한다. 둘을 함께 바꾼다'
    )


@pytest.mark.parametrize('which', ['active', 'alternative'])
def test_turning_radius_clears_the_narrowest_measured_corner(which):
    """R이 작을수록 90° 코너에 더 넓은 통로가 필요하다."""
    params = _load_params()
    active, alternative = _planner_blocks(params)
    block = active if which == 'active' else alternative

    if block['plugin'] not in FOOTPRINT_AWARE_PLUGINS:
        pytest.skip(
            f'{which} planner {block["plugin"]}는 점 로봇이라 이 키가 없다.'
            ' 회전 반경·후진·계획 예산은 격자 기반 planner 전용 개념이다'
        )

    radius = _turning_radius(block)
    needed = _min_corner_width(_padded_footprint(params), radius)

    assert needed is not None, (
        f'{which} planner의 회전 반경 {radius} m로는 2.50 m 통로에서도'
        ' 90° 코너를 돌 수 없다'
    )
    assert needed <= MEASURED_NARROWEST_CORRIDOR, (
        f'{which} planner의 회전 반경 {radius} m는 90° 코너에'
        f' {needed:.3f} m가 필요한데 실측 최협 통로는'
        f' {MEASURED_NARROWEST_CORRIDOR} m다'
    )


@pytest.mark.parametrize('which', ['active', 'alternative'])
def test_planner_never_plans_a_reverse_segment(which):
    """실주행에서는 핸들 뒤에 사람이 따라온다(guideline/vica_scenario.md)."""
    active, alternative = _planner_blocks(_load_params())
    block = active if which == 'active' else alternative

    if block['plugin'] not in FOOTPRINT_AWARE_PLUGINS:
        pytest.skip(
            f'{which} planner {block["plugin"]}는 점 로봇이라 이 키가 없다.'
            ' 회전 반경·후진·계획 예산은 격자 기반 planner 전용 개념이다'
        )

    if block['plugin'].endswith('SmacPlannerHybrid'):
        assert block['motion_model_for_search'] == 'DUBIN', (
            f'{which} planner의 motion_model_for_search'
            f' {block["motion_model_for_search"]}는 후진 경로를 만든다'
        )
    else:
        assert block['allow_reverse_expansion'] is False, (
            f'{which} planner가 후진 primitive로 확장하도록 열려 있다'
        )


@pytest.mark.parametrize('which', ['active', 'alternative'])
def test_planning_budget_fits_the_replan_period(which):
    active, alternative = _planner_blocks(_load_params())
    block = active if which == 'active' else alternative

    if block['plugin'] not in FOOTPRINT_AWARE_PLUGINS:
        pytest.skip(
            f'{which} planner {block["plugin"]}는 점 로봇이라 이 키가 없다.'
            ' 회전 반경·후진·계획 예산은 격자 기반 planner 전용 개념이다'
        )

    assert block['max_planning_time'] <= REPLAN_PERIOD_SEC, (
        f'{which} planner의 max_planning_time {block["max_planning_time"]}가'
        f' 재계획 주기 {REPLAN_PERIOD_SEC} s를 넘는다. 넘기려면 BT의'
        ' RateController도 같이 낮춰야 한다'
    )


def test_active_hybrid_does_not_downsample_the_narrow_corridor_away():
    """맵 해상도 0.05 m는 이미 거칠다."""
    active, _alternative = _planner_blocks(_load_params())
    if not active['plugin'].endswith('SmacPlannerHybrid'):
        pytest.skip('활성 planner가 Hybrid가 아니다')

    assert active['downsample_costmap'] is False
    assert active['downsampling_factor'] == 1


def test_lattice_file_matches_the_map_and_keeps_in_place_rotation():
    if not _has_lattice(_load_params()):
        pytest.skip('활성·대안 어느 블록도 Lattice가 아니다')
    """Lattice를 쓰는 이유가 제자리 회전이므로 격자에 그것이 있는지 확인한다.

    Hybrid(DUBIN)는 반경 R의 호로만 방향을 바꿔서, 목표가 뒤쪽에 있으면
    180° 되돌기에 통로 폭 1.85 m가 필요하다(R=0.50). 맵 통로 폭 중앙값은
    1.40 m, 최협은 1.10 m라 어느 통로에서도 불가하다. diff 격자의 제자리
    회전 primitive가 그 경로를 대신한다.
    """
    params = _load_params()
    active, alternative = _planner_blocks(params)
    block = next(
        (b for b in (active, alternative)
         if b['plugin'].endswith('SmacPlannerLattice')),
        None,
    )
    assert block is not None, 'Lattice 블록이 없다'

    path = Path(block['lattice_filepath'])
    assert path.is_file(), f'격자 파일이 없다: {path}'

    lattice = json.loads(path.read_text(encoding='utf-8'))
    meta = lattice['lattice_metadata']

    assert meta['motion_model'] == 'diff', (
        f'격자 motion_model {meta["motion_model"]}는 차동구동이 아니다'
    )
    resolution = params['global_costmap']['global_costmap']['ros__parameters']['resolution']
    assert meta['grid_resolution'] == resolution, (
        f'격자 해상도 {meta["grid_resolution"]}가 costmap {resolution}과 다르다'
    )
    in_place = [p for p in lattice['primitives'] if p['trajectory_length'] == 0.0]
    assert len(in_place) > 0, (
        '격자에 제자리 회전 primitive가 없다. 그러면 Hybrid와 같은 이유로'
        ' 좁은 통로에서 방향을 되돌릴 수 없다'
    )


def _has_lattice(params):
    planner = params['planner_server']['ros__parameters']
    names = [planner['planner_plugins'][0], ALTERNATIVE_KEY]
    return any(
        planner.get(n, {}).get('plugin', '').endswith('SmacPlannerLattice')
        for n in names
    )


def test_both_planner_blocks_change_only_the_planner():
    if not _has_lattice(_load_params()):
        pytest.skip(
            'Smac 계열끼리 비교할 때만 성립하는 계약이다.'
            ' NavFn은 cost_penalty 같은 공유 키를 갖지 않는다'
        )
    """A/B 비교에서 planner 외의 변수가 섞이면 결과를 해석할 수 없다.

    2026-07-29에 사용자가 명시했다: 한 번에 한 파라미터만 바꾼다.
    두 블록이 공유할 수 있는 값은 모두 같아야 한다.
    """
    active, alternative = _planner_blocks(_load_params())

    shared = [
        'tolerance',
        'allow_unknown',
        'cost_penalty',
        'max_planning_time',
        'max_iterations',
        'max_on_approach_iterations',
        'cache_obstacle_heuristic',
        'smooth_path',
    ]
    for key in shared:
        assert key in active and key in alternative, (
            f'두 블록 모두 {key}를 명시해야 비교가 성립한다'
        )
        assert active[key] == alternative[key], (
            f'{key}가 다르다: 활성 {active[key]} vs 대안 {alternative[key]}.'
            ' planner 교체 효과를 이 차이가 오염시킨다'
        )
    assert active['smoother'] == alternative['smoother']


def test_planner_avoids_obstacles_at_least_as_hard_as_the_2d_baseline():
    if not _has_lattice(_load_params()):
        pytest.skip('cost_penalty는 Smac 계열 전용 키다')
    """cost_penalty는 '장애물에서 얼마나 떨어져 갈 것인가'의 직접 노브다.

    SmacPlanner2D에서는 같은 역할을 cost_travel_multiplier가 했고, 벽에
    붙는 경향을 줄이려고 기본 2.0에서 3.0으로 올려 뒀다. planner를 바꿀 때
    이 세기를 기본값으로 되돌리면 장애물 이격이라는 원래 목적을 잃는다.
    """
    active, _alternative = _planner_blocks(_load_params())
    assert active['cost_penalty'] >= 3.0, (
        f'cost_penalty {active["cost_penalty"]}는 2D 기준선'
        ' cost_travel_multiplier 3.0보다 약하다'
    )
