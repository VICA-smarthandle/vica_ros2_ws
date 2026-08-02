"""global planner가 실제 차체로 충돌을 검사하고, 실측 최협 코너를 돌 수 있는지 감시한다.

2026-07-28~29 주행에서 로봇이 통로 한가운데에 갇혀 20초 이상 진행하지 못했다.
원인은 planner와 controller가 로봇 형태를 다르게 본 것이었다(기계어 확인):

    Node2D::isNodeValid      -> inCollision(unsigned int index, bool)
                                셀 하나만 본다. 즉 planner에게 로봇은 점이다.
    NodeHybrid::isNodeValid   -> inCollision(float x, float y, float theta, bool)
    NodeLattice::isNodeValid  -> inCollision(float x, float y, float theta, bool)
                                padding 포함 footprint 전체를 본다.

    DWB ObstacleFootprint     -> footprint 전체를 본다.

SmacPlanner2D는 내접반경 0.277 m 기준으로 통과 가능하다고 판단한 경로를 내는데,
DWB는 외접반경 0.675 m로 그 경로를 거부한다. 맵 free 영역 106.4 m2에서 planner는
74.9 %를 통과 가능으로 보고 controller는 42.0 %만 회전 가능으로 봐서 32.9 %p가
벌어졌다(analysis/rotatable_area.py). 이 테스트는 그 불일치의 재발을 막는다.

회전 반경 기준은 실측이다. 방2 -> 화장실 구간의 최협 통로 폭이 1.10 m이고,
90° L자 코너를 반경 R의 호로 도는 데 필요한 폭을 footprint 스윕으로 구하면
R이 작을수록 폭이 더 필요하다(전후 0.97 m짜리 긴 차체의 후방이 휘둘려
진입 통로 외벽을 쓸어낸다). R=0.20은 1.16 m가 필요해 실패하고 R=0.50은
0.985 m로 통과한다.
"""
import json
import math
from pathlib import Path

import pytest
import yaml


# 방2 -> 화장실 구간 최협 통로 폭 실측(2026-07-30, 줄자).
MEASURED_NARROWEST_CORRIDOR = 1.10

# nav2 기본 BT(navigate_to_pose_w_replanning_and_recovery.xml)의
# ComputePathToPose는 RateController 1.0 Hz 아래에 있다. planner가 이보다 오래
# 걸리면 controller가 그만큼 낡은 경로를 따라간다.
REPLAN_PERIOD_SEC = 1.0

# 기본 BT의 <ComputePathToPose planner_id="GridBased"/>. planner_server는
# planner_id가 비어있지 않은데 등록된 이름과 다르면 계획을 거부한다.
BT_PLANNER_ID = 'GridBased'

# footprint 전체로 충돌을 검사하는 planner만 허용한다. SmacPlanner2D와
# NavfnPlanner는 점 로봇이라 위 불일치를 되살린다.
FOOTPRINT_AWARE_PLUGINS = {
    'nav2_smac_planner/SmacPlannerHybrid',
    'nav2_smac_planner/SmacPlannerLattice',
}

# 비교 실험용 대안 블록. planner_plugins에 없으므로 Nav2가 로드하지 않는다.
# planner 중립적인 이름을 쓴다 -- 어느 쪽이 활성이든 나머지가 여기 들어간다.
# 전환은 두 키 이름(GridBased <-> GridBasedAlt)을 서로 바꾸는 것으로 한다.
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
    # nav2 padFootprint()과 같은 규칙: 각 좌표를 부호 방향으로 pad만큼 민다.
    return [
        (x + (pad if x >= 0 else -pad), y + (pad if y >= 0 else -pad))
        for x, y in points
    ]


def _fits_corner(padded_footprint, radius, width, step_deg=0.5):
    """폭 width의 L자 통로에서 반경 radius의 90° 좌회전이 벽을 넘지 않는지.

    수평 팔은 y in [0, width] & x <= width, 수직 팔은 x in [0, width] & y >= 0이다.
    로봇 중심은 (width/2 - R, width/2 + R)를 중심으로 한 반경 R의 호를 따라간다.
    """
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
    """블록이 실제로 쓰는 최소 회전 반경.

    Hybrid는 파라미터로 받고, Lattice는 격자 파일 metadata가 결정한다
    (libnav2_smac_planner_lattice.so에 minimum_turning_radius 파라미터가 없다).
    """
    if 'minimum_turning_radius' in block:
        return block['minimum_turning_radius']
    return _lattice_metadata(block)['turning_radius']


def test_exactly_one_planner_is_active_and_matches_the_bt_planner_id():
    planner = _load_params()['planner_server']['ros__parameters']
    plugins = planner['planner_plugins']

    # 두 블록을 동시에 로드하면 각각 obstacle heuristic 캐시를 만들어
    # Orin NX에서 낭비가 된다. 비교는 한 번에 하나만 켜서 한다.
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
    """planner와 DWB critic이 같은 자로 재야 한다. 이것이 진짜 계약이다.

    2026-08-01 정정. 그전까지 이 파일은 "planner가 footprint를 봐야 한다"를
    계약으로 삼았다. 그런데 그날 실기에서 Lattice(footprint) + ObstacleFootprint
    조합이 장애물 앞에서 우회하지 못했고, 사용자는 NavFn(점) + BaseObstacle(점)
    시절이 훨씬 잘 달렸다고 보고했다.

    두 시기를 다시 보면 공통점은 'footprint를 보느냐'가 아니라 **둘이 일치하느냐**다.

        NavFn(점)    + BaseObstacle(점)       -> 일치. 잘 달렸다
        Lattice(면)  + ObstacleFootprint(면)  -> 일치. 2026-08-01 실패
        2D(점)       + ObstacleFootprint(면)  -> 불일치. 2026-07-28 갇힘

    불일치가 재발을 부르는 축이므로 그것만 막는다. 어느 축으로 맞출지는 실측으로
    고르는 튜닝 사항이지 계약이 아니다. 점으로 맞출 때의 대가(긴 차체 후방이
    걸러지지 않는다)는 inflation_radius가 흡수하며 test_footprint_contract가
    그 하한을 지킨다.
    """
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
    """R이 작을수록 90° 코너에 더 넓은 통로가 필요하다.

    직관과 반대인데, 급하게 꺾으면 후방 0.615 m가 회전 중심 반대편으로
    휘둘려 진입 통로의 외벽을 넘기 때문이다(analysis/turning_radius.png).
    """
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
    """실주행에서는 핸들 뒤에 사람이 따라온다(guideline/vica_scenario.md).

    후진 차단을 DWB의 min_vel_x: 0.0에만 맡기면, planner가 후진 경로를 내고
    controller가 그것을 못 따라가는 다른 종류의 불일치가 생긴다. 그래서
    경로 생성 단계에서 후진을 없앤다.
    """
    active, alternative = _planner_blocks(_load_params())
    block = active if which == 'active' else alternative

    if block['plugin'] not in FOOTPRINT_AWARE_PLUGINS:
        pytest.skip(
            f'{which} planner {block["plugin"]}는 점 로봇이라 이 키가 없다.'
            ' 회전 반경·후진·계획 예산은 격자 기반 planner 전용 개념이다'
        )

    if block['plugin'].endswith('SmacPlannerHybrid'):
        # DUBIN은 전진 호와 직선만으로 이루어져 후진 primitive가 없다.
        # REEDS_SHEPP은 후진을 포함하므로 reverse_penalty로도 막을 수 없다.
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
    """맵 해상도 0.05 m는 이미 거칠다.

    최협 통로 반폭이 0.35 m인데 downsampling_factor 2면 셀이 0.10 m가 되어
    통로를 통과 불가로 오판할 수 있다. Lattice에는 이 파라미터가 없다
    (격자 파일의 grid_resolution이 대신 결정한다).
    """
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

    # 차동구동 로봇이므로 ackermann(nav2 기본값)이 아니라 diff여야 한다.
    assert meta['motion_model'] == 'diff', (
        f'격자 motion_model {meta["motion_model"]}는 차동구동이 아니다'
    )
    # 격자 해상도가 costmap과 다르면 primitive 끝점이 셀 경계에 안 맞는다.
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
        'tolerance',            # 목표 허용오차
        'allow_unknown',        # 미탐색 영역 통과 허용
        'cost_penalty',         # 장애물에서 떨어지려는 세기 (핵심 노브)
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
