"""nvblox_layer가 두 costmap에 올바른 프레임으로 들어가 있는지 감시한다."""
import ast
import math
from pathlib import Path

import pytest
import yaml

NVBLOX_PLUGIN = 'nvblox::nav2::NvbloxCostmapLayer'
INFLATION_PLUGIN = 'nav2_costmap_2d::InflationLayer'

MAX_WEIGHT = 5.0
ESDF_MIN_WEIGHT = 0.1
def _maneuver_budget_s():
    fp = _params()['controller_server']['ros__parameters']['FollowPath']
    costmap = _costmap('local_costmap')
    pts = ast.literal_eval(costmap['footprint'])
    xs = [p[0] for p in pts]
    length = (max(xs) - min(xs)) + 2 * costmap['footprint_padding']
    turn_s = (math.pi / 2) / fp['max_vel_theta']
    pass_s = length / fp['max_vel_x']
    return turn_s + pass_s

EXPECTED_SLICE = {
    'local_costmap': '/nvblox_node/static_map_slice',
    'global_costmap': '/nvblox_node/static_map_slice',
}
DYNAMIC_ONLY_SLICES = ('combined_map_slice', 'dynamic_map_slice')
NVBLOX_OVERRIDES = (
    Path(__file__).parents[2]
    / 'vica_nvblox_bringup' / 'config' / 'vica_nvblox_overrides.yaml'
)


def _params():
    path = Path(__file__).parents[1] / 'config' / 'nav2_params.yaml'
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _costmap(name):
    return _params()[name][name]['ros__parameters']


def test_local_nvblox_absence_is_a_recorded_decision():
    """local의 nvblox_layer는 2026-08-28 사용자 판정으로 plugins에서 빠졌다."""
    cm = _costmap('local_costmap')
    assert 'nvblox_layer' in cm, (
        'local_costmap에 nvblox_layer 블록이 아예 없다. plugins에서 빼는 것은'
        ' 되지만 블록은 남겨 두어야 한 줄로 되돌릴 수 있다'
    )
    assert cm['nvblox_layer']['plugin'] == NVBLOX_PLUGIN
    if 'nvblox_layer' in cm['plugins']:
        assert cm['nvblox_layer']['enabled'] is True


def test_global_nvblox_presence_is_recorded_as_an_open_experiment():
    """global의 nvblox_layer 유무는 2026-07-30 현재 실측 중인 A/B다."""
    plugins = _costmap('global_costmap')['plugins']
    has = 'nvblox_layer' in plugins
    assert 'nvblox_layer' in _costmap('global_costmap'), (
        'global_costmap에 nvblox_layer 블록이 아예 없다. plugins에서 빼는 것은'
        ' 되지만 블록은 남겨 두어야 한 줄로 되돌릴 수 있다'
    )
    if has:
        assert _costmap('global_costmap')['nvblox_layer']['enabled'] is True


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_nvblox_layer_frame_matches_the_costmap_global_frame(costmap):
    """프레임이 어긋나면 장애물이 엉뚱한 자리에 얹힌다."""
    cm = _costmap(costmap)
    assert cm['nvblox_layer']['nav2_costmap_global_frame'] == cm['global_frame'], (
        f'{costmap}: nvblox_layer.nav2_costmap_global_frame '
        f'{cm["nvblox_layer"]["nav2_costmap_global_frame"]}가 costmap '
        f'global_frame {cm["global_frame"]}과 다르다'
    )


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_each_costmap_subscribes_the_slice_matching_its_role(costmap):
    layer = _costmap(costmap)['nvblox_layer']
    assert layer['nvblox_map_slice_topic'] == EXPECTED_SLICE[costmap], (
        f'{costmap}: 슬라이스 토픽 {layer["nvblox_map_slice_topic"]}가'
        f' 역할에 맞지 않다. 기대값 {EXPECTED_SLICE[costmap]}'
    )


def test_global_never_takes_the_dynamic_slice():
    """global costmap이 동적 슬라이스를 먹으면 유령이 planner를 막는다."""
    topic = _costmap('global_costmap')['nvblox_layer']['nvblox_map_slice_topic']
    for dyn in DYNAMIC_ONLY_SLICES:
        assert dyn not in topic, (
            f'global_costmap이 동적 슬라이스({dyn})를 구독한다: {topic}'
        )


def test_both_costmaps_use_the_same_slice():
    """다른 슬라이스를 보면 planner와 controller의 장애물이 갈린다."""
    local = _costmap('local_costmap')['nvblox_layer']
    global_ = _costmap('global_costmap')['nvblox_layer']
    assert (local['nvblox_map_slice_topic']
            == global_['nvblox_map_slice_topic']), (
        f'슬라이스가 다르다: local {local["nvblox_map_slice_topic"]} vs '
        f'global {global_["nvblox_map_slice_topic"]}'
    )
    assert (local['convert_to_binary_costmap']
            == global_['convert_to_binary_costmap'])


def test_mapping_type_publishes_the_slices_both_costmaps_subscribe():
    """구독 토픽과 mapping_type의 짝이 맞는지 검사한다."""
    if not NVBLOX_OVERRIDES.is_file():
        pytest.skip(f'nvblox override 없음: {NVBLOX_OVERRIDES}')
    nv = yaml.safe_load(NVBLOX_OVERRIDES.read_text(encoding='utf-8'))
    mapping_type = nv['/**']['ros__parameters']['mapping_type']

    needs_dynamic = any(
        any(d in _costmap(cm)['nvblox_layer']['nvblox_map_slice_topic']
            for d in DYNAMIC_ONLY_SLICES)
        for cm in ('local_costmap', 'global_costmap')
    )
    if needs_dynamic:
        assert mapping_type in ('dynamic', 'human_with_static_tsdf',
                                'human_with_static_occupancy'), (
            f'동적 슬라이스를 구독하는데 mapping_type이 {mapping_type}다.'
            ' 그 토픽은 발행되지 않는다'
        )


def test_decay_actually_runs_on_what_the_robot_is_looking_at():
    """시야 안 유령이 영구히 남지 않아야 한다."""
    if not NVBLOX_OVERRIDES.is_file():
        pytest.skip(f'nvblox override 없음: {NVBLOX_OVERRIDES}')
    p = yaml.safe_load(NVBLOX_OVERRIDES.read_text(encoding='utf-8'))
    p = p['/**']['ros__parameters']
    for mapper in ('static_mapper', 'dynamic_mapper'):
        if mapper not in p:
            continue
        assert p[mapper].get('exclude_last_view_from_decay') is False, (
            f'{mapper}.exclude_last_view_from_decay가 false가 아니다.'
            ' 시야 안 유령이 영구히 남는다'
        )


def test_ghost_clears_within_the_maneuver_budget():
    """유령 소멸 시간이 로봇이 지나칠 시간보다 지나치게 길면 안 된다."""
    if not NVBLOX_OVERRIDES.is_file():
        pytest.skip(f'nvblox override 없음: {NVBLOX_OVERRIDES}')
    if 'nvblox_layer' not in _costmap('local_costmap')['plugins']:
        pytest.skip('local_costmap plugins 에 nvblox_layer 가 없다 - 위 주석 참고')
    p = yaml.safe_load(NVBLOX_OVERRIDES.read_text(encoding='utf-8'))
    p = p['/**']['ros__parameters']
    rate = p['decay_tsdf_rate_hz']
    factor = p.get('static_mapper', {}).get('tsdf_decay_factor', 0.95)
    n = math.log(ESDF_MIN_WEIGHT / MAX_WEIGHT) / math.log(factor)
    seconds = n / rate
    budget = _maneuver_budget_s()
    assert budget * 2 <= seconds <= budget * 6, (
        f'유령 소멸 {seconds:.1f} s가 기동 예산 {budget:.1f} s의'
        f' 2~6배 범위를 벗어난다 (rate {rate}, factor {factor})'
    )


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_inflation_layer_runs_after_nvblox(costmap):
    """nvblox는 binary(lethal/free)만 찍는다. 팽창은 inflation_layer가 한다."""
    plugins = _costmap(costmap)['plugins']
    if 'nvblox_layer' in plugins:
        assert plugins.index('inflation_layer') > plugins.index('nvblox_layer'), (
            f'{costmap} plugins 순서가 잘못됐다: {plugins}'
        )
    assert plugins[-1] == 'inflation_layer', (
        f'{costmap} plugins 마지막이 inflation_layer가 아니다: {plugins}'
    )
