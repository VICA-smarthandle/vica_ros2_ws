"""nvblox_layer가 두 costmap에 올바른 프레임으로 들어가 있는지 감시한다.

2026-07-30까지 nvblox_layer는 local_costmap에만 있었다. 그래서 planner는
카메라만 보는 장애물에 눈이 멀어 있었고, 두 가지가 겹쳐 주행이 굳었다.

  (1) LiDAR 평면 아래 장애물. laser_frame은 지면 0.382 m다. 사선 책상다리처럼
      그 아래에서 올라오는 구조물은 obstacle_layer(scan)가 못 본다.
  (2) 2D 레이트레이싱의 깜빡임. 거울면 스테인리스 청소기가 global costmap에서
      18초간 완전히 사라졌고, 안내소 goal 계획 시점(t+1.9~6.9s)에 LETHAL이
      0개였다. t+8.0s에 뒤늦게 나타났고 t+14.0s에 DWB가 유효 궤적을 잃었다.

프레임 계약이 핵심이다. 플러그인은 nav2_costmap_global_frame과 슬라이스 frame이
같으면 identity로 처리하고 다르면 TF로 변환한다
(nvblox_costmap_layer.cpp 238~250행). 이 값이 각 costmap의 global_frame과
어긋나면 장애물이 엉뚱한 자리에 얹힌다 — 조용히 틀리는 종류의 결함이다.
"""
from pathlib import Path

import pytest
import yaml

NVBLOX_PLUGIN = 'nvblox::nav2::NvbloxCostmapLayer'
INFLATION_PLUGIN = 'nav2_costmap_2d::InflationLayer'

# 두 costmap은 '서로 다른' 슬라이스를 써야 한다. 역할이 다르기 때문이다.
#   local  <- combined : 정적 + 동적. DWB가 사람을 실시간 회피한다.
#   global <- static   : 정적만. planner가 유령에 막히지 않는다.
# 하나로 통일하면 둘 중 하나를 잃는다. static 하나로 global까지 먹였을 때
# (2026-07-30) 사람이 static TSDF에 쌓여 global costmap에 중앙값 9 s / p95 46 s
# 남았고, 그 유령이 "Starting point in lethal space" 22회를 만들었다.
EXPECTED_SLICE = {
    'local_costmap': '/nvblox_node/combined_map_slice',
    'global_costmap': '/nvblox_node/static_map_slice',
}
# combined/dynamic 슬라이스는 dynamic 계열 mapping_type에서만 발행된다
# (nvblox_node.cpp advertiseTopics의 isUsingHumanOrDynamicMapper 분기).
# 짝이 어긋나면 nvblox_layer는 아무것도 받지 못한 채 조용히 무해해진다.
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


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_nvblox_layer_is_enabled_on_both_costmaps(costmap):
    """한쪽에만 있으면 planner와 controller가 다른 장애물을 본다.

    그 상태에서는 planner가 통과 가능으로 만든 경로를 DWB가 전부 거부해
    로봇이 멈춘다. footprint 불일치를 고친 것과 같은 종류의 결함이다.
    """
    cm = _costmap(costmap)
    assert 'nvblox_layer' in cm['plugins'], (
        f'{costmap} plugins에 nvblox_layer가 없다: {cm["plugins"]}'
    )
    layer = cm['nvblox_layer']
    assert layer['plugin'] == NVBLOX_PLUGIN
    assert layer['enabled'] is True


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_nvblox_layer_frame_matches_the_costmap_global_frame(costmap):
    """프레임이 어긋나면 장애물이 엉뚱한 자리에 얹힌다.

    local은 odom, global은 map이다. 플러그인은 이 값을 슬라이스 frame과 비교해
    같으면 identity, 다르면 TF 변환을 쓴다. costmap 자신의 global_frame과
    맞추지 않으면 변환 방향이 틀어진다.
    """
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
    """global costmap이 동적 슬라이스를 먹으면 유령이 planner를 막는다.

    planner는 253 이상을 하드 거부하므로(GridCollisionChecker::inCollision,
    cmp w1 #0xfc), 사람이 지나간 자리가 남아 있으면 그 자리에서 경로를 못 낸다.
    동적 회피는 DWB(local)의 일이다.
    """
    topic = _costmap('global_costmap')['nvblox_layer']['nvblox_map_slice_topic']
    for dyn in DYNAMIC_ONLY_SLICES:
        assert dyn not in topic, (
            f'global_costmap이 동적 슬라이스({dyn})를 구독한다: {topic}'
        )


def test_local_sees_dynamic_obstacles():
    """local이 정적 슬라이스만 보면 DWB가 사람을 못 피한다."""
    topic = _costmap('local_costmap')['nvblox_layer']['nvblox_map_slice_topic']
    assert any(d in topic for d in DYNAMIC_ONLY_SLICES), (
        f'local_costmap이 동적 슬라이스를 구독하지 않는다: {topic}'
    )


def test_mapping_type_publishes_the_slices_both_costmaps_subscribe():
    """구독 토픽과 mapping_type의 짝이 맞는지 검사한다.

    combined/dynamic 슬라이스는 dynamic 계열 mapping_type에서만 발행된다.
    짝이 어긋나면 nvblox_layer가 아무것도 받지 못한 채 조용히 무해해지고,
    planner는 다시 카메라 장애물에 눈이 먼다 — 경고도 나오지 않는다.
    """
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


def test_static_and_dynamic_decay_pull_in_opposite_directions():
    """정적은 붙잡고 동적은 놓아야 한다. 하나의 decay로는 둘을 만족시킬 수 없다.

    TSDF 무게는 소거마다 tsdf_decay_factor를 곱하고 1e-3 밑에서 소멸한다.
      n = ln(1e-3) / ln(factor) 스텝,  소요 = n / decay_tsdf_rate_hz
    2026-07-30 실측: factor 0.95 / 2.5 Hz -> 53.9 s 예측, p95 46 s 관측.
    """
    if not NVBLOX_OVERRIDES.is_file():
        pytest.skip(f'nvblox override 없음: {NVBLOX_OVERRIDES}')
    nv = yaml.safe_load(NVBLOX_OVERRIDES.read_text(encoding='utf-8'))
    p = nv['/**']['ros__parameters']
    if p['mapping_type'] not in ('dynamic', 'human_with_static_tsdf',
                                 'human_with_static_occupancy'):
        pytest.skip('단일 mapper 모드에서는 분리 계약이 성립하지 않는다')

    # 동적 소거 주기가 정적보다 빨라야 한다.
    assert p['decay_dynamic_occupancy_rate_hz'] > p['decay_tsdf_rate_hz'], (
        f'동적 소거 {p["decay_dynamic_occupancy_rate_hz"]} Hz가 정적'
        f' {p["decay_tsdf_rate_hz"]} Hz보다 빠르지 않다'
    )
    # 소거된 복셀은 unknown이 아니라 free여야 한다. DWB의 pointCost는
    # NO_INFORMATION(0xff)에서도 예외를 던져, 미지가 남으면 궤적이 전멸한다.
    assert p['static_mapper']['tsdf_set_free_distance_on_decayed'] is True
    assert p['dynamic_mapper']['occupancy_decay_to_free'] is True
    # 확률 감쇠의 유효 범위 (occupancy_decay_integrator_params.h)
    occ = p['dynamic_mapper']['occupied_region_decay_probability']
    free = p['dynamic_mapper']['free_region_decay_probability']
    assert 0.0 <= occ <= 0.5, f'occupied_region_decay_probability {occ} 범위 밖'
    assert 0.5 <= free <= 1.0, f'free_region_decay_probability {free} 범위 밖'


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_inflation_layer_runs_after_nvblox(costmap):
    """nvblox는 binary(lethal/free)만 찍는다. 팽창은 inflation_layer가 한다.

    plugins 순서가 곧 적용 순서다. inflation이 nvblox보다 앞이면 nvblox가 찍은
    장애물에는 비용 경사가 생기지 않아, 경로가 그 장애물에 그대로 붙는다.
    """
    plugins = _costmap(costmap)['plugins']
    assert plugins.index('inflation_layer') > plugins.index('nvblox_layer'), (
        f'{costmap} plugins 순서가 잘못됐다: {plugins}'
    )
    assert plugins[-1] == 'inflation_layer'
