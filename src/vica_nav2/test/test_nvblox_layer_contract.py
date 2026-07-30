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
# vica_nvblox_bringup의 mapping_type: static_tsdf와 짝이 되는 토픽.
# dynamic 계열로 바꾸면 combined_map_slice가 되고, 짝이 어긋나면 이 레이어는
# 아무것도 받지 못한 채 조용히 무해해진다(그리고 planner는 다시 눈이 먼다).
EXPECTED_SLICE_TOPIC = '/nvblox_node/static_map_slice'


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
def test_nvblox_layer_subscribes_the_slice_that_static_tsdf_publishes(costmap):
    layer = _costmap(costmap)['nvblox_layer']
    assert layer['nvblox_map_slice_topic'] == EXPECTED_SLICE_TOPIC, (
        f'{costmap}: 슬라이스 토픽 {layer["nvblox_map_slice_topic"]}가'
        f' mapping_type static_tsdf의 발행 토픽과 다르다'
    )


def test_both_costmaps_use_the_same_slice_topic():
    """서로 다른 슬라이스를 보면 planner와 controller의 장애물이 또 갈린다."""
    local = _costmap('local_costmap')['nvblox_layer']
    global_ = _costmap('global_costmap')['nvblox_layer']
    assert (local['nvblox_map_slice_topic']
            == global_['nvblox_map_slice_topic'])
    assert (local['convert_to_binary_costmap']
            == global_['convert_to_binary_costmap'])


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
