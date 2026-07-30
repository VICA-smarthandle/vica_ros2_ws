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
import math
from pathlib import Path

import pytest
import yaml

NVBLOX_PLUGIN = 'nvblox::nav2::NvbloxCostmapLayer'
INFLATION_PLUGIN = 'nav2_costmap_2d::InflationLayer'

# nvblox_base.yaml의 값. 유령 소멸 시간 계산에 쓴다.
MAX_WEIGHT = 5.0          # projective_integrator_max_weight (:78)
ESDF_MIN_WEIGHT = 0.1     # esdf_integrator_min_weight (:95). 실질 소멸 기준이다
# 시야 밖으로 나간 낮은 장애물을 로봇이 지나칠 때까지 필요한 기억:
#   90도 회전 1.571 rad / max_vel_theta 0.4 = 3.93 s
#   차체 통과 0.87 m / max_vel_x 0.26       = 3.35 s
MANEUVER_BUDGET_S = 7.3

# 두 costmap은 '같은' 슬라이스를 써야 한다. 다르면 planner와 controller가 다른
# 장애물을 보고, planner가 통과 가능으로 만든 경로를 DWB가 거부해 로봇이 굳는다.
#
# 2026-07-30에 분리(local=combined / global=static)를 시험했다가 되돌렸다. 사람이
# static TSDF에 쌓이는 것을 막으려는 의도였는데 실측이 개선을 보이지 않았다:
#   일시 LETHAL 셀 지속 p95   47.0 s (통일) -> 43.0 s (분리)
#   일시 셀 수                26,270 -> 26,059
# 유령의 원인은 mapping_type이 아니라 exclude_last_view_from_decay였다.
# 그리고 분리 구성으로 완주한 회차가 없다(1/3). 3/3 완주한 조합은 통일 쪽이다.
EXPECTED_SLICE = {
    'local_costmap': '/nvblox_node/static_map_slice',
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


def test_both_costmaps_use_the_same_slice():
    """다른 슬라이스를 보면 planner와 controller의 장애물이 갈린다.

    footprint 불일치(2026-07-28)와 같은 종류의 결함이다 -- planner가 통과 가능으로
    만든 경로를 DWB가 전부 거부해 로봇이 굳는다.
    """
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


def test_decay_actually_runs_on_what_the_robot_is_looking_at():
    """시야 안 유령이 영구히 남지 않아야 한다.

    exclude_last_view_from_decay가 true면 '마지막 깊이 영상에 보이는 블록'이
    decay에서 제외된다(mapper.cpp 368~375). 그런데 '보는 범위'(카메라 수직 FOV 58°)와
    '갱신되는 범위'(관측 표면 앞뒤 0.20 m = truncation 4 vox x 0.05)가 달라서,
    시야 안이면서 truncation 밖인 복셀은 integration으로도 decay로도 지워지지 않는다.
    막혀 멈춘 로봇이 계속 그것을 쳐다보므로 상태가 원인을 유지한다
    (devlog/2026-07-30-nvblox-ghost-obstacle.md 4.3).

    false여도 시야 안 실제 장애물은 integration이 매 관측마다 weight를 되올려
    보호한다(카메라 30 Hz >> decay 2.5 Hz). 시야 밖 거동은 바뀌지 않는다.
    """
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
    """유령 소멸 시간이 로봇이 지나칠 시간보다 지나치게 길면 안 된다.

    실질 소멸은 tsdf_decayed_weight_threshold(0.001)가 아니라
    esdf_integrator_min_weight(0.1, nvblox_base.yaml:95)에서 일어난다. ESDF가 weight
    0.1 미만 복셀을 아예 보지 않는다(esdf_integrator.cu:115).
      projective_integrator_max_weight 5.0 x factor^n <= 0.1
      n = ln(0.02)/ln(factor),  소요 = n / decay_tsdf_rate_hz
    필요 기억은 90도 회전 1.571/0.4 = 3.93 s + 차체 통과 0.87/0.26 = 3.35 s
    = 약 7.3 s다. 상한을 그 6배로 둔다 -- 2026-07-28에 decay 5.0 Hz(15.2 s)가
    책상다리 충돌로 기록됐으므로 하한도 함께 건다.
    """
    if not NVBLOX_OVERRIDES.is_file():
        pytest.skip(f'nvblox override 없음: {NVBLOX_OVERRIDES}')
    p = yaml.safe_load(NVBLOX_OVERRIDES.read_text(encoding='utf-8'))
    p = p['/**']['ros__parameters']
    rate = p['decay_tsdf_rate_hz']
    factor = p.get('static_mapper', {}).get('tsdf_decay_factor', 0.95)
    n = math.log(ESDF_MIN_WEIGHT / MAX_WEIGHT) / math.log(factor)
    seconds = n / rate
    assert MANEUVER_BUDGET_S * 2 <= seconds <= MANEUVER_BUDGET_S * 6, (
        f'유령 소멸 {seconds:.1f} s가 기동 예산 {MANEUVER_BUDGET_S} s의'
        f' 2~6배 범위를 벗어난다 (rate {rate}, factor {factor})'
    )


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
