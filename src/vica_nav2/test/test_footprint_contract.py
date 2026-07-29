"""costmap footprint가 실제 차체(vica_description/meshes/base_link.stl)를 덮는지 감시한다.

2026-07-27 화장실 주행에서 커브 중 전방 좌측 범퍼가 실제로 충돌했다. 원인은
footprint 전방이 +0.15 m로 선언되어 있었으나 실제 차체는 +0.305 m까지 뻗어 있어,
Nav2가 15.5 cm를 free 공간으로 오인한 것이었다. 좌우도 각 4 cm 짧았다.

이 테스트는 같은 종류의 회귀(차체보다 작은 footprint)를 막는다.
"""
import struct
from pathlib import Path

import pytest
import yaml


# base_link.stl은 mm 단위이고 URDF에서 scale 0.001로 쓰인다.
STL_SCALE = 0.001
# URDF의 collision origin이 xyz="0 0 -0.044"로 z만 오프셋이므로
# STL의 x/y는 base_link 좌표와 직접 대응한다.
#
# 후방만은 STL을 신뢰하지 않는다. 2026-07-29 줄자 실측이 중심 -> 핸들 끝 56.5 cm인데
# STL은 -0.505로 6 cm 짧다. 같은 날 핸들 기둥 위치도 CAD(-0.265~-0.305) 대비
# 실측(-0.205)이 7~10 cm 어긋나, CAD 후방부가 실제 차체를 반영하지 못한다.
# 따라서 후방 기준은 STL이 아니라 이 실측값이다.
MEASURED_REAR = -0.565
LASER_X = 0.185  # VICA.xacro laser_x
CAMERA_X = 0.28683  # VICA.xacro camera_x


def _load_params():
    config_path = Path(__file__).parents[1] / 'config' / 'nav2_params.yaml'
    return yaml.safe_load(config_path.read_text(encoding='utf-8'))


def _footprint(costmap_name):
    params = _load_params()
    raw = params[costmap_name][costmap_name]['ros__parameters']['footprint']
    return yaml.safe_load(raw)


def _stl_path():
    return (
        Path(__file__).parents[2]
        / 'vica_description' / 'meshes' / 'base_link.stl'
    )


def _stl_xy_bounds():
    """base_link.stl을 XY로 투영한 (min_x, max_x, max_abs_y)를 m 단위로 돌려준다."""
    data = _stl_path().read_bytes()
    triangle_count = struct.unpack('<I', data[80:84])[0]
    min_x, max_x, max_abs_y = 1e9, -1e9, 0.0
    for i in range(triangle_count):
        base = 84 + i * 50
        for vertex in range(3):
            off = base + 12 + vertex * 12
            x, y, _z = struct.unpack('<fff', data[off:off + 12])
            min_x = min(min_x, x * STL_SCALE)
            max_x = max(max_x, x * STL_SCALE)
            max_abs_y = max(max_abs_y, abs(y * STL_SCALE))
    return min_x, max_x, max_abs_y


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_footprint_covers_the_real_chassis(costmap):
    if not _stl_path().exists():
        pytest.skip('vica_description/meshes/base_link.stl 없음')

    _min_x, max_x, max_abs_y = _stl_xy_bounds()
    points = _footprint(costmap)

    front = max(p[0] for p in points)
    rear = min(p[0] for p in points)
    half_width = max(abs(p[1]) for p in points)

    # 전방·좌우는 실제 차체를 반드시 덮어야 한다. 여기서 짧으면 범퍼가
    # costmap상 free 공간을 쓸고 지나가 충돌한다.
    assert front >= max_x, (
        f'{costmap} footprint 전방 {front}이 실제 차체 {max_x:.3f}보다 짧다'
    )
    assert half_width >= max_abs_y, (
        f'{costmap} footprint 반폭 {half_width}이 실제 차체 {max_abs_y:.3f}보다 좁다'
    )
    # 후방은 줄자 실측(-0.565)을 덮어야 한다. STL(-0.505)로 검사하면 6 cm 짧은
    # footprint를 통과시켜, 2026-07-27 전방 충돌과 같은 종류의 결함을 놓친다.
    assert rear <= MEASURED_REAR, (
        f'{costmap} footprint 후방 {rear}이 실측 차체 {MEASURED_REAR}보다 짧다'
    )


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_footprint_contains_forward_mounted_sensors(costmap):
    # LiDAR·카메라가 footprint 밖에 있으면 그 자체로 footprint가 틀렸다는 신호다.
    # 구 footprint(전방 +0.15)는 둘 다 바깥이었고, 실제로 충돌로 이어졌다.
    front = max(p[0] for p in _footprint(costmap))
    assert front >= LASER_X
    assert front >= CAMERA_X


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_padding_keeps_a_hard_clearance_margin(costmap):
    """padding은 유일한 '하드 여유'다.

    planner는 벽에서 inscribed radius(반폭 + padding)만큼 떨어진 곳까지 통과
    가능으로 본다. padding이 0이면 차체 가장자리가 벽에 정확히 닿는 경로가
    합법이 되어, AMCL 오차가 그대로 충돌이 된다(2026-07-27 실제 충돌).
    """
    params = _load_params()
    padding = params[costmap][costmap]['ros__parameters']['footprint_padding']
    assert padding >= 0.05, (
        f'{costmap} footprint_padding {padding}은 하드 여유로 부족하다'
    )


# 맵 실측 통로 반폭(scratchpad/corridor_width.py): 중앙값 0.70 m, 10%tile 0.35 m.
NARROWEST_CORRIDOR_HALF_WIDTH = 0.35


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_inflation_radius_keeps_the_path_off_the_wall(costmap):
    """inflation_radius는 '벽에서 이만큼은 떨어져라'는 목표 이격거리다.

    너무 작으면(0.35) 경사 구간이 몇 cm뿐이라 통로가 평탄해지고 NavFn이 벽에
    붙은 최단경로를 그린다 — 2026-07-27 전방 범퍼 충돌의 배경이다.

    너무 크면(0.65) 통로 '전체'가 비용 지대가 된다. NavFn은 경로 비용 합을
    최소화하므로 비용 0인 먼 길로 우회하고, 좁은 길에서는 최소비용 경로가
    거의 동점이 되어 경로가 계속 뒤바뀌며 주춤거린다 — 같은 날 실주행 확인.

    그래서 하한(실효 여유 확보)과 상한(통로 중앙부는 평탄 유지)을 함께 건다.
    """
    params = _load_params()
    cm = params[costmap][costmap]['ros__parameters']
    padding = cm['footprint_padding']
    half_width = max(abs(p[1]) for p in _footprint(costmap))
    inscribed = half_width + padding
    inflation_radius = cm['inflation_layer']['inflation_radius']

    # 하한: 차체 가장자리 기준 실여유가 최소 15 cm는 되어야 한다.
    #
    # 2026-07-28에 이 하한을 padding의 2배(0.10)로 낮추고 inflation_radius를
    # 0.38로 시험했다가 되돌렸다. 최협 통로(반폭 0.35)에 비용 0인 중앙선을
    # 만들려는 의도였고 최장 서행은 8.37 -> 2.57 s로 줄었지만, 같은 4구간
    # 경로에서 0.45가 완주한 반면 0.38은 마지막 구간에서 61초간 갇혔다.
    # inflation_radius는 '얼마나 떨어져 갈 것인가'라서, 줄이면 좁은 곳에
    # 진입하는 것 자체를 막지 못한다. 그래서 0.15를 유지한다.
    assert inflation_radius - inscribed >= 0.15, (
        f'{costmap} 실여유 {inflation_radius - inscribed:.3f} m가 너무 좁다'
    )
    # 상한: 최협 통로의 반폭을 넘으면 그 통로 전체가 비용 지대가 되어
    # 우회와 경로 진동을 유발한다.
    assert inflation_radius <= NARROWEST_CORRIDOR_HALF_WIDTH + 0.15, (
        f'{costmap} inflation_radius {inflation_radius}가 최협 통로 반폭'
        f' {NARROWEST_CORRIDOR_HALF_WIDTH}에 비해 과도하다'
    )


def test_local_and_global_costmap_use_the_same_footprint():
    # 두 costmap이 다르면 planner가 통과 가능하다고 만든 경로를
    # controller가 거부해 주행이 멈춘다.
    assert _footprint('local_costmap') == _footprint('global_costmap')

    params = _load_params()
    local = params['local_costmap']['local_costmap']['ros__parameters']
    global_ = params['global_costmap']['global_costmap']['ros__parameters']
    assert local['footprint_padding'] == global_['footprint_padding']
