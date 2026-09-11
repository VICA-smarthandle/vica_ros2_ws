"""costmap footprint가 실제 차체(vica_description/meshes/base_link.stl)를 덮는지 감시한다."""
import struct
from pathlib import Path

import pytest
import yaml


STL_SCALE = 0.001
MEASURED_REAR = -0.415
MEASURED_HALF_WIDTH = 0.225
STL_TOLERANCE = 0.01
LASER_X = 0.185
CAMERA_X = 0.28683


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

    assert front >= max_x, (
        f'{costmap} footprint 전방 {front}이 실제 차체 {max_x:.3f}보다 짧다'
    )
    assert half_width >= MEASURED_HALF_WIDTH, (
        f'{costmap} footprint 반폭 {half_width}이 실측 {MEASURED_HALF_WIDTH}보다 좁다'
    )
    assert rear <= MEASURED_REAR, (
        f'{costmap} footprint 후방 {rear}이 실측 차체 {MEASURED_REAR}보다 짧다'
    )
    assert max_abs_y - half_width <= STL_TOLERANCE, (
        f'{costmap} 반폭 실측 {half_width}이 STL {max_abs_y:.4f}보다 '
        f'{max_abs_y - half_width:.4f} m 작다. 허용 {STL_TOLERANCE} m를 넘었으니 '
        'CAD와 실물 중 어느 쪽이 낡았는지 사람이 확인해야 한다'
    )


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_footprint_contains_forward_mounted_sensors(costmap):
    front = max(p[0] for p in _footprint(costmap))
    assert front >= LASER_X
    assert front >= CAMERA_X


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_padding_keeps_a_hard_clearance_margin(costmap):
    """padding은 유일한 '하드 여유'다."""
    params = _load_params()
    padding = params[costmap][costmap]['ros__parameters']['footprint_padding']
    assert padding >= 0.03, (
        f'{costmap} footprint_padding {padding}은 하드 여유로 부족하다'
    )


NARROWEST_CORRIDOR_HALF_WIDTH = 0.35
CORRIDOR_HALF_WIDTH_MEDIAN = 0.650

DRIVEN_CORRIDOR_CLEARANCE = 0.412

PATH_TRACKING_ERROR_P95 = 0.120

COSTMAP_RESOLUTION = 0.05


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_inflation_radius_keeps_the_path_off_the_wall(costmap):
    """inflation_radius는 '벽에서 이만큼은 떨어져라'는 목표 이격거리다."""
    params = _load_params()
    cm = params[costmap][costmap]['ros__parameters']
    padding = cm['footprint_padding']
    half_width = max(abs(p[1]) for p in _footprint(costmap))
    inscribed = half_width + padding
    inflation_radius = cm['inflation_layer']['inflation_radius']

    assert inflation_radius - inscribed >= PATH_TRACKING_ERROR_P95, (
        f'{costmap} 완충 {inflation_radius - inscribed:.3f} m가 경로 추종 오차'
        f' p95 {PATH_TRACKING_ERROR_P95} m보다 작다. 오차가 내접반경을 먹어'
        ' footprint가 253 밴드에 들어가고, 그 자리에서 Spin이 회전하면 부딪힌다'
        ' (2026-07-30 의자 충돌)'
    )
    assert inflation_radius <= CORRIDOR_HALF_WIDTH_MEDIAN, (
        f'{costmap} inflation_radius {inflation_radius}가 실측 통로 반폭 중앙값'
        f' {CORRIDOR_HALF_WIDTH_MEDIAN} m를 넘어, 협착부가 아니라 일반 통로에도'
        ' 비용 0인 중앙선이 없다. 우회가 아니라 전면 정체가 된다'
    )
    if inflation_radius > DRIVEN_CORRIDOR_CLEARANCE:
        print(
            f'[주의] {costmap} inflation_radius {inflation_radius}는 실주행'
            f' 통로 최협 여유 {DRIVEN_CORRIDOR_CLEARANCE} m를 넘는다.'
            ' 그 통로는 전 구간이 비용 지대이며 우회로가 없으면 진동한다.'
        )


def test_local_and_global_costmap_use_the_same_footprint():
    assert _footprint('local_costmap') == _footprint('global_costmap')

    params = _load_params()
    local = params['local_costmap']['local_costmap']['ros__parameters']
    global_ = params['global_costmap']['global_costmap']['ros__parameters']
    assert local['footprint_padding'] == global_['footprint_padding']
