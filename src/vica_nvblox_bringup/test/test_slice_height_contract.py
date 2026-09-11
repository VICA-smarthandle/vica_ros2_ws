"""nvblox esdf slice가 로봇 전체 높이를 덮는지 감시한다."""
import re
import struct
from pathlib import Path

import pytest
import yaml


STL_SCALE = 0.001

LASER_Z = 0.382
CAMERA_Z = 0.320


def _repo_src():
    return Path(__file__).parents[2]


def _xacro_property(name):
    """VICA.xacro의 xacro:property 값을 읽는다."""
    path = _repo_src() / 'vica_description' / 'urdf' / 'VICA.xacro'
    text = path.read_text(encoding='utf-8')
    match = re.search(
        rf'<xacro:property\s+name="{name}"\s+value="([^"]+)"',
        text,
    )
    assert match is not None, (
        f'VICA.xacro에서 xacro:property "{name}"을 찾지 못했다.'
        ' URDF 구조가 바뀌었다면 이 계약도 다시 봐야 한다'
    )
    return float(match.group(1))


def _overrides():
    path = (
        _repo_src() / 'vica_nvblox_bringup' / 'config'
        / 'vica_nvblox_overrides.yaml'
    )
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _mapper(name):
    return _overrides()['/**']['ros__parameters'][name]


def _stl_path():
    return _repo_src() / 'vica_description' / 'meshes' / 'base_link.stl'


def _robot_top_from_floor():
    """base_link.stl에서 바닥 기준 로봇 최고점을 m 단위로 계산한다."""
    data = _stl_path().read_bytes()
    triangle_count = struct.unpack('<I', data[80:84])[0]
    max_z = -1e9
    for i in range(triangle_count):
        base = 84 + i * 50
        for vertex in range(3):
            off = base + 12 + vertex * 12 + 8
            z = struct.unpack('<f', data[off:off + 4])[0]
            max_z = max(max_z, z * STL_SCALE)
    return (
        max_z
        - _xacro_property('body_center_z')
        + _xacro_property('base_link_height')
    )


@pytest.mark.parametrize('mapper', ['static_mapper', 'dynamic_mapper'])
def test_slice_covers_robot_height(mapper):
    if not _stl_path().exists():
        pytest.skip('vica_description/meshes/base_link.stl 없음')

    top = _robot_top_from_floor()
    max_height = _mapper(mapper)['esdf_slice_max_height']

    assert max_height >= top, (
        f'{mapper} esdf_slice_max_height {max_height}가 로봇 최고점'
        f' {top:.3f} m보다 낮다. 그 사이 높이의 테이블 상판·선반·벽 돌출물을'
        f' 아무 센서도 보지 못해 로봇 상부가 충돌한다'
    )


@pytest.mark.parametrize('mapper', ['static_mapper', 'dynamic_mapper'])
def test_slice_excludes_the_floor(mapper):
    """바닥이 밴드에 들어가면 slice가 바닥 전체를 장애물로 투영한다."""
    min_height = _mapper(mapper)['esdf_slice_min_height']
    assert min_height > 0.0, (
        f'{mapper} esdf_slice_min_height {min_height}는 바닥(z≈0)을 포함한다'
    )


@pytest.mark.parametrize('mapper', ['static_mapper', 'dynamic_mapper'])
def test_slice_band_is_ordered(mapper):
    params = _mapper(mapper)
    assert params['esdf_slice_min_height'] < params['esdf_slice_max_height']
    assert (
        params['esdf_slice_min_height']
        <= params['esdf_slice_height']
        <= params['esdf_slice_max_height']
    )


def test_sensors_alone_cannot_cover_the_robot():
    """이 계약이 왜 필요한지를 고정한다."""
    if not _stl_path().exists():
        pytest.skip('vica_description/meshes/base_link.stl 없음')
    top = _robot_top_from_floor()
    assert max(LASER_Z, CAMERA_Z) < top, (
        '센서가 로봇 최고점보다 높아졌다. slice 상한 계약의 전제를 재검토하라'
    )


def _nav2_params():
    path = (
        _repo_src() / 'vica_nav2' / 'config' / 'nav2_params.yaml'
    )
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _nvblox_layer():
    local = _nav2_params()['local_costmap']['local_costmap']['ros__parameters']
    return local['nvblox_layer']


def test_nav2_subscribes_the_slice_that_this_mapping_type_publishes():
    """mapping_type과 nav2 구독 토픽이 어긋나면 조용히 실패한다."""
    params = _overrides()['/**']['ros__parameters']
    mapping_type = params['mapping_type']
    topic = _nvblox_layer()['nvblox_map_slice_topic']

    if mapping_type in ('dynamic', 'human_with_static_tsdf',
                        'human_with_static_occupancy'):
        assert topic.endswith('/combined_map_slice'), (
            f'mapping_type={mapping_type}인데 nav2가 {topic}을 구독한다.'
            ' 동적 장애물이 costmap에 들어오지 않는다'
        )
    else:
        assert topic.endswith('/static_map_slice'), (
            f'mapping_type={mapping_type}은 combined_map_slice를 발행하지 않는다.'
            f' {topic} 구독은 아무 데이터도 받지 못한다'
        )


def test_static_memory_outlives_dynamic_memory():
    """정적 지도를 동적보다 오래 기억해야 한다."""
    params = _overrides()['/**']['ros__parameters']
    static_decay = params['decay_tsdf_rate_hz']
    dynamic_decay = params['decay_dynamic_occupancy_rate_hz']

    assert static_decay < dynamic_decay, (
        f'정적 decay {static_decay}가 동적 decay {dynamic_decay}보다 빠르다.'
        ' 벽·가구를 사람보다 빨리 잊는다는 뜻이다'
    )
    assert dynamic_decay > 0.0, (
        '동적 decay가 0이면 지나간 사람이 영구히 남아 경로를 막는다'
    )
