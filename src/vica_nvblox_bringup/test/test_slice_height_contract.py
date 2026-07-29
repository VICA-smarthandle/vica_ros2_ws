"""nvblox esdf slice가 로봇 전체 높이를 덮는지 감시한다.

2026-07-28 실측에서 0.50~0.86 m 구간을 아무 센서도 보지 않는 사각지대가
드러났다. LiDAR는 0.382 m 단일 평면이라 테이블 상판을 보지 못하고 다리만
본다. 그래서 "상판 아래로 지나갈 수 있다"고 판단하고 진입하는데, 로봇
최고점은 0.86 m라 상부가 그대로 들이받는다.

이 테스트는 slice 상한이 로봇 최고점 아래로 다시 내려가는 회귀를 막는다.
"""
import struct
from pathlib import Path

import pytest
import yaml


# base_link.stl은 mm 단위이고 URDF에서 scale 0.001로 쓰인다.
STL_SCALE = 0.001
# URDF collision origin이 xyz="0 0 -0.044"라 STL z에서 이만큼 내려간 값이
# base_link 좌표다.
COLLISION_Z_OFFSET = -0.044
# base_footprint -> base_link (VICA.xacro base_link_height). 바닥 기준으로
# 환산하려면 이만큼 더한다. 실측 TF와도 일치한다.
BASE_LINK_HEIGHT = 0.19

# 실측 TF(base_footprint 기준). 이 둘만으로는 로봇 상부를 볼 수 없다는 것이
# 이 계약의 전제다.
LASER_Z = 0.382
CAMERA_Z = 0.320


def _repo_src():
    return Path(__file__).parents[2]


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
            off = base + 12 + vertex * 12 + 8  # x, y 건너뛰고 z
            z = struct.unpack('<f', data[off:off + 4])[0]
            max_z = max(max_z, z * STL_SCALE)
    return max_z + COLLISION_Z_OFFSET + BASE_LINK_HEIGHT


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
    """바닥이 밴드에 들어가면 slice가 바닥 전체를 장애물로 투영한다.

    그러면 nvblox_layer(binary)가 로봇 주변을 LETHAL로 덮어 DWB가 유효
    trajectory를 만들지 못한다. odom 기준 바닥은 z≈0이다.
    """
    min_height = _mapper(mapper)['esdf_slice_min_height']
    assert min_height > 0.0, (
        f'{mapper} esdf_slice_min_height {min_height}는 바닥(z≈0)을 포함한다'
    )


@pytest.mark.parametrize('mapper', ['static_mapper', 'dynamic_mapper'])
def test_slice_band_is_ordered(mapper):
    params = _mapper(mapper)
    assert params['esdf_slice_min_height'] < params['esdf_slice_max_height']
    # slice_height는 밴드 안에 있어야 투영 기준면이 의미를 가진다.
    assert (
        params['esdf_slice_min_height']
        <= params['esdf_slice_height']
        <= params['esdf_slice_max_height']
    )


def test_sensors_alone_cannot_cover_the_robot():
    """이 계약이 왜 필요한지를 고정한다.

    LiDAR·카메라 높이가 로봇 최고점보다 낮은 한, 상부 보호는 slice 상한에만
    달려 있다. 센서 배치가 바뀌어 이 전제가 깨지면 위 계약도 다시 봐야 한다.
    """
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
    """mapping_type과 nav2 구독 토픽이 어긋나면 조용히 실패한다.

    dynamic 모드에서 nvblox는 static/dynamic을 나눠 관리하고 combined_map_slice로
    합쳐 발행한다. nav2가 static_map_slice를 그대로 구독하면 동적 장애물(사람)이
    costmap에 들어오지 않는데, 토픽은 정상 수신되므로 겉으로는 아무 문제가
    없어 보인다. 이 테스트가 그 조합을 막는다.
    """
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
    """정적 지도를 동적보다 오래 기억해야 한다.

    벽과 가구는 움직이지 않으므로 잊을 이유가 없다. 반대로 사람이 지나간
    자리는 빨리 지워야 경로가 막히지 않는다. decay rate는 '초당 몇 번
    감쇠하는가'이므로 값이 작을수록 오래 기억한다.
    """
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
