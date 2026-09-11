"""D455 깊이를 2D 스캔으로 눌러 local_costmap 에 넣은 계약 (2026-08-29)."""
import ast
import math
import re
from pathlib import Path

import yaml

BASE_LINK_Z = 0.190
CAMERA_Z_IN_BASE = 0.855
CAMERA_HEIGHT = BASE_LINK_Z + CAMERA_Z_IN_BASE
CAMERA_PITCH_DEG = -0.71

TILT_BUDGET_DEG = 3.0
ROBOT_TOP_M = 1.10

DEPTH_SCAN_TOPIC = '/camera/depth_scan'
CLOUD_TOPIC = '/camera/camera/depth/color/points'


def _params():
    path = Path(__file__).parents[1] / 'config' / 'nav2_params.yaml'
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _voxel():
    p = _params()
    return p['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']


def _scan_src():
    return _voxel()['depth_scan']


def _launch_text():
    return (Path(__file__).parents[1] / 'launch'
            / 'nav2_map_test.launch.py').read_text(encoding='utf-8')


def _launch_param(name):
    """launch 의 depth_band_to_scan 블록에서 숫자 파라미터를 읽는다."""
    txt = _launch_text()
    i = txt.index('"depth_band_to_scan"')
    blk = txt[i:i + 3000]
    m = re.search(rf'"{name}":\s*(-?[0-9.]+)', blk)
    assert m, f'launch 에 {name} 이 없다'
    return float(m.group(1))


def test_depth_scan_is_an_observation_source():
    """블록만 써 두고 목록에 이름을 안 올리면 nav2 는 통째로 무시한다."""
    sources = _voxel()['observation_sources'].split()
    assert 'depth_scan' in sources, sources
    assert 'scan' in sources, '라이다를 빼면 안 된다. 깊이는 보조다.'


def test_depth_source_is_a_laserscan():
    """2D 로 누른 것이 이 설계의 핵심이다. PointCloud2 로 되돌리면"""
    c = _scan_src()
    assert c['data_type'] == 'LaserScan', (
        'PointCloud2 로 되돌리면 2026-08-29 의 유령 문제가 재발한다.'
    )
    assert c['topic'] == DEPTH_SCAN_TOPIC
    assert c['marking'] is True
    assert c['clearing'] is True


def test_launch_publishes_that_scan():
    """스캔을 만드는 노드가 실제로 launch 에 있고 토픽이 맞물려야 한다."""
    txt = _launch_text()
    assert 'pointcloud_to_laserscan' in txt, '스캔을 만드는 노드가 없다'
    assert f'("scan", "{DEPTH_SCAN_TOPIC}")' in txt, (
        f'출력이 {DEPTH_SCAN_TOPIC} 로 remap 되어야 costmap 이 받는다. '
        'remap 을 빠뜨리면 라이다의 /scan 과 충돌한다.'
    )
    assert f'("cloud_in", "{CLOUD_TOPIC}")' in txt


def test_band_bottom_clears_the_floor():
    """높이 띠의 아래끝이 흔들림을 견디는가."""
    lo = _launch_param('min_height')
    rng = _scan_src()['obstacle_max_range']
    rise = rng * math.sin(math.radians(TILT_BUDGET_DEG))
    assert lo > rise, (
        f'거리 {rng} m 에서 {TILT_BUDGET_DEG}도 기울면 바닥이 {rise*100:.1f} cm '
        f'로 떠오르는데 띠의 아래끝이 {lo*100:.1f} cm 다. 바닥이 벽이 된다.'
    )


def test_band_top_stops_at_the_robot():
    """로봇보다 높은 물건은 볼 필요가 없다. 넓힐수록 헛 장애물만 는다."""
    hi = _launch_param('max_height')
    assert hi <= ROBOT_TOP_M, (
        f'로봇 최고점이 {ROBOT_TOP_M} m 인데 {hi} m 까지 본다. '
        '머리 위로 지나갈 물건을 장애물로 잡는다.'
    )
    assert hi > _launch_param('min_height') + 0.30, (
        '띠가 너무 좁으면 볼 수 있는 것이 거의 없다. '
        '2D 로 누른 뒤에는 넓혀도 지우기가 나빠지지 않는다.'
    )


def test_camera_geometry_matches_the_urdf():
    """이 파일의 계산이 딛고 선 기하가 URDF 와 어긋나면 알려준다."""
    urdf = (Path(__file__).parents[2] / 'vica_description' / 'urdf'
            / 'VICA.xacro').read_text(encoding='utf-8', errors='ignore')
    assert f'"camera_z" value="{CAMERA_Z_IN_BASE}"' in urdf, (
        'URDF 의 camera_z 가 바뀌었다. 띠의 높이를 다시 정하라.'
    )
    rad = math.radians(CAMERA_PITCH_DEG)
    assert f'"camera_pitch" value="{rad:.4f}"' in urdf, (
        f'URDF 의 camera_pitch 가 {rad:.4f} rad 가 아니다. 마스트를 다시 쟀다면 '
        '이 시험의 CAMERA_PITCH_DEG 도 같이 고쳐라. 안 맞으면 바닥이 떠올라 '
        '앞이 막힌다(2026-08-29 실주행에서 겪음).'
    )


def test_depth_scan_and_lidar_use_different_layers():
    """서로 다른 층을 써야 한쪽이 다른 쪽을 잘못 지우지 않는다."""
    assert _launch_param('target_frame') if False else True
    assert '"target_frame": "base_footprint"' in _launch_text(), (
        '스캔이 지면 기준으로 나와야 아래 높이 창(0.0~0.10)과 맞는다.'
    )
    d = _scan_src()
    assert d['min_obstacle_height'] < 0.0 < d['max_obstacle_height'] <= 0.20, (
        f"깊이 스캔의 높이 창 {d['min_obstacle_height']}~"
        f"{d['max_obstacle_height']} 이 지면 층을 벗어났다. 아래끝은 0 보다 "
        '작아야 한다 -- 스캔이 정확히 z=0 이라 경계에 걸리면 통째로 버려진다.'
    )
    lidar = _voxel()['scan']
    assert lidar.get('min_obstacle_height', 0.0) < 0.40, (
        '라이다 평면(0.382 m)이 걸러지면 낮은 장애물을 아무도 못 본다.'
    )
    assert d['max_obstacle_height'] < 0.382, (
        '깊이 스캔의 높이 창이 라이다 평면까지 닿으면 서로의 마크를 지운다.'
    )


def test_marking_range_stays_inside_clearing_range():
    """표시 범위가 소거 범위보다 넓으면 지워지지 않는 마크가 남는다."""
    c = _scan_src()
    assert c['raytrace_max_range'] > c['obstacle_max_range']
    assert c['raytrace_min_range'] <= c['obstacle_min_range']
    assert c['obstacle_max_range'] <= _launch_param('range_max'), (
        '스캔이 내지 않는 거리까지 표시하려 한다.'
    )


def test_depth_is_not_remembered():
    """observation_persistence 0.0 -- 동적 장애물 회피의 핵심."""
    assert _scan_src()['observation_persistence'] == 0.0


def test_camera_dropout_does_not_stop_the_robot():
    """expected_update_rate 0.0 = stale 판정을 하지 않는다."""
    assert _scan_src()['expected_update_rate'] == 0.0


def test_empty_directions_are_cleared():
    """물체가 없는 방향은 inf 로 오고, 그것을 청소에 써야 빈 공간이 열린다."""
    assert _scan_src()['inf_is_valid'] is True
    assert '"use_inf": True' in _launch_text(), (
        'use_inf 를 끄면 빈 방향이 스캔에서 통째로 빠져 청소가 안 된다.'
    )


def test_global_costmap_does_not_take_the_camera():
    """전역 지도에는 깊이를 넣지 않는다."""
    g = _params()['global_costmap']['global_costmap']['ros__parameters']
    assert g['obstacle_layer']['observation_sources'].split() == ['scan']


def test_voxel_grid_is_not_taller_than_needed():
    """2D 로 누른 뒤에는 높은 격자가 필요 없다. 격자는 CPU 를 먹는다."""
    v = _voxel()
    ceiling = v['origin_z'] + v['z_voxels'] * v['z_resolution']
    assert ceiling <= 1.0, (
        f'격자 상한 {ceiling:.2f} m. 라이다(0.382)와 깊이 스캔(지면)만 쓰므로 '
        '이보다 높일 이유가 없다.'
    )
