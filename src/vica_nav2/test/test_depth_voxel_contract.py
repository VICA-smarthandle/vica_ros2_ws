"""D455 깊이를 2D 스캔으로 눌러 local_costmap 에 넣은 계약 (2026-08-29).

왜 2D 인가 -- 하루치 실패에서 나온 결론이다
--------------------------------------------
처음에는 깊이 포인트클라우드를 그대로 3D voxel_layer 에 넣었다. 책상 다리는
잘 봤지만 **찍은 칸을 지우지 못했다.** bag 실측(run2017, 846초):

    0.9~1.8 m (깊이가 찍는 구간)   3,833칸   수명 중앙 21.6초   최대 191초
    1.8 m 밖  (라이다 구간)           451칸   수명 중앙  0.6초

원인은 카메라 높이다. 카메라는 지면 1.025 m 에 있고, 그보다 낮은 칸을 지우려면
그 칸을 스치는 광선이 훨씬 먼 바닥까지 닿아야 한다.

    필요한 광선 길이 R = d / (1 - h / 1.025)
      d 1.8 m · h 0.60 m -> 4.3 m       d 1.8 m · h 0.75 m -> 6.7 m
      d 1.8 m · h 0.90 m -> 9.8 m       (카메라 높이에 가까울수록 발산한다)

실내에서는 3~4 m 앞에 벽이 있어 그만한 바닥이 애초에 안 보인다.
raytrace_max_range 를 8 m 로 늘려도 볼 바닥이 없으면 소용이 없었다.

라이다가 잘 지워지는 이유는 **자기와 같은 높이를 보기 때문**이다. 광선이 그
평면을 그대로 훑는다. 그래서 깊이도 한 평면으로 눌러 같은 성질을 갖게 했다.
높이는 버리고 "이 방향 이 거리에 뭔가 있다"만 남긴다.

이 파일이 지키는 것
-------------------
1. 스캔을 만드는 쪽(launch)과 받는 쪽(costmap)의 값이 어긋나지 않는다
2. 바닥을 장애물로 찍지 않는다
3. 라이다와 다른 층을 써서 서로 잘못 지우지 않는다
"""
import ast
import math
import re
from pathlib import Path

import yaml

# ── URDF 정본에서 온 값 (vica_description/urdf/VICA.xacro) ────────────────
BASE_LINK_Z = 0.190
CAMERA_Z_IN_BASE = 0.835
CAMERA_HEIGHT = BASE_LINK_Z + CAMERA_Z_IN_BASE      # 지면 1.025 m
# 마스트가 실제로 기운 각도(음수 = 위로 들림). 2026-08-29 바닥평면 RANSAC
# 4회 실측(3.05 / 2.88 / 3.16 / 3.06 -> 3.04 +- 0.14).
# 부호 주의: 측정기의 양수는 "URDF 보다 위로 들렸다"는 뜻이라 URDF pitch 에는
# 뒤집어 넣는다. 처음에 +0.0524 를 넣었더니 오차가 두 배가 되어 확정했다.
CAMERA_PITCH_DEG = -3.0

# 마운트 오차 + 주행 중 피칭. 이보다 크게 흔들린다면 파라미터가 아니라 기구로
# 풀 문제다.
TILT_BUDGET_DEG = 3.0
# 로봇의 최고점(카메라 마스트). 그보다 높은 물건은 로봇 위를 지나간다.
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


# ── 1. 배선 ───────────────────────────────────────────────────────────────
def test_depth_scan_is_an_observation_source():
    """블록만 써 두고 목록에 이름을 안 올리면 nav2 는 통째로 무시한다."""
    sources = _voxel()['observation_sources'].split()
    assert 'depth_scan' in sources, sources
    assert 'scan' in sources, '라이다를 빼면 안 된다. 깊이는 보조다.'


def test_depth_source_is_a_laserscan():
    """2D 로 누른 것이 이 설계의 핵심이다. PointCloud2 로 되돌리면
    지우지 못하는 유령이 그대로 돌아온다(이 파일 머리말 참조)."""
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


# ── 2. 바닥을 찍지 않는가 ─────────────────────────────────────────────────
def test_band_bottom_clears_the_floor():
    """높이 띠의 아래끝이 흔들림을 견디는가.

    3D 때와 달리 이제 바닥은 **스캔을 만드는 단계**에서 걸러진다. 그러나
    차체가 앞뒤로 기울면 바닥점이 떠오르는 것은 같다.
        떠오르는 높이 = 표시거리 x sin(기운 각도)
    """
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


# ── 3. 층 분리 · 지우기 ───────────────────────────────────────────────────
def test_depth_scan_and_lidar_use_different_layers():
    """서로 다른 층을 써야 한쪽이 다른 쪽을 잘못 지우지 않는다.

    깊이 스캔은 base_footprint(지면 z=0) 기준으로 나오고, 라이다는 지면
    0.382 m 평면이다. costmap 의 높이 창이 겹치면 한 센서의 청소가 다른
    센서의 마크를 지운다 -- 깊이는 앞만 보므로 옆·뒤를 보는 라이다의 마크를
    지우면 위험하다.
    """
    assert _launch_param('target_frame') if False else True  # 문자열이라 별도 검사
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
    """표시 범위가 소거 범위보다 넓으면 지워지지 않는 마크가 남는다.

    2D 가 된 뒤에는 이 오래된 규칙이 다시 충분조건이 된다 -- 3D 때는
    "raytrace > obstacle" 만으로는 한참 모자랐다(머리말의 광선 길이 표).
    """
    c = _scan_src()
    assert c['raytrace_max_range'] > c['obstacle_max_range']
    assert c['raytrace_min_range'] <= c['obstacle_min_range']
    assert c['obstacle_max_range'] <= _launch_param('range_max'), (
        '스캔이 내지 않는 거리까지 표시하려 한다.'
    )


def test_depth_is_not_remembered():
    """observation_persistence 0.0 -- 동적 장애물 회피의 핵심.

    라이다는 0.5 s 를 보관한다. 단일 평면이라 얇은 다리에 점이 1~2개만 맞아
    검출이 깜빡이기 때문이다. 깊이는 반대다. 한 물체에 점이 수백 개라 깜빡임이
    없고, 보관하면 비켜준 사람만 남는다.
    """
    assert _scan_src()['observation_persistence'] == 0.0


def test_camera_dropout_does_not_stop_the_robot():
    """expected_update_rate 0.0 = stale 판정을 하지 않는다.

    카메라가 끊겼다고 costmap 이 isCurrent()=false 가 되어 주행이 멎으면 안
    된다. 깊이는 라이다를 보조하는 두 번째 눈이지 주행의 전제가 아니다.
    """
    assert _scan_src()['expected_update_rate'] == 0.0


def test_empty_directions_are_cleared():
    """물체가 없는 방향은 inf 로 오고, 그것을 청소에 써야 빈 공간이 열린다."""
    assert _scan_src()['inf_is_valid'] is True
    assert '"use_inf": True' in _launch_text(), (
        'use_inf 를 끄면 빈 방향이 스캔에서 통째로 빠져 청소가 안 된다.'
    )


def test_global_costmap_does_not_take_the_camera():
    """전역 지도에는 깊이를 넣지 않는다.

    planner 는 전역 costmap 위에 경로를 그린다. 거기에 깊이가 들어가면 한 번
    잘못 찍힌 것이 통로를 막아 경로 자체가 안 나온다. 사람과 책상을 피하는
    일은 5 Hz 로 도는 local(DWB)의 몫이다.
    """
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
