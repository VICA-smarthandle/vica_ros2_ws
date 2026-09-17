"""레일(Route Server) 트리와 레일 파일의 계약.

2026-09-16 실주행 세 회차(run1·run3·run5)가 같은 이유로 섰다. route_server 가
점 하나짜리 경로를 내놓았고 controller 가 "Received plan with zero length" 를
2739·999·237회 뱉었다. route 없이 달린 run4 는 0회. 원인 둘, 수리 둘:

  원인 1  레일이 꺾이는 점만 남겨 직선 복도가 6.71 m 엣지 하나였다. route_server
          는 로봇이 첫 노드를 지나치면 그 노드를 잘라내고, 엣지가 하나뿐이면 잘라낸
          뒤 엣지 0개 -> 점 1개 경로가 된다.         -> 엣지 상한 1 m
  원인 2  기성 IsPathValid 는 점 1개를 통과시킨다.   -> IsRoutePathUsable 거름망

여기 시험은 그 둘이 되돌아가지 않게 지킨다. 실제 route_server 동작은 실주행
으로만 확인된다.
"""
import json
import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

ROUTE_BT = 'vica_navigate_to_pose_route.xml'
GUARD = 'IsRoutePathUsable'
GUARD_LIB = 'vica_is_route_path_usable_condition_bt_node'
MAP = 'vica_map_0630'
MAX_EDGE_M = 1.00          # scripts/vica_route_graph.py 의 MAX_EDGE_M 과 같아야 한다
EDGE_SLACK_M = 0.10        # 사이 점이 벽을 피해 옮겨질 때의 여유
DEST_TO_NODE_M = 0.60      # 목적지는 이 안에 노드가 있어야 레일이 '지나간다'

# 뒤에 사람이 손잡이를 잡고 있어 후진 계열은 어느 트리에도 못 들어간다.
REVERSE_CAPABLE_NODES = ('BackUp', 'DriveOnHeading', 'AssistedTeleop')


def _pkg_dir():
    return Path(__file__).resolve().parent.parent


def _ws_maps():
    return _pkg_dir().parent.parent / 'maps'


def _bt_root():
    return ET.parse(_pkg_dir() / 'behavior_trees' / ROUTE_BT).getroot()


def _params():
    return yaml.safe_load(
        (_pkg_dir() / 'config' / 'nav2_params.yaml').read_text(encoding='utf-8'))


def _graph():
    p = _ws_maps() / f'{MAP}_route.geojson'
    if not p.is_file():
        pytest.skip(f'레일 파일이 없다: {p} — scripts/vica_route_graph.py {MAP}')
    g = json.loads(p.read_text(encoding='utf-8'))
    nodes, edges = {}, []
    for f in g['features']:
        if f['geometry']['type'] == 'Point':
            nodes[f['properties']['id']] = tuple(f['geometry']['coordinates'])
        else:
            edges.append((f['properties']['startid'], f['properties']['endid']))
    return nodes, edges


# ── BT ──────────────────────────────────────────────────────────────────────

def test_route_bt_guards_the_route_path_before_following():
    """ComputeRoute -> IsRoutePathUsable -> IsPathValid. 순서가 곧 안전이다.

    거름망이 빠지면 점 1개 경로가 controller 로 간다(run5). IsPathValid 가
    빠지면 레일 위 장애물로 그대로 들어간다.
    """
    seqs = [s for s in _bt_root().iter('Sequence')
            if s.get('name') == 'RouteIfUsableAndClear']
    assert len(seqs) == 1, 'RouteIfUsableAndClear 시퀀스가 하나여야 한다'
    tags = [c.tag for c in seqs[0]]
    assert tags == ['ComputeRoute', GUARD, 'IsPathValid'], tags

    guard = seqs[0][1]
    assert guard.get('path') == '{path}'
    assert int(guard.get('min_poses', '0')) >= 2, '점 2개 미만은 선이 아니다'
    assert 0 < float(guard.get('max_dist_from_path', '0')) < 3.0, (
        'DWB 지역 창 반폭(3 m)보다 작아야 "0 poses" 를 막는다')
    assert guard.get('robot_base_frame') == _params()['bt_navigator'][
        'ros__parameters']['robot_base_frame']
    # run6 입구: 레일 끝 방향(남)과 목적지(북)가 반대라 제자리 180도 회전 40초.
    # 목적지 근처는 planner 가 도착 방향까지 맞춰 그리도록 넘겨야 한다.
    assert guard.get('goal') == '{goal}', '인계 판정에 목적지가 필요하다'
    assert 1.0 <= float(guard.get('handoff_dist_to_goal', '0')) <= 3.0, (
        '1 m 미만이면 제자리 회전이 남고, 3 m 넘으면 레일을 너무 일찍 버린다')


def test_route_server_does_not_smooth_corners():
    """run6: 코너 둥글리기가 NaN 좌표를 만들어 controller abort 970회. 끈다."""
    rs = _params()['route_server']['ros__parameters']
    assert rs.get('smooth_corners') is False, (
        'smooth_corners 가 켜져 있다. 1 m 엣지·일직선 중간역과 함께 쓰면 NaN 경로가 나온다')


def test_route_bt_falls_back_to_freespace_when_route_unusable():
    """레일이 안 되면 Fallback 의 두 번째 자식(자유주행)이 있어야 한다."""
    fb = [f for f in _bt_root().iter('Fallback') if f.get('name') == 'RouteThenFreespace']
    assert len(fb) == 1
    tags = [c.tag for c in fb[0]]
    assert tags == ['Sequence', 'ComputePathToPose'], tags
    assert fb[0][1].get('goal') == '{goal}' and fb[0][1].get('path') == '{path}'


def test_route_bt_has_last_mile_freespace_phase():
    """공식 README 4번(Last-Mile). 레일 끝 노드 -> 목적지는 자유주행으로.

    레일 경로는 가장 가까운 노드에서 끝나고 끝 방향은 마지막 엣지 방향이다
    (nav2_route path_converter.cpp). 목적지 yaw 로 맞추려면 2단계가 있어야 한다.
    """
    root = _bt_root()
    pipes = [p.get('name') for p in root.iter('PipelineSequence')]
    assert pipes == ['RailWithReplanning', 'LastMileWithReplanning'], pipes

    last = next(p for p in root.iter('PipelineSequence')
                if p.get('name') == 'LastMileWithReplanning')
    assert not list(last.iter('ComputeRoute')), '2단계는 레일을 다시 보지 않는다'
    assert len(list(last.iter('ComputePathToPose'))) == 1
    assert len(list(last.iter('FollowPath'))) == 1

    # 두 단계는 Sequence 로 묶여 1단계가 성공해야 2단계로 간다.
    seq = [s for s in root.iter('Sequence') if s.get('name') == 'RailThenLastMile']
    assert len(seq) == 1
    assert [c.get('name') for c in seq[0]] == pipes


@pytest.mark.parametrize('node_name', REVERSE_CAPABLE_NODES)
def test_route_bt_has_no_reverse_capable_node(node_name):
    assert not list(_bt_root().iter(node_name)), f'{node_name} 는 어느 트리에도 못 들어간다'


def test_route_bt_keeps_stationary_recovery_only():
    root = _bt_root()
    assert not list(root.iter('Spin')) and not list(root.iter('BackUp'))
    assert list(root.iter('Wait')), '정지 복구(Wait) 는 남아 있어야 한다'


# ── 플러그인 ───────────────────────────────────────────────────────────────

def test_guard_plugin_is_registered_in_bt_navigator():
    names = _params()['bt_navigator']['ros__parameters']['plugin_lib_names']
    assert GUARD_LIB in names, (
        f'{GUARD_LIB} 가 plugin_lib_names 에 없다. 트리는 {GUARD} 를 쓰는데 '
        'bt_navigator 가 그 노드를 모른 채 XML 을 읽다 멈춘다')


def test_guard_plugin_library_is_built():
    """이름만 맞고 .so 가 없으면 bt_navigator 가 configure 에서 멈춘다."""
    roots = []
    for var in ('LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH'):
        for chunk in os.environ.get(var, '').split(':'):
            if chunk:
                roots += [Path(chunk), Path(chunk) / 'lib']
    roots = [r for r in dict.fromkeys(roots) if r.is_dir()]
    if not roots:
        pytest.skip('라이브러리 경로를 못 찾았다. source 후 실행한다')
    assert any((r / f'lib{GUARD_LIB}.so').is_file() for r in roots), (
        f'lib{GUARD_LIB}.so 가 없다. colcon build --packages-select vica_nav2_bt_plugins')


# ── 레일 파일 ──────────────────────────────────────────────────────────────

def test_route_graph_edges_are_short():
    """모든 엣지 <= 1 m. 6.71 m 엣지가 run5 를 세웠다."""
    nodes, edges = _graph()
    long = [(a, b, round(math.dist(nodes[a], nodes[b]), 2)) for a, b in edges
            if math.dist(nodes[a], nodes[b]) > MAX_EDGE_M + EDGE_SLACK_M]
    assert not long, f'{MAX_EDGE_M} m 를 넘는 엣지: {long}'


def test_route_graph_corners_are_rounded():
    """run8: smooth_corners 를 끄니 코너가 뾰족해 DWB 가 지나쳤다 되돌아왔다(w ±0.29).
    생성기가 15~120° 코너를 호로 바꾼다. 30~120° 로 한 번에 꺾이는 노드가 남으면 안 된다.
    120° 초과는 목적지 스퍼의 되돌림이라 예외."""
    nodes, edges = _graph()
    ids = sorted(nodes); n = len(ids); sharp = []
    for i, k in enumerate(ids):
        a, b, c = nodes[ids[i - 1]], nodes[k], nodes[ids[(i + 1) % n]]
        v1 = (b[0] - a[0], b[1] - a[1]); v2 = (c[0] - b[0], c[1] - b[1])
        ang = abs(math.degrees(math.atan2(v1[0]*v2[1] - v1[1]*v2[0], v1[0]*v2[0] + v1[1]*v2[1])))
        if 30 <= ang <= 120:
            sharp.append((k, round(ang)))
    assert not sharp, f'뾰족한 코너가 남았다 (노드, 각도): {sharp}'


def test_route_graph_is_a_bidirectional_ring():
    """엣지는 방향이 있다. 양쪽으로 다니려면 쌍이어야 하고, 고리는 끊기면 안 된다."""
    nodes, edges = _graph()
    es = set(edges)
    missing = [(a, b) for a, b in edges if (b, a) not in es]
    assert not missing, f'되돌아오는 엣지가 없다: {missing[:5]}'

    adj = {}
    for a, b in edges:
        adj.setdefault(a, set()).add(b)
    assert all(len(adj.get(n, ())) >= 2 for n in nodes), '이웃이 2개 미만인 노드가 있다'

    seen, stack = set(), [next(iter(nodes))]
    while stack:
        n = stack.pop()
        if n in seen:
            continue
        seen.add(n)
        stack.extend(adj.get(n, ()))
    assert seen == set(nodes), '고리가 끊겨 섬이 있다'


def test_route_graph_passes_through_registered_destinations():
    """레일은 미션 매니저가 실제로 쓰는 목적지를 지나가야 한다.

    2026-09-16 1판은 저장소의 locations.json 을 읽어 안내소가 1.03 m 빗나갔다.
    """
    doc = Path.home() / 'vica_data' / 'destinations' / MAP / 'destinations.yaml'
    if not doc.is_file():
        pytest.skip(f'이 기기엔 목적지 파일이 없다: {doc}')
    nodes, _ = _graph()
    far = []
    for d in (yaml.safe_load(doc.read_text(encoding='utf-8')) or {}).get('destinations', []):
        po = d.get('pose') or {}
        if po.get('x') is None:
            continue
        dist = min(math.dist((po['x'], po['y']), xy) for xy in nodes.values())
        if dist > DEST_TO_NODE_M:
            far.append((d.get('name'), round(dist, 2)))
    assert not far, f'레일에서 {DEST_TO_NODE_M} m 넘게 떨어진 목적지: {far}'
