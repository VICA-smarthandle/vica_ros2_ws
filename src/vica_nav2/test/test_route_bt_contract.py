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

def test_route_bt_plans_only_to_a_carrot_on_the_rail():
    """README 방식 1+2 (8판). ComputeRoute -> 거름망 -> 앞 3 m -> [비면 레일 직접 | 막히면 6 m 당근 -> planner].

    run10: 출발 순간 레일 첫 0.8 m 위에 뒤따르는 사람이 서 있어 레일이 무효였고, 그때
    planner 가 최종 목적지까지 그린 자유주행이 로봇을 다른 복도(휴게실 대각선)로 끌고
    갔다. planner 목표를 레일 위 앞 지점(당근)으로 묶으면 사람을 돌아 레일로 복귀한다.
    run15~17(0903_d): 늘 당근이면 코너를 끊어 돌고(planner 경로가 매초 22.5° 눈금에서 새로
    시작) 레일의 호를 못 쓴다 → 앞 3 m 가 비면 레일을 그대로 따른다(8판). 당근이 3 m 고정점이면
    사람을 비켜도 곧 되돌아오는 좁은 S 라 횡 이탈 0.27 m → 막혔을 때 당근은 6 m.
    """
    seqs = [s for s in _bt_root().iter('Sequence') if s.get('name') == 'RouteCarrot']
    assert len(seqs) == 1, 'RouteCarrot 시퀀스가 하나여야 한다'
    tags = [c.tag for c in seqs[0]]
    assert tags == ['ComputeRoute', GUARD, 'TruncatePathLocal', 'Fallback'], tags
    guard, trunc, mode = seqs[0][1], seqs[0][2], seqs[0][3]
    assert mode.get('name') == 'RailIfClearElseCarrot'
    assert [c.tag for c in mode] == ['Sequence', 'Sequence'] and [c.get('name') for c in mode] == ['RailDirect', 'CarrotBeyond']
    rail_direct, carrot_seq = mode[0], mode[1]
    # 8판: 앞 3 m 조각이 비어 있으면 레일 그대로. IsPathValid 는 조각만 본다(레일 전체를 보던 3판 사고 금지).
    assert [c.tag for c in rail_direct] == ['IsPathValid', 'ComputeRoute'], '레일 직접 = 3 m 조각 검사 뒤 레일을 {path} 에'
    assert rail_direct[1].get('path') == '{path}' and rail_direct[1].get('goal') == '{goal}'
    valids = list(seqs[0].iter('IsPathValid'))
    assert len(valids) == 1 and valids[0].get('path') == trunc.get('output_path'), (
        'IsPathValid 는 앞 3 m 조각({rail_ahead})만 검사한다. 레일 전체({rail_path})를 주면 run9 의 "치명 칸 하나에 레일 통째 거부" 가 돌아온다')
    # 막혔을 때만 당근: 6 m 조각 끝점 -> planner, 실패 시 레일 후퇴(6판).
    assert [c.tag for c in carrot_seq] == ['TruncatePathLocal', 'GetPoseFromPath', 'Fallback'], [c.tag for c in carrot_seq]
    far, pick, cor = carrot_seq[0], carrot_seq[1], carrot_seq[2]
    # 6판(run11): 당근 planner 가 실패해도 레일을 다시 받아 그대로 따른다. 매 틱 planner 에
    # 기대면 벽 옆에서 뒤에 사람이 붙을 때 "Starting point in lethal" 로 둘 다 막혀 선다.
    assert cor.get('name') == 'CarrotOrRail'
    assert [c.tag for c in cor] == ['ComputePathToPose', 'ComputeRoute'], (
        '당근 planner 실패 시 ComputeRoute 로 레일을 다시 받아야 한다 (실패한 ComputePathToPose 는 {path} 를 비울 수 있다)')
    plan = cor[0]
    assert cor[1].get('path') == '{path}' and cor[1].get('goal') == '{goal}'

    # 7판(run12): 레일 원본은 {path} 에 두지 않는다. 목적지 옆에서 점 1개(방향 0°)로 줄어든 레일이
    # {path} 를 덮으면, GoalReached 로 재계획을 멈춘 뒤 FollowPath 가 그 점을 목표로 삼아 0° 에서
    # 도착 처리한다(방2 +80°, 입구 −155°). {path} 는 planner 결과와 후퇴용 레일만.
    rail_key = seqs[0][0].get('path')
    assert rail_key and rail_key != '{path}', 'ComputeRoute 원본은 {path} 가 아닌 키에 받아야 한다'
    assert guard.get('path') == rail_key and guard.get('goal') == '{goal}'
    assert int(guard.get('min_poses', '0')) >= 2, '점 2개 미만은 선이 아니다'
    assert 0 < float(guard.get('max_dist_from_path', '0')) < 3.0, (
        'DWB 지역 창 반폭(3 m)보다 작아야 "0 poses" 를 막는다')
    assert 1.0 <= float(guard.get('handoff_dist_to_goal', '0')) <= 3.0, (
        '1 m 미만이면 제자리 회전이 남고, 3 m 넘으면 레일을 너무 일찍 버린다')
    base = _params()['bt_navigator']['ros__parameters']['robot_base_frame']
    assert guard.get('robot_base_frame') == base

    assert trunc.get('input_path') == rail_key
    ahead = trunc.get('output_path')
    assert ahead and ahead != '{path}', '자른 조각이 {path} 를 덮으면 레일 전체를 잃는다'
    assert 2.0 <= float(trunc.get('distance_forward', '0')) <= 4.0, (
        '2 m 미만이면 당근이 너무 가까워 planner 가 매초 급하게 꺾고, 4 m 넘으면 지역 창을 벗어난다')
    assert float(trunc.get('distance_backward', '1')) == 0.0, '뒤(사람이 선 곳)는 보지 않는다'
    assert trunc.get('robot_frame') == base

    # 8판: 막혔을 때 당근은 먼 조각(6 m)의 끝점. 3 m 고정점이면 회피 폭이 0.27 m 에 묶인다(run16).
    assert far.get('input_path') == rail_key and far.get('robot_frame') == base
    far_key = far.get('output_path')
    assert far_key and far_key not in ('{path}', ahead), '먼 조각은 {path}·검사 조각과 다른 키에'
    assert 5.0 <= float(far.get('distance_forward', '0')) <= 8.0, (
        '5 m 미만이면 2 m 안 사람 너머에 당근이 안 놓이고, 8 m 넘으면 표시 범위(6 m)를 벗어나 planner 가 못 본 곳까지 그린다')
    assert float(far.get('distance_forward')) > float(trunc.get('distance_forward')), '당근 조각은 검사 조각보다 길어야 한다'
    assert float(far.get('distance_backward', '1')) == 0.0

    assert pick.get('path') == far_key and pick.get('index') == '-1', '당근은 먼 조각의 마지막 점'
    carrot = pick.get('pose')
    assert carrot and carrot != '{goal}'
    assert plan.get('goal') == carrot and plan.get('path') == '{path}', (
        'planner 는 당근까지만 그리고, 그 결과가 FollowPath 의 {path} 가 된다')

    names = _params()['bt_navigator']['ros__parameters']['plugin_lib_names']
    for lib in ('nav2_truncate_path_local_action_bt_node', 'nav2_get_pose_from_path_action_bt_node',
                'nav2_is_path_valid_condition_bt_node'):
        assert lib in names, f'{lib} 미등록 — bt_navigator 가 XML 을 읽다 멈춘다'


def test_route_bt_falls_back_to_freespace_when_route_unusable():
    """레일(당근)이 안 되면 Fallback 의 두 번째 자식(목적지까지 자유주행)이 있어야 한다."""
    fb = [f for f in _bt_root().iter('Fallback') if f.get('name') == 'RouteThenFreespace']
    assert len(fb) == 1
    tags = [c.tag for c in fb[0]]
    assert tags == ['Sequence', 'Fallback'], tags
    near = fb[0][1]
    # 6판(run11): 목적지 0.25 m 안에서 매초 새 경로를 그리면 lattice 고리를 쫓아 지나친다
    # (안내소 정렬 24·19 s). 닿았으면 마지막 경로를 유지해 RotateToGoal 로 제자리 정렬.
    assert near.get('name') == 'NearGoalKeepPath'
    assert [c.tag for c in near] == ['GoalReached', 'ComputePathToPose'], (
        '0.25 m 안이면 재계획을 멈추고(GoalReached), 아니면 목적지까지 자유주행')
    assert near[0].get('goal') == '{goal}'
    assert near[1].get('goal') == '{goal}' and near[1].get('path') == '{path}'


def test_route_bt_skips_last_mile_when_already_at_goal():
    """run10 안내소→입구 31 s: 이미 도착했는데 2단계 planner 가 1.2 m 고리를 그려 한 바퀴 더.

    2단계는 GoalReached 가 실패할 때만(레일 밖 목적지) 돈다.
    """
    root = _bt_root()
    seq = [s for s in root.iter('Sequence') if s.get('name') == 'RailThenLastMile']
    assert len(seq) == 1
    assert [c.tag for c in seq[0]] == ['PipelineSequence', 'Fallback']
    assert seq[0][0].get('name') == 'RailWithReplanning'
    gate = seq[0][1]
    assert gate.get('name') == 'LastMileIfNeeded'
    assert [c.tag for c in gate] == ['GoalReached', 'PipelineSequence']
    assert gate[0].get('goal') == '{goal}'
    assert gate[0].get('robot_base_frame') == _params()['bt_navigator']['ros__parameters']['robot_base_frame']
    last = gate[1]
    assert last.get('name') == 'LastMileWithReplanning'
    assert not list(last.iter('ComputeRoute')), '2단계는 레일을 다시 보지 않는다'
    assert len(list(last.iter('ComputePathToPose'))) == 1
    assert len(list(last.iter('FollowPath'))) == 1
    assert 'nav2_goal_reached_condition_bt_node' in _params()['bt_navigator']['ros__parameters']['plugin_lib_names']


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
