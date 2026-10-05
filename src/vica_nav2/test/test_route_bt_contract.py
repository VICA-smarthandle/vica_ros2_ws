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
# 2026-10-05 11판(해결안 B): 레일 앞 조각 검사는 IsPathValid 대신 몸통·0.5 m 건너뛰기 검사.
CLEAR = 'IsRailAheadClear'
GUARD_LIB = 'vica_is_route_path_usable_condition_bt_node'
MAP = 'vica_map_0630'
# 레일 파일 시험을 받는 지도. 2026-09-30 전에는 0630 하나만 봐서 0903_d 레일은 시험이 0 이었다.
RAIL_MAPS = ('vica_map_0630', 'vica_map_0903_d')
RING_MAPS = ('vica_map_0630',)   # 고리형. 0903_d 는 나무형이라 끝 노드 이웃이 1개다
JUNCTION_PENALTY = 0.1     # scripts/vica_route_graph.py 의 JUNCTION_PENALTY 와 같아야 한다
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


def _graph(map_name=MAP, with_penalty=False):
    p = _ws_maps() / f'{map_name}_route.geojson'
    if not p.is_file():
        pytest.skip(f'레일 파일이 없다: {p} — scripts/vica_route_graph.py {map_name}')
    g = json.loads(p.read_text(encoding='utf-8'))
    nodes, edges, penalty = {}, [], {}
    for f in g['features']:
        if f['geometry']['type'] == 'Point':
            nodes[f['properties']['id']] = tuple(f['geometry']['coordinates'])
        else:
            a, b = f['properties']['startid'], f['properties']['endid']
            edges.append((a, b))
            meta = f['properties'].get('metadata') or {}
            penalty[(a, b)] = float(meta.get('penalty', 0.0))
    if with_penalty:
        return nodes, edges, penalty
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
    # 8판-2(run19): 모드 셋 — 레일 직접(가깝고 비면) / 복귀 당근 3 m(멀지만 비면) / 회피 당근 6 m(막히면).
    assert [c.tag for c in mode] == ['Sequence', 'Sequence', 'Sequence'] and \
        [c.get('name') for c in mode] == ['RailDirect', 'RejoinCarrot', 'CarrotBeyond']
    rail_direct, rejoin, carrot_seq = mode[0], mode[1], mode[2]
    assert [c.tag for c in rejoin] == [CLEAR, 'GetPoseFromPath', 'Fallback'], '복귀 = 조각 비었는지 → 3 m 조각 끝점 → planner'
    assert rejoin[0].get('path') == trunc.get('output_path') and rejoin[1].get('path') == trunc.get('output_path')
    near_carrot = rejoin[1].get('pose')
    assert near_carrot and near_carrot not in ('{goal}', '{carrot}'), '복귀 당근은 회피 당근과 다른 키(bag 에서 갈라 세기 위해)'
    assert [c.tag for c in rejoin[2]] == ['ComputePathToPose', 'ComputeRoute'] and rejoin[2][0].get('goal') == near_carrot \
        and rejoin[2][0].get('path') == '{path}' and rejoin[2][1].get('path') == '{path}'
    # 8판: 앞 3 m 조각이 비어 있으면 레일 그대로. IsPathValid 는 조각만 본다(레일 전체를 보던 3판 사고 금지).
    # 8판-1(run18): 레일에서 가까울 때만 — 멀리서 레일 모드가 켜지면 DWB 가 옆 선에 붙느라 지그재그(직진 w 표준편차 0.23).
    assert [c.tag for c in rail_direct] == [GUARD, CLEAR, 'ComputeRoute'], '레일 직접 = 가까움 검사 → 3 m 조각 검사 → 레일을 {path} 에'
    near_gate = rail_direct[0]
    assert near_gate.get('path') == seqs[0][0].get('path') and near_gate.get('goal') == '{goal}'
    assert 0.6 <= float(near_gate.get('max_dist_from_path', '0')) <= 1.0, (
        '0.6 m 미만이면 정상 주행(0.5 m 밖 13 %)까지 걸어 당근 모드가 되고(run19), 1.0 m 넘으면 호 U턴 끝(1.3~1.4 m)에서 옆 선에 붙는 지그재그가 돌아온다(run18)')
    assert float(near_gate.get('max_dist_from_path')) < float(seqs[0][1].get('max_dist_from_path')), '안쪽 문은 바깥 거름망보다 좁아야 한다'
    # 10판(10-01): 안쪽 문은 문턱 두 개 — 나갈 때 0.8, 돌아올 때 그보다 안쪽. 같으면 경계에서 1 Hz 로 경로가 번갈아 간다.
    rejoin = float(near_gate.get('rejoin_dist_from_path', '-1'))
    assert 0.0 < rejoin < float(near_gate.get('max_dist_from_path')), (
        'rejoin_dist_from_path 는 0 보다 크고 max_dist_from_path 보다 작아야 한다(run52~54 경로 교대 2.9~6.9회/분)')
    assert rail_direct[2].get('path') == '{path}' and rail_direct[2].get('goal') == '{goal}'
    valids = list(seqs[0].iter(CLEAR))
    assert len(valids) == 2 and all(v.get('path') == trunc.get('output_path') for v in valids), (
        f'{CLEAR} 는 앞 3 m 조각({rail_ahead})만 검사한다(레일 직접·복귀 두 곳). 레일 전체({rail_path})를 주면 run9 의 "치명 칸 하나에 레일 통째 거부" 가 돌아온다')
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

    # 8판-3(run21·22 뒤): 막혔을 때 당근은 3 m. 6 m 로 늘린 근거(회피 폭)는 실주행으로 반증됐고
    # (횡 이탈 3 m 0.27 vs 6 m 0.25~0.37), U턴 창 경로의 옆 폭이 0.52 → 3.0~4.3 m 로 커져
    # 레일 이탈 → 안쪽 문 실패 58 % 를 만들었다. 0630 run13 은 3 m 로 15/15.
    assert far.get('input_path') == rail_key and far.get('robot_frame') == base
    far_key = far.get('output_path')
    assert far_key and far_key not in ('{path}',), '당근 조각은 {path} 를 덮으면 안 된다'
    assert 2.5 <= float(far.get('distance_forward', '0')) <= 6.0, (
        '2.5 m 미만이면 당근이 너무 가까워 planner 가 매초 급하게 꺾고, 6 m 넘으면 U턴에서 경로가 옆으로 3 m 이상 '
        '벌어져 레일 이탈 → 안쪽 문 실패를 만든다(devlog §14)')
    assert float(far.get('distance_forward')) >= float(trunc.get('distance_forward')), (
        '당근 조각은 검사 조각보다 짧으면 안 된다(검사한 곳 너머를 목표로 삼게 된다)')
    assert float(far.get('distance_backward', '1')) == 0.0

    assert pick.get('path') == far_key and pick.get('index') == '-1', '당근은 먼 조각의 마지막 점'
    carrot = pick.get('pose')
    assert carrot and carrot != '{goal}'
    assert plan.get('goal') == carrot and plan.get('path') == '{path}', (
        'planner 는 당근까지만 그리고, 그 결과가 FollowPath 의 {path} 가 된다')

    names = _params()['bt_navigator']['ros__parameters']['plugin_lib_names']
    for lib in ('nav2_truncate_path_local_action_bt_node', 'nav2_get_pose_from_path_action_bt_node',
                'vica_is_rail_ahead_clear_condition_bt_node'):
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
    assert [c.tag for c in near] == ['GoalReached', 'Sequence'], (
        '0.25 m 안이면 재계획을 멈추고(GoalReached), 아니면 목적지까지 자유주행')
    assert near[0].get('goal') == '{goal}'
    _assert_goal_path_then_align_end(near[1])


ALIGN_END = 'AlignPathEndToGoal'
ALIGN_END_LIB = 'vica_align_path_end_to_goal_action_bt_node'


def _assert_goal_path_then_align_end(seq):
    """9판(run49): 목적지 경로 = ComputePathToPose(goal) 바로 뒤 AlignPathEndToGoal, 같은 {path} 에.

    끝점 방향만 목적지 방향으로 바꾼다. 항상 SUCCESS 라 Sequence 가 planner 결과를 그대로 전한다.
    Fallback 에 직접 두면 planner 성공 때 뒤 자식이 안 돌고, 실패 때만 돈다 — 그래서 Sequence 로 묶는다.
    """
    assert seq.tag == 'Sequence'
    assert [c.tag for c in seq] == ['ComputePathToPose', ALIGN_END], [c.tag for c in seq]
    plan, align = seq[0], seq[1]
    assert plan.get('goal') == '{goal}' and plan.get('path') == '{path}'
    assert align.get('goal') == '{goal}'
    assert align.get('input_path') == '{path}' and align.get('output_path') == '{path}'
    # goal checker 도장이 풀리는 거리와 같다 — 그 밖의 끝점은 목적지가 아니다(당근·중간 경로).
    unlatch = _params()['controller_server']['ros__parameters']['general_goal_checker'].get(
        'unlatch_distance', 0.5)
    assert float(align.get('max_dist', '0')) == unlatch


def test_align_end_is_only_after_goal_paths():
    """당근(레일 앞 3 m)·복귀 경로의 끝은 목적지가 아니므로 방향을 바꾸지 않는다."""
    root = _bt_root()
    aligns = list(root.iter(ALIGN_END))
    assert len(aligns) == 2, '목적지 경로는 NearGoalKeepPath 와 LastMile 두 곳뿐이다'
    for rc in root.iter('Sequence'):
        kids = list(rc)
        for i, c in enumerate(kids):
            if c.tag == ALIGN_END:
                assert i > 0 and kids[i - 1].tag == 'ComputePathToPose' and \
                    kids[i - 1].get('goal') == '{goal}'


def test_align_end_plugin_is_registered_and_built():
    names = _params()['bt_navigator']['ros__parameters']['plugin_lib_names']
    assert ALIGN_END_LIB in names, (
        f'{ALIGN_END_LIB} 가 plugin_lib_names 에 없으면 bt_navigator 가 XML 을 읽다 멈춘다')


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
    compute = [r for r in last.iter('RecoveryNode') if r.get('name') == 'ComputeLastMilePath']
    assert len(compute) == 1
    _assert_goal_path_then_align_end(compute[0][0])
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

def _adjacency(edges):
    adj = {}
    for a, b in edges:
        adj.setdefault(a, set()).add(b)
    return adj


def _turn_deg(a, b, c):
    v1 = (b[0] - a[0], b[1] - a[1]); v2 = (c[0] - b[0], c[1] - b[1])
    return abs(math.degrees(math.atan2(v1[0]*v2[1] - v1[1]*v2[0], v1[0]*v2[0] + v1[1]*v2[1])))


@pytest.mark.parametrize('map_name', RAIL_MAPS)
def test_route_graph_edges_are_short(map_name):
    """모든 엣지 <= 1 m. 6.71 m 엣지가 run5 를 세웠다."""
    nodes, edges = _graph(map_name)
    long = [(a, b, round(math.dist(nodes[a], nodes[b]), 2)) for a, b in edges
            if math.dist(nodes[a], nodes[b]) > MAX_EDGE_M + EDGE_SLACK_M]
    assert not long, f'{MAX_EDGE_M} m 를 넘는 엣지: {long}'


@pytest.mark.parametrize('map_name', RAIL_MAPS)
def test_route_graph_corners_are_rounded(map_name):
    """run8: smooth_corners 를 끄니 코너가 뾰족해 DWB 가 지나쳤다 되돌아왔다(w ±0.29).
    생성기가 15~120° 코너를 호로 바꾼다. 30~120° 로 한 번에 꺾이는 노드가 남으면 안 된다.
    120° 초과는 목적지 스퍼의 되돌림이라 예외.

    이웃이 둘인 노드만 본다 — 실제로 이어진 앞뒤 노드의 각이다(2026-09-30). 전에는 id 순서를
    고리 순서로 가정해 나무형(0903_d)에서 엉뚱한 세 점을 쟀다. 갈림길(이웃 3+)은 Y 호가 맡는다."""
    nodes, edges = _graph(map_name)
    sharp = []
    for k, nb in _adjacency(edges).items():
        if len(nb) != 2:
            continue
        a, c = sorted(nb)
        ang = _turn_deg(nodes[a], nodes[k], nodes[c])
        if 30 <= ang <= 120:
            sharp.append((k, round(ang)))
    assert not sharp, f'뾰족한 코너가 남았다 (노드, 각도): {sharp}'


@pytest.mark.parametrize('map_name', RAIL_MAPS)
def test_route_graph_is_bidirectional_and_connected(map_name):
    """엣지는 방향이 있다. 양쪽으로 다니려면 쌍이어야 하고, 섬이 있으면 안 된다."""
    nodes, edges = _graph(map_name)
    es = set(edges)
    missing = [(a, b) for a, b in edges if (b, a) not in es]
    assert not missing, f'되돌아오는 엣지가 없다: {missing[:5]}'
    adj = _adjacency(edges)
    seen, stack = set(), [next(iter(nodes))]
    while stack:
        n = stack.pop()
        if n in seen:
            continue
        seen.add(n)
        stack.extend(adj.get(n, ()))
    assert seen == set(nodes), '레일이 끊겨 섬이 있다'


@pytest.mark.parametrize('map_name', RING_MAPS)
def test_route_graph_ring_has_no_dead_end(map_name):
    """고리형은 모든 노드의 이웃이 2개 이상이다."""
    nodes, edges = _graph(map_name)
    adj = _adjacency(edges)
    assert all(len(adj.get(n, ())) >= 2 for n in nodes), '이웃이 2개 미만인 노드가 있다'


@pytest.mark.parametrize('map_name', RAIL_MAPS)
def test_route_graph_passes_through_registered_destinations(map_name):
    """레일은 미션 매니저가 실제로 쓰는 목적지를 지나가야 한다.

    2026-09-16 1판은 저장소의 locations.json 을 읽어 안내소가 1.03 m 빗나갔다.
    나무형은 등뼈에서 2 m(BT handoff) 안 목적지에 곁가지를 내지 않으므로 그 거리를 쓴다.
    """
    if map_name == 'vica_map_0903_d':
        pytest.xfail('0903_d 레일은 09-17 에 깔아 그 뒤 장소(학과사무실, 레일에서 10.3 m)가 빠져 있다. '
                     '재생성은 벌점과 다른 회차에서(설계서 1.6-4)')
    doc = Path.home() / 'vica_data' / 'destinations' / map_name / 'destinations.yaml'
    if not doc.is_file():
        pytest.skip(f'이 기기엔 목적지 파일이 없다: {doc}')
    nodes, _ = _graph(map_name)
    limit = DEST_TO_NODE_M if map_name in RING_MAPS else 2.0
    far = []
    for d in (yaml.safe_load(doc.read_text(encoding='utf-8')) or {}).get('destinations', []):
        po = d.get('pose') or {}
        if po.get('x') is None:
            continue
        dist = min(math.dist((po['x'], po['y']), xy) for xy in nodes.values())
        if dist > limit:
            far.append((d.get('name'), round(dist, 2)))
    assert not far, f'레일에서 {limit} m 넘게 떨어진 목적지: {far}'


# ── 갈림길 벌점 (2026-09-30, 설계서 2026-09-28-app-route-editor-design.md 1부) ──

def test_route_server_uses_penalty_scorer():
    rs = _params()['route_server']['ros__parameters']
    assert 'PenaltyScorer' in rs['edge_cost_functions']
    assert rs['PenaltyScorer']['plugin'] == 'nav2_route::PenaltyScorer'


def _route_like_server(nodes, adj, cost, start_xy, goal_xy):
    """humble route_server 흉내: 최근접 노드 출발·도착 + 첫 노드 지나쳤으면 잘라냄
    (goal_intent_extractor.cpp, dot>0 · 0.10 m) + 다익스트라(거리+벌점)."""
    import heapq
    s = min(nodes, key=lambda n: math.dist(nodes[n], start_xy))
    t = min(nodes, key=lambda n: math.dist(nodes[n], goal_xy))
    dist, prev, q = {s: 0.0}, {}, [(0.0, s)]
    while q:
        c, u = heapq.heappop(q)
        if u == t:
            break
        if c > dist[u]:
            continue
        for v in adj.get(u, ()):
            nc = c + cost(u, v)
            if nc < dist.get(v, 1e18):
                dist[v], prev[v] = nc, u
                heapq.heappush(q, (nc, v))
    if t not in dist:
        return None
    path = [t]
    while path[-1] != s:
        path.append(prev[path[-1]])
    path.reverse()
    if len(path) > 1:
        a, b = nodes[path[0]], nodes[path[1]]
        vr = (b[0] - a[0], b[1] - a[1]); vp = (start_xy[0] - a[0], start_xy[1] - a[1])
        nr, np_ = math.hypot(*vr), math.hypot(*vp)
        if np_ > 0.10 and nr > 0 and (vr[0]*vp[0] + vr[1]*vp[1]) / (nr * np_) > 1e-4:
            path = path[1:]
    return path


@pytest.mark.parametrize('map_name', RAIL_MAPS)
def test_route_graph_junction_has_no_detour(map_name):
    """갈림길 옆을 직진할 때 경로가 곁가지 호로 V자로 내려갔다 오면 안 된다.

    run40~46: 0903_d 갈림길 직진 6회 중 4회 V자. 호 첫 노드가 큰길에서 0.05 m 라 '가장
    가까운 노드'가 호 위였다. 갈림길(이웃 3+) 옆 곧게 이어진 두 엣지 위를 5 cm·좌우 ±0.15 m
    로 훑어 모든 끝 노드로 경로를 낸다. 경로가 큰길 직선에서 0.3 m 넘게 벗어났다가 0.1 m
    안으로 **돌아오면** V자다. 곁가지로 가는 경로는 벗어난 채 돌아오지 않으므로 걸리지 않는다.
    판정은 모양으로만 한다 — 벌점 표시가 없는 옛 레일(벌점 전)에서 이 시험이 실패해야 한다."""
    nodes, edges, penalty = _graph(map_name, with_penalty=True)
    adj = _adjacency(edges)
    weight = float(_params()['route_server']['ros__parameters']
                   .get('PenaltyScorer', {}).get('weight', 1.0))
    cost = lambda u, v: math.dist(nodes[u], nodes[v]) + weight * penalty.get((u, v), 0.0)  # noqa: E731
    ends = [n for n, nb in adj.items() if len(nb) == 1]

    def off_line(xy, A, C):
        dx, dy = C[0] - A[0], C[1] - A[1]
        return abs(dx * (xy[1] - A[1]) - dy * (xy[0] - A[0])) / math.hypot(dx, dy)

    bad = []
    for j, nb in adj.items():
        if len(nb) < 3:
            continue
        pairs = [(a, c) for a in nb for c in nb if a < c
                 and _turn_deg(nodes[a], nodes[j], nodes[c]) < 15]
        for a, c in pairs:
            A, C = nodes[a], nodes[c]
            for p, q in ((a, j), (j, c)):
                P, Q = nodes[p], nodes[q]
                L = math.dist(P, Q)
                ux, uy = (Q[0] - P[0]) / L, (Q[1] - P[1]) / L
                for i in range(int(L / 0.05) + 1):
                    for off in (-0.15, 0.0, 0.15):
                        x = P[0] + ux * i * 0.05 - uy * off
                        y = P[1] + uy * i * 0.05 + ux * off
                        for e in ends:
                            path = _route_like_server(nodes, adj, cost, (x, y), nodes[e])
                            if not path:
                                continue
                            d = [off_line(nodes[n], A, C) for n in path[:15]]
                            left = next((k for k, v in enumerate(d) if v > 0.3), None)
                            if left is not None and any(v < 0.1 for v in d[left:]):
                                bad.append((j, round(x, 2), round(y, 2), e))
    assert not bad, f'직진인데 갈림길 호로 돌아간 자리 {len(bad)}곳 (갈림길, x, y, 목적 끝): {bad[:5]}'


@pytest.mark.parametrize('map_name', RAIL_MAPS)
def test_route_graph_penalty_only_on_junction_arcs(map_name):
    """벌점은 갈림길 호에만. 갈림길이 있으면 벌점 엣지가 있어야 하고, 벌점 값은 생성기와 같다.
    갈림길 없는 고리형(0630)에는 벌점이 하나도 없어야 한다 — 코너 수로 좌/우 선택이 흔들린다."""
    nodes, edges, penalty = _graph(map_name, with_penalty=True)
    adj = _adjacency(edges)
    junctions = [n for n, nb in adj.items() if len(nb) >= 3]
    values = {v for v in penalty.values() if v > 0}
    if not junctions:
        assert not values, '갈림길이 없는데 벌점 엣지가 있다'
        return
    assert values == {JUNCTION_PENALTY}
    for (a, b), v in penalty.items():
        assert penalty.get((b, a)) == v, f'벌점이 한 방향에만 있다: {(a, b)}'


def test_rail_ahead_check_uses_body_without_handle_and_skips_where_robot_stands():
    """11판(2026-10-05, 해결안 B). run61·63: IsPathValid 막힘 68·31번 중 첫 접촉이 손잡이 꼬리 38·20번 —
    로봇이 서 있는 자리의 뒤 사람·곡선에서 휘는 꼬리 옆 벽을 '앞이 막혔다'로 읽어 레일을 버렸다.

    몸통은 global_costmap footprint 에서 꼬리(뒤 끝보다 뒤로 나온 점)만 뺀 사각형이어야 한다 — 앞·옆을
    줄이면 진짜 막힘을 놓친다. 패딩도 같아야 한다. 건너뛰는 거리는 그 지점 몸통 뒤 끝이 로봇 뒤(손잡이
    뒤 사람)까지 닿지 않을 만큼(>= 0.4), 앞길을 놓치지 않을 만큼(<= 1.0). 막힘 확인은 2번 이상.
    """
    import ast
    gc = _params()['global_costmap']['global_costmap']['ros__parameters']
    full = ast.literal_eval(gc['footprint'])
    pad = float(gc['footprint_padding'])
    front = max(p[0] for p in full)
    half = max(abs(p[1]) for p in full)
    body_rear = max(p[0] for p in full if abs(abs(p[1]) - half) < 1e-9 and p[0] < 0)   # 옆선이 끝나는 뒤 모서리
    for node in _bt_root().iter(CLEAR):
        body = ast.literal_eval(node.get('body_footprint'))
        assert max(p[0] for p in body) == front and max(abs(p[1]) for p in body) == half, '몸통 앞·옆은 footprint 그대로'
        assert min(p[0] for p in body) == body_rear, '몸통 뒤 끝 = 꼬리가 시작되는 모서리(꼬리만 뺀다)'
        assert float(node.get('footprint_padding')) == pad
        skip = float(node.get('skip_distance'))
        assert 0.4 <= skip <= 1.0, skip
        assert skip + (body_rear - pad) > -0.62 + 0.4, '건너뛴 지점 몸통 뒤 끝이 손잡이 뒤 사람 자리에 닿으면 안 된다'
        assert int(node.get('confirm_ticks')) >= 2
