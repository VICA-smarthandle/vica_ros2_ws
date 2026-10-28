"""도착 판정기 이름 계약. 2026-10-07 대기 장소 작업.

controller_server 에 판정기가 둘(general_goal_checker·position_goal_checker)이 되면 Humble 은
goal_checker_id 를 비워 둔 FollowPath 를 실패시킨다(판정기가 하나일 때만 빈 이름을 봐준다).
그래서 우리 트리의 모든 FollowPath 는 등록된 판정기 이름을 적어야 한다.

안내 트리(guided)는 레일 트리와 판정기 이름, 그리고 목적지 2 m 안 곧은 경로 갈래 두 곳(2026-10-28)만
다르다 — 레일 트리를 고치고 안내 트리를 잊으면 사용자 안내만 옛 주행을 하게 된다.

[함정] NavigateThroughPoses 기성 트리(default_nav_through_poses_bt_xml)는 이름이 비어 있다.
우리 코드는 그 액션을 쓰지 않는다. 쓰게 되면 그 트리도 우리 사본으로 바꿔야 한다.
"""
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

BT_DIR = Path(__file__).resolve().parent.parent / 'behavior_trees'
ROUTE_BT = 'vica_navigate_to_pose_route.xml'
GUIDED_BT = 'vica_navigate_to_pose_guided.xml'
GUIDED_NO_RAIL_BT = 'vica_navigate_to_pose_guided_no_rail.xml'
NO_BACKUP_BT = 'vica_navigate_to_pose_no_backup.xml'
WAIT_SPOT_BT = 'vica_navigate_to_pose_wait_spot.xml'
APPROACH_BT = 'vica_navigate_to_pose_approach.xml'
ALL_BTS = sorted(p.name for p in BT_DIR.glob('*.xml'))


def _controller():
    params = yaml.safe_load(
        (BT_DIR.parent / 'config' / 'nav2_params.yaml').read_text(encoding='utf-8'))
    return params['controller_server']['ros__parameters']


def _follows(name):
    return list(ET.parse(BT_DIR / name).getroot().iter('FollowPath'))


def _without_comments(name):
    return re.sub(r'<!--.*?-->', '', (BT_DIR / name).read_text(encoding='utf-8'), flags=re.S)


def test_both_checkers_are_registered():
    controller = _controller()
    assert controller['goal_checker_plugins'] == ['general_goal_checker', 'position_goal_checker']
    for name in controller['goal_checker_plugins']:
        assert name in controller, f'{name} 설정 블록이 없다'


def test_position_checker_only_opens_yaw():
    controller = _controller()
    general, position = controller['general_goal_checker'], controller['position_goal_checker']
    assert position['plugin'] == general['plugin']
    for key in ('xy_goal_tolerance', 'unlatch_distance'):
        assert position[key] == general[key], f'{key} 가 기본 판정기와 다르다'
    # π 를 넘으면 쿼터니언 왕복에서 음수로 감겨 '항상 방향 틀림'이 된다.
    assert 3.0 < position['yaw_goal_tolerance'] < 3.14159


@pytest.mark.parametrize('name', ALL_BTS)
def test_every_follow_path_names_a_registered_checker(name):
    follows = _follows(name)
    assert follows, f'{name} 에 FollowPath 가 없다'
    registered = set(_controller()['goal_checker_plugins'])
    for follow in follows:
        assert follow.get('goal_checker_id') in registered, (
            f'{name} 의 FollowPath 판정기 이름이 비었거나 등록되지 않았다')


def test_only_the_guided_tree_skips_heading_alignment():
    for name in ALL_BTS:
        ids = {f.get('goal_checker_id') for f in _follows(name)}
        guided = name in (GUIDED_BT, GUIDED_NO_RAIL_BT)
        expected = 'position_goal_checker' if guided else 'general_goal_checker'
        assert ids == {expected}, f'{name}: {ids}'


# ── 안내 트리의 곧은 경로 갈래 (2026-10-28) ─────────────────────────────────────
# 목적지 2 m 안을 lattice 가 그리면 저장 yaw 로 끝나는 길로 휘어 들어간다. 안내 트리만 목적지 문
# (NearGoalKeepPath·LastMile)에서 planner 앞에 곧은 경로를 둔다. 머리 주석은 guided.xml.
STRAIGHT = 'StraightPathToGoal'
STRAIGHT_LIB = 'vica_straight_path_to_goal_action_bt_node'
STRAIGHT_SEQ_NEAR = 'GoalStraightPath'            # 1단계 NearGoalKeepPath 안
STRAIGHT_WRAP_LAST = 'LastMileStraightOrPlanner'  # 2단계: Fallback(곧은 경로, 기존 planner)


def _canon(elem):
    """주석·공백을 뺀 트리 모양(태그, 속성, 자식). 두 트리를 구조로 비교한다."""
    return (elem.tag, tuple(sorted(elem.attrib.items())), tuple(_canon(c) for c in elem))


def _guided_without_straight():
    """안내 트리에서 곧은 경로 갈래 두 곳을 걷어내고 판정기 이름을 기본으로 바꾼 것."""
    root = ET.parse(BT_DIR / GUIDED_BT).getroot()
    for parent in list(root.iter()):
        for i, child in enumerate(list(parent)):
            if child.tag == 'Sequence' and child.get('name') == STRAIGHT_SEQ_NEAR:
                parent.remove(child)
            elif child.tag == 'Fallback' and child.get('name') == STRAIGHT_WRAP_LAST:
                planner = list(child)[-1]
                parent.remove(child)
                parent.insert(i, planner)
    for follow in root.iter('FollowPath'):
        if follow.get('goal_checker_id') == 'position_goal_checker':
            follow.set('goal_checker_id', 'general_goal_checker')
    return root


def _straight_seqs():
    root = ET.parse(BT_DIR / GUIDED_BT).getroot()
    near = [s for s in root.iter('Sequence') if s.get('name') == STRAIGHT_SEQ_NEAR]
    wraps = [f for f in root.iter('Fallback') if f.get('name') == STRAIGHT_WRAP_LAST]
    return root, near, wraps


def test_guided_tree_equals_route_tree_except_checker_and_straight_branches():
    route = ET.parse(BT_DIR / ROUTE_BT).getroot()
    assert _canon(_guided_without_straight()) == _canon(route), (
        '안내 트리가 레일 트리와 판정기·곧은 경로 갈래 말고도 다르다')


def test_straight_branch_sits_only_at_the_two_goal_doors():
    """곧은 경로는 목적지 문 두 곳에만, 기존 planner 갈래 **앞**에 있다. 장애물 회피(레일 당근)에는 없다."""
    root, near, wraps = _straight_seqs()
    assert len(near) == 1 and len(wraps) == 1
    keep = [f for f in root.iter('Fallback') if f.get('name') == 'NearGoalKeepPath']
    assert len(keep) == 1
    assert [c.get('name') or c.tag for c in keep[0]] == [
        'GoalReached', STRAIGHT_SEQ_NEAR, 'GoalPathAlignedEnd'], (
        '0.25 m 안이면 재계획 멈춤 -> 곧은 경로 -> 막히면 planner 순서여야 한다')
    kids = list(wraps[0])
    assert len(kids) == 2 and kids[1].get('name') == 'LastMilePathAlignedEnd'
    assert kids[0].tag == 'Sequence'
    # 1단계 레일 갈래(RouteCarrot: 레일 직접·복귀 당근·회피 당근)에는 곧은 경로가 없다.
    carrot = [s for s in root.iter('Sequence') if s.get('name') == 'RouteCarrot']
    assert len(carrot) == 1 and not list(carrot[0].iter(STRAIGHT))
    assert len(list(root.iter(STRAIGHT))) == 4   # 갈래마다 {straight_path} 1번 + {path} 1번


def test_straight_branch_uses_the_rail_handoff_distance():
    """안전장치: 레일을 놓는 거리(목적지 2 m 안)에서만 곧은 선. 목적지 문은 레일에서 1.5 m 넘게
    벗어난 회피·레일 서버 이상 때도 열리는데, 그때 목적지가 멀면 이 거리 검사로 planner 에 넘긴다."""
    root, near, wraps = _straight_seqs()
    carrot = [s for s in root.iter('Sequence') if s.get('name') == 'RouteCarrot'][0]
    gate = list(carrot)[1]
    assert gate.tag == 'IsRoutePathUsable'
    handoff = float(gate.get('handoff_dist_to_goal'))
    for seq in (near[0], list(wraps[0])[0]):
        tags = [c.tag for c in seq]
        assert tags == [STRAIGHT, 'IsRailAheadClear', STRAIGHT], tags
        first, clear, final = list(seq)
        for node in (first, final):
            assert node.get('goal') == '{goal}'
            assert float(node.get('max_dist')) == handoff, '곧은 경로 거리 = 레일 handoff 거리'
        # 검사를 통과한 뒤에만 {path} 를 덮는다(실패한 노드가 FollowPath 경로를 비우지 않게).
        assert first.get('path') == '{straight_path}' and clear.get('path') == '{straight_path}'
        assert final.get('path') == '{path}'


def test_straight_clear_check_is_not_looser_than_the_rail_check():
    """곧은 선은 planner 처럼 벽에서 떨어지려 하지 않는다 — 몸통은 같고 여유는 레일 검사 이상."""
    root, near, wraps = _straight_seqs()
    rail = [c for c in root.iter('IsRailAheadClear') if c.get('path') == '{rail_ahead}']
    assert rail
    for seq in (near[0], list(wraps[0])[0]):
        clear = list(seq)[1]
        assert clear.get('body_footprint') == rail[0].get('body_footprint')
        assert float(clear.get('footprint_padding')) >= float(rail[0].get('footprint_padding'))
        assert int(clear.get('confirm_ticks')) == int(rail[0].get('confirm_ticks'))


@pytest.mark.parametrize('name', ALL_BTS)
def test_only_the_guided_tree_uses_straight_path(name):
    """홈·배송·대기 장소·사람 접근·레일 없는 지도 트리는 그대로 planner 로 도착한다."""
    used = bool(list(ET.parse(BT_DIR / name).getroot().iter(STRAIGHT)))
    assert used == (name == GUIDED_BT), name


def test_straight_plugin_is_registered():
    params = yaml.safe_load(
        (BT_DIR.parent / 'config' / 'nav2_params.yaml').read_text(encoding='utf-8'))
    names = params['bt_navigator']['ros__parameters']['plugin_lib_names']
    assert STRAIGHT_LIB in names, (
        f'{STRAIGHT_LIB} 가 plugin_lib_names 에 없으면 bt_navigator 가 안내 트리를 읽다 멈춘다')


def test_guided_no_rail_tree_equals_default_tree_except_checker():
    # 레일 없는 지도용 안내 트리. 레일 트리는 route_server 가 없으면 만들 때부터 실패한다.
    base = _without_comments(NO_BACKUP_BT)
    guided = _without_comments(GUIDED_NO_RAIL_BT).replace(
        'goal_checker_id="position_goal_checker"', 'goal_checker_id="general_goal_checker"')
    assert guided.strip() == base.strip(), '레일 없는 안내 트리가 기본 트리와 판정기 말고도 다르다'
    assert not list(ET.parse(BT_DIR / GUIDED_NO_RAIL_BT).getroot().iter('ComputeRoute'))


def test_wait_spot_tree_is_the_approach_tree_with_shorter_recovery():
    wait = ET.parse(BT_DIR / WAIT_SPOT_BT).getroot()
    approach = ET.parse(BT_DIR / APPROACH_BT).getroot()
    assert [e.tag for e in wait.iter()] == [e.tag for e in approach.iter()], (
        '대기 장소 트리의 골격이 접근 트리와 다르다')
    outer = wait.find('.//RecoveryNode')
    assert int(outer.get('number_of_retries')) < 20
    for tag in ('BackUp', 'Spin', 'DriveOnHeading', 'ComputeRoute'):
        assert not list(wait.iter(tag)), f'대기 장소 트리에 {tag} 가 있다'
