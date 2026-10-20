"""도착 판정기 이름 계약. 2026-10-07 대기 장소 작업."""
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


def test_guided_tree_equals_route_tree_except_checker():
    route = _without_comments(ROUTE_BT)
    guided = _without_comments(GUIDED_BT).replace(
        'goal_checker_id="position_goal_checker"', 'goal_checker_id="general_goal_checker"')
    assert guided.strip() == route.strip(), '안내 트리가 레일 트리와 판정기 말고도 다르다'


def test_guided_no_rail_tree_equals_default_tree_except_checker():
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
