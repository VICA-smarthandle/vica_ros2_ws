"""사람 접근 트리(vica_navigate_to_pose_approach.xml)의 계약. 2026-10-02 run60.

접근 goal 은 레일을 쓰지 않고 goal 까지 바로 자유주행한다. 레일 트리의 "목적지까지
자유주행" 갈래와 복구를 그대로 옮겼으므로, 여기서는 그 둘이 레일 트리와 어긋나지
않는지, 레일 노드가 섞이지 않았는지, 로봇을 움직이는 복구가 없는지를 지킨다.
실제 접근 동작은 실주행으로만 확인된다.
"""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

APPROACH_BT = 'vica_navigate_to_pose_approach.xml'
ROUTE_BT = 'vica_navigate_to_pose_route.xml'
RAIL_NODES = ('ComputeRoute', 'IsRoutePathUsable', 'TruncatePathLocal', 'GetPoseFromPath')
REVERSE_CAPABLE_NODES = ('BackUp', 'DriveOnHeading', 'AssistedTeleop')
# 각 BT 노드 이름 -> plugin_lib_names 에 있어야 하는 라이브러리
NODE_LIBS = {
    'ComputePathToPose': 'nav2_compute_path_to_pose_action_bt_node',
    'FollowPath': 'nav2_follow_path_action_bt_node',
    'GoalReached': 'nav2_goal_reached_condition_bt_node',
    'ClearEntireCostmap': 'nav2_clear_costmap_service_bt_node',
    'Wait': 'nav2_wait_action_bt_node',
    'RateController': 'nav2_rate_controller_bt_node',
    'RecoveryNode': 'nav2_recovery_node_bt_node',
    'PipelineSequence': 'nav2_pipeline_sequence_bt_node',
    'RoundRobin': 'nav2_round_robin_node_bt_node',
    'GoalUpdated': 'nav2_goal_updated_condition_bt_node',
    'AlignPathEndToGoal': 'vica_align_path_end_to_goal_action_bt_node',
}


def _pkg_dir():
    return Path(__file__).resolve().parent.parent


def _root(name=APPROACH_BT):
    return ET.parse(_pkg_dir() / 'behavior_trees' / name).getroot()


def _params():
    return yaml.safe_load(
        (_pkg_dir() / 'config' / 'nav2_params.yaml').read_text(encoding='utf-8'))


@pytest.mark.parametrize('node_name', RAIL_NODES)
def test_approach_bt_has_no_rail_node(node_name):
    assert not list(_root().iter(node_name)), f'접근 트리에 레일 노드 {node_name} 가 있다'


def test_approach_bt_plans_straight_to_the_goal_with_lattice():
    plans = list(_root().iter('ComputePathToPose'))
    assert plans and all(p.get('goal') == '{goal}' for p in plans)
    assert all(p.get('planner_id') == 'GridBased' for p in plans)


def test_approach_bt_follows_with_the_same_controller():
    follows = list(_root().iter('FollowPath'))
    route = list(_root(ROUTE_BT).iter('FollowPath'))
    assert follows and {f.get('controller_id') for f in follows} == \
        {f.get('controller_id') for f in route}


def test_approach_bt_aligns_path_end_and_keeps_the_near_goal_gate():
    """사람을 마주 보는 goal 방향(9판)과 0.25 m 안 재계획 중단(lattice 고리)을 지킨다."""
    root = _root()
    for seq in root.iter('Sequence'):
        kids = list(seq)
        for i, c in enumerate(kids):
            if c.tag == 'AlignPathEndToGoal':
                assert i > 0 and kids[i - 1].tag == 'ComputePathToPose'
    assert list(root.iter('AlignPathEndToGoal'))
    gate = [f for f in root.iter('Fallback') if f.get('name') == 'NearGoalKeepPath']
    assert gate and list(gate[0])[0].tag == 'GoalReached'


def test_approach_bt_recovery_matches_the_rail_tree():
    """복구 예산 20회 x (costmap 비우기, Wait 1 s) — 레일 트리와 같아야 한다."""
    def outer(root):
        return next(iter(root.iter('RecoveryNode')))

    assert outer(_root()).get('number_of_retries') == outer(_root(ROUTE_BT)).get('number_of_retries')
    assert [w.get('wait_duration') for w in _root().iter('Wait')] == \
        [w.get('wait_duration') for w in _root(ROUTE_BT).iter('Wait')]


@pytest.mark.parametrize('node_name', REVERSE_CAPABLE_NODES + ('Spin',))
def test_approach_bt_has_no_moving_recovery(node_name):
    assert not list(_root().iter(node_name)), f'{node_name} 는 접근 트리에도 못 들어간다'


def test_every_approach_bt_node_plugin_is_registered():
    names = _params()['bt_navigator']['ros__parameters']['plugin_lib_names']
    used = {e.tag for e in _root().iter()} & set(NODE_LIBS)
    missing = [NODE_LIBS[t] for t in used if NODE_LIBS[t] not in names]
    assert not missing, f'plugin_lib_names 에 없으면 bt_navigator 가 트리를 못 읽는다: {missing}'
