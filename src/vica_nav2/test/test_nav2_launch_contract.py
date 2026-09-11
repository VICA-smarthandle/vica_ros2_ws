import importlib.util
import re
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node, SetRemap


def _load_launch_module():
    launch_path = (
        Path(__file__).parents[1] / 'launch' / 'nav2_map_test.launch.py'
    )
    spec = importlib.util.spec_from_file_location(
        'vica_nav2_map_test_launch',
        launch_path,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _remap_pairs(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    launch_description = _load_launch_module().generate_launch_description()
    groups = [
        entity
        for entity in launch_description.entities
        if isinstance(entity, GroupAction)
    ]

    assert len(groups) == 1

    actions = groups[0].get_sub_entities()
    remaps = [action for action in actions if isinstance(action, SetRemap)]
    includes = [
        action
        for action in actions
        if isinstance(action, IncludeLaunchDescription)
    ]

    assert len(includes) == 1

    context = LaunchContext()
    return {
        perform_substitutions(context, remap.src):
            perform_substitutions(context, remap.dst)
        for remap in remaps
    }


def _group_nodes(monkeypatch, tmp_path):
    """GroupAction 안에 launch 가 직접 띄우는 Node 를 이름으로 모은다."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    launch_description = _load_launch_module().generate_launch_description()
    groups = [
        entity
        for entity in launch_description.entities
        if isinstance(entity, GroupAction)
    ]
    assert len(groups) == 1

    context = LaunchContext()

    def text(value):
        if isinstance(value, str):
            return value
        if isinstance(value, (list, tuple)):
            return ''.join(text(item) for item in value)
        return perform_substitutions(context, [value])

    nodes = {}
    for action in groups[0].get_sub_entities():
        if not isinstance(action, Node):
            continue
        nodes[text(action.node_executable)] = text(action.node_package)
    return nodes


def _params():
    config = (
        Path(__file__).parents[1] / 'config' / 'nav2_params.yaml'
    ).read_text(encoding='utf-8')
    return yaml.safe_load(config)


def _smoother_output_topic():
    """velocity_smoother 가 '실제로' 발행하는 토픽 이름을 알아낸다."""
    launch_file = Path(
        '/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py'
    )
    if not launch_file.is_file():
        pytest.skip(f'nav2_bringup launch 없음: {launch_file}')
    text = launch_file.read_text(encoding='utf-8')
    match = re.search(
        r"\(\s*['\"]cmd_vel_smoothed['\"]\s*,\s*['\"]([^'\"]+)['\"]\s*\)", text
    )
    if not match:
        return 'cmd_vel_smoothed'
    return match.group(1)


def test_velocity_smoother_output_reaches_safety_through_collision_monitor(
    monkeypatch,
    tmp_path,
):
    """속도 명령의 마지막 구간이 Collision Monitor 를 거쳐야 한다."""
    pairs = _remap_pairs(monkeypatch, tmp_path)
    assert 'cmd_vel_smoothed' not in pairs, (
        'cmd_vel_smoothed 에 remap 이 남아 있다. velocity_smoother 가'
        ' collision_monitor 를 건너뛰고 /cmd_vel_req 로 직접 발행하게 된다'
    )

    nodes = _group_nodes(monkeypatch, tmp_path)
    assert nodes.get('collision_monitor') == 'nav2_collision_monitor', (
        f'launch 가 collision_monitor 를 띄우지 않는다: {nodes}'
    )

    expected_in = _smoother_output_topic()
    monitor = _params()['collision_monitor']['ros__parameters']
    assert monitor['cmd_vel_in_topic'].lstrip('/') == expected_in.lstrip('/'), (
        f'monitor 입력 {monitor["cmd_vel_in_topic"]} 이 velocity_smoother 의 실제'
        f' 출력 /{expected_in} 과 다르다. 다른 토픽을 보면 아무것도 받지 못하고,'
        ' 그러면 /cmd_vel_req 가 조용히 비어 로봇이 한 발도 못 움직인다'
        ' (2026-08-15 실기에서 실제로 발생)'
    )
    assert monitor['cmd_vel_out_topic'] == '/cmd_vel_req', (
        f'monitor 출력 {monitor["cmd_vel_out_topic"]} 이 Safety 입력이 아니다.'
        ' CLAUDE.md 의 /cmd_vel_req 계약이 깨진다'
    )


def test_collision_monitor_is_lifecycle_managed(monkeypatch, tmp_path):
    """관리자가 없으면 노드가 unconfigured 로 남아 아무 일도 하지 않는다."""
    nodes = _group_nodes(monkeypatch, tmp_path)
    assert nodes.get('lifecycle_manager') == 'nav2_lifecycle_manager', (
        f'collision_monitor 를 올려 줄 lifecycle_manager 가 없다: {nodes}'
    )


def test_collision_monitor_has_no_rear_polygon():
    """후방 감시 영역은 절대 두지 않는다."""
    monitor = _params()['collision_monitor']['ros__parameters']
    front_edge = 0.355

    for name in monitor['polygons']:
        points = monitor[name]['points']
        xs = points[0::2]
        assert min(xs) >= front_edge, (
            f'{name} 이 x={min(xs)} 까지 뻗어 차체 앞단 {front_edge} 안으로'
            ' 들어온다. 뒤쪽이면 손잡이를 잡은 사용자가 상시 검출된다'
        )


def test_recovery_behaviors_reach_the_safety_path(monkeypatch, tmp_path):
    """behavior_server의 속도 명령도 /cmd_vel_req로 나가야 한다."""
    pairs = _remap_pairs(monkeypatch, tmp_path)
    assert pairs['behavior_server:cmd_vel'] == '/cmd_vel_req'


def test_no_unscoped_cmd_vel_remap_hijacks_the_controller(
    monkeypatch,
    tmp_path,
):
    """`cmd_vel` 전역 remap은 절대 넣으면 안 된다."""
    pairs = _remap_pairs(monkeypatch, tmp_path)
    assert 'cmd_vel' not in pairs
    for src in pairs:
        if src.endswith(':cmd_vel'):
            assert ':' in src, f'{src}는 노드 지정 remap이어야 한다'
