import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import SetRemap


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


def test_velocity_smoother_output_is_scoped_to_safety_input(
    monkeypatch,
    tmp_path,
):
    pairs = _remap_pairs(monkeypatch, tmp_path)
    assert pairs['cmd_vel_smoothed'] == '/cmd_vel_req'


def test_recovery_behaviors_reach_the_safety_path(monkeypatch, tmp_path):
    """behavior_server의 속도 명령도 /cmd_vel_req로 나가야 한다.

    Nav2 humble navigation_launch.py는 controller_server에만
    ('cmd_vel', 'cmd_vel_nav')를 준다. behavior_server는 remap을 못 받아
    /cmd_vel로 발행하는데 VICA에는 그 토픽 구독자가 없다.

    2026-07-29 실측: /cmd_vel 발행자 5(전부 behavior_server), 구독자 0.
    Spin·BackUp이 한 번도 로봇을 움직이지 못했고, planner가 실패하면 복구
    6회가 전부 헛돌아 즉시 Goal failed가 됐다. 갇히면 빠져나올 수단이 없었다.
    """
    pairs = _remap_pairs(monkeypatch, tmp_path)
    assert pairs['behavior_server:cmd_vel'] == '/cmd_vel_req'


def test_no_unscoped_cmd_vel_remap_hijacks_the_controller(
    monkeypatch,
    tmp_path,
):
    """`cmd_vel` 전역 remap은 절대 넣으면 안 된다.

    launch_ros node.py(468~477행)가 global remap을 node-level보다 **먼저**
    붙이고 rcl은 첫 일치 규칙을 쓴다. 따라서 접두사 없는 cmd_vel remap은
    controller_server의 cmd_vel:=cmd_vel_nav를 덮어써 velocity_smoother를
    건너뛰고, velocity_smoother는 자기 출력을 구독하는 자기루프가 된다.
    노드 지정 문법(node_name:from:=to)만 허용한다.
    """
    pairs = _remap_pairs(monkeypatch, tmp_path)
    assert 'cmd_vel' not in pairs
    for src in pairs:
        if src.endswith(':cmd_vel'):
            assert ':' in src, f'{src}는 노드 지정 remap이어야 한다'
