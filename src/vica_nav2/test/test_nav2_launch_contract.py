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


def test_velocity_smoother_output_is_scoped_to_safety_input(
    monkeypatch,
    tmp_path,
):
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

    assert len(remaps) == 1
    assert len(includes) == 1

    context = LaunchContext()
    assert perform_substitutions(context, remaps[0].src) == 'cmd_vel_smoothed'
    assert perform_substitutions(context, remaps[0].dst) == '/cmd_vel_req'
