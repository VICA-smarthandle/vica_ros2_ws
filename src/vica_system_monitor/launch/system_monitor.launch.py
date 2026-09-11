"""Launch the VICA observation layer: adapter, aggregator and monitor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE = 'vica_system_monitor'

AGG_TOPIC = '/diagnostics_agg'
RAW_TOPIC = '/diagnostics'


def generate_launch_description() -> LaunchDescription:
    """Build the observation-layer launch description."""
    share = get_package_share_directory(PACKAGE)
    config = os.path.join(share, 'config')

    probes_yaml = os.path.join(config, 'probes.yaml')
    aggregator_yaml = os.path.join(config, 'diagnostic_aggregator.yaml')
    components_yaml = os.path.join(config, 'required_components.yaml')

    enable_aggregator = LaunchConfiguration('enable_aggregator')

    adapter = Node(
        package=PACKAGE,
        executable='external_diagnostics_node',
        name='external_diagnostics_node',
        parameters=[probes_yaml],
        output='screen',
    )

    aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='aggregator_node',
        parameters=[aggregator_yaml],
        output='screen',
        condition=IfCondition(enable_aggregator),
    )

    monitor_with_agg = Node(
        package=PACKAGE,
        executable='robot_health_monitor_node',
        name='robot_health_monitor_node',
        parameters=[components_yaml, {'diagnostics_topic': AGG_TOPIC}],
        output='screen',
        condition=IfCondition(enable_aggregator),
    )

    monitor_standalone = Node(
        package=PACKAGE,
        executable='robot_health_monitor_node',
        name='robot_health_monitor_node',
        parameters=[components_yaml, {'diagnostics_topic': RAW_TOPIC}],
        output='screen',
        condition=UnlessCondition(enable_aggregator),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'enable_aggregator',
                default_value='true',
                description=(
                    'diagnostic_aggregator를 함께 띄운다. false면 모니터가 '
                    f'{RAW_TOPIC}를 직접 읽어 단독 디버깅한다.'
                ),
            ),
            adapter,
            aggregator,
            monitor_with_agg,
            monitor_standalone,
        ]
    )
