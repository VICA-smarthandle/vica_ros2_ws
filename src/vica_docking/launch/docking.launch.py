"""vica_docking_node 단독 실행(실기 시험·주행 묶음에서 포함)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('vica_docking'), 'config', 'docking.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('destinations_path', default_value=''),
        DeclareLaunchArgument('auto_initial_pose', default_value='true'),
        DeclareLaunchArgument('trace_dir', default_value=''),
        Node(
            package='vica_docking',
            executable='vica_docking_node',
            name='vica_docking',
            output='screen',
            parameters=[cfg, {
                'destinations_path': ParameterValue(LaunchConfiguration('destinations_path'), value_type=str),
                'auto_initial_pose': ParameterValue(LaunchConfiguration('auto_initial_pose'), value_type=bool),
                'trace_dir': ParameterValue(LaunchConfiguration('trace_dir'), value_type=str),
            }],
        ),
    ])
