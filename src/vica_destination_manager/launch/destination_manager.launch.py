"""VICA 지도별 목적지 저장 관리자 launch."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_root = PathJoinSubstitution(
        [EnvironmentVariable("HOME"), "vica_data", "destinations"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("storage_root", default_value=default_root),
            Node(
                package="vica_destination_manager",
                executable="destination_manager",
                output="screen",
                parameters=[
                    {"storage_root": LaunchConfiguration("storage_root")}
                ],
            ),
        ]
    )
