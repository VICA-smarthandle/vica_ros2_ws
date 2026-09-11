"""URDF 확인용 launch. robot_state.launch.py를 include하고 RViz를 얹는다."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("vica_description")
    default_model = os.path.join(pkg_share, "urdf", "VICA.xacro")
    default_rviz_config = os.path.join(pkg_share, "rviz", "urdf.rviz")
    robot_state_launch = os.path.join(pkg_share, "launch", "robot_state.launch.py")

    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rviz_config")
    gui = LaunchConfiguration("gui")

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value=default_model),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="true면 슬라이더 창(joint_state_publisher_gui)도 함께 띄운다.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_state_launch),
            launch_arguments={
                "model": model,
                "gui": gui,
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
