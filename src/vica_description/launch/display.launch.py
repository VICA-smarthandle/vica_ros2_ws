"""URDF 확인용 launch. RViz와 슬라이더 창을 함께 띄운다.

TF 발행 자체는 robot_state.launch.py가 소유한다. 이 파일은 거기에 gui:=true를 넘기고
RViz만 얹는다. 실주행은 robot_state.launch.py를 직접 띄운다 — 화면이 없어도 동작한다.
"""
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

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value=default_model),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_state_launch),
            launch_arguments={
                "model": model,
                "gui": "true",
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
