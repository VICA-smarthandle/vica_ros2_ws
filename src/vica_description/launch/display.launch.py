"""URDF 확인용 launch. robot_state.launch.py를 include하고 RViz를 얹는다.

TF 발행 자체는 robot_state.launch.py가 소유한다. 바퀴는 continuous 조인트라
/joint_states가 있어야 TF가 생기고, TF가 없으면 RViz가 그 링크를 그리지 못한다 —
바퀴 메시가 보이려면 joint_state_publisher(GUI 없는 쪽)만 있으면 충분하다.
그래서 gui 기본값은 false다. 슬라이더 창(joint_state_publisher_gui)은 URDF 제작 시
관절 축을 손으로 돌려 검증하는 도구라 평소 확인 작업에는 필요 없고, 원할 때만
gui:=true로 얹는다. 실주행은 robot_state.launch.py를 직접 띄운다 — 화면이 없어도
동작한다.
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
