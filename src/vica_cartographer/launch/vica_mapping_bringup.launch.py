"""Bring up what the app can start for a mapping session."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package: str, launch_file: str, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package), 'launch', launch_file
            )
        ),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    """Start motor (optional), slam and the map preview in that order."""
    can_iface = LaunchConfiguration('can_iface')
    start_motor = LaunchConfiguration('start_motor')
    slam_delay = LaunchConfiguration('slam_delay_sec')
    preview_dir = LaunchConfiguration('preview_output_dir')
    preview_period = LaunchConfiguration('preview_period_sec')

    return LaunchDescription([
        DeclareLaunchArgument(
            'can_iface',
            default_value='can1',
            description='encoder_feedback 이 사용할 SocketCAN 인터페이스입니다.',
        ),
        DeclareLaunchArgument(
            'start_motor',
            default_value='true',
            description=(
                'motor node 를 함께 띄웁니다. 터미널에서 이미 띄웠다면 false 로 '
                '넘기세요. mapping_supervisor_node 는 항상 false 를 넘깁니다 — '
                'motor 는 터미네이터 ⑤ 칸 소유로 고정이고(2026-08-25 실기 결정), '
                '떠 있는지는 시작 전 필수 노드 검사가 확인합니다.'
            ),
        ),
        DeclareLaunchArgument(
            'slam_delay_sec',
            default_value='4.0',
            description='motor 가 뜬 뒤 slam 을 띄우기까지 기다리는 시간입니다.',
        ),
        DeclareLaunchArgument(
            'preview_output_dir',
            default_value='',
            description=(
                '미리보기 PNG 를 쓸 디렉터리입니다. 비우면 $VICA_ROS_WS/maps/_live '
                '를 씁니다.'
            ),
        ),
        DeclareLaunchArgument(
            'preview_period_sec',
            default_value='2.0',
            description='미리보기 갱신 주기입니다. 매핑 속도 0.3 m/s 기준 2초면 60 cm 다.',
        ),

        GroupAction(
            condition=IfCondition(start_motor),
            actions=[_include('mdrobot_can_control', 'motor_bringup.launch.py')],
        ),

        TimerAction(
            period=slam_delay,
            actions=[
                _include(
                    'vica_cartographer',
                    'vica_slam_bringup.launch.py',
                    {'can_iface': can_iface},
                )
            ],
        ),

        Node(
            package='vica_cartographer',
            executable='map_preview_node',
            name='map_preview_node',
            output='screen',
            parameters=[{
                'output_dir': preview_dir,
                'period_sec': preview_period,
            }],
        ),
    ])
