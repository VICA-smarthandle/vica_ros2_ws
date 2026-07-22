import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # =========================================================================
    # [1] 패키지 경로 및 설정 파일 동적 설정
    #     절대경로를 쓰지 않고, colcon build 후 install된 패키지 경로를 자동으로 찾습니다.
    # =========================================================================
    pkg_share_dir = get_package_share_directory('vica_cartographer')
    config_dir = os.path.join(pkg_share_dir, 'config')
    configuration_basename = 'vica_2d.lua'

    # =========================================================================
    # [2] Launch Arguments
    #     실행 시 터미널에서 값을 바꿀 수 있습니다.
    #
    # 예:
    #   ros2 launch vica_cartographer cartographer.launch.py use_sim_time:=true
    #   ros2 launch vica_cartographer cartographer.launch.py resolution:=0.03
    # =========================================================================
    use_sim_time_arg = LaunchConfiguration('use_sim_time')
    resolution_arg = LaunchConfiguration('resolution')
    publish_period_arg = LaunchConfiguration('publish_period_sec')
    odom_topic_arg = LaunchConfiguration('odom_topic')

    return LaunchDescription([
        # ROS2 로그를 버퍼링해서 출력 안정화
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='실제 로봇 구동 시 false, rosbag 재생 매핑 시 true로 설정합니다.'
        ),

        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Occupancy Grid 지도 해상도입니다. 0.05는 5cm 격자를 의미합니다.'
        ),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='2D Occupancy Grid 지도를 /map으로 발행하는 주기입니다.'
        ),

        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Cartographer가 사용할 표준 EKF odometry 토픽입니다.'
        ),

        # =========================================================================
        # [3] Cartographer SLAM 핵심 노드
        #     입력:
        #       /scan : YDLIDAR G2 LaserScan
        #       /odom : 외부 바퀴 오도메트리
        #
        #     Lua 설정에서 use_odometry = true이면 /odom 토픽이 필요합니다.
        # =========================================================================
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time_arg,
            }],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', configuration_basename,
            ],
            remappings=[
                # YDLIDAR G2가 발행하는 스캔 토픽
                ('scan', '/scan'),

                # vica_localization EKF가 발행하는 표준 오도메트리 토픽
                ('odom', odom_topic_arg),
            ],
        ),

        # =========================================================================
        # [4] Occupancy Grid 생성 노드
        #     Cartographer의 submap 정보를 받아서 Nav2/RViz에서 쓰는 /map을 생성합니다.
        #
        #     주의:
        #       cartographer_ros 설치 버전에 따라 실행파일명이 다를 수 있으므로
        #       아래 명령으로 반드시 확인하세요.
        #
        #       ros2 pkg executables cartographer_ros
        #
        #       보통 Humble 빌드에서는 cartographer_occupancy_grid_node를 사용합니다.
        # =========================================================================
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time_arg,
            }],
            arguments=[
                '-resolution', resolution_arg,
                '-publish_period_sec', publish_period_arg,
            ],
        ),
    ])
