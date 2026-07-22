"""Launch VICA wheel odometry, local EKF and Cartographer 2D.

The motor node requests MDROBOT C5 position feedback with RET_TYPE_ODOM=5.
encoder_feedback receives it without sending a second motor command and publishes
/wheel/odom. The EKF publishes /odom and the odom -> base_footprint transform.

This launch does not start the motor, LiDAR driver, IMU adapter or
robot_state_publisher. Set start_localization:=false only when an external process
already owns /odom and odom -> base_footprint.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share_dir = get_package_share_directory('vica_cartographer')
    localization_share_dir = get_package_share_directory('vica_localization')
    wheel_ekf_launch = os.path.join(
        localization_share_dir,
        'launch',
        'wheel_ekf.launch.py',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_localization = LaunchConfiguration('start_localization')
    start_encoder = LaunchConfiguration('start_encoder')
    can_iface = LaunchConfiguration('can_iface')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_localization',
            default_value='true',
            description='wheel encoder와 EKF를 함께 실행합니다.'
        ),
        DeclareLaunchArgument(
            'start_encoder',
            default_value='true',
            description='실기기 CAN C5 encoder receiver를 실행합니다.'
        ),
        DeclareLaunchArgument(
            'can_iface',
            default_value='can1',
            description='encoder_feedback이 사용할 SocketCAN 인터페이스입니다.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='실기기 false, rosbag 재생 시 true 로 설정합니다.'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(wheel_ekf_launch),
            condition=IfCondition(start_localization),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'start_encoder': start_encoder,
                'can_iface': can_iface,
            }.items(),
        ),

        # =====================================================================
        # [2] Cartographer 2D + occupancy grid
        # =====================================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share_dir, 'launch', 'vica_cartographer_2d.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'odom_topic': '/odom',
            }.items(),
        ),
    ])
