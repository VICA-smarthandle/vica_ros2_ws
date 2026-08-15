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
    odom_topic = LaunchConfiguration('odom_topic')

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
        # 2026-08-15: 전에는 아래 include 가 '/odom' 을 하드코딩해서, 명령줄의
        # odom_topic:=... 이 경고 한 줄 없이 무시됐다. ROS 2 launch 는 선언 안 된
        # 인자를 오류로 잡지 않는다(include_launch_description.py 의 raise 는
        # '필수 인자 누락'에만 해당). 그래서 터미네이터 slam 칸이 오래
        # odom_topic:=/wheel/odom 을 넘겼는데도 Cartographer 는 /odom 을 읽었다.
        #
        # 기본값은 /odom 을 유지한다. 2026-08-11~12 매핑 13회가 전부 /odom
        # 이었고 그중 성공한 vica_map_0810 도 같은 값이었다. /wheel/odom 은
        # 아직 한 번도 시험된 적이 없다 — 바꿀 때는 그 축 하나만 바꾼다.
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Cartographer 가 읽을 오도메트리 토픽입니다. '
                        '/odom 은 EKF 출력, /wheel/odom 은 엔코더 원본입니다.'
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
                'odom_topic': odom_topic,
            }.items(),
        ),
    ])
