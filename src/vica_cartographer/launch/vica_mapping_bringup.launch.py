"""Bring up everything the app can start for a mapping session.

**순서에 근거가 있다.**

  ① safety   docs/vica_robot_bringup_manual.md ④ 가 "motor 보다 **먼저** 띄운다"
  ② motor    scripts/vica_terminator_layout.py 의 vica_map 프로파일:
             "motor 는 뺄 수 없다. 엔코더 피드백을 요청하는 쪽이 motor node 라서,
              없으면 /wheel/odom 이 나오지 않는다"
  ③ slam     encoder_feedback + EKF + cartographer. odom -> base_footprint TF 의
             유일한 발행자라 끄면 map -> odom 이 나오지 않는다
  ④ preview  /map 을 PNG 로 떠서 앱이 보게 한다

**여기 없는 것과 그 이유.**

  전원·CAN   물리 조작이다
  d455       Docker `exec` 이라 launch 범위 밖이다
  imu        띄운 뒤 **20초 완전 정지**해야 'Gyro bias calibrated' 가 뜬다.
             사람이 기다려야 하는 일이라 자동화하면 오히려 무효 회차를 만든다
  teleop     키보드 입력을 받는 대화형 프로세스다. 앱이 대신한다(앱 teleop)
  E-stop reset  앱이 /app_estop_reset 으로 한다. 중앙 래치는 기동 직후 latched 로
             시작하고 /motor/can_ok 가 원인의 하나라, safety·motor 가 다 뜬 뒤에야
             풀린다. 풀지 않으면 /cmd_vel_safe 가 안 나가 로봇을 끌고 다닐 수 없다
  nav2       **넣으면 안 된다.** SLAM 과 EKF·map->odom TF 가 충돌한다

**Nav2 를 절대 include 하지 않는다.** 이 파일에 nav2 를 넣으면
mapping_supervisor_node 의 중복 검사가 자기가 만든 충돌을 잡게 된다.

TimerAction 으로 사이를 벌리는 이유: launch 는 include 를 순서대로 '시작'만 하고
기다려 주지 않는다. safety 가 뜨기 전에 motor 가 /cmd_vel_safe 를 찾으면 초기
프레임을 놓친다. 지연값은 실기에서 조정할 수 있게 인자로 뺐다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
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
    """Start safety, motor, slam and the map preview in that order."""
    can_iface = LaunchConfiguration('can_iface')
    motor_delay = LaunchConfiguration('motor_delay_sec')
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
            'motor_delay_sec',
            default_value='3.0',
            description='safety 가 뜬 뒤 motor 를 띄우기까지 기다리는 시간입니다.',
        ),
        DeclareLaunchArgument(
            'slam_delay_sec',
            default_value='6.0',
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

        # ① Safety — motor 보다 먼저.
        _include('vica_safety', 'safety_bringup.launch.py'),

        # ② motor — /wheel/odom 의 전제.
        TimerAction(
            period=motor_delay,
            actions=[_include('mdrobot_can_control', 'motor_bringup.launch.py')],
        ),

        # ③ SLAM — encoder + EKF + cartographer.
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

        # ④ 미리보기 — /map 이 나오기 시작하면 알아서 쓴다. 지연이 필요 없다.
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
