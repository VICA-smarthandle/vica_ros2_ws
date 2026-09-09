from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start only the MDROBOT CAN actuator adapter."""
    return LaunchDescription([
        # 속도 조절 줄이 **실제로** 움직이는 knob 구간. 이 사이를 0~100 으로 편다.
        #
        # 2026-09-09 실기 실측: 이 로봇의 줄은 55~98 만 움직인다. 노드 기본값은
        # (0, 100)이라 아무것도 바꾸지 않으므로, 이 장비의 값은 여기서 준다.
        # 그대로 두면 눈금의 가운데 43 %만 쓰게 되어 줄이 아무 역할도 못 한다 —
        # 완전히 당겨도 55 % 라 정지 기준(5 %)에 못 미치고, 조금 놓으면 60~98 %
        # 가 전부 주행 상한 0.5 m/s 위여서 속도가 변하지 않는다.
        #
        # 줄이나 연결부를 고쳐 실제로 0~100 이 나오게 되면 두 값을 0, 100 으로
        # 되돌린다. 다른 장비에 올릴 때도 그 장비에서 실측한 값으로 준다.
        DeclareLaunchArgument('knob_min_pct', default_value='55'),
        DeclareLaunchArgument('knob_max_pct', default_value='98'),
        Node(
            package='mdrobot_can_control',
            executable='keyboard_knob',
            name='mdrobot_can_keyboard_knob_node',
            output='screen',
            parameters=[{
                'can_iface': 'can1',
                'estop_bit_pressed_value': 0,
                'knob_min_pct': ParameterValue(
                    LaunchConfiguration('knob_min_pct'), value_type=int),
                'knob_max_pct': ParameterValue(
                    LaunchConfiguration('knob_max_pct'), value_type=int),
            }],
        ),
    ])
