from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# 같은 노드의 두 판. 실행 파일·노드 이름·파라미터·토픽이 같다.
#
# 2026-10-12 C++ 판(mdrobot_can_control_cpp)을 기본으로 바꿨다. 이 노드는 주행·정지와
# 무관하게 30 Hz 로 돌아 코어 하나의 평균 8.7 %(기록 25회차)를 썼고, 격리 벤치에서 C++ 가
# 같은 일을 약 1/5 로 했다. 파이썬 원본과 무작위 시나리오의 출력이 글자까지 같다
# (mdrobot_can_control_cpp/test/parity). 되돌리려면 motor_impl:=python.
MOTOR_PACKAGES = {
    'cpp': 'mdrobot_can_control_cpp',
    'python': 'mdrobot_can_control',
}


def _motor_node(context):
    impl = LaunchConfiguration('motor_impl').perform(context)
    if impl not in MOTOR_PACKAGES:
        # 조용히 아무것도 안 띄우면 바퀴가 왜 안 도는지 찾기 어렵다.
        raise RuntimeError(
            f"motor_impl 은 'cpp' 또는 'python' 이어야 합니다: {impl!r}")
    return [Node(
        package=MOTOR_PACKAGES[impl],
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
    )]


def generate_launch_description():
    """Start only the MDROBOT CAN actuator adapter."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'motor_impl', default_value='cpp',
            description="모터 노드 판: 'cpp'(기본) 또는 'python'(옛 판으로 되돌릴 때)"),
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
        OpaqueFunction(function=_motor_node),
    ])
