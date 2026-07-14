from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # VICA safety bringup 테스트용 launch.
    #
    # 여기서는 emergency_stop_node와 safety_supervisor_node만 실행한다.
    # motor node(keyboard_knob)는 can1으로 실제 CAN 0xCF 속도 명령을 보낼 수 있으므로
    # 이 launch에 포함하지 않는다. 모터 연결 테스트는 바퀴를 띄운 뒤 별도 터미널에서 실행한다.
    return LaunchDescription([
        Node(
            package="mdrobot_can_control",
            executable="emergency_stop_node",
            name="emergency_stop_node",
            output="screen",
        ),
        Node(
            package="mdrobot_can_control",
            executable="safety_supervisor_node",
            name="safety_supervisor_node",
            output="screen",
        ),
    ])
