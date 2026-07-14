from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # VICA motor safety bringup.
    #
    # 이 launch는 실제 mdrobot_can_keyboard_knob_node까지 실행한다.
    # keyboard_knob 노드는 can1으로 MDROBOT CAN 0xCF 속도 명령을 송신할 수 있으므로,
    # 바퀴를 띄운 상태에서만 사용한다.
    return LaunchDescription([
        Node(
            package="mdrobot_can_control",
            executable="emergency_stop_node",
            name="emergency_stop_node",
            output="screen",
            parameters=[{
                "input_mode": "test_topic",
                "log_f1_frames": True,
            }],
        ),
        Node(
            package="mdrobot_can_control",
            executable="safety_supervisor_node",
            name="safety_supervisor_node",
            output="screen",
        ),
        Node(
            package="mdrobot_can_control",
            executable="keyboard_knob",
            name="mdrobot_can_keyboard_knob_node",
            output="screen",
            parameters=[{
                "can_iface": "can1",
                "estop_bit_pressed_value": 0,
            }],
        ),
    ])
