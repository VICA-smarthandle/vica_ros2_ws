from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start only the MDROBOT CAN actuator adapter."""
    return LaunchDescription([
        Node(
            package='mdrobot_can_control',
            executable='keyboard_knob',
            name='mdrobot_can_keyboard_knob_node',
            output='screen',
            parameters=[{
                'can_iface': 'can1',
                'estop_bit_pressed_value': 0,
            }],
        ),
    ])
