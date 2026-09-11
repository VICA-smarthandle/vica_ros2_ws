from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start only the MDROBOT CAN actuator adapter."""
    return LaunchDescription([
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
