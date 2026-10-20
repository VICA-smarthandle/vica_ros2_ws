from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


MOTOR_PACKAGES = {
    'cpp': 'mdrobot_can_control_cpp',
    'python': 'mdrobot_can_control',
}


def _motor_node(context):
    impl = LaunchConfiguration('motor_impl').perform(context)
    if impl not in MOTOR_PACKAGES:
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
        DeclareLaunchArgument('knob_min_pct', default_value='55'),
        DeclareLaunchArgument('knob_max_pct', default_value='98'),
        OpaqueFunction(function=_motor_node),
    ])
