from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    """Start the independent VICA software safety layer without the motor."""
    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
        Node(
            package="vica_safety",
            executable="emergency_stop_node",
            name="emergency_stop_node",
            output="screen",
            parameters=[{
                "input_mode": "can_f1",
                "can_iface": "can1",
                "driver_response_id": 0x701,
                "log_f1_frames": True,
            }],
        ),
        Node(
            package="vica_safety",
            executable="safety_supervisor_node",
            name="safety_supervisor_node",
            output="screen",
        ),
        Node(
            package="vica_safety",
            executable="app_emergency_node",
            name="app_emergency_node",
            output="screen",
        ),
    ])
