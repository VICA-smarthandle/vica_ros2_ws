"""Launch raw wheel odometry and the VICA local EKF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("vica_localization")
    default_ekf_params = os.path.join(package_share, "config", "ekf.yaml")
    default_encoder_params = os.path.join(package_share, "config", "encoder.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_encoder = LaunchConfiguration("start_encoder")
    can_iface = LaunchConfiguration("can_iface")
    ekf_params_file = LaunchConfiguration("ekf_params_file")
    encoder_params_file = LaunchConfiguration("encoder_params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation or rosbag clock when true.",
        ),
        DeclareLaunchArgument(
            "start_encoder",
            default_value="true",
            description="Start the read-only MDROBOT C5 encoder receiver.",
        ),
        DeclareLaunchArgument(
            "can_iface",
            default_value="can1",
            description="SocketCAN interface used by encoder_feedback.",
        ),
        DeclareLaunchArgument(
            "ekf_params_file",
            default_value=default_ekf_params,
            description="Installed VICA robot_localization EKF configuration.",
        ),
        DeclareLaunchArgument(
            "encoder_params_file",
            default_value=default_encoder_params,
            description="Installed VICA wheel encoder configuration.",
        ),
        Node(
            package="encoder_feedback",
            executable="encoder_feedback",
            name="encoder_feedback",
            output="screen",
            condition=IfCondition(start_encoder),
            parameters=[
                encoder_params_file,
                {
                    "can_iface": can_iface,
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "publish_tf": False,
                    "request_position_feedback": False,
                    "odom_topic": "/wheel/odom",
                },
            ],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[
                ekf_params_file,
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                },
            ],
            remappings=[
                ("odometry/filtered", "/odom"),
            ],
        ),
    ])
