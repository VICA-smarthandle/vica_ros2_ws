"""로봇 TF 발행의 기반 launch. 실주행은 이것만 띄운다."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("vica_description")
    default_model = os.path.join(pkg_share, "urdf", "VICA.xacro")

    model = LaunchConfiguration("model")
    gui = LaunchConfiguration("gui")

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"),
                " ",
                model,
            ]),
            value_type=str,
        )
    }

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value=default_model),
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="true면 슬라이더 창(joint_state_publisher_gui)을 띄운다. 화면이 있어야 한다.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            parameters=[robot_description],
            condition=UnlessCondition(gui),
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            parameters=[robot_description],
            condition=IfCondition(gui),
            output="screen",
        ),
    ])
