"""로봇 TF 발행의 기반 launch. 실주행은 이것만 띄운다.

joint_state_publisher_gui는 Qt가 필요해 헤드리스 환경에서 뜨지 않는다. 그런데
기동 매뉴얼은 TF 발행을 필수 구성으로 지정하므로, 실주행 경로는 화면 없이 동작해야
한다. 그래서 기본값은 GUI 없는 joint_state_publisher다.

joint_state_publisher_gui는 joint_state_publisher를 의존성으로 포함하고 Qt 창만 얹은
래퍼다. 발행 로직이 같으므로 /joint_states 내용과 발행량은 동일하고, RViz에서 바퀴
메시도 그대로 보인다. 바뀌는 것은 슬라이더 창뿐이다.

gui 인자로 둘 중 하나만 띄운다. 둘 다 띄우면 /joint_states를 이중 발행한다.

나중에 엔코더 기반 각도를 넣을 때는 joint_state_publisher의 source_list 파라미터에
부분 발행 토픽을 더하면 된다. 캐스터 4개는 센서가 없어도 기본값으로 자동 보충된다.
"""
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
