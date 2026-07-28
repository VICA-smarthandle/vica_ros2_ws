"""Smart Handle 사용자 안내 bringup.

이 launch는 주행 명령 노드를 기동하지 않는다. 안내 표시 계층만 띄운다.

하드웨어 없이 로직만 확인하려면:
    ros2 launch vica_user_guidance user_guidance.launch.py enable_serial:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch the turn guide node and the Smart Handle driver."""
    package_share = get_package_share_directory("vica_user_guidance")
    default_params = os.path.join(package_share, "config", "user_guidance.yaml")

    params_file = LaunchConfiguration("params_file")
    enable_serial = LaunchConfiguration("enable_serial")

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Smart Handle 안내 파라미터 YAML 경로",
            ),
            DeclareLaunchArgument(
                "enable_serial",
                default_value="true",
                description="false면 시리얼을 열지 않고 로직만 동작한다 (mock)",
            ),
            # 노드 이름은 YAML 최상위 키와 반드시 일치해야 한다.
            Node(
                package="vica_user_guidance",
                executable="turn_guide_node",
                name="turn_guide_node",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="vica_user_guidance",
                executable="user_guidance_driver_node",
                name="user_guidance_driver_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "enable_serial": ParameterValue(
                            enable_serial, value_type=bool
                        )
                    },
                ],
            ),
        ]
    )
