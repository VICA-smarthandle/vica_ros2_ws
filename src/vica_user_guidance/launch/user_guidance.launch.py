"""Smart Handle 사용자 안내 bringup.

이 launch는 주행 명령 노드를 기동하지 않는다. 안내 표시 계층만 띄운다.

하드웨어 없이 로직만 확인하려면:
    ros2 launch vica_user_guidance user_guidance.launch.py enable_serial:=false

손잡이 드라이버를 파이썬 판으로 되돌리려면:
    ros2 launch vica_user_guidance user_guidance.launch.py driver_impl:=python
"""

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 손잡이 드라이버의 두 판. 실행 파일·노드 이름·파라미터·토픽·로그가 같다.
#
# 2026-10-13 C++ 판(vica_user_guidance_cpp)을 기본으로 바꿨다. 이 노드는 주행·정지와 무관하게
# 상향 시리얼 20 Hz·전송 10 Hz·진단 2 Hz 를 돌리고 초음파 TF 게이트 때문에 /tf 를 모두 받아,
# 파이썬에서 코어 하나의 15~23 %를 썼다. 파이썬 원본과 무작위 시나리오의 출력이 글자까지 같다
# (vica_user_guidance_cpp/test/parity). 되돌리려면 driver_impl:=python.
DRIVER_PACKAGES = {
    "cpp": "vica_user_guidance_cpp",
    "python": "vica_user_guidance",
}


def _driver_node(context, params_file, enable_serial):
    impl = LaunchConfiguration("driver_impl").perform(context)
    if impl not in DRIVER_PACKAGES:
        # 조용히 아무것도 안 띄우면 손잡이·초음파가 왜 안 되는지 찾기 어렵다.
        raise RuntimeError(
            f"driver_impl 은 'cpp' 또는 'python' 이어야 합니다: {impl!r}")
    actions = []
    if impl == "cpp":
        try:
            get_package_prefix(DRIVER_PACKAGES["cpp"])
        except PackageNotFoundError:
            # C++ 판을 빌드하지 않은 작업공간(예: vica_user_guidance 만 골라 빌드). 여기서 멈추면
            # 방향지시(turn_guide_node)까지 함께 안 떠 손잡이 안내가 통째로 꺼진다 — 파이썬 판으로
            # 띄우고 크게 알린다.
            impl = "python"
            actions.append(LogInfo(msg=(
                "[경고] vica_user_guidance_cpp 가 설치돼 있지 않아 손잡이 드라이버를 파이썬 판으로 "
                "띄웁니다. C++ 판을 쓰려면 colcon build --packages-up-to vica_user_guidance")))
    return actions + [
        Node(
            package=DRIVER_PACKAGES[impl],
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
        )
    ]


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
            DeclareLaunchArgument(
                "driver_impl",
                default_value="cpp",
                description="손잡이 드라이버 판: 'cpp'(기본) 또는 'python'(옛 판으로 되돌릴 때)",
            ),
            # 노드 이름은 YAML 최상위 키와 반드시 일치해야 한다.
            Node(
                package="vica_user_guidance",
                executable="turn_guide_node",
                name="turn_guide_node",
                output="screen",
                parameters=[params_file],
            ),
            OpaqueFunction(
                function=_driver_node,
                kwargs={"params_file": params_file, "enable_serial": enable_serial},
            ),
        ]
    )
