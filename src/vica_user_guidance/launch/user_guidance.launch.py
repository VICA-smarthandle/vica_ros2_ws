"""Smart Handle 사용자 안내 bringup."""

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

IMPL_PACKAGES = {
    "cpp": "vica_user_guidance_cpp",
    "python": "vica_user_guidance",
}


def _pick_package(context, arg_name, executable, label):
    """(패키지, 앞에 둘 경고) — C++ 판 실행 파일이 없으면 파이썬 판으로 바꾸고 크게 알린다."""
    impl = LaunchConfiguration(arg_name).perform(context)
    if impl not in IMPL_PACKAGES:
        raise RuntimeError(
            f"{arg_name} 은 'cpp' 또는 'python' 이어야 합니다: {impl!r}")
    if impl == "cpp":
        package = IMPL_PACKAGES["cpp"]
        try:
            exe = os.path.join(get_package_prefix(package), "lib", package, executable)
        except PackageNotFoundError:
            exe = ""
        if not os.path.isfile(exe):
            return IMPL_PACKAGES["python"], [LogInfo(msg=(
                f"[경고] vica_user_guidance_cpp 의 {executable} 가 설치돼 있지 않아 {label}를 파이썬 판으로 "
                "띄웁니다. C++ 판을 쓰려면 colcon build --packages-up-to vica_user_guidance"))]
    return IMPL_PACKAGES[impl], []


def _turn_node(context, params_file):
    package, actions = _pick_package(context, "turn_impl", "turn_guide_node", "방향지시")
    return actions + [
        Node(
            package=package,
            executable="turn_guide_node",
            name="turn_guide_node",
            output="screen",
            parameters=[params_file],
        )
    ]


def _driver_node(context, params_file, enable_serial):
    package, actions = _pick_package(
        context, "driver_impl", "user_guidance_driver_node", "손잡이 드라이버")
    return actions + [
        Node(
            package=package,
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
            DeclareLaunchArgument(
                "turn_impl",
                default_value="cpp",
                description="방향지시 판: 'cpp'(기본) 또는 'python'(옛 판으로 되돌릴 때)",
            ),
            OpaqueFunction(function=_turn_node, kwargs={"params_file": params_file}),
            OpaqueFunction(
                function=_driver_node,
                kwargs={"params_file": params_file, "enable_serial": enable_serial},
            ),
        ]
    )
