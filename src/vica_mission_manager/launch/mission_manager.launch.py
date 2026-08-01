"""vica_mission_manager launch.

목적지/지도 파일 경로는 ROS parameter 로 절대경로를 넘긴다
(env var·상대경로는 배포/권한 취약 — 계획 결정 자동⑥).

사용 예:
    ros2 launch vica_mission_manager mission_manager.launch.py
    ros2 launch vica_mission_manager mission_manager.launch.py \
        destinations_yaml:=/path/to/destinations.yaml map_yaml:=/path/to/map.yaml
"""
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    map_id = LaunchConfiguration("map_id")
    storage_root = LaunchConfiguration("destination_storage_root")
    default_storage_root = PathJoinSubstitution(
        [EnvironmentVariable("HOME"), "vica_data", "destinations"]
    )
    default_destinations = PathJoinSubstitution(
        [storage_root, map_id, "destinations.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_id", default_value="vica_map_0630"),
            DeclareLaunchArgument(
                "destination_storage_root",
                default_value=default_storage_root,
            ),
            DeclareLaunchArgument(
                "destinations_yaml",
                default_value=default_destinations,
            ),
            DeclareLaunchArgument("map_yaml", default_value=""),
            DeclareLaunchArgument("confirm_timeout_sec", default_value="30.0"),
            DeclareLaunchArgument("estop_release_grace_sec", default_value="2.0"),
            # 접근 감속 단계. 두 배열은 순번끼리 짝이며 개수가 같아야 한다.
            # 잔여거리가 1.0 m 이하면 80 %, 0.5 m 이하면 60 %로 최대속도 상한을
            # 내린다. 한 번 내려간 제한은 그 Goal 동안 풀리지 않는다.
            # 2026-08-01 실주행 뒤 (1.5,70)(1.0,55)(0.5,40)에서 조정했다 —
            # 제한이 회전에도 걸려 도착 직전 제자리 회전이 9도/초로 느려졌다.
            # 값의 근거와 위험(마지막 구간 회전 지연)은 approach_speed.py 참조.
            DeclareLaunchArgument(
                "approach_slowdown_distances_m",
                default_value="[1.0, 0.5]",
            ),
            DeclareLaunchArgument(
                "approach_speed_limit_percents",
                default_value="[80.0, 60.0]",
            ),
            DeclareLaunchArgument("current_floor", default_value="-1"),
            DeclareLaunchArgument("current_building", default_value=""),
            DeclareLaunchArgument("estop_pulse_sec", default_value="3.0"),
            # name= 을 지정하지 않는다: launch 의 name 리매핑은 프로세스 안의
            # 모든 노드(BasicNavigator 포함)에 적용되어 이름 충돌을 일으킨다.
            Node(
                package="vica_mission_manager",
                executable="mission_manager",
                output="screen",
                parameters=[
                    {
                        "destinations_yaml": LaunchConfiguration("destinations_yaml"),
                        "map_id": map_id,
                        "map_yaml": LaunchConfiguration("map_yaml"),
                        "confirm_timeout_sec": LaunchConfiguration("confirm_timeout_sec"),
                        "estop_release_grace_sec": LaunchConfiguration(
                            "estop_release_grace_sec"
                        ),
                        # 노드는 double 배열로 선언한다. launch 인자는 문자열이라
                        # value_type 을 지정해야 "[1.5, 1.0, 0.5]" 가 배열로 해석된다.
                        "approach_slowdown_distances_m": ParameterValue(
                            LaunchConfiguration("approach_slowdown_distances_m"),
                            value_type=List[float],
                        ),
                        "approach_speed_limit_percents": ParameterValue(
                            LaunchConfiguration("approach_speed_limit_percents"),
                            value_type=List[float],
                        ),
                        "current_floor": LaunchConfiguration("current_floor"),
                        "current_building": LaunchConfiguration("current_building"),
                    }
                ],
            ),
            # 진행순서 ③: 긴급어 → E-stop 래치 체인 배선.
            # 정지의 권위는 vica_safety/emergency_stop_node 중앙 래치이고
            # (vica_safety safety_bringup으로 별도 기동), 이 브리지는 방아쇠만 당긴다.
            Node(
                package="vica_mission_manager",
                executable="emergency_estop_bridge",
                output="screen",
                parameters=[{"pulse_sec": LaunchConfiguration("estop_pulse_sec")}],
            ),
        ]
    )
