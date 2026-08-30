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
            #
            # 2026-08-30: 거리만 [1.0, 0.5] -> [0.8, 0.4] 로 당긴다. 비율은 그대로다.
            # (approach_speed.py 의 DEFAULT_APPROACH_STAGES 는 설계 기본값이므로
            #  건드리지 않는다. 실기 값은 여기서 정한다 — 그 파일 주석의 원칙이다.)
            #
            # 왜: max_vel_x 를 0.26 -> 0.5 로 올린 뒤 이 사다리가 재검토되지 않았다.
            # 같은 60 % 라도 속도가 두 배라 도착 직전 상황이 달라졌다.
            #
            #                    설계(0.26/0.4)   지금(0.50/0.5)
            #   0.5 m 이내 60 %   직진 0.156 m/s   직진 0.300 m/s
            #                     정지거리 5.2 cm  정지거리 10.8 cm  <- 두 배
            #                     회전 13.8 도/초  회전 17.2 도/초   <- 오히려 빠름
            #
            # 즉 **회전은 설계보다 빨라졌고 정지 낙차만 두 배가 됐다.** 사용자가
            # 느낀 "도착 직전이 답답하다"는 무제한 구간(28.6 도/초)과의 대비였다.
            #
            # 두 요구가 부딪힌다 — 부드럽게 멈추려면 비율을 낮춰야 하는데
            # /speed_limit 은 비율 하나로 max_vel_x 와 max_vel_theta 를 **같이**
            # 줄이므로 회전이 함께 느려진다. 그래서 이번에는 비율을 건드리지 않고
            # **감속 구간을 짧게** 하는 쪽을 택했다(사용자 판정).
            #
            #   60 % 구간 길이   25 cm -> 15 cm   (도착 판정 0.25 m 기준)
            #   0.5 -> 0.3 m/s 감속에 필요한 거리는 3.2 cm 라 여유가 남는다
            #
            # 정지 낙차(10.8 cm)는 그대로다. 그것까지 잡으려면 비율을 낮추거나,
            # /speed_limit 대신 max_vel_x 만 직접 낮추는 길을 열어야 한다
            # (후자는 표준 통로를 벗어나므로 필요가 확실해질 때 연다).
            DeclareLaunchArgument(
                "approach_slowdown_distances_m",
                default_value="[0.8, 0.4]",
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
