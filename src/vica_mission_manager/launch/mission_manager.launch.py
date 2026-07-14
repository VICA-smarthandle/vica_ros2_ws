"""vica_mission_manager launch.

목적지/지도 파일 경로는 ROS parameter 로 절대경로를 넘긴다
(env var·상대경로는 배포/권한 취약 — 계획 결정 자동⑥).

사용 예:
    ros2 launch vica_mission_manager mission_manager.launch.py
    ros2 launch vica_mission_manager mission_manager.launch.py \
        destinations_yaml:=/path/to/destinations.yaml map_yaml:=/path/to/map.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 현재 실기 기준 기본 경로 (~/tony 통합 워크스페이스).
# destinations.yaml 의 단일 소스는 음성 저장소다 (함정 3번).
DEFAULT_DESTINATIONS = "/home/ji_w/tony/vica-voice-llm/config/destinations.yaml"
DEFAULT_MAP = "/home/ji_w/tony/vica_ros2_ws/maps/vica_map_0604.yaml"


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("destinations_yaml", default_value=DEFAULT_DESTINATIONS),
            DeclareLaunchArgument("map_yaml", default_value=DEFAULT_MAP),
            DeclareLaunchArgument("confirm_timeout_sec", default_value="30.0"),
            DeclareLaunchArgument("estop_release_grace_sec", default_value="2.0"),
            DeclareLaunchArgument("current_floor", default_value="-1"),
            DeclareLaunchArgument("current_building", default_value=""),
            # name= 을 지정하지 않는다: launch 의 name 리매핑은 프로세스 안의
            # 모든 노드(BasicNavigator 포함)에 적용되어 이름 충돌을 일으킨다.
            Node(
                package="vica_mission_manager",
                executable="mission_manager",
                output="screen",
                parameters=[
                    {
                        "destinations_yaml": LaunchConfiguration("destinations_yaml"),
                        "map_yaml": LaunchConfiguration("map_yaml"),
                        "confirm_timeout_sec": LaunchConfiguration("confirm_timeout_sec"),
                        "estop_release_grace_sec": LaunchConfiguration(
                            "estop_release_grace_sec"
                        ),
                        "current_floor": LaunchConfiguration("current_floor"),
                        "current_building": LaunchConfiguration("current_building"),
                    }
                ],
            ),
        ]
    )
