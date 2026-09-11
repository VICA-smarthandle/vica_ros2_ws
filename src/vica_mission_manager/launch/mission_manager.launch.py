"""vica_mission_manager launch."""
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
            DeclareLaunchArgument("estop_release_grace_sec", default_value="1.0"),
            DeclareLaunchArgument(
                "person_approach_speed_percent", default_value="100.0"),
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
            DeclareLaunchArgument("auto_return_home", default_value="true"),
            DeclareLaunchArgument("wake_doa_sign", default_value="1.0"),
            DeclareLaunchArgument("seek_look_sec", default_value="6.0"),
            DeclareLaunchArgument("near_call_max_m", default_value="1.5"),
            DeclareLaunchArgument("near_call_no_spin_m", default_value="1.0"),
            DeclareLaunchArgument("handle_side_min_yaw_deg", default_value="135.0"),
            DeclareLaunchArgument("return_resume_sec", default_value="15.0"),
            DeclareLaunchArgument("dest_retry_return_sec", default_value="0.0"),
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
                        "person_approach_speed_percent": ParameterValue(
                            LaunchConfiguration("person_approach_speed_percent"),
                            value_type=float,
                        ),
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
                        "auto_return_home": ParameterValue(
                            LaunchConfiguration("auto_return_home"),
                            value_type=bool,
                        ),
                        "wake_doa_sign": ParameterValue(
                            LaunchConfiguration("wake_doa_sign"),
                            value_type=float,
                        ),
                        "seek_look_sec": ParameterValue(
                            LaunchConfiguration("seek_look_sec"),
                            value_type=float,
                        ),
                        "near_call_max_m": ParameterValue(
                            LaunchConfiguration("near_call_max_m"),
                            value_type=float,
                        ),
                        "near_call_no_spin_m": ParameterValue(
                            LaunchConfiguration("near_call_no_spin_m"),
                            value_type=float,
                        ),
                        "handle_side_min_yaw_deg": ParameterValue(
                            LaunchConfiguration("handle_side_min_yaw_deg"),
                            value_type=float,
                        ),
                        "return_resume_sec": ParameterValue(
                            LaunchConfiguration("return_resume_sec"),
                            value_type=float,
                        ),
                        "dest_retry_return_sec": ParameterValue(
                            LaunchConfiguration("dest_retry_return_sec"),
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="vica_mission_manager",
                executable="emergency_estop_bridge",
                output="screen",
                parameters=[{"pulse_sec": LaunchConfiguration("estop_pulse_sec")}],
            ),
        ]
    )
