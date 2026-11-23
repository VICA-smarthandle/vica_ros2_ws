"""vica_mission_manager launch."""
import os
from typing import List

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _docking_nodes(map_id) -> List[Node]:
    """도킹 노드(2026-11-18) — 미션이 유일한 지휘자라 미션과 같이 뜨고 같이 내린다."""
    try:
        share = get_package_share_directory("vica_docking")
    except PackageNotFoundError:
        return []
    return [
        Node(
            package="vica_docking",
            executable="vica_docking_node",
            name="vica_docking",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_docking")),
            parameters=[
                os.path.join(share, "config", "docking.yaml"),
                {
                    "destinations_path": ParameterValue(
                        LaunchConfiguration("destinations_yaml"), value_type=str),
                    "map_id": ParameterValue(map_id, value_type=str),
                    "trace_dir": ParameterValue(
                        LaunchConfiguration("docking_trace_dir"), value_type=str),
                },
            ],
        )
    ]


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
            DeclareLaunchArgument("use_docking", default_value="true"),
            DeclareLaunchArgument("docking_trace_dir", default_value=""),
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
            DeclareLaunchArgument("approach_bt_xml", default_value="auto"),
            DeclareLaunchArgument("guided_bt_xml", default_value="auto"),
            DeclareLaunchArgument("guided_no_rail_bt_xml", default_value="auto"),
            DeclareLaunchArgument("wait_spot_bt_xml", default_value="auto"),
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
            DeclareLaunchArgument("grip_release_grace_sec", default_value="0.1"),
            DeclareLaunchArgument("grip_enter_window_sec", default_value="1.0"),
            DeclareLaunchArgument("grip_wait_timeout_sec", default_value="10.0"),
            DeclareLaunchArgument("grip_assume_held", default_value="false"),
            DeclareLaunchArgument("grip_release_stop", default_value="false"),
            DeclareLaunchArgument("speaker_gate", default_value="false"),
            DeclareLaunchArgument("spk_joined_threshold", default_value="0.50"),
            DeclareLaunchArgument("speaker_gate_grip", default_value="false"),
            DeclareLaunchArgument("reask_ladder", default_value="false"),
            DeclareLaunchArgument("wake_revive", default_value="false"),
            DeclareLaunchArgument("answer_field", default_value="false"),
            DeclareLaunchArgument("obstacle_narration", default_value="true"),
            DeclareLaunchArgument("patrol_dwell_sec", default_value="3.0"),
            DeclareLaunchArgument("patrol_intruder_repeat_sec", default_value="3.0"),
            DeclareLaunchArgument("patrol_light_repeat_sec", default_value="10.0"),
            DeclareLaunchArgument("patrol_alarm_resend_sec", default_value="10.0"),
            DeclareLaunchArgument("patrol_detector_timeout_sec", default_value="3.0"),
            DeclareLaunchArgument("patrol_arming_timeout_sec", default_value="15.0"),
            DeclareLaunchArgument("patrol_speed_mps", default_value="0.6"),
            DeclareLaunchArgument("patrol_max_leg_failures", default_value="3"),
            DeclareLaunchArgument(
                "patrol_media_dir",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "workspaces", "isaac_ros-dev", "vica_patrol"]),
            ),
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
                        "approach_bt_xml": ParameterValue(
                            LaunchConfiguration("approach_bt_xml"), value_type=str),
                        "guided_bt_xml": ParameterValue(
                            LaunchConfiguration("guided_bt_xml"), value_type=str),
                        "guided_no_rail_bt_xml": ParameterValue(
                            LaunchConfiguration("guided_no_rail_bt_xml"), value_type=str),
                        "wait_spot_bt_xml": ParameterValue(
                            LaunchConfiguration("wait_spot_bt_xml"), value_type=str),
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
                        "grip_release_grace_sec": ParameterValue(
                            LaunchConfiguration("grip_release_grace_sec"),
                            value_type=float,
                        ),
                        "grip_wait_timeout_sec": ParameterValue(
                            LaunchConfiguration("grip_wait_timeout_sec"),
                            value_type=float,
                        ),
                        "grip_enter_window_sec": ParameterValue(
                            LaunchConfiguration("grip_enter_window_sec"),
                            value_type=float,
                        ),
                        "grip_assume_held": ParameterValue(
                            LaunchConfiguration("grip_assume_held"),
                            value_type=bool,
                        ),
                        "grip_release_stop": ParameterValue(
                            LaunchConfiguration("grip_release_stop"),
                            value_type=bool,
                        ),
                        "speaker_gate": ParameterValue(
                            LaunchConfiguration("speaker_gate"),
                            value_type=bool,
                        ),
                        "spk_joined_threshold": ParameterValue(
                            LaunchConfiguration("spk_joined_threshold"),
                            value_type=float,
                        ),
                        "speaker_gate_grip": ParameterValue(
                            LaunchConfiguration("speaker_gate_grip"),
                            value_type=bool,
                        ),
                        "reask_ladder": ParameterValue(
                            LaunchConfiguration("reask_ladder"),
                            value_type=bool,
                        ),
                        "wake_revive": ParameterValue(
                            LaunchConfiguration("wake_revive"),
                            value_type=bool,
                        ),
                        "answer_field": ParameterValue(
                            LaunchConfiguration("answer_field"),
                            value_type=bool,
                        ),
                        "obstacle_narration": ParameterValue(
                            LaunchConfiguration("obstacle_narration"),
                            value_type=bool,
                        ),
                        **{
                            name: ParameterValue(
                                LaunchConfiguration(name), value_type=float)
                            for name in (
                                "patrol_dwell_sec", "patrol_intruder_repeat_sec",
                                "patrol_light_repeat_sec", "patrol_alarm_resend_sec",
                                "patrol_detector_timeout_sec", "patrol_arming_timeout_sec",
                                "patrol_speed_mps")
                        },
                        "patrol_max_leg_failures": ParameterValue(
                            LaunchConfiguration("patrol_max_leg_failures"),
                            value_type=int,
                        ),
                        "patrol_media_dir": ParameterValue(
                            LaunchConfiguration("patrol_media_dir"), value_type=str),
                    }
                ],
            ),
            Node(
                package="vica_mission_manager",
                executable="emergency_estop_bridge",
                output="screen",
                parameters=[{"pulse_sec": LaunchConfiguration("estop_pulse_sec")}],
            ),
            *_docking_nodes(map_id),
        ]
    )
