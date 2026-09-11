import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    vica_nav2_dir = get_package_share_directory("vica_nav2")
    vica_localization_dir = get_package_share_directory("vica_localization")

    bringup_launch = os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
    default_params = os.path.join(vica_nav2_dir, "config", "nav2_params.yaml")
    active_bt = os.path.join(
        vica_nav2_dir,
        "behavior_trees",
        "vica_navigate_to_pose_no_backup.xml",
    )
    wheel_ekf_launch = os.path.join(
        vica_localization_dir,
        "launch",
        "wheel_ekf.launch.py",
    )

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    start_localization = LaunchConfiguration("start_localization")
    start_encoder = LaunchConfiguration("start_encoder")
    can_iface = LaunchConfiguration("can_iface")

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={"default_nav_to_pose_bt_xml": active_bt},
        convert_types=True,
    )

    def keepout_actions(context):
        """금지구역 마스크 서버 두 개를 띄운다. 마스크 파일이 있을 때만 띄운다."""
        explicit = LaunchConfiguration("keepout_map").perform(context).strip()
        if explicit:
            keepout_yaml = explicit
        else:
            map_path = LaunchConfiguration("map").perform(context)
            stem = os.path.splitext(os.path.basename(map_path))[0]
            keepout_yaml = os.path.join(
                os.path.dirname(map_path), f"{stem}_keepout.yaml"
            )

        if not os.path.isfile(keepout_yaml):
            return [
                LogInfo(
                    msg=(
                        "[keepout] 마스크가 없어 금지구역 없이 실행한다: "
                        f"{keepout_yaml}"
                    )
                )
            ]

        return [
            LogInfo(msg=f"[keepout] 마스크를 적용한다: {keepout_yaml}"),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="keepout_filter_mask_server",
                output="screen",
                parameters=[configured_params, {"yaml_filename": keepout_yaml}],
                respawn=False,
            ),
            Node(
                package="nav2_map_server",
                executable="costmap_filter_info_server",
                name="keepout_costmap_filter_info_server",
                output="screen",
                parameters=[configured_params],
                respawn=False,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_keepout",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": True},
                    {
                        "node_names": [
                            "keepout_filter_mask_server",
                            "keepout_costmap_filter_info_server",
                        ]
                    },
                ],
                respawn=False,
            ),
        ]

    return LaunchDescription([
        DeclareLaunchArgument("map"),
        DeclareLaunchArgument(
            "keepout_map",
            default_value="",
            description=(
                "금지구역 마스크 YAML. 비우면 map 인자에서 <이름>_keepout.yaml 을 "
                "찾고, 그 파일이 없으면 금지구역 없이 실행한다."
            ),
        ),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("use_composition", default_value="False"),
        DeclareLaunchArgument(
            "start_localization",
            default_value="true",
            description="Start VICA wheel odometry and EKF.",
        ),
        DeclareLaunchArgument(
            "start_encoder",
            default_value="true",
            description="Start the read-only MDROBOT C5 encoder receiver.",
        ),
        DeclareLaunchArgument(
            "can_iface",
            default_value="can1",
            description="SocketCAN interface used by encoder_feedback.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(wheel_ekf_launch),
            condition=IfCondition(start_localization),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "start_encoder": start_encoder,
                "can_iface": can_iface,
            }.items(),
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="depth_band_to_scan",
            output="screen",
            remappings=[
                ("cloud_in", "/camera/camera/depth/color/points"),
                ("scan", "/camera/depth_scan"),
            ],
            parameters=[{
                "use_sim_time": use_sim_time,
                "target_frame": "base_footprint",
                "transform_tolerance": 0.05,
                "min_height": 0.30,
                "max_height": 1.05,
                "angle_min": -0.75,
                "angle_max": 0.75,
                "angle_increment": 0.0087,
                "scan_time": 0.0667,
                "range_min": 0.30,
                "range_max": 4.0,
                "use_inf": True,
                "queue_size": 1,
            }],
            respawn=False,
        ),
        OpaqueFunction(function=keepout_actions),
        GroupAction(
            actions=[
                SetRemap(
                    src="behavior_server:cmd_vel",
                    dst="/cmd_vel_req",
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(bringup_launch),
                    launch_arguments={
                        "slam": "False",
                        "map": map_yaml,
                        "params_file": configured_params,
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "use_composition": use_composition,
                        "use_respawn": "False",
                    }.items(),
                ),
                Node(
                    package="nav2_collision_monitor",
                    executable="collision_monitor",
                    name="collision_monitor",
                    output="screen",
                    parameters=[configured_params],
                    respawn=False,
                ),
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_collision_monitor",
                    output="screen",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"autostart": True},
                        {"node_names": ["collision_monitor"]},
                    ],
                    respawn=False,
                ),
            ],
        ),
    ])
