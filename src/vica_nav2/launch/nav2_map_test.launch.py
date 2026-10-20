import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def select_bt_expression(route_bt, active_bt):
    """use_route:=true **이고 레일 파일이 있을 때만** 레일 트리를 고른다."""
    return PythonExpression([
        "'", route_bt, "' if ('", LaunchConfiguration("use_route"),
        "'.lower() in ('true', '1') and __import__('os').path.isfile(('",
        LaunchConfiguration("route_graph"),
        "'.strip()) or __import__('os').path.splitext('",
        LaunchConfiguration("map"),
        "')[0] + '_route.geojson')) else '", active_bt, "'",
    ])


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
    route_bt = os.path.join(
        vica_nav2_dir,
        "behavior_trees",
        "vica_navigate_to_pose_route.xml",
    )
    selected_bt = select_bt_expression(route_bt, active_bt)
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
        param_rewrites={"default_nav_to_pose_bt_xml": selected_bt},
        convert_types=True,
    )

    def route_actions(context):
        """레일(Route Server)을 띄운다. use_route:=true 이고 파일이 있을 때만."""
        if LaunchConfiguration("use_route").perform(context).lower() not in (
            "true", "1",
        ):
            return []

        explicit = LaunchConfiguration("route_graph").perform(context).strip()
        if explicit:
            graph = explicit
        else:
            map_path = LaunchConfiguration("map").perform(context)
            stem = os.path.splitext(os.path.basename(map_path))[0]
            graph = os.path.join(os.path.dirname(map_path), f"{stem}_route.geojson")

        if not os.path.isfile(graph):
            return [
                LogInfo(msg=f"[route] **레일 파일이 없다**: {graph}"),
                LogInfo(msg="[route] scripts/vica_route_graph.py <지도> 로 만든다. "
                            "지금은 레일 없이(자유 주행) 달린다."),
            ]

        return [
            LogInfo(msg=f"[route] 레일을 적용한다: {graph}"),
            Node(
                package="nav2_route",
                executable="route_server",
                name="route_server",
                output="screen",
                parameters=[configured_params, {"graph_filepath": graph}],
                remappings=[("plan", "/rail_plan")],
                respawn=False,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_route",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": True},
                    {"node_names": ["route_server"]},
                ],
                respawn=False,
            ),
        ]

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

        keepout_rescue_sh = (
            "for n in keepout_filter_mask_server keepout_costmap_filter_info_server; do "
            "st=$(ros2 lifecycle get \"/$n\" 2>&1 | head -n 1 | awk '{print $1}'); "
            "case \"$st\" in "
            "active) echo \"[keepout] $n: 이미 active, 할 일 없음\";; "
            "unconfigured) echo \"[keepout] $n: unconfigured, configure+activate 시킨다\"; "
            "ros2 lifecycle set \"/$n\" configure && ros2 lifecycle set \"/$n\" activate;; "
            "inactive) echo \"[keepout] $n: inactive, activate 시킨다\"; "
            "ros2 lifecycle set \"/$n\" activate;; "
            "*) echo \"[keepout] $n: 상태를 못 읽었다: $st\";; "
            "esac; done"
        )

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
            TimerAction(
                period=12.0,
                actions=[
                    ExecuteProcess(
                        cmd=["bash", "-c", keepout_rescue_sh],
                        name="keepout_rescue",
                        output="screen",
                        shell=False,
                    ),
                ],
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
        DeclareLaunchArgument(
            "use_route",
            default_value="false",
            description=(
                "레일(Route Server)을 쓴다. 켜면 BT 도 레일 트리로 함께 바뀐다. "
                "레일 파일은 scripts/vica_route_graph.py 로 만든다."
            ),
        ),
        DeclareLaunchArgument(
            "route_graph",
            default_value="",
            description=(
                "레일 GeoJSON. 비우면 map 인자에서 <이름>_route.geojson 을 찾는다."
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
        OpaqueFunction(function=route_actions),
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
                TimerAction(
                    period=12.0,
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                "ros2", "service", "call",
                                "/lifecycle_manager_collision_monitor/manage_nodes",
                                "nav2_msgs/srv/ManageLifecycleNodes",
                                "{command: 0}",
                            ],
                            output="screen",
                            shell=False,
                        ),
                    ],
                ),
            ],
        ),
    ])
