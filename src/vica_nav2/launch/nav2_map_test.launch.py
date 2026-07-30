import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    vica_nav2_dir = get_package_share_directory("vica_nav2")
    vica_localization_dir = get_package_share_directory("vica_localization")

    bringup_launch = os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
    default_params = os.path.join(vica_nav2_dir, "config", "nav2_params.yaml")
    # 기본 BT에서 <BackUp/>만 제거한 트리. 근거는 nav2_params.yaml의
    # bt_navigator 주석과 이 파일 상단 주석을 참고한다.
    no_backup_bt = os.path.join(
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

    # yaml에는 자리표시자만 두고 실제 경로를 여기서 넣는다. yaml은 설치 경로를
    # 계산할 수 없고, 소스 트리 절대경로를 박으면 다른 장비에서 깨진다.
    # RewrittenYaml은 '이미 존재하는 키'만 치환하므로 yaml에 키가 있어야 한다.
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={"default_nav_to_pose_bt_xml": no_backup_bt},
        convert_types=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument("map"),
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
        GroupAction(
            actions=[
                # Humble Nav2는 velocity_smoother의 출력을 기본적으로
                # /cmd_vel로 remap한다. 모든 주행 명령이 VICA Safety Supervisor를
                # 거치도록 최종 출력만 /cmd_vel_req로 변경한다.
                SetRemap(
                    src="cmd_vel_smoothed",
                    dst="/cmd_vel_req",
                ),
                # behavior_server(Spin/BackUp/Wait/DriveOnHeading)는 Nav2
                # navigation_launch.py에서 cmd_vel을 remap받지 못한다. controller만
                # ('cmd_vel', 'cmd_vel_nav')를 받는다. 그래서 복구 동작은 /cmd_vel로
                # 나가는데 VICA에는 그 토픽 구독자가 없다.
                #
                # 2026-07-29 실측: /cmd_vel 발행자 5(전부 behavior_server), 구독자 0.
                # 즉 Spin·BackUp이 한 번도 로봇을 움직인 적이 없다. planner가
                # "Starting point in lethal space"로 실패하면 복구 6회가 전부 헛돌고
                # 곧바로 Goal failed가 된다 — 한 번 갇히면 빠져나올 수단이 없었다.
                #
                # Nav2 기본 구성에서도 behavior_server는 velocity_smoother를 거치지
                # 않고 로봇의 최종 속도 토픽으로 직접 발행한다(복구 동작은 자체
                # 램프를 갖는다). VICA의 그 토픽은 /cmd_vel_req이므로 Safety
                # Supervisor 경로(CLAUDE.md)는 그대로 유지된다.
                #
                # 노드 지정 문법(node_name:from:=to)을 쓴다. 접두사 없는 전역
                # remap은 global이 node-level보다 먼저 적용되어(launch_ros
                # node.py 468~477행) controller_server의 cmd_vel:=cmd_vel_nav를
                # 덮어쓰고 velocity_smoother를 자기루프로 만든다.
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
            ],
        ),
    ])
