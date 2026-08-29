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
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    vica_nav2_dir = get_package_share_directory("vica_nav2")
    vica_localization_dir = get_package_share_directory("vica_localization")

    bringup_launch = os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
    default_params = os.path.join(vica_nav2_dir, "config", "nav2_params.yaml")
    # 2026-07-31: 복구가 costmap 초기화뿐인 측정용 트리로 바꾼다.
    #
    # 지금까지의 완주 성적은 recovery가 흡수해서 나온 것이고 순수 주행 실력은
    # 측정된 적이 없다. 풀어야 하는 문제가 "빠져나오기"가 아니라 "못 움직일
    # 자리에 애초에 안 들어가기"이므로, 로봇을 움직이는 복구(Spin·BackUp·Wait)를
    # 걷어내고 그 실력만 본다. ClearEntireCostmap은 로봇을 움직이지 않고,
    # 유령 장애물이 미해결이라 최소한의 원만한 주행을 위해 남긴다.
    # 상세는 behavior_trees/vica_navigate_to_pose_clearing_only.xml 주석 참고.
    #
    # 되돌릴 때는 아래 파일명을 vica_navigate_to_pose_no_backup.xml로 바꾼다.
    # 그 트리(BackUp 제거 + 좌우 Spin + Wait)는 그대로 남겨 두었다.
    # 2026-08-01: 측정용 clearing_only에서 제품용 no_backup으로 되돌린다.
    #
    # clearing_only는 "복구가 실패를 흡수해서 순수 주행 실력이 한 번도 측정된
    # 적이 없다"를 풀려고 만든 실험 트리였다. 오늘 그 측정을 했고 답이 나왔다 —
    # 안내소 -> 방2에서 7.1 m를 간 뒤 목표 3 m 앞에서 ABORT했고, 오른쪽이
    # 3.75 m 뚫려 있는데도 우회하지 못했다. 복구 없이는 못 빠져나온다.
    #
    # 측정이 끝났으므로 Spin(좌우 ±0.30 rad)과 Wait(5초)이 있는 트리로 돌아간다.
    # BackUp은 여전히 없다 — 핸들 뒤에 사람이 따라온다.
    #
    # 되돌릴 때는 아래 파일명을 vica_navigate_to_pose_clearing_only.xml로 바꾼다.
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

    # yaml에는 자리표시자만 두고 실제 경로를 여기서 넣는다. yaml은 설치 경로를
    # 계산할 수 없고, 소스 트리 절대경로를 박으면 다른 장비에서 깨진다.
    # RewrittenYaml은 '이미 존재하는 키'만 치환하므로 yaml에 키가 있어야 한다.
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={"default_nav_to_pose_bt_xml": active_bt},
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
        # ===== D455 깊이를 2D 스캔으로 눌러 costmap 에 넣는다 (2026-08-29) =====
        # 왜 3D 가 아니라 2D 인가:
        #   깊이를 voxel_layer 에 포인트클라우드로 그대로 넣었더니 **찍은 칸을
        #   지우지 못했다.** bag 실측(run2017, 846초)에서 깊이가 찍는 구간
        #   0.9~1.8 m 의 칸 수명이 중앙 21.6초(최대 191초)였고, 라이다 구간은
        #   0.6초였다.
        #   원인은 카메라 높이다. 지면 1.025 m 에서 그보다 낮은 칸을 지우려면
        #   그 칸을 스치는 광선이 훨씬 먼 바닥까지 닿아야 하는데(1.8 m·0.75 m
        #   칸이면 6.7 m), 실내에서는 3~4 m 앞 벽에 막혀 그만한 바닥이 안 보인다.
        #   라이다가 잘 지워지는 이유는 **자기와 같은 높이를 보기 때문**이다.
        #   그래서 깊이도 한 평면으로 눌러 같은 성질을 갖게 한다.
        #
        # GroupAction 밖에 둔다. 안쪽 SetRemap(148행)이 이 노드의 scan 출력을
        # 건드리면 라이다의 /scan 과 충돌할 수 있다.
        #
        # 카메라가 없어도 이 노드는 조용히 기다린다. 그때 costmap 의 depth_scan
        # 소스는 아무 일도 하지 않고 라이다만으로 돌아간다.
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
                # 지면 기준으로 스캔을 만든다. 아래 min/max_height 도 지면 기준이
                # 되어 nav2_params.yaml 의 설명과 단위가 맞는다.
                "target_frame": "base_footprint",
                "transform_tolerance": 0.05,
                # 볼 높이 띠. 근거는 nav2_params.yaml 의 depth_scan 주석에 있다.
                #   0.30  차체가 흔들려도 바닥이 안 걸리는 선(3 m 에서 5.7도)
                #   1.05  로봇 최고점(카메라 마스트). 그 위는 로봇을 넘어간다
                "min_height": 0.30,
                "max_height": 1.05,
                # D455 수평 시야 87도(+-43.5도)에서 가장자리를 조금 던다.
                "angle_min": -0.75,
                "angle_max": 0.75,
                "angle_increment": 0.0087,   # 0.5도. 빔 173개
                "scan_time": 0.0667,          # 15 Hz
                "range_min": 0.30,
                "range_max": 4.0,
                # 아무것도 없는 방향을 inf 로 낸다. costmap 의 inf_is_valid 가
                # 그것을 최대거리로 바꿔 빈 공간 청소에 쓴다.
                "use_inf": True,
                "queue_size": 1,
            }],
            respawn=False,
        ),
        GroupAction(
            actions=[
                # 2026-08-15 [NAV2-B5]: velocity_smoother와 /cmd_vel_req 사이에
                # collision_monitor를 끼운다. 배선은 아래와 같다.
                #
                #   velocity_smoother --cmd_vel_smoothed--> collision_monitor
                #       --/cmd_vel_req--> Safety --> motor
                #
                # 종전의 SetRemap(cmd_vel_smoothed -> /cmd_vel_req)은 지운다.
                # 남겨 두면 velocity_smoother가 monitor를 건너뛰고 /cmd_vel_req로
                # 직접 발행해 감시가 통째로 무의미해진다. 입출력 토픽은 이제
                # remap이 아니라 nav2_params.yaml의 collision_monitor 블록
                # (cmd_vel_in_topic / cmd_vel_out_topic)이 정한다.
                # /cmd_vel_req가 Safety Supervisor의 유일한 입력이라는 계약
                # (CLAUDE.md)은 그대로다. 발행자만 바뀐다.
                #
                # 왜 필요한가: 2026-08-15 run9 에서 사람이 정면 0.36 m 까지 붙자
                # DWB 궤적 419개가 전멸하고 24초를 복구에 썼다. footprint 정면
                # 경계가 0.355 m 이므로 5 mm 차이로 닿기 직전이었다. 그 자리에서
                # 빠져나오려면 회전해야 하는데, 손잡이 때문에 후방 반경이
                # 0.546 m 라 뒤가 막히면 회전도 못 한다(같은 회차 실측: 전방 0,
                # 후방·측면 100 이 46초 지속). 갇힌 뒤에 푸는 것보다 애초에
                # 그 거리까지 붙지 않게 하는 편이 확실하다.
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
                # collision_monitor는 nav2_bringup이 띄우지 않으므로 여기서 직접
                # 띄운다. 파라미터는 위와 같은 configured_params를 쓴다 — 같은
                # nav2_params.yaml의 collision_monitor 블록을 읽는다.
                #
                # GroupAction 안에 두지만 SetRemap의 영향은 받지 않는다. 남은
                # remap은 노드 지정(behavior_server:cmd_vel) 하나뿐이고,
                # collision_monitor의 입출력 토픽은 remap이 아니라 yaml
                # 파라미터로 정하기 때문이다.
                Node(
                    package="nav2_collision_monitor",
                    executable="collision_monitor",
                    name="collision_monitor",
                    output="screen",
                    parameters=[configured_params],
                    respawn=False,
                ),
                # 전용 lifecycle_manager. nav2_bringup의 lifecycle_nodes 목록
                # (navigation_launch.py 43행)에 collision_monitor가 없어서 기존
                # lifecycle_manager_navigation은 이 노드를 configure·activate하지
                # 않는다. 관리자를 따로 붙이지 않으면 노드는 unconfigured로 남아
                # 구독도 발행도 하지 않는다 — 살아 있는 것처럼 보이면서 아무 일도
                # 안 하므로 알아채기 어렵다.
                # lifecycle_manager_navigation은 건드리지 않는다.
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
