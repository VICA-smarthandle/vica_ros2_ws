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

    def keepout_actions(context):
        """금지구역 마스크 서버 두 개를 띄운다. 마스크 파일이 있을 때만 띄운다.

        OpaqueFunction 인 이유: 지도 경로는 실행할 때 정해지는 값이라
        LaunchConfiguration 을 여기서 perform 해야 파일 존재를 볼 수 있다.
        IfCondition 은 파일이 있는지 물어볼 수 없다.

        keepout_map 을 비워 두면 map 인자에서 <이름>_keepout.yaml 을 유도한다.
        사람이 지도 이름을 두 번 적게 만들면 언젠가 서로 다른 지도의 마스크를
        물린다 — 그때 로봇은 엉뚱한 자리를 막고도 아무 말을 하지 않는다.

        파일이 없으면 서버를 아예 띄우지 않는다. 그러면 KeepoutFilter 가
        2초마다 "Filter mask was not received" 만 찍고 주행은 종전과 같다.
        앱에서 금지구역을 한 번이라도 저장하면 keepout_map_node 가 빈 마스크를
        만들어 두므로, 그 뒤로는 이 분기가 항상 참이 된다.
        """
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
            # 원본 지도를 읽는 map_server 와 별개인 두 번째 Map Server 다.
            # 이름이 nav2_params.yaml 의 블록 이름과 같아야 파라미터를 받는다.
            #
            # 이 노드가 /keepout_filter_mask_server/load_map 서비스를 연다.
            # 앱에서 사각형을 고치면 keepout_map_node 가 그 서비스를 불러
            # 마스크만 갈아끼운다 — Nav2 재시작도, AMCL 재수렴도 없다.
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
            # 전용 lifecycle_manager. 이유는 collision_monitor 쪽 주석과 같다 —
            # nav2_bringup 의 lifecycle_nodes 목록에 이 둘이 없어서 관리자를
            # 붙이지 않으면 unconfigured 로 남아 아무것도 발행하지 않는다.
            # lifecycle_manager_navigation 은 건드리지 않는다.
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
        # 금지구역 마스크 서버. GroupAction 밖에 둔다 — 안쪽 SetRemap 은
        # behavior_server 지정이라 여기까지 오지 않지만, 마스크 서버는 Nav2
        # 주행 배선과 무관한 데이터 공급자라 섞지 않는 편이 읽기 쉽다.
        OpaqueFunction(function=keepout_actions),
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
                # 기동을 한 번 더 시킨다 (2026-09-02 실기 재발 후).
                #
                # **왜 필요한가.** collision_monitor 는 /cmd_vel_req 를 만드는
                # 유일한 노드다. 활성화에 실패하면 Nav2 가 멀쩡히 경로를 계산해도
                # 명령이 밖으로 못 나가고, 노드는 살아 있어서 목록에는 보인다 —
                # 알아채기 가장 어려운 고장이다. 실제로 두 번 겪었다
                # (2026-09-02 오전·저녁).
                #
                # **왜 실패하나.** 관리자가 get_state 를 부르면 nav2_util 의
                # ServiceClient 가 이렇게 동작한다.
                #
                #     while (!wait_for_service(1s)) { ... }      # 상대가 뜰 때까지 무한 대기
                #     async_send_request(...)
                #     spin_until_future_complete(future, 2s)     # 응답을 2초만 기다린다
                #
                # 요청 경로는 wait_for_service 가 보장하지만 **응답 경로는 보장하지
                # 않는다.** 기동 순간 DDS 가 아직 응답 writer 를 매칭하지 못하면
                # collision_monitor 가 답을 보내려다 버린다 — 그 순간이 로그에
                # "failed to send response ... client will not receive response"
                # 로 남는다. 관리자는 2초 뒤 포기하고 "Aborting bringup" 한다.
                # **재시도가 없다.**
                #
                # **왜 타이머 지연이 아닌가.** 실측이 반증한다 — 실패한 회차가
                # 오히려 더 늦게(프로세스 시작 +8.0초) 시작했고, 가장 빨리 시작한
                # 회차(+6.3초)는 성공했다. 지연과 성패에 상관이 없다. 이 경합은
                # 기다려서 피하는 것이 아니라 다시 해서 넘는 것이다.
                #
                # **왜 무해한가.** manage_nodes STARTUP 은 이미 active 인 노드에
                # 대해 아무 일도 하지 않는다. 성공한 회차에서는 로그 한 줄만 남는다.
                # 그래서 "실패했을 때만 살아나는" 안전핀이 된다.
                #
                # 12초로 두는 것은 실측 기준이다 — 정상 회차에서 Activating 이
                # 프로세스 시작 +9.0초에 끝났다. 그보다 뒤여야 정상 기동을
                # 방해하지 않는다.
                TimerAction(
                    period=12.0,
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                "ros2", "service", "call",
                                "/lifecycle_manager_collision_monitor/manage_nodes",
                                "nav2_msgs/srv/ManageLifecycleNodes",
                                "{command: 0}",   # 0 = STARTUP
                            ],
                            output="screen",
                            shell=False,
                        ),
                    ],
                ),
            ],
        ),
    ])
