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
            DeclareLaunchArgument("estop_release_grace_sec", default_value="1.0"),
            # 사람에게 다가가는 구간의 최대속도(주행 상한의 %). 기본 100 % = 0.5 m/s.
            # 등록 목적지와 달리 감속 사다리 없이 처음부터 끝까지 이 값이다.
            # 2026-09-09 실기: 60 % 일 때 접근 19.6 초 -> 100 % 에서 7.9 초.
            DeclareLaunchArgument(
                "person_approach_speed_percent", default_value="100.0"),
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
            # 접근(거절·무응답) 뒤 홈 복귀. 2026-09-04 사용자 결정으로 기본 켬 —
            # 촬영에서 "물러납니다" 하고 제자리에 서 있는 것이 어색했다.
            # 끄려면 auto_return_home:=false. 켜면 사람이 부르지 않아도 로봇이
            # 홈까지 달리므로, 통행이 잦은 곳에서는 끄는 편이 안전하다.
            DeclareLaunchArgument("auto_return_home", default_value="true"),
            # 마이크 각도 증가 방향(+1 반시계 / -1 시계, 호출 접근 설계 §5).
            # 실측값 +1.0 확정(2026-09-10, 컨트롤러가 사용자와 2회 걷기 측정) —
            # 로봇을 마주 보고 사용자 기준 오른쪽으로 이동 -> 마이크 각도 0°에서
            # 119°로 증가 -> 반시계 증가 -> Nav2 규약과 같은 부호. 틀리면
            # 로봇이 호출 방향의 정반대로 돈다.
            DeclareLaunchArgument("wake_doa_sign", default_value="1.0"),
            # 고개를 돌린 뒤 사람을 찾는 시간(초). 회전 완료 갱신(1 Hz, 최대
            # 1.0 s)과 detection_gate 의 stable 1.0 s + still window 3.0 s 를
            # 더한 바닥값이 4.2~4.5 s 라 여유가 1.5 s 뿐이었다 — 8.0 으로 올린다
            # (2026-09-10 재검토. 근거는 mission_logic.SEEK_LOOK_SEC 주석).
            DeclareLaunchArgument("seek_look_sec", default_value="8.0"),
            # 근접 호출(2026-09-10 확장). 부른 사람이 이보다 가까우면 접근 goal
            # (1.1 m)이 이미 지나간 자리라 걸어가지 않고 그 자리에서 바로
            # 질문한다. vica_perception detection_gate 의 min_distance_m 과 값은
            # 같지만(1.5) 별개 파라미터다 — Mission 은 그 감지기 상수를 모른다.
            DeclareLaunchArgument("near_call_max_m", default_value="1.5"),
            # 이보다 가까우면 수락해도 회전하지 않는다 — 손잡이가 뒤로 길게 나와
            # 있어 이 거리의 180도 회전은 손잡이가 사람을 칠 수 있다
            # (mission_logic.NEAR_CALL_NO_SPIN_M 주석, 2026-09-10 사용자 결정).
            DeclareLaunchArgument("near_call_no_spin_m", default_value="1.0"),
            # 핸들 쪽(로봇 뒤) 호출 사각지대(도) — 정면 사각지대(10°)의
            # 거울쌍이다. 회전량이 이보다 크면(부채꼴 180°±45°) 소리가 핸들
            # 옆에서 왔다는 뜻이라 카메라 확인 없이 곧바로 접근 질문을 낸다
            # (mission_logic.HANDLE_SIDE_MIN_YAW_RAD 주석, 2026-09-10 사용자
            # 결정). 실기에서 뒤쪽 호출의 DOA 가 163°~185° 안에 들어와 ±45°는
            # 넉넉한 여유다 — 실기에서 폭을 조정한다.
            DeclareLaunchArgument("handle_side_min_yaw_deg", default_value="135.0"),
            # 홈 복귀 중 호출로 브레이크가 걸린 뒤 이만큼 침묵하면 떠나기
            # 예고를 내고(MSG_LEAVING_NOTICE 재사용) LEAVING_GRACE_SEC(3초)
            # 뒤 복귀를 재개한다(2026-09-10 사용자 승인 흐름). 기준은
            # 브레이크가 걸린 시각 — 청취 창(음성 쪽) 길이와는 무관하다.
            DeclareLaunchArgument("return_resume_sec", default_value="15.0"),
            # 온보딩("이제 어디로 가고 싶으신가요?") 뒤 STT 가 빈손으로
            # 닫히면 한 번 되묻고, 그래도 빈손이면 이만큼 더 기다렸다 떠남을
            # 예고한다(실기 2026-09-11). return_resume_sec 과 값·뜻이 같다 —
            # 근거는 mission_logic.DEST_RETRY_RETURN_SEC 주석.
            DeclareLaunchArgument("dest_retry_return_sec", default_value="15.0"),
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
                        "person_approach_speed_percent": ParameterValue(
                            LaunchConfiguration("person_approach_speed_percent"),
                            value_type=float,
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
