"""Launch raw wheel odometry and the VICA local EKF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("vica_localization")
    default_ekf_params = os.path.join(package_share, "config", "ekf.yaml")
    default_encoder_params = os.path.join(package_share, "config", "encoder.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_encoder = LaunchConfiguration("start_encoder")
    can_iface = LaunchConfiguration("can_iface")
    ekf_params_file = LaunchConfiguration("ekf_params_file")
    encoder_params_file = LaunchConfiguration("encoder_params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation or rosbag clock when true.",
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
        DeclareLaunchArgument(
            "ekf_params_file",
            default_value=default_ekf_params,
            description="Installed VICA robot_localization EKF configuration.",
        ),
        DeclareLaunchArgument(
            "encoder_params_file",
            default_value=default_encoder_params,
            description="Installed VICA wheel encoder configuration.",
        ),
        Node(
            package="encoder_feedback",
            executable="encoder_feedback",
            name="encoder_feedback",
            output="screen",
            condition=IfCondition(start_encoder),
            # 죽으면 스스로 다시 뜬다 (2026-09-07 실기).
            #
            # 이 노드가 죽으면 `/wheel/odom` 이 끊기고 EKF 의 odom TF 가 멈춘다.
            # 그 뒤 로봇은 조용히 무력화된다 — 회전 안내가 끊기고, TF 공백이
            # 길어지면 Nav2 의 RangeSensorLayer 가 미포착 예외로
            # controller_server 를 죽인다. 9/7 20:56 에 실제로 그렇게 됐고,
            # **모터 노드를 다시 띄워도 살아나지 않았다** — 엔코더는 이 launch
            # 소속이라 그쪽 재기동과 무관하기 때문이다.
            #
            # 2초를 두는 이유: 즉시 재시작하면 CAN 이 아직 정리되지 않은 상태로
            # 다시 열려 같은 자리에서 또 죽는다. 재기동에 걸리는 총 공백은
            # 2초 + 초기화 약 1.3초이고, CPU 여유가 있으면 그동안 EKF 가 예측
            # 으로 TF 를 메운다(같은 날 실측: load 13.65 에서 최대 공백 2.29초).
            #
            # 재시작이 반복되면 그 자체가 신호다. 로그에서 "Initial position
            # set" 이 계속 찍히면 근본 원인(CAN·전원)을 봐야 한다.
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                encoder_params_file,
                {
                    "can_iface": can_iface,
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "publish_tf": False,
                    "request_position_feedback": False,
                    "odom_topic": "/wheel/odom",
                },
            ],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[
                ekf_params_file,
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                },
            ],
            remappings=[
                ("odometry/filtered", "/odom"),
            ],
        ),
    ])
