from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument("can_iface", default_value="can1"),
        DeclareLaunchArgument("driver_id", default_value="1"),
        # 실측/보정 대상: 오돔 거리 스케일에 직접 영향.
        DeclareLaunchArgument("wheel_radius_m", default_value="0.065"),
        # 실측/보정 대상: 제자리 회전 각도 스케일에 직접 영향.
        DeclareLaunchArgument("wheel_base_m", default_value="0.37"),
        # 주행 테스트 중 자주 낮춰 쓰는 제한값.
        DeclareLaunchArgument("max_linear_mps", default_value="1.0"),
        DeclareLaunchArgument("max_angular_radps", default_value="2.0"),
        DeclareLaunchArgument("max_rpm", default_value="400"),
        DeclareLaunchArgument("send_hz", default_value="30.0"),
        DeclareLaunchArgument("resend_interval_sec", default_value="0.05"),
        DeclareLaunchArgument("deadzone_pct", default_value="5"),
        DeclareLaunchArgument("knob_timeout_sec", default_value="0.8"),
        DeclareLaunchArgument("cmd_timeout_sec", default_value="0.5"),
        # 방향 테스트에서 전진/회전 부호가 반대일 때만 변경.
        DeclareLaunchArgument("invert_mot1", default_value="false"),
        DeclareLaunchArgument("invert_mot2", default_value="false"),
        # 저속에서 모터가 꿈틀거리기만 할 때만 30~50 정도로 변경.
        DeclareLaunchArgument("min_rpm_when_moving", default_value="0"),
    ]

    motor_node = Node(
        package="mdrobot_can_control",
        executable="keyboard_knob",
        name="mdrobot_can_keyboard_knob_node",
        output="screen",
        parameters=[
            {
                "can_iface": LaunchConfiguration("can_iface"),
                "driver_id": ParameterValue(
                    LaunchConfiguration("driver_id"), value_type=int
                ),
                "wheel_radius_m": ParameterValue(
                    LaunchConfiguration("wheel_radius_m"), value_type=float
                ),
                "wheel_base_m": ParameterValue(
                    LaunchConfiguration("wheel_base_m"), value_type=float
                ),
                "max_linear_mps": ParameterValue(
                    LaunchConfiguration("max_linear_mps"), value_type=float
                ),
                "max_angular_radps": ParameterValue(
                    LaunchConfiguration("max_angular_radps"), value_type=float
                ),
                "max_rpm": ParameterValue(
                    LaunchConfiguration("max_rpm"), value_type=int
                ),
                "send_hz": ParameterValue(
                    LaunchConfiguration("send_hz"), value_type=float
                ),
                "resend_interval_sec": ParameterValue(
                    LaunchConfiguration("resend_interval_sec"), value_type=float
                ),
                "deadzone_pct": ParameterValue(
                    LaunchConfiguration("deadzone_pct"), value_type=int
                ),
                "knob_timeout_sec": ParameterValue(
                    LaunchConfiguration("knob_timeout_sec"), value_type=float
                ),
                "cmd_timeout_sec": ParameterValue(
                    LaunchConfiguration("cmd_timeout_sec"), value_type=float
                ),
                "invert_mot1": ParameterValue(
                    LaunchConfiguration("invert_mot1"), value_type=bool
                ),
                "invert_mot2": ParameterValue(
                    LaunchConfiguration("invert_mot2"), value_type=bool
                ),
                "min_rpm_when_moving": ParameterValue(
                    LaunchConfiguration("min_rpm_when_moving"), value_type=int
                ),
            }
        ],
    )

    return LaunchDescription(launch_args + [motor_node])
