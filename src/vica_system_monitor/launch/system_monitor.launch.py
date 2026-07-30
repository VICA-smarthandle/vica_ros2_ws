"""Launch the VICA observation layer: adapter, aggregator and monitor.

세 프로세스를 함께 띄운다.

    external_diagnostics_node   외부 대상(센서·프로세스)을 대신 진단
    aggregator_node             표준 진단 집계기 (diagnostic_aggregator 패키지)
    robot_health_monitor_node   등급 결정·문구·폭주 억제·앱 표시용 발행

`enable_aggregator:=false`로 aggregator 없이 단독 디버깅할 수 있다. 그때는 모니터의
`diagnostics_topic`을 `/diagnostics`로 넘겨 어댑터·모터 진단을 직접 읽는다.
`agg_parser`가 계층 name과 평면 name을 모두 파싱하므로 두 구성이 모두 동작한다.

이 launch는 정지 경로를 만들지 않는다. 모니터가 죽어도 모터 안전 정지는 유지된다
(guideline/vica_system_health_monitoring_draft.md 3.2절).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE = 'vica_system_monitor'

# aggregator를 쓸 때와 쓰지 않을 때 모니터가 읽는 토픽.
AGG_TOPIC = '/diagnostics_agg'
RAW_TOPIC = '/diagnostics'


def generate_launch_description() -> LaunchDescription:
    """Build the observation-layer launch description."""
    share = get_package_share_directory(PACKAGE)
    config = os.path.join(share, 'config')

    probes_yaml = os.path.join(config, 'probes.yaml')
    aggregator_yaml = os.path.join(config, 'diagnostic_aggregator.yaml')
    components_yaml = os.path.join(config, 'required_components.yaml')

    enable_aggregator = LaunchConfiguration('enable_aggregator')

    adapter = Node(
        package=PACKAGE,
        executable='external_diagnostics_node',
        # [중요] name은 probes.yaml의 최상위 키와 정확히 같아야 한다.
        # 다르면 파라미터가 조용히 무시되고 전부 기본값으로 동작한다.
        name='external_diagnostics_node',
        parameters=[probes_yaml],
        output='screen',
    )

    aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='aggregator_node',
        parameters=[aggregator_yaml],
        output='screen',
        condition=IfCondition(enable_aggregator),
    )

    # 같은 노드를 두 번 선언하는 대신 diagnostics_topic만 다르게 준다.
    # 파라미터 파일의 값을 뒤에서 덮어쓴다.
    monitor_with_agg = Node(
        package=PACKAGE,
        executable='robot_health_monitor_node',
        name='robot_health_monitor_node',
        parameters=[components_yaml, {'diagnostics_topic': AGG_TOPIC}],
        output='screen',
        condition=IfCondition(enable_aggregator),
    )

    monitor_standalone = Node(
        package=PACKAGE,
        executable='robot_health_monitor_node',
        name='robot_health_monitor_node',
        parameters=[components_yaml, {'diagnostics_topic': RAW_TOPIC}],
        output='screen',
        condition=UnlessCondition(enable_aggregator),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'enable_aggregator',
                default_value='true',
                description=(
                    'diagnostic_aggregator를 함께 띄운다. false면 모니터가 '
                    f'{RAW_TOPIC}를 직접 읽어 단독 디버깅한다.'
                ),
            ),
            adapter,
            aggregator,
            monitor_with_agg,
            monitor_standalone,
        ]
    )
