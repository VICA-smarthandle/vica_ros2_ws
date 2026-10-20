"""Fault code catalog: the single source of truth for VICA fault descriptions."""

from typing import Dict, NamedTuple, Optional


SEVERITY_OK = 0
SEVERITY_WARN = 1
SEVERITY_DEGRADED = 2
SEVERITY_STOP = 3
SEVERITY_FAULT = 4

SEVERITY_NAMES: Dict[int, str] = {
    SEVERITY_OK: 'OK',
    SEVERITY_WARN: 'WARN',
    SEVERITY_DEGRADED: 'DEGRADED',
    SEVERITY_STOP: 'STOP',
    SEVERITY_FAULT: 'FAULT',
}

COMPONENTS = (
    'motor',
    'safety',
    'localization',
    'navigation',
    'lidar',
    'perception',
    'guidance',
    'voice',
    'app',
    'computer',
    'monitor',
)


class FaultSpec(NamedTuple):
    """One catalog entry."""

    component: str
    severity: int
    detail_template: str
    suggested_action: str


CATALOG: Dict[str, FaultSpec] = {
    'MOTOR_CAN_TIMEOUT': FaultSpec(
        'motor',
        SEVERITY_STOP,
        'CAN 응답이 {age_sec}초 동안 수신되지 않았습니다.',
        '로봇을 확인한 뒤 앱에서 안전 초기화를 실행해 주세요.',
    ),
    'MOTOR_CAN_FAILED': FaultSpec(
        'motor',
        SEVERITY_STOP,
        'CAN 통신이 끊겨 모터 출력을 0으로 유지하고 있습니다.',
        'CAN 케이블과 can1 링크를 확인한 뒤 앱에서 안전 초기화를 실행해 주세요.',
    ),
    'MOTOR_NODE_SILENT': FaultSpec(
        'motor',
        SEVERITY_STOP,
        '모터 노드 진단이 수신되지 않습니다. 노드가 종료됐을 수 있습니다.',
        '모터 노드 실행 상태를 확인한 뒤 앱에서 안전 초기화를 실행해 주세요.',
    ),
    'SAFETY_ESTOP_LATCHED': FaultSpec(
        'safety',
        SEVERITY_STOP,
        '비상정지가 걸려 있습니다. 원인: {reason}',
        '위험 원인을 해소한 뒤 앱에서 안전 초기화를 실행해 주세요.',
    ),
    'SAFETY_STATE_STALE': FaultSpec(
        'safety',
        SEVERITY_STOP,
        'Safety 상태가 {age_sec}초 동안 갱신되지 않았습니다.',
        'safety_supervisor_node 실행 상태를 확인해 주세요.',
    ),
    'SAFETY_SUPERVISOR_FAULT': FaultSpec(
        'safety',
        SEVERITY_STOP,
        '비상정지 신호 갱신이 늦어 FAULT 상태입니다.',
        '앱에서 비상정지를 걸었다 풀어주세요.',
    ),
    'SAFETY_RESET_REQUIRED': FaultSpec(
        'safety',
        SEVERITY_STOP,
        '정지 원인은 해소됐으나 관리자 초기화를 기다리고 있습니다.',
        '주변이 안전한지 확인한 뒤 앱에서 안전 초기화를 실행해 주세요.',
    ),
    'LOCALIZATION_ODOM_STALE': FaultSpec(
        'localization',
        SEVERITY_STOP,
        'odometry가 {age_sec}초 동안 갱신되지 않았습니다.',
        'encoder와 EKF 노드 실행 상태를 확인해 주세요.',
    ),
    'LOCALIZATION_WHEEL_ODOM_STALE': FaultSpec(
        'localization',
        SEVERITY_STOP,
        'encoder odometry가 {age_sec}초 동안 갱신되지 않았습니다.',
        'encoder_feedback 노드와 CAN 연결을 확인해 주세요.',
    ),
    'LOCALIZATION_TF_STALE': FaultSpec(
        'localization',
        SEVERITY_STOP,
        'map에서 base_footprint까지의 위치 변환이 {age_sec}초 동안 갱신되지 않았습니다.',
        'Nav2와 localization 실행 상태를 확인해 주세요.',
    ),
    'NAV2_NOT_ACTIVE': FaultSpec(
        'navigation',
        SEVERITY_STOP,
        'Nav2가 active 상태가 아닙니다. 현재 상태: {state}',
        'Nav2 lifecycle 상태를 확인하고 필요하면 다시 실행해 주세요.',
    ),
    'LIDAR_SCAN_STALE': FaultSpec(
        'lidar',
        SEVERITY_STOP,
        '/scan이 {age_sec}초 동안 수신되지 않았습니다.',
        'LiDAR USB 연결과 rplidar 노드 실행 상태를 확인해 주세요.',
    ),
    'DEPTH_SCAN_STALE': FaultSpec(
        'perception',
        SEVERITY_DEGRADED,
        '카메라 기반 장애물 스캔이 {age_sec}초 동안 수신되지 않았습니다. '
        'LiDAR 기반 2D 장애물 회피는 계속 동작합니다.',
        '카메라 연결과 depth_band_to_scan 노드 실행 상태를 확인해 주세요.',
    ),
    'CAMERA_DEPTH_STALE': FaultSpec(
        'perception',
        SEVERITY_DEGRADED,
        'depth 카메라 데이터가 {age_sec}초 동안 수신되지 않았습니다.',
        'D455 연결과 카메라 컨테이너 실행 상태를 확인해 주세요.',
    ),
    'CAMERA_COLOR_STALE': FaultSpec(
        'perception',
        SEVERITY_WARN,
        'color 카메라 데이터가 {age_sec}초 동안 수신되지 않았습니다.',
        'D455 연결 상태를 확인해 주세요.',
    ),
    'NAV2_CMD_PATH_DOWN': FaultSpec(
        'navigation',
        SEVERITY_STOP,
        '주행 명령이 나갈 길이 열리지 않았습니다. 목적지를 보내도 로봇이 움직이지 않습니다.',
        'Nav2 를 다시 띄워 주세요. collision_monitor 가 활성화되지 않은 상태입니다.',
    ),
    'GUIDANCE_HANDLE_DISCONNECTED': FaultSpec(
        'guidance',
        SEVERITY_STOP,
        'Smart Handle 하향 통신이 끊겼습니다. 원인 코드: {fault}',
        '핸들 USB 연결을 확인해 주세요.',
    ),
    'GUIDANCE_UPLINK_STALE': FaultSpec(
        'guidance',
        SEVERITY_DEGRADED,
        '핸들 아두이노에서 오는 신호가 {age_sec}초 동안 없습니다. '
        '주행은 계속할 수 있습니다.',
        '핸들 USB 연결과 아두이노 전원을 확인해 주세요. 드라이버는 살아 있으나 '
        '핸들 LED·서보와 초음파 거리 감지가 함께 멈춥니다.',
    ),
    'GUIDANCE_NODE_SILENT': FaultSpec(
        'guidance',
        SEVERITY_DEGRADED,
        '스마트 핸들 드라이버가 상태를 보내지 않습니다. 주행은 계속할 수 있습니다.',
        'user_guidance 드라이버 노드 실행 상태를 확인해 주세요. '
        '방향 안내와 초음파 거리 감지가 함께 멈춘 상태입니다.',
    ),
    'VOICE_NODE_SILENT': FaultSpec(
        'voice',
        SEVERITY_DEGRADED,
        '음성 노드가 실행되지 않고 있습니다. 주행은 계속할 수 있습니다.',
        '음성 파이프라인 실행 상태를 확인해 주세요. '
        '(이 항목은 노드 실행 여부만 확인합니다 — 마이크가 실제로 들리는지는 '
        '확인하지 못합니다.)',
    ),
    'APP_BRIDGE_SILENT': FaultSpec(
        'app',
        SEVERITY_WARN,
        '앱 브리지가 로봇 상태를 보내지 않습니다. 로봇은 정상 주행 중일 수 있습니다.',
        '앱 브리지 연결 상태를 확인해 주세요. 화면의 위치·주행 표시만 멈춘 것입니다.',
    ),
    'DIAG_COMPONENT_ERROR': FaultSpec(
        '',
        SEVERITY_STOP,
        '{message}',
        '해당 장치와 노드 상태를 확인해 주세요.',
    ),
    'DIAG_COMPONENT_WARN': FaultSpec(
        '',
        SEVERITY_WARN,
        '{message}',
        '해당 장치와 노드 상태를 확인해 주세요.',
    ),
    'DIAG_COMPONENT_STALE': FaultSpec(
        '',
        SEVERITY_STOP,
        '{name} 진단이 갱신되지 않았습니다.',
        '해당 노드 실행 상태를 확인해 주세요.',
    ),
    'MONITOR_DIAG_INPUT_STALE': FaultSpec(
        'monitor',
        SEVERITY_DEGRADED,
        '{source} 진단이 {age_sec}초째 오지 않습니다. 그 진단에 기대는 부품'
        '(라이다·위치추정·인지·안내·음성·앱)은 관측 불가로 표시합니다.',
        'monitor 칸(⑪-1)에서 external_diagnostics_node 실행 상태를 확인해 주세요. '
        '주행과 모터 안전 정지에는 영향이 없습니다.',
    ),
}


class FaultDescription(NamedTuple):
    """Resolved fault text ready to put into a RobotFault message."""

    component: str
    severity: int
    detail: str
    suggested_action: str


def describe(
    fault_code: str,
    component: Optional[str] = None,
    severity: Optional[int] = None,
    **measurements: object,
) -> FaultDescription:
    """Resolve a fault code into component, severity and Korean text."""
    spec = CATALOG.get(fault_code)
    if spec is None:
        return FaultDescription(
            component=component or 'monitor',
            severity=severity if severity is not None else SEVERITY_WARN,
            detail=f'알 수 없는 진단 코드입니다: {fault_code}',
            suggested_action='개발자에게 이 코드를 알려 주세요.',
        )

    try:
        detail = spec.detail_template.format(**measurements)
    except (KeyError, IndexError):
        detail = _fallback_detail(spec.detail_template)

    return FaultDescription(
        component=component or spec.component or 'monitor',
        severity=severity if severity is not None else spec.severity,
        detail=detail,
        suggested_action=spec.suggested_action,
    )


def _fallback_detail(template: str) -> str:
    """Strip format placeholders so the user never sees ``{age_sec}``."""
    result = []
    depth = 0
    for char in template:
        if char == '{':
            depth += 1
            continue
        if char == '}':
            depth = max(0, depth - 1)
            continue
        if depth == 0:
            result.append(char)
    return ''.join(result).replace('  ', ' ').strip()


def severity_name(severity: int) -> str:
    """Return the display name for a severity value."""
    return SEVERITY_NAMES.get(severity, f'UNKNOWN({severity})')
