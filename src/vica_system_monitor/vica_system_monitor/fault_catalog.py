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
    'GUIDANCE_HANDLE_DISCONNECTED': FaultSpec(
        'guidance',
        SEVERITY_STOP,
        'Smart Handle 하향 통신이 끊겼습니다. 원인 코드: {fault}',
        '핸들 USB 연결을 확인해 주세요.',
    ),
    'GUIDANCE_UPLINK_STALE': FaultSpec(
        'guidance',
        SEVERITY_STOP,
        '핸들에서 오는 신호가 {age_sec}초 동안 수신되지 않았습니다. '
        '터치 상태를 확인할 수 없습니다.',
        '핸들 USB 연결과 아두이노 전원을 확인해 주세요.',
    ),
    'GUIDANCE_NODE_SILENT': FaultSpec(
        'guidance',
        SEVERITY_DEGRADED,
        '안내 장치 진단이 수신되지 않습니다.',
        'user_guidance_driver_node 실행 상태를 확인해 주세요.',
    ),
    'VOICE_NODE_SILENT': FaultSpec(
        'voice',
        SEVERITY_DEGRADED,
        '음성 기능 노드가 실행되지 않고 있습니다.',
        '음성 파이프라인 실행 상태를 확인해 주세요. 주행에는 영향이 없습니다.',
    ),
    'APP_BRIDGE_SILENT': FaultSpec(
        'app',
        SEVERITY_WARN,
        '앱 상태 브리지 노드가 실행되지 않고 있습니다.',
        'rosbridge와 상태 노드 실행 상태를 확인해 주세요.',
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
        SEVERITY_WARN,
        '진단 입력이 {age_sec}초 동안 수신되지 않았습니다. 상태 감시가 제한됩니다.',
        'diagnostic aggregator 실행 상태를 확인해 주세요. 모터 안전 정지는 유지됩니다.',
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
