"""Fault code catalog: the single source of truth for VICA fault descriptions.

fault code 하나가 컴포넌트, 기본 등급, 한국어 상세 문구 템플릿, 관리자 조치를
결정한다. ROS 의존이 없는 순수 모듈이므로 개발용 컴퓨터에서 그대로 검증한다.

문구를 로봇 쪽에 두는 이유:
    vica_system_health_monitoring_draft.md 3.5절은 "한국어 안내 문구는 앱과 TTS에서
    fault code를 기준으로 생성한다"고 제안했다. 그러나 앱에 문구 테이블을 두면 정본이
    두 저장소로 갈라지고, 문구 한 줄을 고치려고 앱을 다시 배포해야 한다. 그래서 이
    카탈로그가 정본이고 detail/suggested_action을 메시지에 실어 보낸다. fault_code도
    함께 보내므로 앱이 필요하면 자체 문구로 덮어쓸 수 있다.

등급을 여기서 확정하지 않는 이유:
    아래 severity는 기본값이다. 실제 등급은 required_components.yaml이 컴포넌트별로
    덮어쓸 수 있다. nvblox·카메라처럼 팀 위험성 평가가 필요한 항목(초안 19절)을
    코드 수정 없이 바꾸기 위한 구조다.
"""

from typing import Dict, NamedTuple, Optional


# RobotFault.msg의 SEVERITY_* 상수와 같은 값이다. 순수 모듈이 ROS 메시지를
# import하지 않도록 여기에 다시 정의하고, test_fault_catalog가 두 정의의 일치를
# 검증한다.
# 비상정지는 이 축에 없다. RobotFault.msg의 주석을 함께 본다 — E-stop은 STOP보다
# 심각한 등급이 아니라 종류가 다른 것이며 latched 플래그가 나타낸다.
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

# 알려진 컴포넌트. required_components.yaml의 key와 이 집합이 일치해야 한다
# (test_aggregator_config가 검증한다).
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
    """One catalog entry.

    detail_template은 str.format으로 채운다. 측정값이 없으면 템플릿의 자리표시자를
    그대로 노출하지 않고 fallback 문구를 쓴다(describe 참조).
    """

    component: str
    severity: int
    detail_template: str
    suggested_action: str


CATALOG: Dict[str, FaultSpec] = {
    # --- motor -------------------------------------------------------------
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
    # --- safety ------------------------------------------------------------
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
    # 감독이 /emergency_stop 하트비트를 제 시간에 못 받은 상태다. **아무도 누르지
    # 않았는데** 주행이 막히는 유일한 안전 상태라, 화면에 이름이 없으면 관리자가
    # 원인을 찾을 단서가 하나도 없다(2026-09-01 실기: "일부 기능 저하" 다섯 글자만
    # 떴다). 자동 복구에 넣지 않기로 한 결정(2026-09-02 사용자 판정)이 이 문구를
    # 더 중요하게 만든다 — 사람이 직접 풀어야 하므로 무엇을 할지 알아야 한다.
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
    # --- localization ------------------------------------------------------
    'LOCALIZATION_ODOM_STALE': FaultSpec(
        'localization',
        SEVERITY_STOP,
        'odometry가 {age_sec}초 동안 갱신되지 않았습니다.',
        'encoder와 EKF 노드 실행 상태를 확인해 주세요.',
    ),
    # /wheel/odom은 encoder_feedback의 원시 출력이고 /odom은 EKF 표준 출력이다
    # (vica_architecture.md 6절). 둘을 구분해야 encoder 문제와 EKF 문제를 가를 수 있다.
    # encoder_feedback이 죽으면 조용히 아무것도 나오지 않는다.
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
    # --- navigation --------------------------------------------------------
    'NAV2_NOT_ACTIVE': FaultSpec(
        'navigation',
        SEVERITY_STOP,
        'Nav2가 active 상태가 아닙니다. 현재 상태: {state}',
        'Nav2 lifecycle 상태를 확인하고 필요하면 다시 실행해 주세요.',
    ),
    # --- lidar -------------------------------------------------------------
    'LIDAR_SCAN_STALE': FaultSpec(
        'lidar',
        SEVERITY_STOP,
        '/scan이 {age_sec}초 동안 수신되지 않았습니다.',
        'LiDAR USB 연결과 rplidar 노드 실행 상태를 확인해 주세요.',
    ),
    # --- perception --------------------------------------------------------
    # nvblox·카메라 등급은 팀 위험성 평가 대상이다(초안 19절). 기본값은 DEGRADED이며
    # required_components.yaml에서 STOP으로 올릴 수 있다.
    # [2026-08-31] NVBLOX_SLICE_STALE 을 뺐다. Nav2 가 nvblox 를 쓰지 않는다 —
    # nav2_params.yaml 의 global·local plugins 어디에도 nvblox_layer 가 없다.
    # 쓰지 않는 것을 감시하면 앱에 결함이 상시로 떠서, 사람이 결함 표시 자체를
    # 무시하게 된다. 카메라는 아래 두 코드로 본다.
    #
    # 카메라 -> costmap 경로가 두 칸이라 결함도 둘로 갈린다.
    #   /camera/camera/depth/color/points -> depth_band_to_scan -> /camera/depth_scan
    # CAMERA_DEPTH_STALE 도 함께 뜨면 카메라가, 이것만 뜨면 변환 노드가 죽은 것이다.
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
    # --- guidance ----------------------------------------------------------
    'GUIDANCE_HANDLE_DISCONNECTED': FaultSpec(
        'guidance',
        SEVERITY_STOP,
        'Smart Handle 하향 통신이 끊겼습니다. 원인 코드: {fault}',
        '핸들 USB 연결을 확인해 주세요.',
    ),
    # 아두이노 → 젯슨 상향 통신 단절. 현재 상향 경로가 없어 발행되지 않으며
    # 터치센서와 상향 프로토콜이 추가되면 값만 흐른다(vica_scenario.md 2-1.3절).
    #
    # [중요] 손 놓음과 다른 사건이다. 통신이 끊기면 "다시 잡았다"를 감지할 수단도 함께
    # 사라지므로, 손 놓음처럼 "재접촉하면 재개"로 처리하면 영구히 재개되지 않는다.
    # 등급은 초안 19절 팀 확정 대상이며 활성 모드 STOP / 비활성 모드 DEGRADED를 제안한다.
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
    # --- voice / app -------------------------------------------------------
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
    # --- 진단 통과 통로 ----------------------------------------------------
    # 다른 노드에 diagnostic_updater가 추가되면 이 두 코드로 그대로 전달된다.
    # 카탈로그와 모니터 코드를 고치지 않고 aggregator yaml에 항목만 추가하면 된다.
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
    # --- 자기 진단 --------------------------------------------------------
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
    """Resolve a fault code into component, severity and Korean text.

    component와 severity를 넘기면 카탈로그 기본값을 덮어쓴다.
    required_components.yaml이 등급을 재정의하는 경로이며, DIAG_* 통로가 진단
    발행자에서 component를 받는 경로이기도 하다.

    알 수 없는 fault_code는 예외를 던지지 않는다. 감시 노드가 예외로 죽으면
    상태 표시 자체가 사라지므로, 원문을 그대로 노출하고 계속 동작한다.

    측정값이 부족해 템플릿을 채울 수 없으면 자리표시자를 그대로 보여주지 않고
    템플릿의 첫 문장만 fallback으로 쓴다.
    """
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
