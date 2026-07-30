"""Unit tests for the fault code catalog.

이 카탈로그가 한국어 문구의 정본이다. 사용자에게 자리표시자(`{age_sec}`)가 그대로 노출되면
안 되고, 알 수 없는 코드로 감시 노드가 죽어도 안 된다.
"""

from vica_system_monitor.fault_catalog import (
    CATALOG,
    COMPONENTS,
    describe,
    SEVERITY_DEGRADED,
    SEVERITY_ESTOP,
    severity_name,
    SEVERITY_NAMES,
    SEVERITY_OK,
    SEVERITY_STOP,
    SEVERITY_WARN,
)


# ---------------------------------------------------------------------------
# 카탈로그 정합성
# ---------------------------------------------------------------------------


def test_every_entry_has_known_component_or_passthrough():
    """component가 비어 있는 항목은 DIAG_ 통로뿐이다."""
    for code, spec in CATALOG.items():
        if spec.component == '':
            assert code.startswith('DIAG_'), code
        else:
            assert spec.component in COMPONENTS, code


def test_every_entry_has_severity_in_range():
    """등급이 정의된 범위 안이다."""
    for code, spec in CATALOG.items():
        assert spec.severity in SEVERITY_NAMES, code


def test_every_entry_has_nonempty_text():
    """문구와 조치가 비어 있으면 앱에 빈 칸이 뜬다."""
    for code, spec in CATALOG.items():
        assert spec.detail_template.strip(), code
        assert spec.suggested_action.strip(), code


def test_motor_and_estop_faults_are_estop_severity():
    """모터 통신 계열은 ESTOP 등급이다(초안 9.1절)."""
    for code in ('MOTOR_CAN_TIMEOUT', 'MOTOR_CAN_FAILED', 'MOTOR_NODE_SILENT'):
        assert CATALOG[code].severity == SEVERITY_ESTOP, code


def test_lidar_is_stop_severity():
    """LiDAR는 두 costmap의 유일한 장애물 입력이므로 STOP이다."""
    assert CATALOG['LIDAR_SCAN_STALE'].severity == SEVERITY_STOP


def test_perception_defaults_to_degraded():
    """nvblox·depth 기본 등급은 DEGRADED다. 초안 19-11의 팀 확정 대상이다."""
    assert CATALOG['NVBLOX_SLICE_STALE'].severity == SEVERITY_DEGRADED
    assert CATALOG['CAMERA_DEPTH_STALE'].severity == SEVERITY_DEGRADED


def test_voice_and_app_do_not_block_driving():
    """비필수 컴포넌트는 DEGRADED 이하다."""
    assert CATALOG['VOICE_NODE_SILENT'].severity <= SEVERITY_DEGRADED
    assert CATALOG['APP_BRIDGE_SILENT'].severity <= SEVERITY_DEGRADED


# ---------------------------------------------------------------------------
# describe: 문구 생성
# ---------------------------------------------------------------------------


def test_describe_fills_measurement():
    """측정값이 문구에 들어간다."""
    result = describe('LIDAR_SCAN_STALE', age_sec='2.1')

    assert '2.1' in result.detail
    assert result.component == 'lidar'
    assert result.severity == SEVERITY_STOP


def test_describe_never_leaks_placeholder():
    """측정값이 없어도 사용자에게 중괄호를 보여주지 않는다."""
    result = describe('LIDAR_SCAN_STALE')

    assert '{' not in result.detail
    assert '}' not in result.detail
    assert result.detail.strip()


def test_describe_unknown_code_does_not_raise():
    """알 수 없는 코드로 예외를 던지지 않는다.

    감시 노드가 예외로 죽으면 상태 표시 자체가 사라진다. 원문을 노출하고 계속 동작한다.
    """
    result = describe('SOMETHING_NEW_2026')

    assert 'SOMETHING_NEW_2026' in result.detail
    assert result.severity == SEVERITY_WARN
    assert result.component == 'monitor'


def test_describe_severity_override_wins():
    """required_components.yaml이 등급을 덮어쓸 수 있다.

    nvblox 등급을 팀이 DEGRADED에서 STOP으로 올릴 때 코드를 고치지 않게 하는 경로다.
    """
    default = describe('NVBLOX_SLICE_STALE')
    overridden = describe('NVBLOX_SLICE_STALE', severity=SEVERITY_STOP)

    assert default.severity == SEVERITY_DEGRADED
    assert overridden.severity == SEVERITY_STOP


def test_describe_component_override_wins():
    """DIAG_ 통로는 진단 발행자에서 컴포넌트를 받는다."""
    result = describe(
        'DIAG_COMPONENT_ERROR', component='navigation', message='planner failed'
    )

    assert result.component == 'navigation'
    assert 'planner failed' in result.detail


def test_diag_passthrough_without_component_falls_back():
    """component를 못 받아도 빈 문자열을 내지 않는다."""
    result = describe('DIAG_COMPONENT_ERROR', message='something')

    assert result.component == 'monitor'


def test_uplink_stale_exists_for_smart_handle():
    """상향 통신 단절 코드를 미리 둔다.

    터치센서와 상향 프로토콜이 추가되면 값만 흐르고 카탈로그는 고치지 않는다
    (vica_scenario.md 2-1.3절).
    """
    assert 'GUIDANCE_UPLINK_STALE' in CATALOG
    assert CATALOG['GUIDANCE_UPLINK_STALE'].component == 'guidance'


# ---------------------------------------------------------------------------
# 표시 보조
# ---------------------------------------------------------------------------


def test_severity_name_covers_all_levels():
    """모든 등급에 표시 이름이 있다."""
    for level in (
        SEVERITY_OK,
        SEVERITY_WARN,
        SEVERITY_DEGRADED,
        SEVERITY_STOP,
        SEVERITY_ESTOP,
    ):
        assert severity_name(level) != ''
        assert 'UNKNOWN' not in severity_name(level)


def test_severity_name_handles_unexpected_value():
    """정의되지 않은 값에도 예외를 던지지 않는다."""
    assert 'UNKNOWN' in severity_name(99)
