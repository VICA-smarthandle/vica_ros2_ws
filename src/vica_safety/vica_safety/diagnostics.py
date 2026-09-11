"""Diagnostic summaries for the three vica_safety nodes."""

OK = b'\x00'
WARN = b'\x01'
ERROR = b'\x02'
STALE = b'\x03'

LABEL_LATCH = 'safety: estop latch'
LABEL_GATE = 'safety: cmd_vel gate'
LABEL_BRIDGE = 'safety: app bridge'


def latch_summary(
    *,
    can_ready: bool,
    physical_fresh: bool,
    motor_can_fresh: bool,
    latch_state: str,
) -> tuple[int, str]:
    """Summarise whether the central latch node can still judge safety."""
    if not can_ready:
        return ERROR, '물리 비상정지 입력을 열지 못했습니다'
    if latch_state == 'WAITING_INPUT':
        return WARN, '부팅 후 첫 입력 대기 중입니다'
    if not physical_fresh:
        return ERROR, '물리 비상정지 입력이 끊겼습니다'
    if not motor_can_fresh:
        return WARN, '모터 CAN 신호가 없어 비상정지를 해제할 수 없습니다'
    return OK, f'정상 (래치 {latch_state})'


def gate_summary(*, estop_fresh: bool, gate_state: str) -> tuple[int, str]:
    """Summarise whether the drive gate still trusts the central latch."""
    if not estop_fresh:
        return ERROR, '중앙 비상정지 신호가 끊겨 주행 출력을 차단했습니다'
    return OK, f'정상 (게이트 {gate_state})'


def bridge_summary(
    *,
    emergency_fresh: bool,
    safety_state_fresh: bool,
) -> tuple[int, str]:
    """Summarise whether the app-facing bridge can still report and reset."""
    if not emergency_fresh:
        return ERROR, '중앙 비상정지 노드의 신호를 받지 못했습니다'
    if not safety_state_fresh:
        return WARN, 'Safety Supervisor 상태를 받지 못했습니다'
    return OK, '정상'


def sources_text(active_sources) -> str:
    """Render active latch sources for a diagnostic value."""
    return ','.join(active_sources) if active_sources else 'none'
