"""Pure readiness and overall-state computation.

ROS 의존이 없다. 시각은 정수 나노초로 주입받고 스스로 조회하지 않는다.

READY 판정은 vica_system_health_monitoring_draft.md 7.2절을 그대로 옮긴다.

    READY = safety state가 IDLE 또는 READY_TO_GO
        AND 모든 필수 컴포넌트가 준비됨
        AND 필수 topic과 TF가 정해진 시간 안에 갱신됨

세 가지 판정 원칙:

1. **startup grace**: 기동 직후에는 미수신을 결함으로 올리지 않고 STARTING을 유지한다
   (초안 13절). motor·safety의 자체 timeout에는 영향을 주지 않는다.
2. **fail-closed**: grace 이후의 미수신은 고장으로 본다.
3. **관측 불가와 정상을 구분한다**: 관측 수단이 없는 컴포넌트는 READY가 아니라 UNKNOWN이다.
   READY로 보고하면 관리자 앱이 초록불을 띄워 잘못된 안심을 준다
   (SmartHandleState.msg 주석의 경고와 같은 문제).
"""

from typing import Dict, List, NamedTuple, Optional, Tuple

from .fault_catalog import (
    describe,
    SEVERITY_DEGRADED,
    SEVERITY_ESTOP,
    SEVERITY_FAULT,
    SEVERITY_OK,
    SEVERITY_STOP,
    SEVERITY_WARN,
)
from .freshness import is_fresh_ns


# RobotHealth.msg의 READINESS_* 상수와 같은 값이다.
UNKNOWN = 0
NOT_READY = 1
READY = 2

# RobotHealth.msg의 STATE_* 상수와 같은 값이다.
STATE_STARTING = 0
STATE_READY = 1
STATE_DEGRADED = 2
STATE_STOPPED = 3
STATE_ESTOPPED = 4
STATE_FAULT = 5

# vica_architecture.md 9.3절의 확정 enum.
SAFETY_STATES = (
    'IDLE',
    'RUNNING',
    'ESTOP_ACTIVE',
    'ESTOP_RELEASED_WAIT_RESET',
    'READY_TO_GO',
    'FAULT',
)

# 이 상태에서만 새 주행을 시작할 수 있다(초안 13.1절).
SAFETY_STATES_ALLOWING_START = ('IDLE', 'READY_TO_GO')


class ComponentProbe(NamedTuple):
    """감시 대상 하나의 현재 관측 결과.

    observable=False는 "관측 수단이 없다"는 뜻이다. 이때 last_seen_ns/ok는 무시하고
    UNKNOWN을 보고하며 결함으로 올리지 않는다. 하드웨어나 진단이 추가되면 호출자가
    observable=True로 바꾸기만 하면 된다.

    fault_code를 지정하지 않으면 컴포넌트 이름으로 기본 코드를 만든다.
    """

    name: str
    required: bool
    observable: bool
    last_seen_ns: Optional[int]
    ok: bool
    timeout_ns: int
    grace_ns: int
    severity: int
    fault_code: str = ''
    detail: str = ''


class SafetyInput(NamedTuple):
    """Safety Supervisor 상태. aggregator를 거치지 않고 직접 구독한다."""

    state: str
    estop_latched: bool
    fresh: bool


class Fault(NamedTuple):
    """판정된 결함 하나. severity 내림차순으로 정렬해 돌려준다."""

    component: str
    fault_code: str
    severity: int
    detail: str
    suggested_action: str
    latched: bool


class HealthSnapshot(NamedTuple):
    """한 tick의 판정 결과."""

    state: int
    readiness: Dict[str, int]
    faults: List[Fault]
    active_fault_count: int
    highest_severity: int
    primary_fault_code: str


def evaluate(
    probes: List[ComponentProbe],
    safety: SafetyInput,
    now_ns: int,
    started_ns: int,
) -> HealthSnapshot:
    """Compute readiness, faults and the overall state for this tick."""
    in_grace_globally = False
    readiness: Dict[str, int] = {}
    faults: List[Fault] = []

    for item in probes:
        level, fault = _judge_probe(item, now_ns, started_ns)
        readiness[item.name] = level
        if fault is not None:
            faults.append(fault)
        if level == UNKNOWN and item.observable:
            # 관측 가능하지만 아직 grace 중이라 판정을 보류한 경우다.
            in_grace_globally = True

    safety_fault = _judge_safety(safety)
    if safety_fault is not None:
        faults.append(safety_fault)

    faults.sort(key=lambda f: (-f.severity, f.component, f.fault_code))

    highest = faults[0].severity if faults else SEVERITY_OK
    primary = faults[0].fault_code if faults else ''

    state = _overall_state(
        faults=faults,
        highest=highest,
        readiness=readiness,
        probes=probes,
        safety=safety,
        in_grace=in_grace_globally,
    )

    return HealthSnapshot(
        state=state,
        readiness=readiness,
        faults=faults,
        active_fault_count=len(faults),
        highest_severity=highest,
        primary_fault_code=primary,
    )


def _judge_probe(
    item: ComponentProbe,
    now_ns: int,
    started_ns: int,
) -> Tuple[int, Optional[Fault]]:
    """Return (readiness level, fault or None) for one probe."""
    if not item.observable:
        # 관측 수단이 없다. 모른다고 보고하고 결함으로 올리지 않는다.
        return UNKNOWN, None

    fresh = is_fresh_ns(item.last_seen_ns, now_ns=now_ns, timeout_ns=item.timeout_ns)

    if not fresh and _in_grace(now_ns, started_ns, item.grace_ns):
        # 기동 유예 중이다. 아직 판정하지 않는다.
        return UNKNOWN, None

    if fresh and item.ok:
        return READY, _no_fault()

    return NOT_READY, _build_fault(item)


def _no_fault() -> None:
    """Readable placeholder for "healthy, no fault"."""
    return None


def _in_grace(now_ns: int, started_ns: int, grace_ns: int) -> bool:
    """Return True while still inside the startup grace window.

    기준은 마지막 수신 시각이 아니라 노드 기동 시각이다. 한 번도 못 받은 입력에
    유예를 주는 것이 목적이기 때문이다.
    """
    if grace_ns <= 0:
        return False
    elapsed = now_ns - started_ns
    if elapsed < 0:
        # 시간 역전. 유예를 주지 않는 쪽이 안전하다.
        return False
    return elapsed < grace_ns


def _build_fault(item: ComponentProbe) -> Fault:
    """Turn a failing probe into a Fault using the catalog."""
    code = item.fault_code or _default_fault_code(item.name)
    description = describe(
        code,
        component=item.name,
        severity=item.severity,
        name=item.name,
        message=item.detail,
    )
    detail = item.detail or description.detail
    return Fault(
        component=description.component,
        fault_code=code,
        severity=description.severity,
        detail=detail,
        suggested_action=description.suggested_action,
        latched=False,
    )


def _default_fault_code(component: str) -> str:
    """Fallback code when a probe does not name one."""
    return f'{component.upper()}_NOT_READY'


def _judge_safety(safety: SafetyInput) -> Optional[Fault]:
    """Turn the safety input into a fault when it is stale, latched or unknown."""
    if not safety.fresh:
        description = describe('SAFETY_STATE_STALE', age_sec='?')
        return Fault(
            component='safety',
            fault_code='SAFETY_STATE_STALE',
            severity=SEVERITY_STOP,
            detail=description.detail,
            suggested_action=description.suggested_action,
            latched=False,
        )

    if safety.estop_latched or safety.state == 'ESTOP_ACTIVE':
        description = describe('SAFETY_ESTOP_LATCHED', reason=safety.state)
        return Fault(
            component='safety',
            fault_code='SAFETY_ESTOP_LATCHED',
            severity=SEVERITY_ESTOP,
            detail=description.detail,
            suggested_action=description.suggested_action,
            latched=True,
        )

    if safety.state == 'ESTOP_RELEASED_WAIT_RESET':
        description = describe('SAFETY_RESET_REQUIRED')
        return Fault(
            component='safety',
            fault_code='SAFETY_RESET_REQUIRED',
            severity=SEVERITY_STOP,
            detail=description.detail,
            suggested_action=description.suggested_action,
            latched=True,
        )

    if safety.state not in SAFETY_STATES:
        # 정의되지 않은 값이다. 원인 불명으로 본다.
        return Fault(
            component='safety',
            fault_code='SAFETY_STATE_UNKNOWN',
            severity=SEVERITY_FAULT,
            detail=f'정의되지 않은 Safety 상태입니다: {safety.state}',
            suggested_action='Safety 상태 계약과 코드를 확인해 주세요.',
            latched=False,
        )

    return None


def _overall_state(
    faults: List[Fault],
    highest: int,
    readiness: Dict[str, int],
    probes: List[ComponentProbe],
    safety: SafetyInput,
    in_grace: bool,
) -> int:
    """Map faults, readiness and safety state onto one overall state.

    우선순위: ESTOP > FAULT > STOP > STARTING > DEGRADED > READY
    STARTING이 DEGRADED보다 뒤에 오는 이유는, 기동 중이라는 사실이 "일부 기능 저하"보다
    사용자에게 더 정확한 설명이기 때문이다. 단 ESTOP·FAULT·STOP은 기동 중에도 그대로
    보고한다.
    """
    if highest >= SEVERITY_ESTOP and _has_severity(faults, SEVERITY_ESTOP):
        return STATE_ESTOPPED
    if _has_severity(faults, SEVERITY_FAULT):
        return STATE_FAULT
    if _has_severity(faults, SEVERITY_STOP):
        return STATE_STOPPED

    if in_grace:
        return STATE_STARTING

    if _has_severity(faults, SEVERITY_DEGRADED):
        return STATE_DEGRADED

    if not _required_all_ready(readiness, probes):
        return STATE_STOPPED

    if safety.state not in SAFETY_STATES_ALLOWING_START:
        # RUNNING처럼 정상이지만 새 주행을 시작할 수 없는 상태다. 결함은 아니다.
        return STATE_DEGRADED

    # SEVERITY_WARN만 남은 경우는 READY를 유지한다(초안 9절: 조건부 허용).
    return STATE_READY


def _has_severity(faults: List[Fault], severity: int) -> bool:
    """Return True when any fault has exactly this severity."""
    return any(f.severity == severity for f in faults)


def _required_all_ready(
    readiness: Dict[str, int],
    probes: List[ComponentProbe],
) -> bool:
    """Return True when every required and observable probe is READY."""
    for item in probes:
        if not item.required:
            continue
        if not item.observable:
            # 관측 불가는 READY를 막지 않는다. UNKNOWN으로 표시할 뿐이다.
            continue
        if readiness.get(item.name) != READY:
            return False
    return True


def severity_to_state(severity: int) -> int:
    """Map a single severity onto the state it would cause. 표시용 보조 함수."""
    if severity >= SEVERITY_FAULT:
        return STATE_FAULT
    if severity >= SEVERITY_ESTOP:
        return STATE_ESTOPPED
    if severity >= SEVERITY_STOP:
        return STATE_STOPPED
    if severity >= SEVERITY_DEGRADED:
        return STATE_DEGRADED
    if severity >= SEVERITY_WARN:
        return STATE_READY
    return STATE_READY
