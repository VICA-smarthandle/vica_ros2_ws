"""Pure readiness and overall-state computation."""

from typing import Dict, List, NamedTuple, Optional, Tuple

from .fault_catalog import (
    describe,
    SEVERITY_DEGRADED,
    SEVERITY_FAULT,
    SEVERITY_OK,
    SEVERITY_STOP,
    SEVERITY_WARN,
)
from .freshness import is_fresh_ns


UNKNOWN = 0
NOT_READY = 1
READY = 2

STATE_STARTING = 0
STATE_READY = 1
STATE_DEGRADED = 2
STATE_STOPPED = 3
STATE_ESTOPPED = 4
STATE_FAULT = 5

SAFETY_STATES = (
    'IDLE',
    'RUNNING',
    'ESTOP_ACTIVE',
    'ESTOP_RELEASED_WAIT_RESET',
    'READY_TO_GO',
    'FAULT',
)

SAFETY_STATES_ALLOWING_START = ('IDLE', 'READY_TO_GO')


class ComponentProbe(NamedTuple):
    """감시 대상 하나의 현재 관측 결과."""

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
    ever_ok: bool = False


class SafetyInput(NamedTuple):
    """Safety Supervisor 상태. aggregator를 거치지 않고 직접 구독한다."""

    state: str
    estop_latched: bool
    fresh: bool
    age_sec: Optional[float] = None
    ever_fresh: bool = False


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
            in_grace_globally = True

    safety_fault = _judge_safety(
        safety,
        suppress_never_received=_safety_in_grace(
            safety, probes, now_ns=now_ns, started_ns=started_ns
        ),
    )
    if safety_fault is not None:
        faults.append(safety_fault)

    faults.sort(key=lambda f: (-f.severity, f.component, f.fault_code))

    highest = faults[0].severity if faults else SEVERITY_OK
    primary = faults[0].fault_code if faults else ''

    state = _overall_state(
        faults=faults,
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
        return UNKNOWN, None

    fresh = is_fresh_ns(item.last_seen_ns, now_ns=now_ns, timeout_ns=item.timeout_ns)
    healthy = fresh and item.ok

    if healthy:
        return READY, _no_fault()

    if not item.ever_ok and _in_grace(now_ns, started_ns, item.grace_ns):
        return UNKNOWN, None

    return NOT_READY, _build_fault(item)


def _no_fault() -> None:
    """Readable placeholder for "healthy, no fault"."""
    return None


def _in_grace(now_ns: int, started_ns: int, grace_ns: int) -> bool:
    """Return True while still inside the startup grace window."""
    if grace_ns <= 0:
        return False
    elapsed = now_ns - started_ns
    if elapsed < 0:
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


def _safety_in_grace(
    safety: SafetyInput,
    probes: List[ComponentProbe],
    now_ns: int,
    started_ns: int,
) -> bool:
    """Return True while a never-received safety state is still inside grace."""
    if safety.ever_fresh or safety.fresh:
        return False
    for item in probes:
        if item.name == 'safety':
            return _in_grace(now_ns, started_ns, item.grace_ns)
    return False


def _judge_safety(
    safety: SafetyInput,
    suppress_never_received: bool = False,
) -> Optional[Fault]:
    """Turn the safety input into a fault when it is stale, latched or unknown."""
    if safety.estop_latched:
        description = describe('SAFETY_ESTOP_LATCHED', reason=safety.state)
        return Fault(
            component='safety',
            fault_code='SAFETY_ESTOP_LATCHED',
            severity=SEVERITY_STOP,
            detail=description.detail,
            suggested_action=description.suggested_action,
            latched=True,
        )

    if not safety.fresh:
        if suppress_never_received:
            return None
        if safety.age_sec is None:
            detail = 'Safety 상태를 한 번도 수신하지 못했습니다.'
            action = 'safety_supervisor_node 실행 상태를 확인해 주세요.'
        else:
            description = describe(
                'SAFETY_STATE_STALE', age_sec=f'{safety.age_sec:.1f}'
            )
            detail = description.detail
            action = description.suggested_action
        return Fault(
            component='safety',
            fault_code='SAFETY_STATE_STALE',
            severity=SEVERITY_STOP,
            detail=detail,
            suggested_action=action,
            latched=False,
        )

    if safety.state == 'ESTOP_ACTIVE':
        description = describe('SAFETY_ESTOP_LATCHED', reason=safety.state)
        return Fault(
            component='safety',
            fault_code='SAFETY_ESTOP_LATCHED',
            severity=SEVERITY_STOP,
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
    readiness: Dict[str, int],
    probes: List[ComponentProbe],
    safety: SafetyInput,
    in_grace: bool,
) -> int:
    """Map faults, readiness and safety state onto one overall state."""
    if safety.estop_latched:
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
        return STATE_DEGRADED

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
            continue
        if readiness.get(item.name) != READY:
            return False
    return True


def severity_to_state(severity: int) -> int:
    """Map a single severity onto the state it would cause. 표시용 보조 함수."""
    if severity >= SEVERITY_FAULT:
        return STATE_FAULT
    if severity >= SEVERITY_STOP:
        return STATE_STOPPED
    if severity >= SEVERITY_DEGRADED:
        return STATE_DEGRADED
    if severity >= SEVERITY_WARN:
        return STATE_READY
    return STATE_READY
