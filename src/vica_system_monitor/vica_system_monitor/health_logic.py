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

from typing import Dict, List, NamedTuple, Optional, Sequence, Tuple

from .fault_catalog import (
    describe,
    SEVERITY_DEGRADED,
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

    ever_ok는 "기동 이후 한 번이라도 정상이었는가"다. 기동 유예의 판정 기준이며 호출자가
    누적한다. 신선도만으로는 유예를 판정할 수 없다 — aggregator는 아직 뜨지 않은 부품에
    대해 "Missing"을 1 Hz로 계속 발행하므로, 그 입력은 언제나 신선하다.
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
    ever_ok: bool = False


class SafetyInput(NamedTuple):
    """Safety Supervisor 상태. aggregator를 거치지 않고 직접 구독한다.

    age_sec은 표시용 측정값이다. 미수신이면 None이며, 그때 문구는 "한 번도 수신되지
    않았습니다"로 바뀐다. "?초"처럼 자리표시자가 사용자에게 보이면 안 된다.

    ever_fresh는 ComponentProbe.ever_ok와 같은 역할이다. 기동 유예를 "아직 안 뜬 것"에만
    적용하기 위해 필요하다.
    """

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
    extra_faults: Sequence[Fault] = (),
) -> HealthSnapshot:
    """Compute readiness, faults and the overall state for this tick.

    ``extra_faults`` 는 진단(/diagnostics_agg) 계열 밖에서 온 결함을 같은 판정에
    얹는 통로다. 주행 실패(`nav_failure.NavFailureTracker`)가 여기로 들어온다.

    호출부가 만든 observation 목록에 그냥 덧붙이면 안 되는 이유:
    그렇게 하면 `active_faults`(dedup 출처)에는 들어가지만 `highest_severity`·
    `primary_fault_code`(snapshot 출처)에는 반영되지 않아 **한 메시지 안에서 값이
    어긋난다**(docs/proposal_nav_failure_to_app.md 3.4절).

    probe 를 하나 더 만드는 방법도 쓰지 않는다 — 아래 ``readiness[item.name]`` 이
    같은 이름을 덮어써 기존 `NAV2_NOT_ACTIVE` 판정을 망가뜨린다. **extra_faults 는
    readiness 를 건드리지 않는다**: goal 하나가 실패해도 Nav2 는 살아 있다.
    """
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

    safety_fault = _judge_safety(
        safety,
        suppress_never_received=_safety_in_grace(
            safety, probes, now_ns=now_ns, started_ns=started_ns
        ),
    )
    if safety_fault is not None:
        faults.append(safety_fault)

    faults.extend(extra_faults)

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
        # 관측 수단이 없다. 모른다고 보고하고 결함으로 올리지 않는다.
        return UNKNOWN, None

    fresh = is_fresh_ns(item.last_seen_ns, now_ns=now_ns, timeout_ns=item.timeout_ns)
    healthy = fresh and item.ok

    if healthy:
        return READY, _no_fault()

    # 기동 유예는 "아직 안 뜬 것"만 봐준다.
    #
    # 판정 기준이 신선도가 아니라 ever_ok인 이유: aggregator는 아직 뜨지 않은 부품에도
    # "Missing"을 1 Hz로 계속 발행한다. 그 입력은 항상 신선하므로 신선도로 판정하면
    # 유예 분기에 도달조차 못 한다(2026-07-31 실기 전 검증에서 확인).
    #
    # 한 번이라도 정상이었다면 유예 안이라도 즉시 보고한다. 떴다가 죽은 것을 감추면
    # 기동 직후의 실제 고장이 최대 45초 동안 묻힌다.
    if not item.ever_ok and _in_grace(now_ns, started_ns, item.grace_ns):
        return UNKNOWN, None

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


def _safety_in_grace(
    safety: SafetyInput,
    probes: List[ComponentProbe],
    now_ns: int,
    started_ns: int,
) -> bool:
    """Return True while a never-received safety state is still inside grace.

    유예 창은 safety 컴포넌트의 정책을 그대로 쓴다. 별도 파라미터를 만들면 두 값이
    갈라져 "어느 쪽이 이기는지" 모호해진다.
    """
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
    """Turn the safety input into a fault when it is stale, latched or unknown.

    E-stop 래치는 suppress_never_received와 무관하게 항상 보고한다. 모니터는 정지
    권한이 없지만, 래치 사실을 늦게 알리면 관리자가 원인을 찾는 시간이 늘어난다.
    """
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
            # 기동 유예 중이고 한 번도 받은 적이 없다. 아직 판정하지 않는다.
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
    readiness: Dict[str, int],
    probes: List[ComponentProbe],
    safety: SafetyInput,
    in_grace: bool,
) -> int:
    """Map faults, readiness and safety state onto one overall state.

    우선순위: ESTOPPED > FAULT > STOPPED > STARTING > DEGRADED > READY
    STARTING이 DEGRADED보다 뒤에 오는 이유는, 기동 중이라는 사실이 "일부 기능 저하"보다
    사용자에게 더 정확한 설명이기 때문이다. 단 ESTOPPED·FAULT·STOPPED는 기동 중에도
    그대로 보고한다.

    **ESTOPPED는 실제 래치가 걸렸을 때만이다.** 이 상태 이름은 `/emergency_stop` 중앙
    래치가 소유하는 의미이고, 해제하려면 관리자 reset이 필요하다는 뜻을 담고 있다.
    등급(severity)에는 ESTOP이 없다 — 그 축은 "얼마나 나쁜가"만 답한다.
    """
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
    if severity >= SEVERITY_STOP:
        return STATE_STOPPED
    if severity >= SEVERITY_DEGRADED:
        return STATE_DEGRADED
    if severity >= SEVERITY_WARN:
        return STATE_READY
    return STATE_READY
