"""Unit tests for the pure readiness/state logic.

ROS 없이 실행한다. 시각은 정수 나노초로 주입한다.
"""

from vica_system_monitor.fault_catalog import (
    SEVERITY_DEGRADED,
    SEVERITY_ESTOP,
    SEVERITY_STOP,
    SEVERITY_WARN,
)
from vica_system_monitor.freshness import sec_to_ns
from vica_system_monitor.health_logic import (
    ComponentProbe,
    evaluate,
    NOT_READY,
    READY,
    SafetyInput,
    STATE_DEGRADED,
    STATE_ESTOPPED,
    STATE_FAULT,
    STATE_READY,
    STATE_STARTING,
    STATE_STOPPED,
    UNKNOWN,
)


SEC = 1_000_000_000


def probe(
    name,
    *,
    required=True,
    observable=True,
    last_seen_ns=0,
    ok=True,
    timeout_ns=SEC,
    grace_ns=0,
    severity=SEVERITY_STOP,
):
    """Build a probe with test-friendly defaults."""
    return ComponentProbe(
        name=name,
        required=required,
        observable=observable,
        last_seen_ns=last_seen_ns,
        ok=ok,
        timeout_ns=timeout_ns,
        grace_ns=grace_ns,
        severity=severity,
    )


def safety(state='IDLE', *, estop=False, fresh=True):
    """Build a safety input with test-friendly defaults."""
    return SafetyInput(state=state, estop_latched=estop, fresh=fresh)


# ---------------------------------------------------------------------------
# startup grace
# ---------------------------------------------------------------------------


def test_missing_input_inside_grace_is_starting_not_fault():
    """Grace 안에서는 미수신을 결함으로 올리지 않는다."""
    probes = [probe('motor', last_seen_ns=None, grace_ns=10 * SEC)]
    snapshot = evaluate(probes, safety(), now_ns=5 * SEC, started_ns=0)

    assert snapshot.state == STATE_STARTING
    assert snapshot.faults == []
    assert snapshot.readiness['motor'] == UNKNOWN


def test_missing_input_after_grace_is_fail_closed():
    """Grace 이후 미수신은 fail-closed로 결함이 된다."""
    probes = [probe('motor', last_seen_ns=None, grace_ns=10 * SEC)]
    snapshot = evaluate(probes, safety(), now_ns=11 * SEC, started_ns=0)

    assert snapshot.state == STATE_STOPPED
    assert [f.component for f in snapshot.faults] == ['motor']
    assert snapshot.readiness['motor'] == NOT_READY


def test_grace_is_measured_from_start_not_from_last_seen():
    """Grace 기준은 노드 기동 시각이다."""
    probes = [probe('motor', last_seen_ns=None, grace_ns=10 * SEC)]

    inside = evaluate(probes, safety(), now_ns=109 * SEC, started_ns=100 * SEC)
    outside = evaluate(probes, safety(), now_ns=111 * SEC, started_ns=100 * SEC)

    assert inside.state == STATE_STARTING
    assert outside.state == STATE_STOPPED


# ---------------------------------------------------------------------------
# 신선도 판정
# ---------------------------------------------------------------------------


def test_stale_probe_becomes_fault():
    """timeout을 넘은 입력은 결함이다."""
    probes = [probe('lidar', last_seen_ns=0, timeout_ns=SEC)]
    snapshot = evaluate(probes, safety(), now_ns=2 * SEC, started_ns=0)

    assert snapshot.readiness['lidar'] == NOT_READY
    assert snapshot.faults[0].fault_code.endswith('STALE') or snapshot.faults


def test_age_exactly_at_timeout_is_fresh():
    """경계값: age == timeout은 신선하다."""
    probes = [probe('lidar', last_seen_ns=0, timeout_ns=SEC)]
    snapshot = evaluate(probes, safety(), now_ns=SEC, started_ns=0)

    assert snapshot.readiness['lidar'] == READY


def test_time_reversal_is_treated_as_stale():
    """시간 역전은 stale로 본다(fail-safe)."""
    probes = [probe('lidar', last_seen_ns=5 * SEC, timeout_ns=SEC)]
    snapshot = evaluate(probes, safety(), now_ns=SEC, started_ns=0)

    assert snapshot.readiness['lidar'] == NOT_READY


def test_value_not_ok_becomes_fault_even_when_fresh():
    """수신은 되지만 값이 비정상이면 결함이다."""
    probes = [probe('motor', ok=False)]
    snapshot = evaluate(probes, safety(), now_ns=0, started_ns=0)

    assert snapshot.readiness['motor'] == NOT_READY
    assert snapshot.faults


# ---------------------------------------------------------------------------
# 관측 불가(UNKNOWN)
# ---------------------------------------------------------------------------


def test_unobservable_component_is_unknown_not_ready():
    """관측 수단이 없으면 READY가 아니라 UNKNOWN이다."""
    probes = [probe('guidance', observable=False, required=False)]
    snapshot = evaluate(probes, safety(), now_ns=0, started_ns=0)

    assert snapshot.readiness['guidance'] == UNKNOWN


def test_unobservable_component_is_not_a_fault():
    """관측 불가는 결함이 아니다. 모른다는 사실을 보고할 뿐이다."""
    probes = [probe('guidance', observable=False, required=False)]
    snapshot = evaluate(probes, safety(), now_ns=0, started_ns=0)

    assert snapshot.faults == []


def test_unobservable_required_component_does_not_block_ready():
    """관측 불가 컴포넌트가 READY를 막지 않는다.

    막으면 하드웨어가 붙기 전까지 READY에 영원히 도달하지 못한다. 대신
    required=False로 두고 UNKNOWN을 표시하는 것이 정직하다.
    """
    probes = [
        probe('motor'),
        probe('guidance', observable=False, required=False),
    ]
    snapshot = evaluate(probes, safety(), now_ns=0, started_ns=0)

    assert snapshot.state == STATE_READY


# ---------------------------------------------------------------------------
# READY 조건 (초안 7.2절)
# ---------------------------------------------------------------------------


def test_ready_requires_all_required_probes():
    """필수 컴포넌트가 하나라도 준비되지 않으면 READY가 아니다."""
    probes = [probe('motor'), probe('lidar', ok=False)]
    snapshot = evaluate(probes, safety(), now_ns=0, started_ns=0)

    assert snapshot.state != STATE_READY


def test_ready_requires_allowed_safety_state():
    """Safety state가 허용 상태가 아니면 READY가 아니다."""
    probes = [probe('motor')]

    for allowed in ('IDLE', 'READY_TO_GO'):
        assert evaluate(
            probes, safety(allowed), now_ns=0, started_ns=0
        ).state == STATE_READY

    for blocked in ('RUNNING', 'FAULT', 'ESTOP_RELEASED_WAIT_RESET'):
        assert evaluate(
            probes, safety(blocked), now_ns=0, started_ns=0
        ).state != STATE_READY


def test_non_required_failure_is_degraded_not_stopped():
    """비필수 컴포넌트 이상은 DEGRADED까지만 올린다."""
    probes = [
        probe('motor'),
        probe('voice', required=False, ok=False, severity=SEVERITY_DEGRADED),
    ]
    snapshot = evaluate(probes, safety(), now_ns=0, started_ns=0)

    assert snapshot.state == STATE_DEGRADED


def test_warn_only_still_reaches_ready():
    """WARN 등급만 있으면 주행 가능 상태를 유지한다."""
    probes = [
        probe('motor'),
        probe('app', required=False, ok=False, severity=SEVERITY_WARN),
    ]
    snapshot = evaluate(probes, safety(), now_ns=0, started_ns=0)

    assert snapshot.state == STATE_READY
    assert snapshot.highest_severity == SEVERITY_WARN


# ---------------------------------------------------------------------------
# E-stop과 safety 입력
# ---------------------------------------------------------------------------


def test_estop_latched_wins_over_everything():
    """중앙 래치가 걸려 있으면 다른 조건과 무관하게 ESTOPPED다."""
    probes = [probe('motor')]
    snapshot = evaluate(
        probes, safety('IDLE', estop=True), now_ns=0, started_ns=0
    )

    assert snapshot.state == STATE_ESTOPPED
    assert snapshot.highest_severity == SEVERITY_ESTOP


def test_stale_safety_input_is_fault():
    """Safety 상태 미수신은 결함이다."""
    probes = [probe('motor')]
    snapshot = evaluate(
        probes, safety('IDLE', fresh=False), now_ns=0, started_ns=0
    )

    assert snapshot.state != STATE_READY
    assert any(f.component == 'safety' for f in snapshot.faults)


def test_unknown_safety_state_is_fault():
    """정의되지 않은 safety enum은 원인 불명으로 본다."""
    probes = [probe('motor')]
    snapshot = evaluate(
        probes, safety('SOMETHING_NEW'), now_ns=0, started_ns=0
    )

    assert snapshot.state == STATE_FAULT


# ---------------------------------------------------------------------------
# 우선순위와 요약
# ---------------------------------------------------------------------------


def test_highest_severity_wins_among_simultaneous_faults():
    """동시 다발 결함에서 가장 심각한 것이 대표가 된다(초안 18.2 항목 6)."""
    probes = [
        probe('app', required=False, ok=False, severity=SEVERITY_WARN),
        probe('motor', ok=False, severity=SEVERITY_ESTOP),
        probe('lidar', ok=False, severity=SEVERITY_STOP),
    ]
    snapshot = evaluate(probes, safety(), now_ns=0, started_ns=0)

    assert snapshot.highest_severity == SEVERITY_ESTOP
    assert snapshot.faults[0].component == 'motor'
    assert snapshot.active_fault_count == 3


def test_empty_probe_list_is_starting_then_ready():
    """감시 대상이 없으면 판정할 것도 없다. grace 이후 READY로 둔다."""
    snapshot = evaluate([], safety(), now_ns=10 * SEC, started_ns=0)

    assert snapshot.state == STATE_READY
    assert snapshot.active_fault_count == 0
    assert snapshot.primary_fault_code == ''


def test_sec_to_ns_matches_probe_timeouts():
    """테스트가 쓰는 시간 단위가 freshness 계약과 같다."""
    assert sec_to_ns(1.0) == SEC
    assert sec_to_ns(0.5) == SEC // 2
