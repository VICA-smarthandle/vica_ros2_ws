"""Unit tests for the pure readiness/state logic.

ROS 없이 실행한다. 시각은 정수 나노초로 주입한다.
"""

from vica_system_monitor.fault_catalog import (
    SEVERITY_DEGRADED,
    SEVERITY_FAULT,
    SEVERITY_STOP,
    SEVERITY_WARN,
)
from vica_system_monitor.freshness import sec_to_ns
from vica_system_monitor.health_logic import (
    ComponentProbe,
    evaluate,
    Fault,
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
    ever_ok=False,
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
        ever_ok=ever_ok,
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
    # 래치 결함은 주행 불가 등급이고, 래치라는 사실은 latched 플래그가 나타낸다.
    assert snapshot.highest_severity == SEVERITY_STOP
    assert any(f.latched for f in snapshot.faults)


def test_stale_safety_input_is_fault():
    """Safety 상태 미수신은 결함이다."""
    probes = [probe('motor')]
    snapshot = evaluate(
        probes, safety('IDLE', fresh=False), now_ns=0, started_ns=0
    )

    assert snapshot.state != STATE_READY
    assert any(f.component == 'safety' for f in snapshot.faults)


def test_never_received_safety_says_so_instead_of_showing_placeholder():
    """한 번도 못 받은 것과 오래된 것을 문구로 구분한다.

    age_sec이 없을 때 '?초'처럼 자리표시자를 노출하면 사용자가 읽을 수 없다.
    실기동에서 실제로 그렇게 나왔던 회귀를 고정한다.
    """
    probes = [probe('motor')]
    snapshot = evaluate(
        probes, safety('IDLE', fresh=False), now_ns=0, started_ns=0
    )

    detail = next(f.detail for f in snapshot.faults if f.component == 'safety')
    assert '?' not in detail
    assert '{' not in detail
    assert '한 번도' in detail


def test_stale_safety_with_age_shows_the_measurement():
    """age를 받으면 실제 값을 문구에 넣는다."""
    probes = [probe('motor')]
    snapshot = evaluate(
        probes,
        SafetyInput(state='IDLE', estop_latched=False, fresh=False, age_sec=2.4),
        now_ns=0,
        started_ns=0,
    )

    detail = next(f.detail for f in snapshot.faults if f.component == 'safety')
    assert '2.4' in detail
    assert '?' not in detail


def test_no_fault_detail_ever_leaks_a_placeholder():
    """어떤 결함 문구에도 중괄호나 물음표가 남지 않는다.

    앱이 그대로 표시하므로 자리표시자가 새면 사용자에게 보인다.
    """
    probes = [
        probe('motor', ok=False),
        probe('lidar', last_seen_ns=None, grace_ns=0),
        probe('perception', ok=False, required=False, severity=SEVERITY_DEGRADED),
    ]
    snapshot = evaluate(
        probes, safety('IDLE', fresh=False), now_ns=SEC, started_ns=0
    )

    for fault in snapshot.faults:
        assert '{' not in fault.detail, fault
        assert '}' not in fault.detail, fault
        assert fault.detail.strip(), fault
        assert fault.suggested_action.strip(), fault


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
        probe('motor', ok=False, severity=SEVERITY_FAULT),
        probe('lidar', ok=False, severity=SEVERITY_STOP),
    ]
    snapshot = evaluate(probes, safety(), now_ns=0, started_ns=0)

    assert snapshot.highest_severity == SEVERITY_FAULT
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


# ---------------------------------------------------------------------------
# 기동 유예 — aggregator 경로
#
# 2026-07-31 실행 검증에서 유예가 사실상 작동하지 않는 것을 확인했다. aggregator는
# 아직 뜨지 않은 부품에 대해 "Missing"을 1 Hz로 **계속 발행**한다. 모니터는 그것을
# 방금 받은 신선한 관측으로 보고 즉시 결함으로 올려, `not fresh` 조건을 쓰던 유예
# 분기에 도달조차 하지 못했다. TF 경로(미수신 = None)만 유예를 지켰다.
#
# 판정 기준을 "신선하지 않다"에서 "한 번도 정상인 적이 없다"로 바꾼다.
# ---------------------------------------------------------------------------


def test_fresh_but_failing_probe_inside_grace_is_starting_not_fault():
    """aggregator가 Missing을 신선하게 계속 보내도 유예 안에서는 결함이 아니다."""
    now = 5 * SEC
    snap = evaluate(
        probes=[probe('motor', last_seen_ns=now, ok=False, grace_ns=15 * SEC)],
        safety=safety(),
        now_ns=now,
        started_ns=0,
    )
    assert snap.readiness['motor'] == UNKNOWN
    assert snap.faults == []
    assert snap.state == STATE_STARTING


def test_fresh_but_failing_probe_after_grace_is_a_fault():
    """유예가 끝나면 같은 입력이 결함이 된다. 억제가 아니라 연기다."""
    now = 20 * SEC
    snap = evaluate(
        probes=[probe('motor', last_seen_ns=now, ok=False, grace_ns=15 * SEC)],
        safety=safety(),
        now_ns=now,
        started_ns=0,
    )
    assert snap.readiness['motor'] == NOT_READY
    assert [f.component for f in snap.faults] == ['motor']


def test_component_that_was_healthy_then_broke_faults_inside_grace():
    """한 번 정상이었다가 고장 나면 유예 안이라도 즉시 보고한다.

    유예의 목적은 "아직 안 뜬 것"을 봐주는 것이지 "떴다가 죽은 것"을 감추는 것이
    아니다. 이 구분이 없으면 기동 직후 실제 고장이 최대 45초 동안 묻힌다.
    """
    now = 5 * SEC
    snap = evaluate(
        probes=[
            probe(
                'motor',
                last_seen_ns=now,
                ok=False,
                grace_ns=15 * SEC,
                ever_ok=True,
            )
        ],
        safety=safety(),
        now_ns=now,
        started_ns=0,
    )
    assert snap.readiness['motor'] == NOT_READY
    assert [f.component for f in snap.faults] == ['motor']


def test_never_received_input_still_gets_grace():
    """미수신(None) 경로의 기존 동작이 유지된다. TF가 이 경로다."""
    now = 5 * SEC
    snap = evaluate(
        probes=[probe('localization', last_seen_ns=None, grace_ns=30 * SEC)],
        safety=safety(),
        now_ns=now,
        started_ns=0,
    )
    assert snap.readiness['localization'] == UNKNOWN
    assert snap.faults == []


def test_grace_applies_to_every_input_path_alike():
    """미수신·오래됨·신선하지만 실패 — 세 경로가 유예 안에서 같게 동작한다.

    실기에서 갈라졌던 지점이라 셋을 한 자리에 묶어 고정한다.
    """
    now = 5 * SEC
    snap = evaluate(
        probes=[
            probe('localization', last_seen_ns=None, grace_ns=30 * SEC),
            probe('lidar', last_seen_ns=0, timeout_ns=SEC, grace_ns=15 * SEC),
            probe('navigation', last_seen_ns=now, ok=False, grace_ns=45 * SEC),
        ],
        safety=safety(),
        now_ns=now,
        started_ns=0,
    )
    assert set(snap.readiness.values()) == {UNKNOWN}
    assert snap.faults == []
    assert snap.state == STATE_STARTING


# ---------------------------------------------------------------------------
# ESTOPPED는 실제 래치가 있을 때만
#
# 2026-07-31 실행 검증에서 E-stop이 하나도 안 걸린 노트북 단독 실행이
# state=ESTOPPED(비상 정지)로 보고됐다. motor 진단이 없다는 이유만으로 등급이
# ESTOP이었기 때문이다. ESTOPPED는 `/emergency_stop` 래치가 소유하는 의미이며,
# 진단 결함이 그 이름을 빌려 쓰면 관리자가 있지도 않은 E-stop 버튼을 찾는다.
# ---------------------------------------------------------------------------


def test_estop_severity_fault_without_latch_is_stopped_not_estopped():
    """진단 결함은 등급이 ESTOP이어도 주행 불가까지다. 비상 정지가 아니다."""
    now = 20 * SEC
    snap = evaluate(
        probes=[
            probe('motor', last_seen_ns=now, ok=False, severity=SEVERITY_STOP)
        ],
        safety=safety(estop=False),
        now_ns=now,
        started_ns=0,
    )
    assert snap.highest_severity == SEVERITY_STOP
    assert snap.state == STATE_STOPPED


def test_estop_latch_makes_state_estopped():
    """실제 래치가 걸리면 ESTOPPED다."""
    now = 20 * SEC
    snap = evaluate(
        probes=[probe('motor', last_seen_ns=now)],
        safety=safety(state='ESTOP_ACTIVE', estop=True),
        now_ns=now,
        started_ns=0,
    )
    assert snap.state == STATE_ESTOPPED


def test_estop_latch_wins_even_during_startup_grace():
    """비상 정지는 기동 유예보다 우선한다."""
    now = 1 * SEC
    snap = evaluate(
        probes=[probe('motor', last_seen_ns=None, grace_ns=15 * SEC)],
        safety=safety(state='ESTOP_ACTIVE', estop=True),
        now_ns=now,
        started_ns=0,
    )
    assert snap.state == STATE_ESTOPPED


# ---------------------------------------------------------------------------
# 안전 입력도 같은 유예 규칙을 따른다
#
# 결함 1을 고친 뒤 safety만 남아 기동 1초에 STOP을 올렸다. 판정 경로가 달랐을 뿐
# "한 번도 수신하지 못했다"는 사실은 motor와 똑같다. 다르게 처리할 근거가 없다.
#
# 단 E-stop 래치는 유예 대상이 아니다. 모니터는 정지 권한이 없지만, 래치가 걸린 사실을
# 늦게 알리면 관리자가 원인을 찾는 시간이 늘어난다.
# ---------------------------------------------------------------------------


def test_never_received_safety_inside_grace_is_not_a_fault():
    """기동 유예 안에서는 safety 미수신을 결함으로 올리지 않는다."""
    now = 5 * SEC
    snap = evaluate(
        probes=[probe('safety', last_seen_ns=None, grace_ns=15 * SEC)],
        safety=SafetyInput(state='IDLE', estop_latched=False, fresh=False),
        now_ns=now,
        started_ns=0,
    )
    assert snap.faults == []
    assert snap.state == STATE_STARTING


def test_never_received_safety_after_grace_is_a_fault():
    """유예가 끝나면 fail-closed로 보고한다."""
    now = 20 * SEC
    snap = evaluate(
        probes=[probe('safety', last_seen_ns=None, grace_ns=15 * SEC)],
        safety=SafetyInput(state='IDLE', estop_latched=False, fresh=False),
        now_ns=now,
        started_ns=0,
    )
    assert 'SAFETY_STATE_STALE' in [f.fault_code for f in snap.faults]


def test_safety_that_was_received_then_went_stale_faults_inside_grace():
    """한 번 받았다가 끊긴 것은 유예 대상이 아니다."""
    now = 5 * SEC
    snap = evaluate(
        probes=[probe('safety', last_seen_ns=None, grace_ns=15 * SEC)],
        safety=SafetyInput(
            state='IDLE',
            estop_latched=False,
            fresh=False,
            age_sec=2.0,
            ever_fresh=True,
        ),
        now_ns=now,
        started_ns=0,
    )
    assert 'SAFETY_STATE_STALE' in [f.fault_code for f in snap.faults]


def test_estop_latch_is_never_suppressed_by_grace():
    """래치는 기동 유예와 무관하게 즉시 보고한다."""
    now = 1 * SEC
    snap = evaluate(
        probes=[probe('safety', last_seen_ns=None, grace_ns=15 * SEC)],
        safety=SafetyInput(
            state='ESTOP_ACTIVE', estop_latched=True, fresh=True, age_sec=0.1
        ),
        now_ns=now,
        started_ns=0,
    )
    assert snap.state == STATE_ESTOPPED
    assert 'SAFETY_ESTOP_LATCHED' in [f.fault_code for f in snap.faults]


def test_bare_bringup_reports_starting_not_stopped():
    """아무것도 안 뜬 노트북 단독 실행이 STARTING으로 보고된다.

    2026-07-31 실행 검증의 재현 조건이다. 고치기 전에는 ESTOPPED였다.
    """
    now = 2 * SEC
    snap = evaluate(
        probes=[
            probe('motor', last_seen_ns=now, ok=False, grace_ns=15 * SEC,
                  severity=SEVERITY_STOP),
            probe('safety', last_seen_ns=now, ok=False, grace_ns=15 * SEC),
            probe('lidar', last_seen_ns=now, ok=False, grace_ns=15 * SEC),
            probe('localization', last_seen_ns=None, grace_ns=30 * SEC),
            probe('navigation', last_seen_ns=now, ok=False, grace_ns=45 * SEC),
        ],
        safety=SafetyInput(state='IDLE', estop_latched=False, fresh=False),
        now_ns=now,
        started_ns=0,
    )
    assert snap.state == STATE_STARTING
    assert snap.faults == []


# ---------------------------------------------------------------------------
# extra_faults — 진단 계열 밖에서 온 결함을 같은 판정에 얹는다
# ---------------------------------------------------------------------------
#
# 주행 실패(goal_failed)는 /diagnostics_agg 계열이 아니다. 그것을 여기로 들이는 통로가
# extra_faults다. 삽입 지점이 중요하다 — publish_health가 만든 observation 목록에
# 그냥 덧붙이면 active_faults(dedup 출처)에는 들어가지만 highest_severity와
# primary_fault_code(snapshot 출처)에는 반영되지 않아 **한 메시지 안에서 값이 어긋난다**
# (docs/proposal_nav_failure_to_app.md 3.4절).


def _nav_fault(severity=SEVERITY_DEGRADED):
    """주행 실패 결함 하나. nav_failure.NavFailureTracker가 만드는 것과 같은 모양이다."""
    return Fault(
        component='navigation',
        fault_code='NAV_GOAL_FAILED',
        severity=severity,
        detail='화장실까지 가지 못했습니다. 사유: Nav2 task failed (실패 1회)',
        suggested_action='로봇 주변에 장애물이 있는지 확인해 주세요.',
        latched=False,
    )


def _healthy_probes(now):
    return [probe('motor', last_seen_ns=now), probe('safety', last_seen_ns=now)]


def test_extra_faults_reach_the_summary_fields():
    """highest_severity·primary_fault_code·active_fault_count에 모두 반영된다.

    셋 중 하나라도 빠지면 앱 배너와 결함 목록이 서로 다른 말을 한다.
    """
    now = 100 * SEC
    snap = evaluate(
        probes=_healthy_probes(now),
        safety=SafetyInput(state='IDLE', estop_latched=False, fresh=True, age_sec=0.0),
        now_ns=now,
        started_ns=0,
        extra_faults=[_nav_fault()],
    )
    assert snap.highest_severity == SEVERITY_DEGRADED
    assert snap.primary_fault_code == 'NAV_GOAL_FAILED'
    assert snap.active_fault_count == 1
    assert snap.state == STATE_DEGRADED


def test_extra_faults_do_not_touch_readiness():
    """주행 실패는 Nav2가 죽었다는 뜻이 아니다.

    goal 하나가 실패해도 Nav2 lifecycle은 active이고 새 goal을 받을 수 있다.
    readiness까지 끌어내리면 관리자가 '스택이 죽었다'로 오해한다.
    """
    now = 100 * SEC
    snap = evaluate(
        probes=[probe('navigation', last_seen_ns=now)],
        safety=SafetyInput(state='IDLE', estop_latched=False, fresh=True, age_sec=0.0),
        now_ns=now,
        started_ns=0,
        extra_faults=[_nav_fault()],
    )
    assert snap.readiness['navigation'] == READY


def test_escalated_extra_fault_stops_the_robot_state():
    """반복 실패로 STOP까지 오르면 전체 상태도 STOPPED가 된다."""
    now = 100 * SEC
    snap = evaluate(
        probes=_healthy_probes(now),
        safety=SafetyInput(state='IDLE', estop_latched=False, fresh=True, age_sec=0.0),
        now_ns=now,
        started_ns=0,
        extra_faults=[_nav_fault(severity=SEVERITY_STOP)],
    )
    assert snap.state == STATE_STOPPED


def test_extra_faults_are_sorted_with_the_others():
    """더 심각한 결함이 있으면 그쪽이 primary다. 정렬 규칙이 하나여야 한다."""
    now = 100 * SEC
    snap = evaluate(
        probes=[probe('motor', last_seen_ns=None, ok=False, severity=SEVERITY_STOP)],
        safety=SafetyInput(state='IDLE', estop_latched=False, fresh=True, age_sec=0.0),
        now_ns=now,
        started_ns=0,
        extra_faults=[_nav_fault()],
    )
    assert snap.highest_severity == SEVERITY_STOP
    assert snap.primary_fault_code != 'NAV_GOAL_FAILED'
    assert 'NAV_GOAL_FAILED' in [f.fault_code for f in snap.faults]


def test_omitting_extra_faults_keeps_the_old_behaviour():
    """기존 호출부는 인자를 넘기지 않는다. 기본값이 빈 목록이어야 한다."""
    now = 100 * SEC
    snap = evaluate(
        probes=_healthy_probes(now),
        safety=SafetyInput(state='IDLE', estop_latched=False, fresh=True, age_sec=0.0),
        now_ns=now,
        started_ns=0,
    )
    assert snap.faults == []
    assert snap.state == STATE_READY
