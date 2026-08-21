"""Unit tests for notification rate limiting.

vica_system_health_monitoring_draft.md 10.3절의 여섯 규칙을 각각 고정한다.

    1. 정상 -> fault 전이 시 한 번 알린다
    2. 같은 fault는 occurrence count만 증가시킨다
    3. 장시간 유지되면 설정된 간격으로만 재알림한다
    4. 복구되면 recovery event를 한 번 발행한다
    5. 더 높은 등급이 발생하면 즉시 알린다
    6. E-stop 알림은 rate limit보다 높은 우선순위를 갖는다
"""

from vica_system_monitor.event_deduplicator import (
    EventDeduplicator,
    Observation,
    TRANSITION_CLEARED,
    TRANSITION_ESCALATED,
    TRANSITION_RAISED,
    TRANSITION_REMINDER,
)
from vica_system_monitor.fault_catalog import (
    SEVERITY_DEGRADED,
    SEVERITY_FAULT,
    SEVERITY_STOP,
    SEVERITY_WARN,
)


SEC = 1_000_000_000
REMINDER = 30 * SEC


def obs(
    component='motor',
    code='MOTOR_CAN_TIMEOUT',
    severity=SEVERITY_STOP,
    latched=False,
):
    """Build an observation with test-friendly defaults."""
    return Observation(
        component=component,
        fault_code=code,
        severity=severity,
        detail='detail',
        suggested_action='action',
        latched=latched,
    )


def make():
    """Build a deduplicator with a 30 second reminder interval."""
    return EventDeduplicator(reminder_interval_ns=REMINDER)


# ---------------------------------------------------------------------------
# 규칙 1 — 전이 시 한 번
# ---------------------------------------------------------------------------


def test_new_fault_raises_once():
    """처음 관측하면 RAISED를 한 번 낸다."""
    dedup = make()
    events, active = dedup.update([obs()], now_ns=0, wall_sec=100.0)

    assert [e.transition for e in events] == [TRANSITION_RAISED]
    assert len(active) == 1
    assert active[0].occurrence_count == 1
    assert active[0].active is True


def test_first_seen_is_recorded():
    """최초 관측 시각을 보관한다."""
    dedup = make()
    events, _ = dedup.update([obs()], now_ns=0, wall_sec=100.0)

    assert events[0].fault.first_seen_sec == 100.0
    assert events[0].fault.last_seen_sec == 100.0


# ---------------------------------------------------------------------------
# 규칙 2 — 재관측은 카운트만
# ---------------------------------------------------------------------------


def test_same_fault_does_not_emit_again():
    """같은 fault를 다시 봐도 이벤트를 내지 않는다."""
    dedup = make()
    dedup.update([obs()], now_ns=0, wall_sec=100.0)
    events, active = dedup.update([obs()], now_ns=SEC, wall_sec=101.0)

    assert events == []
    assert active[0].occurrence_count == 2


def test_occurrence_count_accumulates():
    """관측 횟수가 누적된다."""
    dedup = make()
    for tick in range(5):
        dedup.update([obs()], now_ns=tick * SEC, wall_sec=100.0 + tick)

    _, active = dedup.update([obs()], now_ns=5 * SEC, wall_sec=105.0)
    assert active[0].occurrence_count == 6


def test_last_seen_advances_but_first_seen_does_not():
    """지속 시간을 계산할 수 있도록 두 시각을 따로 유지한다."""
    dedup = make()
    dedup.update([obs()], now_ns=0, wall_sec=100.0)
    _, active = dedup.update([obs()], now_ns=SEC, wall_sec=142.0)

    assert active[0].first_seen_sec == 100.0
    assert active[0].last_seen_sec == 142.0


# ---------------------------------------------------------------------------
# 규칙 3 — 재알림 간격
# ---------------------------------------------------------------------------


def test_reminder_not_emitted_before_interval():
    """간격 이전에는 재알림하지 않는다."""
    dedup = make()
    dedup.update([obs()], now_ns=0, wall_sec=100.0)
    events, _ = dedup.update([obs()], now_ns=REMINDER - 1, wall_sec=129.0)

    assert events == []


def test_reminder_emitted_at_interval():
    """간격에 도달하면 REMINDER를 낸다."""
    dedup = make()
    dedup.update([obs()], now_ns=0, wall_sec=100.0)
    events, _ = dedup.update([obs()], now_ns=REMINDER, wall_sec=130.0)

    assert [e.transition for e in events] == [TRANSITION_REMINDER]


def test_reminder_interval_restarts_after_each_reminder():
    """재알림 후 간격이 다시 시작된다. 매 tick 알리지 않는다."""
    dedup = make()
    dedup.update([obs()], now_ns=0, wall_sec=100.0)
    dedup.update([obs()], now_ns=REMINDER, wall_sec=130.0)

    events, _ = dedup.update([obs()], now_ns=REMINDER + SEC, wall_sec=131.0)
    assert events == []

    events, _ = dedup.update([obs()], now_ns=2 * REMINDER, wall_sec=160.0)
    assert [e.transition for e in events] == [TRANSITION_REMINDER]


# ---------------------------------------------------------------------------
# 규칙 4 — 복구 알림 한 번
# ---------------------------------------------------------------------------


def test_cleared_emitted_once_when_fault_disappears():
    """관측되지 않으면 CLEARED를 한 번 낸다."""
    dedup = make()
    dedup.update([obs()], now_ns=0, wall_sec=100.0)
    events, active = dedup.update([], now_ns=SEC, wall_sec=101.0)

    assert [e.transition for e in events] == [TRANSITION_CLEARED]
    assert events[0].fault.active is False
    assert active == []


def test_cleared_not_repeated():
    """해소된 fault를 계속 알리지 않는다."""
    dedup = make()
    dedup.update([obs()], now_ns=0, wall_sec=100.0)
    dedup.update([], now_ns=SEC, wall_sec=101.0)
    events, _ = dedup.update([], now_ns=2 * SEC, wall_sec=102.0)

    assert events == []


def test_fault_can_be_raised_again_after_clear():
    """해소 후 재발생은 새 RAISED다. 카운트도 새로 센다."""
    dedup = make()
    dedup.update([obs()], now_ns=0, wall_sec=100.0)
    dedup.update([], now_ns=SEC, wall_sec=101.0)
    events, active = dedup.update([obs()], now_ns=2 * SEC, wall_sec=102.0)

    assert [e.transition for e in events] == [TRANSITION_RAISED]
    assert active[0].occurrence_count == 1


# ---------------------------------------------------------------------------
# 규칙 5 — 등급 상승은 즉시
# ---------------------------------------------------------------------------


def test_escalation_emits_immediately():
    """등급이 오르면 간격을 무시하고 즉시 알린다."""
    dedup = make()
    dedup.update([obs(severity=SEVERITY_DEGRADED)], now_ns=0, wall_sec=100.0)
    events, active = dedup.update(
        [obs(severity=SEVERITY_STOP)], now_ns=SEC, wall_sec=101.0
    )

    assert [e.transition for e in events] == [TRANSITION_ESCALATED]
    assert active[0].severity == SEVERITY_STOP


def test_severity_drop_does_not_emit_escalation():
    """등급이 내려가면 ESCALATED가 아니다."""
    dedup = make()
    dedup.update([obs(severity=SEVERITY_STOP)], now_ns=0, wall_sec=100.0)
    events, active = dedup.update(
        [obs(severity=SEVERITY_WARN)], now_ns=SEC, wall_sec=101.0
    )

    assert events == []
    assert active[0].severity == SEVERITY_WARN


# ---------------------------------------------------------------------------
# 규칙 6 — 래치된 결함은 rate limit 무시
#
# 기준이 등급이 아니라 latched인 이유: E-stop은 "STOP보다 한 단계 심각한 것"이 아니라
# 종류가 다른 것이다. 관리자 reset이 있어야 풀린다. 등급으로 판정하면 모터 진단이
# 안 올 뿐인데도 초당 한 건씩 알림이 나간다(2026-07-31 실기동에서 223회 관측).
# ---------------------------------------------------------------------------


def test_latched_fault_reminds_every_tick():
    """래치된 결함은 간격과 무관하게 매 tick 재알림한다.

    관리자가 reset하기 전까지 풀리지 않는 상태를 놓치지 않는 것이 폭주 억제보다
    우선이다.
    """
    dedup = make()
    dedup.update([obs(latched=True)], now_ns=0, wall_sec=100.0)

    for tick in (1, 2, 3):
        events, _ = dedup.update(
            [obs(latched=True)],
            now_ns=tick * SEC,
            wall_sec=100.0 + tick,
        )
        assert [e.transition for e in events] == [TRANSITION_REMINDER]


def test_unlatched_fault_does_not_remind_every_tick():
    """대조: 래치되지 않은 STOP은 간격을 지킨다."""
    dedup = make()
    dedup.update([obs(severity=SEVERITY_STOP)], now_ns=0, wall_sec=100.0)
    events, _ = dedup.update([obs(severity=SEVERITY_STOP)], now_ns=SEC, wall_sec=101.0)

    assert events == []


def test_highest_severity_alone_does_not_defeat_rate_limit():
    """등급이 가장 높아도 래치가 아니면 폭주 억제가 유지된다.

    모터 진단 미수신이 매 tick 알림을 내던 실제 결함의 회귀 테스트다.
    """
    dedup = make()
    dedup.update([obs(severity=SEVERITY_FAULT)], now_ns=0, wall_sec=100.0)

    for tick in (1, 2, 3):
        events, _ = dedup.update(
            [obs(severity=SEVERITY_FAULT)],
            now_ns=tick * SEC,
            wall_sec=100.0 + tick,
        )
        assert events == []


# ---------------------------------------------------------------------------
# 여러 fault와 정렬
# ---------------------------------------------------------------------------


def test_tracks_faults_independently_by_component_and_code():
    """(component, fault_code)가 키다."""
    dedup = make()
    events, active = dedup.update(
        [
            obs(component='motor', code='MOTOR_CAN_TIMEOUT'),
            obs(component='lidar', code='LIDAR_SCAN_STALE'),
        ],
        now_ns=0,
        wall_sec=100.0,
    )

    assert len(events) == 2
    assert len(active) == 2


def test_same_component_different_code_are_separate():
    """같은 컴포넌트의 다른 코드는 별개 fault다."""
    dedup = make()
    _, active = dedup.update(
        [
            obs(component='motor', code='MOTOR_CAN_TIMEOUT'),
            obs(component='motor', code='MOTOR_NODE_SILENT'),
        ],
        now_ns=0,
        wall_sec=100.0,
    )

    assert len(active) == 2


def test_active_list_is_sorted_most_severe_first():
    """앱이 그대로 표시할 수 있도록 severity 내림차순으로 준다."""
    dedup = make()
    _, active = dedup.update(
        [
            obs(component='app', code='APP_BRIDGE_SILENT', severity=SEVERITY_WARN),
            obs(component='motor', code='MOTOR_CAN_TIMEOUT', severity=SEVERITY_STOP),
            obs(component='lidar', code='LIDAR_SCAN_STALE', severity=SEVERITY_STOP),
        ],
        now_ns=0,
        wall_sec=100.0,
    )

    assert [f.severity for f in active] == [
        SEVERITY_STOP,
        SEVERITY_STOP,
        SEVERITY_WARN,
    ]


def test_highest_returns_most_severe_active_fault():
    """대표 fault를 알려준다."""
    dedup = make()
    dedup.update(
        [
            obs(component='app', code='APP_BRIDGE_SILENT', severity=SEVERITY_WARN),
            obs(component='motor', code='MOTOR_CAN_TIMEOUT', severity=SEVERITY_STOP),
        ],
        now_ns=0,
        wall_sec=100.0,
    )

    top = dedup.highest()
    assert top is not None
    assert top.component == 'motor'


def test_highest_is_none_when_healthy():
    """활성 fault가 없으면 None이다."""
    assert make().highest() is None


def test_partial_clear_keeps_the_other_fault():
    """하나만 해소되면 나머지는 유지된다."""
    dedup = make()
    dedup.update(
        [
            obs(component='motor', code='MOTOR_CAN_TIMEOUT'),
            obs(component='lidar', code='LIDAR_SCAN_STALE'),
        ],
        now_ns=0,
        wall_sec=100.0,
    )
    events, active = dedup.update(
        [obs(component='motor', code='MOTOR_CAN_TIMEOUT')],
        now_ns=SEC,
        wall_sec=101.0,
    )

    assert [e.transition for e in events] == [TRANSITION_CLEARED]
    assert events[0].fault.component == 'lidar'
    assert [f.component for f in active] == ['motor']


# ---------------------------------------------------------------------------
# 이상 입력
# ---------------------------------------------------------------------------


def test_time_reversal_emits_reminder_rather_than_going_silent():
    """시간이 역전되면 조용해지는 대신 알린다. 시끄러운 쪽이 안전하다."""
    dedup = make()
    dedup.update([obs()], now_ns=10 * SEC, wall_sec=100.0)
    events, _ = dedup.update([obs()], now_ns=SEC, wall_sec=101.0)

    assert [e.transition for e in events] == [TRANSITION_REMINDER]


def test_detail_and_action_are_updated_on_reobservation():
    """측정값이 바뀌면 문구도 갱신된다."""
    dedup = make()
    dedup.update([obs()], now_ns=0, wall_sec=100.0)
    changed = Observation(
        component='motor',
        fault_code='MOTOR_CAN_TIMEOUT',
        severity=SEVERITY_STOP,
        detail='CAN 응답 1.4초 미수신',
        suggested_action='action',
    )
    _, active = dedup.update([changed], now_ns=SEC, wall_sec=101.0)

    assert active[0].detail == 'CAN 응답 1.4초 미수신'


def test_latched_flag_is_carried_through():
    """래치 표시가 이벤트와 활성 목록에 전달된다."""
    dedup = make()
    latched = Observation(
        component='safety',
        fault_code='SAFETY_ESTOP_LATCHED',
        severity=SEVERITY_STOP,
        detail='d',
        suggested_action='a',
        latched=True,
    )
    events, active = dedup.update([latched], now_ns=0, wall_sec=100.0)

    assert events[0].fault.latched is True
    assert active[0].latched is True


# ---------------------------------------------------------------------------
# 2026-08-21 추가 — 래치 전용 간격과 해소 확정 지연
# ---------------------------------------------------------------------------


LATCHED_REMINDER = 10 * SEC


def make_tuned(clear_confirm_ticks=2):
    """Build a deduplicator with the values shipped in required_components.yaml."""
    return EventDeduplicator(
        reminder_interval_ns=REMINDER,
        latched_reminder_interval_ns=LATCHED_REMINDER,
        clear_confirm_ticks=clear_confirm_ticks,
    )


def test_latched_fault_still_reminds_every_tick_by_default():
    """기본값은 종전 동작이다. 규칙 6을 인자 없이 바꾸지 않는다."""
    dedup = make()
    dedup.update([obs(latched=True)], 0, 0.0)
    events, _ = dedup.update([obs(latched=True)], SEC, 1.0)
    assert [event.transition for event in events] == [TRANSITION_REMINDER]


def test_latched_fault_honours_its_own_interval_when_given():
    """전용 간격을 주면 매 tick이 아니라 그 간격으로 재알림한다."""
    dedup = make_tuned()
    dedup.update([obs(latched=True)], 0, 0.0)

    # 1초 뒤 — 아직 10초가 안 지났다.
    events, _ = dedup.update([obs(latched=True)], SEC, 1.0)
    assert events == []

    # 10초 뒤 — 재알림한다.
    events, _ = dedup.update([obs(latched=True)], 10 * SEC, 10.0)
    assert [event.transition for event in events] == [TRANSITION_REMINDER]


def test_latched_interval_is_shorter_than_the_normal_one():
    """래치는 일반 결함보다 자주 알려야 한다. 값을 뒤바꿔 넣는 것을 막는다."""
    assert LATCHED_REMINDER < REMINDER


def test_clear_waits_for_confirmation_ticks():
    """해소는 연속 미관측이 확정 tick 수에 도달해야 발행한다."""
    dedup = make_tuned(clear_confirm_ticks=3)
    dedup.update([obs()], 0, 0.0)

    for tick in range(1, 3):
        events, active = dedup.update([], tick * SEC, float(tick))
        assert events == [], f'tick {tick}에서 성급히 해소로 봤다'
        assert len(active) == 1, '확정 전에는 활성 목록에 남아 있어야 한다'

    events, active = dedup.update([], 3 * SEC, 3.0)
    assert [event.transition for event in events] == [TRANSITION_CLEARED]
    assert active == []


def test_flapping_observation_produces_no_event_churn():
    """임계값 근처에서 한 tick 걸렀다가 돌아오면 아무 이벤트도 나오지 않는다.

    이것이 '같은 알람이 자주 뜬다'의 실제 기전이다. 확정 지연이 없으면 이 패턴이
    tick마다 해소 -> 발생 두 건을 만들어 이력을 같은 항목으로 채운다.
    """
    dedup = make_tuned()
    dedup.update([obs()], 0, 0.0)

    transitions = []
    for tick in range(1, 11):
        observations = [] if tick % 2 else [obs()]
        events, _ = dedup.update(observations, tick * SEC, float(tick))
        transitions.extend(event.transition for event in events)

    assert transitions == [], f'churn 이 남았다: {transitions}'


def test_clear_confirm_ticks_below_one_is_treated_as_one():
    """0이나 음수를 넣어도 해소가 영영 안 나오는 상태가 되지 않게 한다."""
    dedup = EventDeduplicator(reminder_interval_ns=REMINDER, clear_confirm_ticks=0)
    dedup.update([obs()], 0, 0.0)
    events, _ = dedup.update([], SEC, 1.0)
    assert [event.transition for event in events] == [TRANSITION_CLEARED]
