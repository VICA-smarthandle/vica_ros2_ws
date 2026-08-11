"""주행 실패를 관리자에게 전달하는 판정을 시험한다.

`docs/proposal_nav_failure_to_app.md` 구현분이다. ROS 없이 실행하며 시각은 정수
나노초로 주입한다.

이 계층이 푸는 문제는 하나다 — **일회성 사건을 지속 상태로 바꾸는 것**이다.

감시 계층은 "지금 참인 결함"의 목록을 매 tick 다시 만들고, 이번 tick에 없는 결함은
해소된 것으로 처리한다(`event_deduplicator` 규칙 4). 그런데 `goal_failed`는 한 번 오고
끝나는 사건이라, 받은 tick에만 넣으면 다음 tick에 곧바로 CLEARED가 나가 관리자가
보기 전에 지나간다.

그래서 **보류 창(hold)** 을 둔다. 그 창은 두 가지 일을 한다.

    1. 결함을 창이 닫힐 때까지 붙들어 관리자가 볼 시간을 번다
    2. "반복"의 정의가 된다 — 창이 열려 있는 동안 또 실패하면 같은 곤경이다

반복 판정에 새 기준을 만들지 않고 창을 재사용하는 이유는, 기준이 둘이 되면 "몇 초
안에 몇 번"이라는 값을 하나 더 관리해야 하고 그 둘이 어긋날 수 있기 때문이다.
"""

from vica_system_monitor.fault_catalog import (
    CATALOG,
    SEVERITY_DEGRADED,
    SEVERITY_STOP,
)
from vica_system_monitor.nav_failure import (
    NAV_GOAL_FAILED,
    NavFailureTracker,
    parse_goal_failure,
)


SEC = 1_000_000_000
HOLD_NS = 60 * SEC
NOW = 1_000 * SEC


def _payload(event='goal_failed', name='화장실', reason='Nav2 task failed'):
    """Mission Manager가 실제로 보내는 모양의 JSON 문자열을 만든다.

    필드 구성은 mission_manager_node._publish_goal_event 를 그대로 따른다.
    """
    import json
    return json.dumps(
        {
            'event': event,
            'map_id': 'vica_map_0630',
            'location_id': 'dest-1',
            'destination_id': 'dest-1',
            'name': name,
            'x': 1.0,
            'y': 2.0,
            'yaw': 90.0,
            'reason': reason,
            'timestamp': '2026-08-11T14:22:31',
        },
        ensure_ascii=False,
    )


def _tracker(hold_ns=HOLD_NS):
    return NavFailureTracker(hold_ns=hold_ns)


# ---------------------------------------------------------------------------
# 파싱 — 무엇을 실패로 볼 것인가
# ---------------------------------------------------------------------------


def test_goal_failed_is_a_failure():
    """Nav2 ABORT. 경로 없음·복구 소진·갇힘이 여기로 온다."""
    failure = parse_goal_failure(_payload('goal_failed'))
    assert failure is not None
    assert failure.event == 'goal_failed'
    assert failure.name == '화장실'
    assert failure.reason == 'Nav2 task failed'


def test_goal_rejected_is_a_failure():
    """Nav2가 goal 자체를 거부한 경우도 '못 갔다'는 같다."""
    failure = parse_goal_failure(_payload('goal_rejected', reason='Nav2 goal rejected'))
    assert failure is not None
    assert failure.event == 'goal_rejected'


def test_normal_flow_events_are_not_failures():
    """정상 흐름을 실패로 읽으면 주행할 때마다 관리자에게 경보가 간다."""
    for event in ('goal_sent', 'goal_accepted', 'goal_succeeded', 'goal_canceled'):
        assert parse_goal_failure(_payload(event)) is None, event


def test_broken_payload_never_raises():
    """잘못된 payload 하나가 감시 노드를 죽이면 상태 표시 자체가 사라진다.

    guidance_priority.parse_goal_event 와 같은 방어 계약이다.
    """
    for payload in ('', 'not json', '[]', '{}', None, '{"event": 3}'):
        assert parse_goal_failure(payload) is None


def test_missing_text_fields_fall_back_instead_of_crashing():
    """이름·사유가 비어도 결함은 보고돼야 한다. 문구만 덜 친절해진다."""
    import json
    failure = parse_goal_failure(json.dumps({'event': 'goal_failed'}))
    assert failure is not None
    assert failure.name  # 빈 문자열이 아니라 대체 문구가 들어간다
    assert failure.reason


# ---------------------------------------------------------------------------
# 보류 — 일회성 사건을 지속 상태로
# ---------------------------------------------------------------------------


def test_nothing_is_reported_before_any_failure():
    """평소에는 결함이 없다."""
    assert _tracker().fault(NOW) is None


def test_failure_is_held_for_the_whole_window():
    """받은 tick에만 넣으면 다음 tick에 CLEARED가 나가 관리자가 못 본다.

    창이 닫히기 직전까지 같은 결함이 계속 나와야 EventDeduplicator가 이것을 '지속되는
    하나의 결함'으로 본다.
    """
    tracker = _tracker()
    tracker.record(parse_goal_failure(_payload()), NOW)

    for elapsed in (0, 1 * SEC, 30 * SEC, HOLD_NS - 1):
        assert tracker.fault(NOW + elapsed) is not None, elapsed


def test_failure_clears_when_the_window_closes():
    """창이 닫히면 사라진다. 이미 해결된 실패가 영원히 남으면 안 된다."""
    tracker = _tracker()
    tracker.record(parse_goal_failure(_payload()), NOW)

    assert tracker.fault(NOW + HOLD_NS) is None
    assert tracker.fault(NOW + HOLD_NS + SEC) is None


def test_zero_hold_reports_nothing():
    """hold를 0으로 두면 기능이 꺼진다. 설정으로 끌 수 있어야 한다."""
    tracker = _tracker(hold_ns=0)
    tracker.record(parse_goal_failure(_payload()), NOW)
    assert tracker.fault(NOW) is None


def test_time_going_backwards_keeps_holding():
    """시계가 뒤로 가도 조용해지지 않는다.

    event_deduplicator._reminder_due 와 같은 판단이다 — 조용해지는 방향보다
    시끄러워지는 방향이 안전하다.
    """
    tracker = _tracker()
    tracker.record(parse_goal_failure(_payload()), NOW)
    assert tracker.fault(NOW - 10 * SEC) is not None


# ---------------------------------------------------------------------------
# 승격 — 반복되는 실패가 곧 갇힘이다
# ---------------------------------------------------------------------------


def test_a_single_failure_is_degraded():
    """한 번의 실패는 흔한 일이다. 로봇은 여전히 새 goal을 받을 수 있다."""
    tracker = _tracker()
    tracker.record(parse_goal_failure(_payload()), NOW)

    fault = tracker.fault(NOW)
    assert fault.severity == SEVERITY_DEGRADED
    assert fault.component == 'navigation'
    assert fault.fault_code == NAV_GOAL_FAILED


def test_a_repeat_inside_the_window_escalates_to_stop():
    """창이 열려 있는 동안 또 실패했다면 같은 곤경이다.

    앱은 STOP(3) 이상만 알림 목록에 남긴다(fault_severity.dart:47 blocksDriving).
    그래서 승격 여부가 곧 '관리자에게 알림이 뜨는가'다.
    """
    tracker = _tracker()
    tracker.record(parse_goal_failure(_payload()), NOW)
    tracker.record(parse_goal_failure(_payload()), NOW + 10 * SEC)

    assert tracker.fault(NOW + 10 * SEC).severity == SEVERITY_STOP


def test_escalation_keeps_the_same_fault_code():
    """등급만 올라가고 코드는 같아야 EventDeduplicator가 ESCALATED를 낸다.

    코드가 바뀌면 키가 달라져 '하나가 해소되고 다른 하나가 발생'으로 보인다.
    같은 문제가 나빠진 것이지 다른 문제가 아니다.
    """
    tracker = _tracker()
    tracker.record(parse_goal_failure(_payload()), NOW)
    first = tracker.fault(NOW)
    tracker.record(parse_goal_failure(_payload()), NOW + SEC)
    second = tracker.fault(NOW + SEC)

    assert first.fault_code == second.fault_code
    assert second.severity > first.severity


def test_a_failure_after_the_window_starts_over_at_degraded():
    """창이 닫힌 뒤의 실패는 새 사건이다.

    하루에 한 번씩 실패한 것을 '반복'으로 세면 결국 항상 STOP이 된다.
    """
    tracker = _tracker()
    tracker.record(parse_goal_failure(_payload()), NOW)
    tracker.record(parse_goal_failure(_payload()), NOW + HOLD_NS + SEC)

    assert tracker.fault(NOW + HOLD_NS + SEC).severity == SEVERITY_DEGRADED


def test_third_failure_stays_at_stop():
    """더 올라갈 등급이 없다. FAULT는 '원인 불명'이라 의미가 다르다."""
    tracker = _tracker()
    for i in range(3):
        tracker.record(parse_goal_failure(_payload()), NOW + i * SEC)

    assert tracker.fault(NOW + 2 * SEC).severity == SEVERITY_STOP


def test_each_failure_extends_the_window():
    """실패가 이어지는 동안에는 창이 계속 열려 있어야 한다."""
    tracker = _tracker()
    tracker.record(parse_goal_failure(_payload()), NOW)
    tracker.record(parse_goal_failure(_payload()), NOW + HOLD_NS - SEC)

    # 첫 실패만 있었다면 이미 닫혔을 시각이다.
    assert tracker.fault(NOW + HOLD_NS + SEC) is not None


# ---------------------------------------------------------------------------
# 문구 — 관리자가 무엇을 해야 하는지 알 수 있는가
# ---------------------------------------------------------------------------


def test_detail_names_the_destination_and_the_reason():
    """어디로 가다 왜 못 갔는지가 없으면 관리자가 로봇을 찾아가야 안다."""
    tracker = _tracker()
    tracker.record(parse_goal_failure(_payload(name='방2', reason='경로 없음')), NOW)

    detail = tracker.fault(NOW).detail
    assert '방2' in detail
    assert '경로 없음' in detail


def test_detail_shows_how_many_times_it_failed():
    """반복 여부가 곧 갇힘 여부다. 숫자가 보여야 판단할 수 있다."""
    tracker = _tracker()
    tracker.record(parse_goal_failure(_payload()), NOW)
    assert '1' in tracker.fault(NOW).detail

    tracker.record(parse_goal_failure(_payload()), NOW + SEC)
    assert '2' in tracker.fault(NOW + SEC).detail


def test_detail_never_leaks_a_format_placeholder():
    """관리자에게 '{name}'이 그대로 보이면 안 된다(fault_catalog._fallback_detail)."""
    tracker = _tracker()
    tracker.record(parse_goal_failure(_payload()), NOW)

    detail = tracker.fault(NOW).detail
    assert '{' not in detail and '}' not in detail


def test_catalog_owns_the_wording():
    """문구 정본은 카탈로그다. 이 모듈이 한국어 문장을 새로 만들지 않는다."""
    assert NAV_GOAL_FAILED in CATALOG
    spec = CATALOG[NAV_GOAL_FAILED]
    assert spec.component == 'navigation'
    # 기본 등급은 DEGRADED이며 승격은 이 모듈이 판정한다.
    assert spec.severity == SEVERITY_DEGRADED
    assert spec.suggested_action
