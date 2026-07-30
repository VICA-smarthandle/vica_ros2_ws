"""Fault state tracking and notification rate limiting.

vica_system_health_monitoring_draft.md 10.3절의 여섯 규칙을 그대로 구현한다.

    1. 상태가 정상에서 fault로 바뀔 때 한 번 알린다.
    2. 같은 fault는 occurrence count만 증가시킨다.
    3. 장시간 유지되면 설정된 간격으로만 재알림한다.
    4. 복구되면 recovery event를 한 번 발행한다.
    5. 더 높은 등급이 발생하면 즉시 기존 알림을 덮어쓴다.
    6. E-stop 알림은 rate limit보다 높은 우선순위를 갖는다.

ROS 의존이 없는 순수 모듈이다. 시각은 정수 나노초로 주입받고 스스로 조회하지
않는다. 표시용 wall clock 시각(first_seen/last_seen)은 별도 인자로 받는다 —
STEADY_TIME과 SYSTEM_TIME을 섞어 계산하지 않기 위해서다.
"""

from typing import Dict, List, NamedTuple, Optional, Tuple

from .fault_catalog import SEVERITY_ESTOP


TRANSITION_RAISED = 0
TRANSITION_ESCALATED = 1
TRANSITION_REMINDER = 2
TRANSITION_CLEARED = 3


class Observation(NamedTuple):
    """One fault observed in the current tick."""

    component: str
    fault_code: str
    severity: int
    detail: str
    suggested_action: str
    latched: bool = False


class ActiveFault(NamedTuple):
    """Current state of one tracked fault.

    first_seen_sec/last_seen_sec는 표시용 wall clock(SYSTEM_TIME) 초 값이다.
    신선도 판정에는 쓰지 않는다.
    """

    component: str
    fault_code: str
    severity: int
    detail: str
    suggested_action: str
    latched: bool
    active: bool
    occurrence_count: int
    first_seen_sec: float
    last_seen_sec: float


class Event(NamedTuple):
    """One event to publish on /robot/events this tick."""

    fault: ActiveFault
    transition: int


class _Record:
    """Mutable bookkeeping for one (component, fault_code) key."""

    def __init__(
        self,
        observation: Observation,
        now_ns: int,
        wall_sec: float,
    ) -> None:
        self.component = observation.component
        self.fault_code = observation.fault_code
        self.severity = observation.severity
        self.detail = observation.detail
        self.suggested_action = observation.suggested_action
        self.latched = observation.latched
        self.occurrence_count = 1
        self.first_seen_sec = wall_sec
        self.last_seen_sec = wall_sec
        self.last_notified_ns = now_ns

    def snapshot(self, active: bool) -> ActiveFault:
        """Build an immutable view for publishing."""
        return ActiveFault(
            component=self.component,
            fault_code=self.fault_code,
            severity=self.severity,
            detail=self.detail,
            suggested_action=self.suggested_action,
            latched=self.latched,
            active=active,
            occurrence_count=self.occurrence_count,
            first_seen_sec=self.first_seen_sec,
            last_seen_sec=self.last_seen_sec,
        )


class EventDeduplicator:
    """Turn per-tick fault observations into a minimal event stream."""

    def __init__(self, reminder_interval_ns: int) -> None:
        """Store the re-notification interval in integer nanoseconds."""
        self._reminder_interval_ns = reminder_interval_ns
        self._records: Dict[Tuple[str, str], _Record] = {}

    def update(
        self,
        observations: List[Observation],
        now_ns: int,
        wall_sec: float,
    ) -> Tuple[List[Event], List[ActiveFault]]:
        """Advance state by one tick.

        Returns (events to publish now, current active fault list). 활성 목록은
        severity 내림차순으로 정렬해 앱이 그대로 표시할 수 있게 한다.
        """
        events: List[Event] = []
        seen_keys = set()

        for observation in observations:
            key = (observation.component, observation.fault_code)
            seen_keys.add(key)
            record = self._records.get(key)

            if record is None:
                record = _Record(observation, now_ns, wall_sec)
                self._records[key] = record
                events.append(Event(record.snapshot(active=True), TRANSITION_RAISED))
                continue

            previous_severity = record.severity
            record.severity = observation.severity
            record.detail = observation.detail
            record.suggested_action = observation.suggested_action
            record.latched = observation.latched
            record.occurrence_count += 1
            record.last_seen_sec = wall_sec

            if observation.severity > previous_severity:
                # 규칙 5: 등급 상승은 rate limit과 무관하게 즉시 알린다.
                record.last_notified_ns = now_ns
                events.append(
                    Event(record.snapshot(active=True), TRANSITION_ESCALATED)
                )
                continue

            if self._reminder_due(record, now_ns):
                record.last_notified_ns = now_ns
                events.append(
                    Event(record.snapshot(active=True), TRANSITION_REMINDER)
                )

        # 규칙 4: 이번 tick에 관측되지 않은 fault는 해소된 것으로 보고 한 번만 알린다.
        for key in list(self._records):
            if key in seen_keys:
                continue
            record = self._records.pop(key)
            record.last_seen_sec = wall_sec
            events.append(Event(record.snapshot(active=False), TRANSITION_CLEARED))

        return events, self._active_faults()

    def _reminder_due(self, record: _Record, now_ns: int) -> bool:
        """Return True when the re-notification interval has elapsed.

        규칙 6: ESTOP은 간격을 무시하고 매 tick 재알림한다. 가장 위험한 상태를
        사용자가 놓치지 않는 것이 폭주 억제보다 우선이다.

        시간이 역전되면(now_ns < last_notified_ns) 알린다. 조용해지는 방향보다
        시끄러워지는 방향이 안전하다.
        """
        if record.severity >= SEVERITY_ESTOP:
            return True
        elapsed_ns = now_ns - record.last_notified_ns
        if elapsed_ns < 0:
            return True
        return elapsed_ns >= self._reminder_interval_ns

    def _active_faults(self) -> List[ActiveFault]:
        """Return current faults, most severe first."""
        faults = [record.snapshot(active=True) for record in self._records.values()]
        faults.sort(key=lambda fault: (-fault.severity, fault.component, fault.fault_code))
        return faults

    def highest(self) -> Optional[ActiveFault]:
        """Return the most severe active fault, or None when healthy."""
        faults = self._active_faults()
        return faults[0] if faults else None
