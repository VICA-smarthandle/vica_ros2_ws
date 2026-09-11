"""Fault state tracking and notification rate limiting."""

from typing import Dict, List, NamedTuple, Optional, Tuple


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
    """Current state of one tracked fault."""

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
        self.missed_ticks = 0

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

    def __init__(
        self,
        reminder_interval_ns: int,
        latched_reminder_interval_ns: Optional[int] = None,
        clear_confirm_ticks: int = 1,
    ) -> None:
        """Store the re-notification interval in integer nanoseconds."""
        self._reminder_interval_ns = reminder_interval_ns
        self._latched_reminder_interval_ns = latched_reminder_interval_ns
        self._clear_confirm_ticks = max(1, int(clear_confirm_ticks))
        self._records: Dict[Tuple[str, str], _Record] = {}

    def update(
        self,
        observations: List[Observation],
        now_ns: int,
        wall_sec: float,
    ) -> Tuple[List[Event], List[ActiveFault]]:
        """Advance state by one tick."""
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
            record.missed_ticks = 0

            if observation.severity > previous_severity:
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

        for key in list(self._records):
            if key in seen_keys:
                continue
            record = self._records[key]
            record.missed_ticks += 1
            if record.missed_ticks < self._clear_confirm_ticks:
                continue
            self._records.pop(key)
            record.last_seen_sec = wall_sec
            events.append(Event(record.snapshot(active=False), TRANSITION_CLEARED))

        return events, self._active_faults()

    def _reminder_due(self, record: _Record, now_ns: int) -> bool:
        """Return True when the re-notification interval has elapsed."""
        if record.latched:
            if self._latched_reminder_interval_ns is None:
                return True
            elapsed_ns = now_ns - record.last_notified_ns
            if elapsed_ns < 0:
                return True
            return elapsed_ns >= self._latched_reminder_interval_ns
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
