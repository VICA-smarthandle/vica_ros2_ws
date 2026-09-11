"""Pure policy deciding when a comm-caused latch may clear itself."""

from typing import Iterable, Optional


HUMAN_SOURCES = frozenset({"physical_f1", "app", "voice"})

COMM_SOURCES = frozenset({
    "motor_can",
    "physical_stale",
    "motor_can_stale",
    "physical_waiting",
    "motor_can_waiting",
})

BLOCKING_STATES = frozenset({"FAULT", "ESTOP_ACTIVE"})


class AutoRecoveryPolicy:
    """Decide whether the comm-caused latch may be cleared without an operator."""

    def __init__(self, settle_ns: int):
        self.settle_ns = settle_ns
        self.blocked = False
        self.armed = False
        self.clear_since_ns: Optional[int] = None

    def observe_sources(self, sources: Iterable[str], now: int) -> None:
        """Record one central-latch source snapshot."""
        names = set(sources)
        if names & HUMAN_SOURCES:
            self.blocked = True
        if names & COMM_SOURCES:
            self.armed = True
        if names:
            self.clear_since_ns = None
        elif self.clear_since_ns is None:
            self.clear_since_ns = now

    def observe_safety_state(self, previous: str, current: str) -> None:
        """Record a supervisor state transition to spot a stop while driving."""
        if previous == "RUNNING" and current in BLOCKING_STATES:
            self.blocked = True

    def should_recover(self, now: int) -> bool:
        """Return True only for a settled, comm-only, stopped-at-the-time event."""
        if self.blocked or not self.armed:
            return False
        if self.clear_since_ns is None:
            return False
        return now - self.clear_since_ns >= self.settle_ns

    def mark_attempted(self, success: bool) -> None:
        """Record one automatic attempt; a failure hands the event to the operator."""
        self.armed = False
        self.clear_since_ns = None
        if not success:
            self.blocked = True

    def notify_manual_reset(self) -> None:
        """Return to the initial state after an operator cleared the latch."""
        self.blocked = False
        self.armed = False
        self.clear_since_ns = None
