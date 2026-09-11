"""Pure emergency-stop latch model."""

from dataclasses import dataclass
from typing import Optional

from .freshness import is_fresh_ns


@dataclass(frozen=True)
class LatchSnapshot:
    """Observable central latch state."""

    latched: bool
    active_sources: tuple[str, ...]
    physical_fresh: bool
    reset_allowed: bool


class EmergencyLatch:
    """Central latch API implemented independently from ROS wiring."""

    def __init__(
        self,
        f1_timeout_ns: int,
        motor_can_timeout_ns: int,
        initially_latched: bool = True,
        input_grace_ns: int = 0,
        start_ns: Optional[int] = None,
    ):
        self.f1_timeout_ns = f1_timeout_ns
        self.motor_can_timeout_ns = motor_can_timeout_ns
        self.input_grace_ns = input_grace_ns
        self.start_ns = start_ns
        self.latched = initially_latched
        self.sources = {
            "physical_f1": False,
            "app": False,
            "voice": False,
            "motor_can": False,
        }
        self.last_physical_ns: Optional[int] = None
        self.last_motor_can_ns: Optional[int] = None

    def update_source(self, name: str, active: bool, now: int) -> None:
        del now
        if name not in ("app", "voice"):
            raise ValueError(f"unsupported source: {name}")
        self.sources[name] = active
        if active:
            self.latched = True

    def mark_physical_seen(self, active: bool, now: int) -> None:
        self.sources["physical_f1"] = active
        self.last_physical_ns = now
        if active:
            self.latched = True

    def mark_motor_can_seen(self, ok: bool, now: int) -> None:
        """Record the motor node CAN link report."""
        self.sources["motor_can"] = not ok
        self.last_motor_can_ns = now
        if not ok:
            self.latched = True

    def _within_grace(self, now: int) -> bool:
        """Report whether the boot grace window is still open."""
        if self.input_grace_ns <= 0:
            return False
        if self.start_ns is None:
            self.start_ns = now
        elapsed = now - self.start_ns
        return 0 <= elapsed <= self.input_grace_ns

    def evaluate(self, now: int) -> LatchSnapshot:
        physical_fresh = is_fresh_ns(
            self.last_physical_ns,
            now_ns=now,
            timeout_ns=self.f1_timeout_ns,
        )
        motor_can_fresh = is_fresh_ns(
            self.last_motor_can_ns,
            now_ns=now,
            timeout_ns=self.motor_can_timeout_ns,
        )
        within_grace = self._within_grace(now)

        active_sources = [
            name for name, active in self.sources.items() if active
        ]
        latching_sources = list(active_sources)

        if not physical_fresh:
            if self.last_physical_ns is None and within_grace:
                active_sources.append("physical_waiting")
            else:
                active_sources.append("physical_stale")
                latching_sources.append("physical_stale")
        if not motor_can_fresh:
            if self.last_motor_can_ns is None and within_grace:
                active_sources.append("motor_can_waiting")
            else:
                active_sources.append("motor_can_stale")
                latching_sources.append("motor_can_stale")

        if latching_sources:
            self.latched = True
        return LatchSnapshot(
            latched=self.latched,
            active_sources=tuple(sorted(active_sources)),
            physical_fresh=physical_fresh,
            reset_allowed=self.latched and not active_sources,
        )

    def try_reset(self, now: int) -> tuple[bool, str]:
        snapshot = self.evaluate(now)
        if snapshot.active_sources:
            return False, "active sources: " + ",".join(snapshot.active_sources)
        self.latched = False
        return True, "central estop latch cleared"
