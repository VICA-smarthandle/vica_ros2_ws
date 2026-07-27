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
    """Central latch API implemented independently from ROS wiring.

    All ``now`` arguments are integer nanoseconds from a single STEADY_TIME
    clock owned by the caller (the ROS node).
    """

    def __init__(self, f1_timeout_ns: int, initially_latched: bool = True):
        self.f1_timeout_ns = f1_timeout_ns
        self.latched = initially_latched
        self.sources = {
            "physical_f1": False,
            "app": False,
            "voice": False,
        }
        # None = 물리 F1을 한 번도 수신하지 않음 (0.0 sentinel 금지).
        self.last_physical_ns: Optional[int] = None

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

    def evaluate(self, now: int) -> LatchSnapshot:
        physical_fresh = is_fresh_ns(
            self.last_physical_ns,
            now_ns=now,
            timeout_ns=self.f1_timeout_ns,
        )
        active_sources = [
            name for name, active in self.sources.items() if active
        ]
        if not physical_fresh:
            active_sources.append("physical_stale")
        if active_sources:
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
