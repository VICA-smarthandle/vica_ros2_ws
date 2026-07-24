"""Pure emergency-stop latch model."""

from dataclasses import dataclass


@dataclass(frozen=True)
class LatchSnapshot:
    """Observable central latch state."""

    latched: bool
    active_sources: tuple[str, ...]
    physical_fresh: bool
    reset_allowed: bool


class EmergencyLatch:
    """Central latch API implemented independently from ROS wiring."""

    def __init__(self, f1_timeout_sec: float, initially_latched: bool = True):
        self.f1_timeout_sec = f1_timeout_sec
        self.latched = initially_latched
        self.sources = {
            "physical_f1": False,
            "app": False,
            "voice": False,
        }
        self.last_physical_time = 0.0

    def update_source(self, name: str, active: bool, now: float) -> None:
        del now
        if name not in ("app", "voice"):
            raise ValueError(f"unsupported source: {name}")
        self.sources[name] = active
        if active:
            self.latched = True

    def mark_physical_seen(self, active: bool, now: float) -> None:
        self.sources["physical_f1"] = active
        self.last_physical_time = now
        if active:
            self.latched = True

    def evaluate(self, now: float) -> LatchSnapshot:
        physical_fresh = (
            self.last_physical_time > 0.0
            and now - self.last_physical_time <= self.f1_timeout_sec
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

    def try_reset(self, now: float) -> tuple[bool, str]:
        snapshot = self.evaluate(now)
        if snapshot.active_sources:
            return False, "active sources: " + ",".join(snapshot.active_sources)
        self.latched = False
        return True, "central estop latch cleared"
