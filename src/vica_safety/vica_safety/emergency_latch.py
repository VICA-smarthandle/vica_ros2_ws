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
        # 부팅 유예: 첫 수신 전의 미수신을 고장으로 승격하지 않는 창.
        # 0 이면 유예가 없고 기존 동작과 완전히 같다(되돌리기 경로).
        self.input_grace_ns = input_grace_ns
        self.start_ns = start_ns
        self.latched = initially_latched
        self.sources = {
            "physical_f1": False,
            "app": False,
            "voice": False,
            "motor_can": False,
        }
        # None = 물리 F1을 한 번도 수신하지 않음 (0.0 sentinel 금지).
        self.last_physical_ns: Optional[int] = None
        # None = motor node의 CAN 상태를 한 번도 수신하지 않음.
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
        """Record the motor node CAN link report.

        ``ok=False``는 CAN 장애이므로 즉시 latch한다. 보고가 끊기는 경우는
        ``evaluate``가 stale로 처리한다(motor node 프로세스 사망 포함).
        """
        self.sources["motor_can"] = not ok
        self.last_motor_can_ns = now
        if not ok:
            self.latched = True

    def _within_grace(self, now: int) -> bool:
        """Report whether the boot grace window is still open.

        시간 역전(음수 경과)은 유예 밖으로 본다. freshness와 같은 fail-safe
        방향이다 -- 시계가 튀었을 때 유예를 늘려 주면 안 된다.
        """
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
        # 래치를 걸 근거가 되는 원인만 따로 센다. `*_waiting`은 원인 목록에는
        # 실리지만(그동안 reset도 자동복구도 막아야 하므로) 래치를 새로 걸지는
        # 않는다. 미수신은 고장이 아니라 아직 오지 않은 것이다.
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
