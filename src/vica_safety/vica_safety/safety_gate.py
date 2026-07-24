"""Pure state model for VICA drive-command approval."""

from dataclasses import dataclass
from enum import Enum


class SafetyState(str, Enum):
    """Externally visible safety supervisor states."""

    IDLE = "IDLE"
    RUNNING = "RUNNING"
    ESTOP_ACTIVE = "ESTOP_ACTIVE"
    ESTOP_RELEASED_WAIT_RESET = "ESTOP_RELEASED_WAIT_RESET"
    READY_TO_GO = "READY_TO_GO"
    FAULT = "FAULT"


@dataclass(frozen=True)
class ResetDecision:
    """Result returned by an internal supervisor reset request."""

    accepted: bool
    state: SafetyState
    reason: str


class SafetyGate:
    """Keep drive output locked until every reset precondition is safe."""

    def __init__(self) -> None:
        self.state = SafetyState.IDLE
        self.reset_armed = False

    @property
    def can_forward_command(self) -> bool:
        return self.state is SafetyState.RUNNING

    def request_reset(
        self,
        estop_active: bool,
        estop_fresh: bool,
        cmd_zero: bool,
    ) -> ResetDecision:
        if not estop_fresh:
            self.reset_armed = False
            self.state = SafetyState.FAULT
            return ResetDecision(False, self.state, "emergency stop is stale")
        if estop_active:
            self.reset_armed = False
            self.state = SafetyState.ESTOP_ACTIVE
            return ResetDecision(False, self.state, "emergency stop is active")
        if not cmd_zero:
            return ResetDecision(
                False,
                self.state,
                "/cmd_vel_req is not zero",
            )
        self.reset_armed = True
        self.state = SafetyState.READY_TO_GO
        return ResetDecision(True, self.state, "safety supervisor reset accepted")

    def state_for_command(
        self,
        estop_active: bool,
        estop_fresh: bool,
        cmd_alive: bool,
        cmd_zero: bool,
    ) -> SafetyState:
        previous = self.state
        if not estop_fresh:
            self.reset_armed = False
            self.state = SafetyState.FAULT
            return self.state
        if estop_active:
            self.reset_armed = False
            self.state = SafetyState.ESTOP_ACTIVE
            return self.state
        if not self.reset_armed:
            if previous in (SafetyState.ESTOP_ACTIVE, SafetyState.FAULT):
                self.state = SafetyState.ESTOP_RELEASED_WAIT_RESET
            elif previous is not SafetyState.ESTOP_RELEASED_WAIT_RESET:
                self.state = SafetyState.IDLE
            return self.state
        if not cmd_alive or cmd_zero:
            self.state = SafetyState.READY_TO_GO
            return self.state
        self.state = SafetyState.RUNNING
        return self.state
