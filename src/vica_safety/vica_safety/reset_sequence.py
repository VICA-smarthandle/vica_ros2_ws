"""Pure ordered state for the public safety reset orchestration."""

from dataclasses import dataclass


RESET_STEPS = (
    "nav_goal_check",
    "estop_reset",
    "emergency_clear",
    "supervisor_reset",
    "ready",
)


@dataclass(frozen=True)
class SequenceResult:
    """Result of recording one reset orchestration step."""

    success: bool
    step: str
    message: str


class ResetSequence:
    """Require each fail-closed reset step to complete in order."""

    def __init__(self) -> None:
        self._index = None

    @property
    def next_step(self):
        """Return the required next step or None after failure/completion."""
        if self._index is None:
            return None
        return RESET_STEPS[self._index]

    def begin(self) -> None:
        self._index = 0

    def record(self, step: str, accepted: bool, message: str) -> SequenceResult:
        expected = self.next_step
        if expected is None:
            raise ValueError("reset sequence is not active")
        if step != expected:
            raise ValueError(f"expected {expected}, got {step}")
        if not accepted:
            self._index = None
            return SequenceResult(False, step, message)

        self._index += 1
        complete = self._index == len(RESET_STEPS)
        if complete:
            self._index = None
        return SequenceResult(complete, step, message)
