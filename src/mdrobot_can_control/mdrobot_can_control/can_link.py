"""CAN 링크 상태 모델."""

from typing import Optional


class CanLink:
    """Track CAN bus health and pace reconnect attempts."""

    def __init__(self, retry_interval_ns: int):
        """Start in the healthy state; created only after the bus opened."""
        self.retry_interval_ns = retry_interval_ns
        self._ok = True
        self.last_error: Optional[str] = None
        self._last_attempt_ns: Optional[int] = None

    def record_success(self) -> None:
        """Return to the healthy state after a successful CAN operation."""
        self._ok = True
        self._last_attempt_ns = None

    def record_error(self, exc: BaseException, now_ns: int) -> None:
        """Move to the failed state and remember the reason."""
        self._ok = False
        self.last_error = str(exc)
        self._last_attempt_ns = now_ns

    def is_ok(self) -> bool:
        """Return True only while CAN traffic is believed to work."""
        return self._ok

    def should_retry(self, now_ns: int) -> bool:
        """Return True when a reconnect attempt is due."""
        if self._ok:
            return False
        if self._last_attempt_ns is None:
            return True
        elapsed = now_ns - self._last_attempt_ns
        if elapsed < 0:
            return True
        return elapsed >= self.retry_interval_ns

    def mark_retry_attempted(self, now_ns: int) -> None:
        """Record a reconnect attempt so the next one waits one interval."""
        self._last_attempt_ns = now_ns
