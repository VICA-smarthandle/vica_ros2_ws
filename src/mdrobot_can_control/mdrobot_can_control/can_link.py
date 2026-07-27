"""CAN 링크 상태 모델.

주행 중 CAN 인터페이스가 사라지면 python-can이 예외를 던진다. 이를 잡지 않으면
최종 구동단 노드가 종료되어 정지 상태를 유지하거나 보고할 주체가 사라진다.
이 모델은 예외를 상태로 바꾸고, 재연결 시도 시점을 결정한다.

시각은 모두 정수 나노초(STEADY_TIME)이며 호출자가 주입한다. 이 모델은 걸쇠가
아니다. 걸쇠는 vica_safety의 중앙 래치가 소유한다.
"""

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
        """Return True when a reconnect attempt is due.

        시간 역전(음수 경과)에서도 True를 돌려준다. 재연결이 영구히 막히면
        `/motor/can_ok`가 복구되지 않아 관리자 reset이 불가능해진다.
        """
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
