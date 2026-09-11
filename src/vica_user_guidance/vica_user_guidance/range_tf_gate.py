"""초음파 Range 발행 게이트 (순수 로직, rclpy·tf2 비의존)."""
from __future__ import annotations

from typing import List, Optional


class RangeTfGate:
    """채널별로 "지금 발행해도 되는가"를 정한다."""

    def __init__(self, channels: int) -> None:
        if channels <= 0:
            raise ValueError(f"채널 수는 1 이상이어야 한다: {channels}")
        self._allowed: List[bool] = [True] * channels
        self._pending: List[Optional[bool]] = [None] * channels
        self._blocked: List[int] = [0] * channels

    def allow(self, channel: int, tf_ok: bool) -> bool:
        """이 채널을 지금 발행할 것인가. `tf_ok` 는 노드의 tf2 조회 결과다."""
        self._check(channel)
        ok = bool(tf_ok)
        if ok != self._allowed[channel]:
            self._allowed[channel] = ok
            self._pending[channel] = ok
        if not ok:
            self._blocked[channel] += 1
        return ok

    def take_transition(self, channel: int) -> Optional[bool]:
        """상태가 바뀌었으면 새 상태를 1회만 돌려준다(없으면 None)."""
        self._check(channel)
        pending = self._pending[channel]
        self._pending[channel] = None
        return pending

    def blocked_count(self, channel: int) -> int:
        """막은 횟수. 진단에서 '조용히 안 나간 것'을 셀 수 있어야 한다."""
        self._check(channel)
        return self._blocked[channel]

    def _check(self, channel: int) -> None:
        if not 0 <= channel < len(self._allowed):
            raise IndexError(
                f"채널 범위를 벗어났다: {channel} (0..{len(self._allowed) - 1})"
            )
