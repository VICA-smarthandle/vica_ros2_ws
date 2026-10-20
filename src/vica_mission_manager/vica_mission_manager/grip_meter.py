"""손잡이 접촉을 시간으로 잰다 (순수 로직, ROS 모름)."""
from __future__ import annotations

import math
from typing import List, Optional, Tuple


class GripMeter:
    def __init__(self, stale_sec: float = 1.0, history_sec: float = 10.0) -> None:
        if stale_sec <= 0.0:
            raise ValueError("stale_sec 는 양수여야 합니다")
        self.stale_sec = stale_sec
        self.history_sec = history_sec
        self._points: List[Tuple[float, bool]] = []
        self._last_msg_t: Optional[float] = None
        self._last_fresh: bool = False

    def update(self, now: float, contact: bool, fresh: bool) -> None:
        """메시지 하나. fresh 가 아니면 접촉값과 무관하게 '놓음'이다."""
        value = bool(contact) and bool(fresh)
        if self._last_msg_t is not None and self._points:
            cutoff = self._last_msg_t + self.stale_sec
            if now > cutoff and self._points[-1][1]:
                self._points.append((cutoff, False))
        if not self._points or self._points[-1][1] != value:
            self._points.append((now, value))
        self._last_msg_t = now
        self._last_fresh = bool(fresh)
        self._prune(now)

    def fresh(self, now: float) -> bool:
        """최근 `stale_sec` 안에 신선한 메시지가 왔는가."""
        if self._last_msg_t is None or now - self._last_msg_t > self.stale_sec:
            return False
        return self._last_fresh

    def contact(self, now: float) -> bool:
        """지금 잡고 있는가 (신선할 때만)."""
        return self._value_at(now)

    def ratio(self, now: float, window: float, since: Optional[float] = None) -> float:
        """[now-window, now] 중 잡고 있던 시간 비율(0~1)."""
        if window <= 0.0:
            return 0.0
        start = now - window
        if since is not None:
            start = max(start, since)
        if start >= now:
            return 0.0
        held = 0.0
        for seg_start, seg_end, value in self._segments(now):
            if not value:
                continue
            lo = max(seg_start, start)
            hi = min(seg_end, now)
            if hi > lo:
                held += hi - lo
        return held / window

    def released_for(self, now: float) -> float:
        """지금까지 계속 '놓음'이었던 시간(초). 잡고 있으면 0, 기록이 없으면 무한대."""
        if self._value_at(now):
            return 0.0
        last_true_end: Optional[float] = None
        for seg_start, seg_end, value in self._segments(now):
            if value:
                last_true_end = seg_end
        if last_true_end is None:
            return math.inf
        return max(0.0, now - last_true_end)

    def _segments(self, now: float):
        """(시작, 끝, 값) 구간들. 마지막 메시지 뒤 stale 시한부터는 '놓음'."""
        pts = self._points
        for i, (t, v) in enumerate(pts):
            end = pts[i + 1][0] if i + 1 < len(pts) else now
            if i + 1 == len(pts) and v and self._last_msg_t is not None:
                cutoff = self._last_msg_t + self.stale_sec
                if cutoff < end:
                    yield (t, cutoff, True)
                    yield (cutoff, end, False)
                    continue
            yield (t, end, v)

    def _value_at(self, now: float) -> bool:
        if not self._points or self._last_msg_t is None:
            return False
        if now - self._last_msg_t > self.stale_sec:
            return False
        return self._points[-1][1]

    def _prune(self, now: float) -> None:
        horizon = now - self.history_sec
        while len(self._points) >= 2 and self._points[1][0] <= horizon:
            self._points.pop(0)
