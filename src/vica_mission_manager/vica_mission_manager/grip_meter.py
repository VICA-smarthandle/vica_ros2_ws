"""손잡이 접촉을 시간으로 잰다 (순수 로직, ROS 모름).

`/vica/smart_handle_state` 는 값이 **바뀐 순간**과 2 Hz 주기에만 온다
(user_guidance_driver_node). 그래서 "2초 중 80 %"를 메시지 개수로 세면 틀린다 —
가만히 2초를 쥐면 메시지는 4개뿐이고, 손을 두드리면 수십 개가 온다. 여기서는
값이 바뀐 시각만 기록하고, 창 안에서 '잡음'이었던 **시간의 합 ÷ 창 길이**를 낸다.

신선도: 메시지가 `stale_sec` 넘게 안 오면 그 시각부터 '놓음'으로 본다. 드라이버가
죽으면 마지막 '잡음'이 붙들린 채 남아 죽은 센서가 "잡고 있다"고 말하게 되기
때문이다(SmartHandleState.msg 의 uplink_fresh 규약과 같은 뜻).

정본: docs/superpowers/specs/2026-09-28-touch-haptic-integration-final.md 4.1·4.2절.
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple


class GripMeter:
    def __init__(self, stale_sec: float = 1.0, history_sec: float = 10.0) -> None:
        if stale_sec <= 0.0:
            raise ValueError("stale_sec 는 양수여야 합니다")
        self.stale_sec = stale_sec
        self.history_sec = history_sec
        # (시각, 실효 접촉) — 값이 바뀐 자리만 담는다.
        self._points: List[Tuple[float, bool]] = []
        self._last_msg_t: Optional[float] = None
        # 마지막 메시지의 fresh 값. 접촉값은 fresh=False 를 '놓음'으로 뭉개 적으므로
        # "센서가 끊겼다"와 "손을 놓았다"를 가르려면 따로 들어야 한다(설계 4.3 (라)).
        self._last_fresh: bool = False

    # -- 입력 ------------------------------------------------------------------

    def update(self, now: float, contact: bool, fresh: bool) -> None:
        """메시지 하나. fresh 가 아니면 접촉값과 무관하게 '놓음'이다."""
        value = bool(contact) and bool(fresh)
        if self._last_msg_t is not None and self._points:
            cutoff = self._last_msg_t + self.stale_sec
            if now > cutoff and self._points[-1][1]:
                # 끊겼던 구간을 되살려 적는다 — 그 사이는 '놓음'이었다.
                self._points.append((cutoff, False))
        if not self._points or self._points[-1][1] != value:
            self._points.append((now, value))
        self._last_msg_t = now
        self._last_fresh = bool(fresh)
        self._prune(now)

    # -- 조회 ------------------------------------------------------------------

    def fresh(self, now: float) -> bool:
        """최근 `stale_sec` 안에 신선한 메시지가 왔는가."""
        if self._last_msg_t is None or now - self._last_msg_t > self.stale_sec:
            return False
        return self._last_fresh

    def contact(self, now: float) -> bool:
        """지금 잡고 있는가 (신선할 때만)."""
        return self._value_at(now)

    def ratio(self, now: float, window: float, since: Optional[float] = None) -> float:
        """[now-window, now] 중 잡고 있던 시간 비율(0~1).

        `since` 이전 시간은 '놓음'으로 친다 — 창 길이는 그대로라 "대기를 시작한
        뒤로 최소 window 만큼은 지켜본다"는 뜻이 된다.
        """
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

    # -- 내부 ------------------------------------------------------------------

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
        # 값이 필요한 가장 이른 시각보다 앞선 점은 버린다. 그 시각의 값을 알려면
        # 직전 점 하나는 남겨야 한다.
        horizon = now - self.history_sec
        while len(self._points) >= 2 and self._points[1][0] <= horizon:
            self._points.pop(0)
