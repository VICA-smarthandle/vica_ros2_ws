"""RequestApproach 호출 억제 정책 — 순수 로직."""
from __future__ import annotations

DEFAULT_MIN_INTERVAL_NS = 1_000_000_000


class ApproachRequestThrottle:
    """track 별 마지막 호출 시각을 기억하고 최소 간격을 강제한다."""

    def __init__(self, min_interval_ns: int = DEFAULT_MIN_INTERVAL_NS) -> None:
        if min_interval_ns < 0:
            raise ValueError("min_interval_ns 는 음수일 수 없다")
        self._min_interval_ns = int(min_interval_ns)
        self._last_sent_ns: dict[int, int] = {}

    def should_send(self, track_id: int, now_ns: int) -> bool:
        """지금 이 track 에 요청을 보내도 되는가. True 면 보낸 것으로 기록한다."""
        last = self._last_sent_ns.get(track_id)
        if last is not None and now_ns - last < self._min_interval_ns:
            return False
        self._last_sent_ns[track_id] = now_ns
        return True

    def forget(self, track_id: int) -> None:
        """track 이 끝났을 때(접근 완료·소멸) 기록을 지운다. 없어도 무해하다."""
        self._last_sent_ns.pop(track_id, None)

    def prune(self, now_ns: int, older_than_ns: int = 60_000_000_000) -> None:
        """오래된 기록을 청소한다. track_id 는 재사용되지 않지만 계속 늘어난다."""
        stale = [t for t, ns in self._last_sent_ns.items()
                 if now_ns - ns > older_than_ns]
        for t in stale:
            del self._last_sent_ns[t]
