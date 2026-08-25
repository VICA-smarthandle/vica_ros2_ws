"""RequestApproach 호출 억제 정책 — 순수 로직.

detector 는 5 Hz 로 판정하므로, approachable 인 동안 아무 제어 없이 부르면
Mission Manager 에 초당 5건의 서비스 요청이 몰린다. 접근 중의 재요청은 goal
갱신으로 쓰이지만(mission_logic 이 0.5 m 임계로 재계획 폭주를 이미 막는다),
호출 자체의 상한은 요청자 몫이다.

여기서는 track 별로 최소 간격만 강제한다. 거절당한 track 을 더 오래 쉬게 하는
것(억제 60초의 존중)은 Mission Manager 응답을 보고 노드가 결정한다 — 정책과
응답 해석을 섞지 않는다.
"""
from __future__ import annotations

# 같은 track 에 대한 호출 최소 간격. 5 Hz 판정 -> 초당 1건으로 줄인다.
# 걷는 사람 1.2 m/s 기준 1초에 1.2 m 움직이므로 goal 갱신 주기로도 충분하다
# (갱신 임계 0.5 m 를 놓치지 않는다).
DEFAULT_MIN_INTERVAL_NS = 1_000_000_000


class ApproachRequestThrottle:
    """track 별 마지막 호출 시각을 기억하고 최소 간격을 강제한다.

    시각은 호출자가 소유한 STEADY_TIME 정수 나노초다(detection_gate 와 같은
    규약). 벽시계를 쓰면 시간이 뒤로 갈 수 있다.
    """

    def __init__(self, min_interval_ns: int = DEFAULT_MIN_INTERVAL_NS) -> None:
        if min_interval_ns < 0:
            raise ValueError("min_interval_ns 는 음수일 수 없다")
        self._min_interval_ns = int(min_interval_ns)
        self._last_sent_ns: dict[int, int] = {}

    def should_send(self, track_id: int, now_ns: int) -> bool:
        """지금 이 track 에 요청을 보내도 되는가. True 면 보낸 것으로 기록한다.

        판단과 기록을 한 호출로 묶은 이유: 나눠 두면 호출부가 기록을 잊었을 때
        폭주가 조용히 재발한다. 보낼 수 없으면 기록도 남기지 않는다.
        """
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
