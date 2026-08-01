"""Shared steady-clock freshness predicate for the system monitor.

상태 감시도 Safety·motor watchdog과 같은 시간 계약을 쓴다. 단일 monotonic
(STEADY_TIME) clock, 정수 나노초 저장, 미수신은 ``None``(``0.0`` 아님), 시간
역전(음수 age)은 stale로 처리해 fail-safe한다.

이 파일은 ``vica_safety/vica_safety/freshness.py``와
``mdrobot_can_control/mdrobot_can_control/freshness.py``의 로직 동일 사본이다.
감시 패키지가 기능 패키지에 의존하지 않도록 사본을 둔다
(vica_system_health_monitoring_draft.md 5.1절). 사본이 세 개가 되었으므로 향후
공용 시간 패키지로 통합할 여지가 있다 — README에 기록해 두었다.
"""

from typing import Optional


def sec_to_ns(seconds: float) -> int:
    """Convert a timeout expressed in seconds to integer nanoseconds."""
    return int(seconds * 1_000_000_000)


def is_fresh_ns(
    last_ns: Optional[int],
    now_ns: int,
    timeout_ns: int,
) -> bool:
    """Return True only for a received, non-reversed, in-window timestamp.

    fresh = last_ns is not None and 0 <= (now_ns - last_ns) <= timeout_ns
    """
    if last_ns is None:
        return False
    age_ns = now_ns - last_ns
    return 0 <= age_ns <= timeout_ns
