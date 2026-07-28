"""Steady-clock freshness predicate for the Smart Handle guidance layer.

All watchdogs judge age with a single monotonic (STEADY_TIME) clock and store
timestamps as integer nanoseconds. A missing timestamp is ``None`` (never
``0.0``), and time reversal (negative age) is treated as stale so any clock
jump fails safe.

계약 정본은 ``vica_safety/freshness.py``다. 이 파일은 안전 계층에 대한 역방향
의존을 만들지 않기 위한 의도적 복제이며, ``test/test_timebase.py``가 두 구현의
계약 동일성을 고정한다. 계약을 바꿀 때는 반드시 양쪽을 함께 바꾼다.
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
