"""Shared steady-clock freshness predicate for Safety watchdogs.

All Safety and motor watchdogs must judge age with a single monotonic
(STEADY_TIME) clock and store timestamps as integer nanoseconds. A missing
timestamp is ``None`` (never ``0.0``), and time reversal (negative age) is
treated as stale so any clock jump fails safe.
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
