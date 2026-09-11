"""Shared steady-clock freshness predicate for the system monitor."""

from typing import Optional


def sec_to_ns(seconds: float) -> int:
    """Convert a timeout expressed in seconds to integer nanoseconds."""
    return int(seconds * 1_000_000_000)


def is_fresh_ns(
    last_ns: Optional[int],
    now_ns: int,
    timeout_ns: int,
) -> bool:
    """Return True only for a received, non-reversed, in-window timestamp."""
    if last_ns is None:
        return False
    age_ns = now_ns - last_ns
    return 0 <= age_ns <= timeout_ns
