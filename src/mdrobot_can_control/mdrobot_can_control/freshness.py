"""Shared steady-clock freshness predicate for the motor final stage.

최종 구동단은 ROS 시간이 아니라 monotonic(STEADY_TIME) clock을 기준으로 timeout을
계산해야 시간 동기화나 simulation time 변경에 영향을 덜 받는다. 타임스탬프는 정수
나노초로 저장하고, 미수신은 ``None``(``0.0`` 아님), 시간 역전(음수 age)은 stale로
처리해 fail-safe한다.
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
