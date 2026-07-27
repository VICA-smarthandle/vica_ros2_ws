"""Pure motor final-stage watchdog decision.

cmd(`/cmd_vel_safe`)와 knob(F1 monitor) 두 입력의 신선도를 단일 STEADY_TIME
clock으로 판정한다. 둘 중 하나라도 미수신·시간 역전·timeout이면 0.0을 돌려
motor를 정지시킨다(fail-safe). 정책·토픽·timeout 값은 노드가 그대로 소유한다.
"""

from typing import Optional

from .freshness import is_fresh_ns


def motor_speed_ratio(
    cmd_last_ns: Optional[int],
    knob_last_ns: Optional[int],
    knob_pct: int,
    now_ns: int,
    cmd_timeout_ns: int,
    knob_timeout_ns: int,
    deadzone_pct: int,
) -> float:
    """Return the knob-derived speed ratio, or 0.0 when any input is stale.

    cmd 또는 knob 중 하나만 stale이어도 0.0(정지)을 반환한다.
    """
    cmd_fresh = is_fresh_ns(
        cmd_last_ns, now_ns=now_ns, timeout_ns=cmd_timeout_ns
    )
    knob_fresh = is_fresh_ns(
        knob_last_ns, now_ns=now_ns, timeout_ns=knob_timeout_ns
    )
    if not cmd_fresh or not knob_fresh:
        return 0.0
    if knob_pct <= deadzone_pct:
        return 0.0
    return knob_pct / 100.0
