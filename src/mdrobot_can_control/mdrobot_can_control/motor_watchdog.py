"""Pure motor final-stage watchdog decision."""

from typing import Optional

from .freshness import is_fresh_ns


def normalize_knob_pct(raw_pct: int, min_pct: int, max_pct: int) -> int:
    """줄이 실제로 움직이는 구간(min~max)을 0~100 으로 편다."""
    if max_pct <= min_pct:
        return raw_pct
    span = max_pct - min_pct
    scaled = round((raw_pct - min_pct) * 100.0 / span)
    return max(0, min(100, int(scaled)))


def motor_speed_ratio_ignoring_knob(
    cmd_last_ns: Optional[int],
    now_ns: int,
    cmd_timeout_ns: int,
) -> float:
    """시험용 ignore_knob(2026-11-02 사용자 요청 — 바퀴 점검)."""
    if is_fresh_ns(cmd_last_ns, now_ns=now_ns, timeout_ns=cmd_timeout_ns):
        return 1.0
    return 0.0


def motor_speed_ratio(
    cmd_last_ns: Optional[int],
    knob_last_ns: Optional[int],
    knob_pct: int,
    now_ns: int,
    cmd_timeout_ns: int,
    knob_timeout_ns: int,
    deadzone_pct: int,
) -> float:
    """Return the knob-derived speed ratio, or 0.0 when any input is stale."""
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
