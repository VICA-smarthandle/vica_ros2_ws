"""E-stop > 도착 > 회전 > 기본 우선순위 병합 (순수 로직)."""

import json
from dataclasses import dataclass
from typing import Optional

from . import protocol
from .timebase import is_fresh_ns
from .turn_detector import DIRECTION_LEFT, DIRECTION_RIGHT

ARRIVAL_EVENT = "goal_succeeded"


@dataclass(frozen=True)
class GuidanceInputs:
    """한 tick의 입력 스냅샷. 전부 호출자가 채운다."""

    estop_active: bool
    estop_last_ns: Optional[int]
    turn_direction: int
    turn_last_ns: Optional[int]
    arrival_started_ns: Optional[int]


@dataclass(frozen=True)
class GuidanceOutcome:
    """병합 결과. state_code는 반드시 전송 가능한 코드다."""

    state_code: int
    reason: str


def parse_goal_event(payload: str) -> Optional[str]:
    """/vica_goal_event JSON에서 event 문자열만 꺼낸다."""
    try:
        data = json.loads(payload)
    except (ValueError, TypeError):
        return None
    if not isinstance(data, dict):
        return None
    event = data.get("event")
    return event if isinstance(event, str) else None


def is_arrival_event(event: Optional[str]) -> bool:
    """goal_succeeded만 도착이다."""
    return event == ARRIVAL_EVENT


def resolve_state_code(
    inputs: GuidanceInputs,
    now_ns: int,
    estop_timeout_ns: int,
    cue_timeout_ns: int,
    arrival_hold_ns: int,
    estop_required: bool = True,
) -> GuidanceOutcome:
    """우선순위를 배타적으로 적용해 상태코드 1개를 고른다."""
    if inputs.estop_active:
        return GuidanceOutcome(protocol.STATE_ESTOP, "estop_active")

    if estop_required and not is_fresh_ns(
        inputs.estop_last_ns, now_ns, estop_timeout_ns
    ):
        return GuidanceOutcome(protocol.STATE_ESTOP, "estop_stale")

    if inputs.arrival_started_ns is not None:
        age_ns = now_ns - inputs.arrival_started_ns
        if 0 <= age_ns <= arrival_hold_ns:
            return GuidanceOutcome(protocol.STATE_ARRIVED, "arrival")

    if is_fresh_ns(inputs.turn_last_ns, now_ns, cue_timeout_ns):
        if inputs.turn_direction == DIRECTION_LEFT:
            return GuidanceOutcome(protocol.STATE_LEFT, "turn_left")
        if inputs.turn_direction == DIRECTION_RIGHT:
            return GuidanceOutcome(protocol.STATE_RIGHT, "turn_right")

    return GuidanceOutcome(protocol.STATE_NORMAL, "normal")
