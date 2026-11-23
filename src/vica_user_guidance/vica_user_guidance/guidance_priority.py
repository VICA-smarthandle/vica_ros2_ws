"""E-stop > 순찰 > 도착 > 회전 > 기본 우선순위 병합 (순수 로직)."""

import json
from dataclasses import dataclass
from typing import Optional

from . import protocol
from .timebase import is_fresh_ns
from .turn_detector import DIRECTION_LEFT, DIRECTION_RIGHT

ARRIVAL_EVENT = "goal_succeeded"

PATROL_NONE = 0
PATROL_PREFIX = "patrol"
PATROL_LIGHT_STATE = "patrol_light"
PATROL_ALARM_STATE = "patrol_alarm"
DOCKED_STATE = "docked"
PATROL_STATE_TIMEOUT_NS = 3_000_000_000
PATROL_REASONS = {
    protocol.STATE_PATROL: "patrol",
    protocol.STATE_PATROL_LIGHT: "patrol_light",
    protocol.STATE_PATROL_ALARM: "patrol_alarm",
    protocol.STATE_DOCKED: "docked",
}


@dataclass(frozen=True)
class GuidanceInputs:
    """한 tick의 입력 스냅샷. 전부 호출자가 채운다."""

    estop_active: bool
    estop_last_ns: Optional[int]
    turn_direction: int
    turn_last_ns: Optional[int]
    arrival_started_ns: Optional[int]
    patrol_code: int = PATROL_NONE
    patrol_last_ns: Optional[int] = None


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


def patrol_state_code(dialog_state: str) -> int:
    """dialog_state → 순찰 상태코드. 순찰이 아니면 PATROL_NONE."""
    if dialog_state == DOCKED_STATE:
        return protocol.STATE_DOCKED
    if not dialog_state.startswith(PATROL_PREFIX):
        return PATROL_NONE
    if dialog_state == PATROL_ALARM_STATE:
        return protocol.STATE_PATROL_ALARM
    if dialog_state == PATROL_LIGHT_STATE:
        return protocol.STATE_PATROL_LIGHT
    return protocol.STATE_PATROL


def active_patrol_code(
    patrol_code: int,
    patrol_last_ns: Optional[int],
    now_ns: int,
    timeout_ns: int = PATROL_STATE_TIMEOUT_NS,
) -> int:
    """지금 유효한 순찰 코드. 순찰이 아니거나 robot_state 가 끊겼으면 PATROL_NONE."""
    if patrol_code not in PATROL_REASONS:
        return PATROL_NONE
    if not is_fresh_ns(patrol_last_ns, now_ns, timeout_ns):
        return PATROL_NONE
    return patrol_code


def resolve_state_code(
    inputs: GuidanceInputs,
    now_ns: int,
    estop_timeout_ns: int,
    cue_timeout_ns: int,
    arrival_hold_ns: int,
    estop_required: bool = True,
    patrol_timeout_ns: int = PATROL_STATE_TIMEOUT_NS,
) -> GuidanceOutcome:
    """우선순위를 배타적으로 적용해 상태코드 1개를 고른다."""
    if inputs.estop_active:
        return GuidanceOutcome(protocol.STATE_ESTOP, "estop_active")

    if estop_required and not is_fresh_ns(
        inputs.estop_last_ns, now_ns, estop_timeout_ns
    ):
        return GuidanceOutcome(protocol.STATE_ESTOP, "estop_stale")

    patrol = active_patrol_code(
        inputs.patrol_code, inputs.patrol_last_ns, now_ns, patrol_timeout_ns
    )
    if patrol != PATROL_NONE:
        return GuidanceOutcome(patrol, PATROL_REASONS[patrol])

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
