"""우선순위 병합 순수 로직 테스트.

E-stop > 도착 > 회전 > 기본 순서가 배타적으로 지켜져야 한다.
"""

import json

from vica_user_guidance import protocol
from vica_user_guidance.guidance_priority import (
    GuidanceInputs,
    is_arrival_event,
    parse_goal_event,
    resolve_state_code,
)
from vica_user_guidance.turn_detector import (
    DIRECTION_LEFT,
    DIRECTION_NONE,
    DIRECTION_RIGHT,
)

SEC = 1_000_000_000
NOW = 10 * SEC

ESTOP_TIMEOUT = SEC
CUE_TIMEOUT = SEC
ARRIVAL_HOLD = 4 * SEC


def resolve(**overrides):
    """기본값이 '모두 정상'인 입력으로 병합을 수행한다."""
    inputs = GuidanceInputs(
        estop_active=overrides.pop("estop_active", False),
        estop_last_ns=overrides.pop("estop_last_ns", NOW),
        turn_direction=overrides.pop("turn_direction", DIRECTION_NONE),
        turn_last_ns=overrides.pop("turn_last_ns", NOW),
        arrival_started_ns=overrides.pop("arrival_started_ns", None),
    )
    assert not overrides, f"알 수 없는 인자: {overrides}"
    return resolve_state_code(
        inputs,
        now_ns=NOW,
        estop_timeout_ns=ESTOP_TIMEOUT,
        cue_timeout_ns=CUE_TIMEOUT,
        arrival_hold_ns=ARRIVAL_HOLD,
    )


# ── E-stop 최우선 ─────────────────────────────────────


def test_estop_overrides_turn():
    """E-stop은 회전 cue를 덮어쓴다."""
    out = resolve(estop_active=True, turn_direction=DIRECTION_LEFT)
    assert out.state_code == protocol.STATE_ESTOP


def test_estop_overrides_arrival():
    """E-stop은 도착 표시도 덮어쓴다."""
    out = resolve(estop_active=True, arrival_started_ns=NOW)
    assert out.state_code == protocol.STATE_ESTOP


def test_estop_stale_is_treated_as_estop():
    """/estop_state가 끊기면 안전한지 모르는 상태이므로 ESTOP으로 다룬다."""
    out = resolve(estop_last_ns=None)
    assert out.state_code == protocol.STATE_ESTOP
    assert out.reason == "estop_stale"


def test_estop_expired_timestamp_is_stale():
    """timeout을 넘긴 estop 타임스탬프도 stale이다."""
    out = resolve(estop_last_ns=NOW - 2 * SEC)
    assert out.state_code == protocol.STATE_ESTOP


def test_estop_can_be_disabled_for_development():
    """estop_required=False면 stale을 ESTOP으로 보지 않는다(개발용)."""
    inputs = GuidanceInputs(
        estop_active=False,
        estop_last_ns=None,
        turn_direction=DIRECTION_LEFT,
        turn_last_ns=NOW,
        arrival_started_ns=None,
    )
    out = resolve_state_code(
        inputs,
        now_ns=NOW,
        estop_timeout_ns=ESTOP_TIMEOUT,
        cue_timeout_ns=CUE_TIMEOUT,
        arrival_hold_ns=ARRIVAL_HOLD,
        estop_required=False,
    )
    # 좌회전 cue인데 STATE_RIGHT다 — 2026-08-01 하드웨어 임시 교환 때문이다.
    # 여기서 보는 것은 "E-stop을 끄면 회전 안내가 나온다"이지 좌우가 아니다.
    assert out.state_code == protocol.STATE_RIGHT


def test_released_estop_returns_to_turn():
    """E-stop이 풀리면 회전 안내로 돌아간다.

    좌회전 cue인데 STATE_RIGHT를 기대하는 것은 2026-08-01 하드웨어 임시 교환
    때문이다. 이 시험이 보는 것은 좌우가 아니라 "E-stop 해제 후 회전 안내 복귀"다.
    """
    out = resolve(estop_active=False, turn_direction=DIRECTION_LEFT)
    assert out.state_code == protocol.STATE_RIGHT


# ── 도착이 회전보다 우선 ───────────────────────────────


def test_arrival_overrides_turn():
    """도착 표시 중에는 회전 cue보다 도착이 우선이다."""
    out = resolve(arrival_started_ns=NOW, turn_direction=DIRECTION_RIGHT)
    assert out.state_code == protocol.STATE_ARRIVED


def test_arrival_hold_boundary_still_arrived():
    """경계값(age == hold)까지는 도착 표시를 유지한다."""
    out = resolve(arrival_started_ns=NOW - ARRIVAL_HOLD)
    assert out.state_code == protocol.STATE_ARRIVED


def test_arrival_hold_expired_falls_through():
    """hold를 넘기면 기본 상태로 떨어진다."""
    out = resolve(arrival_started_ns=NOW - ARRIVAL_HOLD - 1)
    assert out.state_code == protocol.STATE_NORMAL


def test_arrival_time_reversal_is_ignored():
    """시작 시각이 미래면(시간 역전) 도착으로 보지 않는다."""
    out = resolve(arrival_started_ns=NOW + SEC)
    assert out.state_code == protocol.STATE_NORMAL


# ── 회전 cue ──────────────────────────────────────────


def test_left_cue_maps_to_right_code_by_hardware_workaround():
    """좌회전 cue는 STATE_RIGHT를 보낸다. **일부러 뒤집은 것이다.**

    2026-08-01 실주행에서 사용자가 "서보는 좌우 피드백이 맞는데 황색 점멸등만
    좌우가 바뀌어 온다"고 보고했다. 올바른 자리는 펌웨어(.ino)의 WAVE_A/WAVE_B
    이지만 젯슨에 arduino-cli도 Arduino IDE도 없어 올릴 수 없어서, 오늘은
    ROS 쪽에서 뒤집어 대응한다.

    **펌웨어를 고칠 수 있게 되면 이 교환과 이 시험을 함께 되돌린다.** 양쪽을
    다 뒤집으면 원위치가 되어 다시 반대로 보인다.
    """
    out = resolve(turn_direction=DIRECTION_LEFT)
    assert out.state_code == protocol.STATE_RIGHT
    assert out.reason == "turn_left"


def test_right_cue_maps_to_left_code_by_hardware_workaround():
    """우회전 cue는 STATE_LEFT를 보낸다. 위와 같은 임시 조치다."""
    out = resolve(turn_direction=DIRECTION_RIGHT)
    assert out.state_code == protocol.STATE_LEFT
    assert out.reason == "turn_right"


def test_stale_cue_falls_back_to_normal():
    """cue가 오래되면 회전 안내를 유지하지 않는다."""
    out = resolve(turn_direction=DIRECTION_LEFT, turn_last_ns=NOW - 2 * SEC)
    assert out.state_code == protocol.STATE_NORMAL


def test_never_received_cue_is_normal():
    """cue 미수신은 기본 상태다."""
    out = resolve(turn_direction=DIRECTION_LEFT, turn_last_ns=None)
    assert out.state_code == protocol.STATE_NORMAL


def test_default_is_normal():
    out = resolve()
    assert out.state_code == protocol.STATE_NORMAL


# ── goal_event 파싱 ───────────────────────────────────


def test_only_goal_succeeded_is_arrival():
    assert is_arrival_event("goal_succeeded") is True


def test_goal_failed_is_not_arrival():
    assert is_arrival_event("goal_failed") is False


def test_goal_canceled_is_not_arrival():
    """비상정지로 인한 취소를 도착으로 오인하면 안 된다."""
    assert is_arrival_event("goal_canceled") is False


def test_goal_sent_and_accepted_are_not_arrival():
    """mission_manager는 goal_sent와 goal_accepted를 연속 발행한다."""
    assert is_arrival_event("goal_sent") is False
    assert is_arrival_event("goal_accepted") is False


def test_parse_invalid_json_returns_none():
    """잘못된 JSON에 예외를 던지지 않는다."""
    assert parse_goal_event("not json at all") is None
    assert parse_goal_event("") is None


def test_parse_json_without_event_key_returns_none():
    assert parse_goal_event('{"map_id": "1f"}') is None


def test_parse_real_mission_manager_payload():
    """mission_manager_node의 실제 payload 형식을 파싱한다."""
    payload = json.dumps(
        {
            "event": "goal_succeeded",
            "map_id": "1f",
            "location_id": "cafe",
            "destination_id": "cafe",
            "name": "카페",
            "x": 1.0,
            "y": 2.0,
            "yaw": 90.0,
            "reason": "",
            "timestamp": "2026-07-28T18:00:00",
        },
        ensure_ascii=False,
    )
    assert parse_goal_event(payload) == "goal_succeeded"
    assert is_arrival_event(parse_goal_event(payload)) is True


def test_parse_none_event_is_not_arrival():
    assert is_arrival_event(None) is False


# ── 출력 안전성 ───────────────────────────────────────


def test_all_outputs_are_sendable():
    """어떤 입력 조합에서도 전송 불가 코드(특히 4)가 나오지 않는다."""
    cases = [
        {},
        {"estop_active": True},
        {"estop_last_ns": None},
        {"arrival_started_ns": NOW},
        {"turn_direction": DIRECTION_LEFT},
        {"turn_direction": DIRECTION_RIGHT},
        {"turn_direction": DIRECTION_LEFT, "estop_active": True},
        {"turn_direction": DIRECTION_RIGHT, "arrival_started_ns": NOW},
        {"turn_last_ns": None},
    ]
    for case in cases:
        out = resolve(**case)
        assert protocol.is_sendable(out.state_code), f"{case} -> {out}"
        assert out.state_code != protocol.STATE_LINK_LOST
