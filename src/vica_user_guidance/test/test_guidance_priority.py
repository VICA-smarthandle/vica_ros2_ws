"""우선순위 병합 순수 로직 테스트.

E-stop > 순찰 > 도착 > 회전 > 기본 순서가 배타적으로 지켜져야 한다.
"""

import json

import pytest

from vica_user_guidance import protocol
from vica_user_guidance.guidance_priority import (
    PATROL_NONE,
    PATROL_REASONS,
    PATROL_STATE_TIMEOUT_NS,
    GuidanceInputs,
    active_patrol_code,
    is_arrival_event,
    parse_goal_event,
    patrol_state_code,
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
    # 여기서 보는 것은 "E-stop을 끄면 회전 안내가 나온다"이지 좌우가 아니다.
    assert out.state_code == protocol.STATE_LEFT


def test_released_estop_returns_to_turn():
    """E-stop이 풀리면 회전 안내로 돌아간다.

    이 시험이 보는 것은 좌우가 아니라 "E-stop 해제 후 회전 안내 복귀"다.
    """
    out = resolve(estop_active=False, turn_direction=DIRECTION_LEFT)
    assert out.state_code == protocol.STATE_LEFT


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


def test_left_cue_sends_left_code():
    """좌회전 cue는 STATE_LEFT를 보낸다. **여기서 좌우를 뒤집지 않는다.**

    이 시험의 목적은 값 확인이 아니라 **계약을 못 박는 것**이다. 배선이 거꾸로
    달린 것을 보정하는 자리는 펌웨어 한 곳이다. 상위에서 한 번 더 뒤집으면 두
    반전이 상쇄돼 어느 층이 무엇을 뒤집는지 추적할 수 없게 된다.

    2026-08-01에 실제로 그 일이 있었다. LED가 반대로 보인다는 보고를 이 자리에서
    대응했는데, 아두이노는 상태코드 하나로 LED와 서보를 같은 case에서 함께 정하므로
    **서보까지 뒤집혔다.** LED를 맞추면서 서보를 깨뜨린 셈이다.

    2026-08-02 실측(ROS를 거치지 않고 펌웨어에 코드 직접 전송)으로 확정했다.

        코드 1 STATE_LEFT   서보 왼쪽  정상 · 주황 LED 오른쪽  반대
        코드 2 STATE_RIGHT  서보 오른쪽 정상 · 주황 LED 왼쪽   반대

    남은 것은 펌웨어의 LED 배정뿐이다. **이 시험이 빨간불이면 고칠 자리는 여기가
    아니라 .ino다.**
    """
    out = resolve(turn_direction=DIRECTION_LEFT)
    assert out.state_code == protocol.STATE_LEFT
    assert out.reason == "turn_left"


def test_right_cue_sends_right_code():
    """우회전 cue는 STATE_RIGHT를 보낸다. 위와 같은 계약이다."""
    out = resolve(turn_direction=DIRECTION_RIGHT)
    assert out.state_code == protocol.STATE_RIGHT
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


# ── 순찰 표시 (2026-11-14, 경비 모드 명세 4.D) ─────────


def resolve_patrol(patrol_code, patrol_last_ns=NOW, **overrides):
    """순찰 입력을 얹어 병합한다. 나머지 기본값은 resolve() 와 같다."""
    inputs = GuidanceInputs(
        estop_active=overrides.pop("estop_active", False),
        estop_last_ns=overrides.pop("estop_last_ns", NOW),
        turn_direction=overrides.pop("turn_direction", DIRECTION_NONE),
        turn_last_ns=overrides.pop("turn_last_ns", NOW),
        arrival_started_ns=overrides.pop("arrival_started_ns", None),
        patrol_code=patrol_code,
        patrol_last_ns=patrol_last_ns,
    )
    assert not overrides, f"알 수 없는 인자: {overrides}"
    return resolve_state_code(
        inputs,
        now_ns=NOW,
        estop_timeout_ns=ESTOP_TIMEOUT,
        cue_timeout_ns=CUE_TIMEOUT,
        arrival_hold_ns=ARRIVAL_HOLD,
    )


@pytest.mark.parametrize(
    "dialog_state, code",
    [
        ("patrolling", protocol.STATE_PATROL),
        ("patrol_dwell", protocol.STATE_PATROL),
        ("patrol_paused", protocol.STATE_PATROL),
        ("patrol_light", protocol.STATE_PATROL_LIGHT),
        ("patrol_alarm", protocol.STATE_PATROL_ALARM),
    ],
)
def test_patrol_dialog_state_maps_to_code(dialog_state, code):
    """명세 4.D 의 대응표: 이동·지점 정지·일시정지 8, 불 켜짐 9, 감지 10."""
    assert patrol_state_code(dialog_state) == code
    out = resolve_patrol(code)
    assert out.state_code == code
    assert out.reason == PATROL_REASONS[code]


@pytest.mark.parametrize(
    "dialog_state",
    ["", "idle", "navigating", "paused", "waiting", "returning", "estopped",
     "Patrolling", " patrol_alarm", "patro"],
)
def test_non_patrol_dialog_state_is_none(dialog_state):
    """순찰이 아닌 값은 PATROL_NONE — 기존 판단이 그대로 돈다."""
    assert patrol_state_code(dialog_state) == PATROL_NONE


def test_unknown_patrol_prefixed_state_is_plain_patrol():
    """'patrol' 로 시작하는 모르는 값은 순찰(8)로 본다(명세 2절: 순찰 여부는 앞 글자로)."""
    assert patrol_state_code("patrol_new_state") == protocol.STATE_PATROL


def test_no_patrol_input_keeps_existing_priority():
    """순찰 값이 없으면 결과가 순찰 추가 전과 같다."""
    assert resolve_patrol(PATROL_NONE).state_code == protocol.STATE_NORMAL
    assert resolve_patrol(
        PATROL_NONE, turn_direction=DIRECTION_LEFT
    ).state_code == protocol.STATE_LEFT
    assert resolve_patrol(
        PATROL_NONE, arrival_started_ns=NOW, turn_direction=DIRECTION_RIGHT
    ).state_code == protocol.STATE_ARRIVED


@pytest.mark.parametrize(
    "code",
    [protocol.STATE_PATROL, protocol.STATE_PATROL_LIGHT, protocol.STATE_PATROL_ALARM],
)
def test_patrol_overrides_turn_and_arrival(code):
    """순찰 중에는 방향지시(1·2)·도착 깜빡임(5) 코드를 내지 않는다."""
    assert resolve_patrol(code, turn_direction=DIRECTION_LEFT).state_code == code
    assert resolve_patrol(code, turn_direction=DIRECTION_RIGHT).state_code == code
    assert resolve_patrol(code, arrival_started_ns=NOW).state_code == code
    assert resolve_patrol(
        code, arrival_started_ns=NOW, turn_direction=DIRECTION_LEFT
    ).state_code == code


@pytest.mark.parametrize(
    "code",
    [protocol.STATE_PATROL, protocol.STATE_PATROL_LIGHT, protocol.STATE_PATROL_ALARM],
)
def test_estop_overrides_patrol(code):
    """비상정지(래치·/estop_state 끊김)는 순찰 표시보다 위다."""
    out = resolve_patrol(code, estop_active=True)
    assert (out.state_code, out.reason) == (protocol.STATE_ESTOP, "estop_active")
    out = resolve_patrol(code, estop_last_ns=None)
    assert (out.state_code, out.reason) == (protocol.STATE_ESTOP, "estop_stale")


def test_patrol_timeout_is_three_seconds():
    assert PATROL_STATE_TIMEOUT_NS == 3 * SEC


def test_patrol_kept_until_exactly_three_seconds():
    """경계값(age == 3 s)까지는 순찰 표시를 유지한다."""
    out = resolve_patrol(protocol.STATE_PATROL, patrol_last_ns=NOW - 3 * SEC)
    assert out.state_code == protocol.STATE_PATROL


def test_patrol_dropped_after_three_seconds_of_silence():
    """robot_state 가 3 s 넘게 끊기면 순찰 코드를 버리고 기존 판단으로 돌아간다."""
    stale = NOW - 3 * SEC - 1
    assert resolve_patrol(
        protocol.STATE_PATROL_ALARM, patrol_last_ns=stale
    ).state_code == protocol.STATE_NORMAL
    assert resolve_patrol(
        protocol.STATE_PATROL, patrol_last_ns=stale, turn_direction=DIRECTION_RIGHT
    ).state_code == protocol.STATE_RIGHT
    assert resolve_patrol(
        protocol.STATE_PATROL, patrol_last_ns=stale, arrival_started_ns=NOW
    ).state_code == protocol.STATE_ARRIVED


def test_patrol_never_received_or_time_reversed_is_ignored():
    """받은 적 없거나(None) 시각이 미래면(시간 역전) 순찰로 보지 않는다."""
    assert resolve_patrol(
        protocol.STATE_PATROL, patrol_last_ns=None
    ).state_code == protocol.STATE_NORMAL
    assert resolve_patrol(
        protocol.STATE_PATROL, patrol_last_ns=NOW + 1
    ).state_code == protocol.STATE_NORMAL


@pytest.mark.parametrize("code", [1, 3, 4, 5, 6, 7, 11, -1, 255])
def test_non_patrol_code_in_patrol_slot_is_ignored(code):
    """순찰 칸에 8·9·10 이 아닌 값이 와도 그대로 내보내지 않는다(특히 4·6·7)."""
    assert active_patrol_code(code, NOW, NOW) == PATROL_NONE
    assert resolve_patrol(code).state_code == protocol.STATE_NORMAL


def test_active_patrol_code_is_what_the_haptic_gate_uses():
    """드라이버가 진동 요청을 버릴지 정하는 판정 — 순찰 값이고 3 s 안일 때만 순찰이다."""
    assert active_patrol_code(protocol.STATE_PATROL, NOW, NOW) == protocol.STATE_PATROL
    assert active_patrol_code(protocol.STATE_PATROL, NOW - 3 * SEC - 1, NOW) == PATROL_NONE
    assert active_patrol_code(PATROL_NONE, NOW, NOW) == PATROL_NONE
    assert active_patrol_code(protocol.STATE_PATROL, None, NOW) == PATROL_NONE


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
        # 순찰 칸에 무엇이 와도(없는 코드 포함) 전송 가능한 코드만 나온다.
        for patrol in (0, 4, 6, 7, 8, 9, 10, 11):
            out = resolve_patrol(patrol, **case)
            assert protocol.is_sendable(out.state_code), f"{case} patrol={patrol} -> {out}"
