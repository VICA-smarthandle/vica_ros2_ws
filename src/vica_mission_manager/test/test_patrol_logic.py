"""경비(순찰) 모드 — 순수 로직 (2026-11-14, ~/vica_data/patrol_plan/SPEC.md 1~3절·4.A).

흐름(시작 → 준비 대기 → 지점 이동 → 3초 정지 → 다음 지점), 경보 4종과 확인·재개,
일시정지, 비상정지, 구간 실패 건너뛰기, 카메라 끊김, 음성 반복 간격, 이벤트 재전송,
그리고 순찰 중에 받지 않는 것들(호출어·음성 요청·사람 접근·목적지 요청)을 못 박는다.
노드 배선은 test_patrol_node.py, 파일은 test_patrol_storage.py 가 본다.
"""
from datetime import datetime

import pytest

from vica_mission_manager.mission_logic import (
    MSG_ESTOPPED,
    MSG_PATROL_INTRUDER,
    MSG_PATROL_LIGHT,
    MSG_TIPPED,
    PATROL_DESTINATION_PREFIX,
    ApproachRequest,
    CancelNav,
    Destination,
    GateReason,
    GoalEvent,
    IntentData,
    MapBounds,
    MissionLogic,
    Navigate,
    NavStatus,
    PatrolEvent,
    PatrolRecordClip,
    Pose2D,
    Say,
    SetNavSpeedCap,
    SetNavSpeedLimit,
    State,
    StopSpeech,
    WakeReply,
    patrol_leg_destination,
)

BOUNDS = MapBounds(min_x=-50, min_y=-50, max_x=50, max_y=50)
A = Destination(id="a", name="로비", pose=Pose2D(1.0, 0.0, 10.0, "map"), calibrated=True)
B = Destination(id="b", name="복도 끝", pose=Pose2D(1.0, 2.0, 20.0, "map"), calibrated=True)
C = Destination(id="c", name="창고 앞", pose=Pose2D(-1.0, 2.0, 30.0, "map"), calibrated=True)
ROUTE = (A, B, C)
HOME = Destination(id="__home__", name="홈", pose=Pose2D(5.0, 5.0, 0.0, "map"))

DETECTOR_OK = {"src": "detector", "kind": "status", "ready": True, "camera_ok": True}
INTRUDER = {"src": "detector", "kind": "intruder", "conf": 0.82, "hits": 2,
            "clip_id": "20261114_203455_intruder"}
LIGHT = {"src": "detector", "kind": "light", "brightness": 35.0, "photo": "light_203455.jpg"}
SOUND = {"src": "audio", "kind": "sound", "db": 71.5, "threshold": 67.0}
WALL = datetime(2026, 11, 14, 20, 34, 55)


def new_logic(**kw) -> MissionLogic:
    logic = MissionLogic(**kw)
    logic.patrol_wall_clock = lambda: WALL
    return logic


def started(route=ROUTE, **kw) -> MissionLogic:
    """경비 시작 직후(t=0) — 감지 노드 준비를 기다리는 중."""
    logic = new_logic(**kw)
    actions, reason = logic.on_patrol_start(route, BOUNDS, True, True, 0.0)
    assert reason == GateReason.OK
    assert actions == [PatrolEvent("patrol_started", route=tuple(d.name for d in route))]
    assert logic.state == State.PATROL_DWELL
    return logic


def tick(logic, t, status=NavStatus.RUNNING, ok=True) -> list:
    """한 틱. ok 면 그 직전에 감지 노드 정상 신호가 온 것으로 친다."""
    if ok:
        logic.on_patrol_event(DETECTOR_OK, t)
    return logic.on_tick(t, status)


def patrolling(route=ROUTE, **kw) -> MissionLogic:
    """준비가 끝나 첫 지점으로 가는 중(t=1)."""
    logic = started(route, **kw)
    tick(logic, 1.0, NavStatus.NONE)
    assert logic.state == State.PATROLLING
    return logic


def dwelling(route=ROUTE, **kw) -> MissionLogic:
    """첫 지점에 도착해 서 있는 중(도착 t=5, 정지는 t=8 까지)."""
    logic = patrolling(route, **kw)
    tick(logic, 5.0, NavStatus.SUCCEEDED)
    assert logic.state == State.PATROL_DWELL
    return logic


def says(actions) -> list:
    return [a.text for a in actions if isinstance(a, Say)]


def events(actions) -> list:
    return [a.event for a in actions if isinstance(a, PatrolEvent)]


def navigates(actions) -> list:
    return [a for a in actions if isinstance(a, Navigate)]


# ---- 계약 값 ---------------------------------------------------------------


def test_sentences_and_state_values_match_the_spec():
    assert MSG_PATROL_INTRUDER == "침입자 발생"
    assert MSG_PATROL_LIGHT == "불 켜짐 감지"
    values = [State.PATROLLING.value, State.PATROL_DWELL.value, State.PATROL_ALARM.value,
              State.PATROL_LIGHT.value, State.PATROL_PAUSED.value]
    assert values == ["patrolling", "patrol_dwell", "patrol_alarm", "patrol_light", "patrol_paused"]
    # 다른 노드는 앞머리로 순찰을 안다 — 순찰이 아닌 상태가 같은 앞머리를 쓰면 안 된다.
    assert {s for s in State if s.value.startswith("patrol")} == {
        State.PATROLLING, State.PATROL_DWELL, State.PATROL_ALARM, State.PATROL_LIGHT,
        State.PATROL_PAUSED}


def test_defaults_match_the_spec():
    logic = MissionLogic()
    assert (logic.patrol_dwell_sec, logic.patrol_intruder_repeat_sec,
            logic.patrol_light_repeat_sec, logic.patrol_alarm_resend_sec,
            logic.patrol_detector_timeout_sec, logic.patrol_arming_timeout_sec,
            logic.patrol_speed_mps, logic.patrol_max_leg_failures) == (
        3.0, 3.0, 10.0, 10.0, 3.0, 15.0, 0.6, 3)


@pytest.mark.parametrize("state", [State.PATROLLING, State.PATROL_DWELL, State.PATROL_ALARM,
                                   State.PATROL_LIGHT, State.PATROL_PAUSED])
def test_dialog_state_is_the_state_value(state):
    logic = MissionLogic()
    logic.state = state
    assert logic.dialog_state == state.value and logic.patrol_active


# ---- 시작 관문 -------------------------------------------------------------


def test_start_needs_a_route_of_two_places():
    logic = new_logic()
    assert logic.on_patrol_start(None, BOUNDS, True, True, 0.0) == ([], GateReason.NO_PATROL_ROUTE)
    assert logic.on_patrol_start([A], BOUNDS, True, True, 0.0) == ([], GateReason.NO_PATROL_ROUTE)
    assert logic.state == State.IDLE and not logic.patrol_active


def test_start_is_rejected_while_estopped_busy_or_already_patrolling():
    logic = new_logic()
    logic.on_estop(True, 0.0)
    assert logic.on_patrol_start(ROUTE, BOUNDS, True, True, 1.0)[1] == GateReason.ESTOP_ACTIVE

    busy = new_logic()
    busy.on_intent(IntentData("navigate", "a", False, "normal"), A, BOUNDS, True, 0.0)
    assert busy.state == State.NAVIGATING
    assert busy.on_patrol_start(ROUTE, BOUNDS, True, True, 1.0) == ([], GateReason.BUSY_NAVIGATING)
    assert busy.state == State.NAVIGATING

    again = started()
    assert again.on_patrol_start(ROUTE, BOUNDS, True, True, 1.0) == ([], GateReason.ALREADY_PATROLLING)


def test_start_is_rejected_without_nav2_or_a_known_pose():
    logic = new_logic()
    assert logic.on_patrol_start(ROUTE, BOUNDS, False, True, 0.0)[1] == GateReason.NAV_NOT_READY
    assert logic.on_patrol_start(ROUTE, BOUNDS, True, False, 0.0)[1] == GateReason.POSE_UNKNOWN
    assert logic.state == State.IDLE


def test_start_checks_every_place_like_a_destination_request():
    blocked = Destination(id="x", name="공사 중", pose=Pose2D(2.0, 2.0, 0.0, "map"),
                          calibrated=True, is_approachable=False)
    outside = Destination(id="y", name="지도 밖", pose=Pose2D(99.0, 0.0, 0.0, "map"), calibrated=True)
    logic = new_logic()
    assert logic.on_patrol_start([A, blocked], BOUNDS, True, True, 0.0)[1] == GateReason.POSE_INVALID
    assert logic.on_patrol_start([A, outside], BOUNDS, True, True, 0.0)[1] == GateReason.POSE_INVALID


def test_private_places_may_be_patrolled():
    """순찰은 관리자만 시작한다 — 배송처럼 비공개 장소도 돈다."""
    office = Destination(id="o", name="교수 사무실", pose=Pose2D(3.0, 1.0, 0.0, "map"),
                         calibrated=True, authorization="private")
    assert new_logic().on_patrol_start([A, office], BOUNDS, True, True, 0.0)[1] == GateReason.OK


def test_start_clears_what_idle_was_holding():
    logic = new_logic(return_destination=HOME)
    logic._return_interrupted = True
    logic._seek_deadline = 5.0
    logic._approach_lock = True
    logic.on_patrol_start(ROUTE, BOUNDS, True, True, 0.0)
    assert not logic.return_interrupted and logic._seek_deadline is None
    assert not logic._approach_lock


# ---- 준비 대기 → 첫 지점 ---------------------------------------------------


def test_waits_for_the_detector_before_the_first_leg():
    logic = started()
    assert logic.on_tick(1.0, NavStatus.NONE) == []          # 아직 신호 없음 — 출발하지 않는다
    assert logic.state == State.PATROL_DWELL
    logic.on_patrol_event({"src": "detector", "kind": "status", "ready": True,
                           "camera_ok": False}, 1.5)          # 카메라가 아직이다
    logic.on_patrol_event({"src": "audio", "kind": "status", "ready": True}, 1.5)   # 음성 쪽은 조건 아님
    assert logic.on_tick(2.0, NavStatus.NONE) == []

    logic.on_patrol_event(DETECTOR_OK, 2.5)
    actions = logic.on_tick(2.6, NavStatus.NONE)
    assert logic.state == State.PATROLLING
    assert actions == [
        PatrolEvent("patrol_waypoint", destination=A, index=0, lap=1, count=3),
        SetNavSpeedCap(0.6),
        Navigate(patrol_leg_destination(A, B)),
    ]


def test_stops_when_the_detector_never_gets_ready():
    logic = started()
    assert logic.on_tick(14.9, NavStatus.NONE) == []
    actions = logic.on_tick(15.0, NavStatus.NONE)
    assert actions == [PatrolEvent("patrol_stopped", reason="detector_not_ready")]
    assert logic.state == State.IDLE and not logic.patrol_active


def test_sound_before_the_detector_is_ready_is_dropped():
    """준비 대기 중의 큰 소리는 시작 버튼을 누른 관리자의 소리다 — 버린다."""
    logic = started()
    assert logic.on_patrol_event(SOUND, 2.0) == []
    assert logic.on_patrol_event(dict(INTRUDER, src="audio"), 2.0) == []   # 감지 노드 것이 아니면 버린다
    assert logic.on_patrol_event({"kind": "intruder", "clip_id": "c"}, 2.0) == []
    assert logic.state == State.PATROL_DWELL
    assert logic.on_tick(15.0, NavStatus.NONE) == [
        PatrolEvent("patrol_stopped", reason="detector_not_ready")]       # 준비된 것으로 치지도 않는다


def test_intruder_from_the_detector_during_arming_proves_it_is_ready():
    """경비 시작 직후에도 사람 감지를 쉬지 않는다(2026-11-14 사용자 결정) — 감지 노드의 경보는
    그 자체가 준비됐다는 증거다. status 를 받기 전이어도 받는다."""
    logic = started()
    actions = logic.on_patrol_event(INTRUDER, 2.0)
    assert logic.state == State.PATROL_ALARM
    assert actions == [
        PatrolEvent("patrol_intruder", alarm_id="20261114_203455_intruder", kind="intruder",
                    time="2026-11-14T20:34:55", clip_id="20261114_203455_intruder"),
        Say("침입자 발생", priority="emergency"),
    ]
    assert logic.on_tick(20.0, NavStatus.NONE) != [
        PatrolEvent("patrol_stopped", reason="detector_not_ready")]       # 준비 시한은 끝났다
    assert logic.state == State.PATROL_ALARM
    # 재개하면 아직 가지 않은 첫 지점으로 간다.
    logic.on_patrol_event(DETECTOR_OK, 21.0)
    actions, reason = logic.on_resume_request(True, 21.5)
    assert reason == GateReason.OK
    assert navigates(actions) == [Navigate(patrol_leg_destination(A, B))]


def test_light_from_the_detector_during_arming_is_taken_too():
    logic = started()
    actions = logic.on_patrol_event(LIGHT, 2.0)
    assert logic.state == State.PATROL_LIGHT and says(actions) == ["불 켜짐 감지"]
    # 그 경보가 준비의 증거라 곧바로(3초 안) 재개할 수 있다.
    assert logic.on_resume_request(True, 3.0)[1] == GateReason.OK


def test_once_ready_sound_is_taken_as_before():
    logic = started()
    logic.on_patrol_event(INTRUDER, 2.0)
    logic.on_resume_request(True, 3.0)
    assert events(logic.on_patrol_event(SOUND, 4.0)) == ["patrol_sound"]


# ---- 구간 goal --------------------------------------------------------------


def test_leg_goal_faces_the_next_place_not_the_saved_heading():
    leg = patrol_leg_destination(A, B)
    assert leg.id == PATROL_DESTINATION_PREFIX + "a" and leg.name == "로비"
    assert (leg.pose.x, leg.pose.y) == (1.0, 0.0)
    assert leg.pose.yaw_deg == pytest.approx(90.0)                          # A(1,0) → B(1,2)
    assert patrol_leg_destination(B, C).pose.yaw_deg == pytest.approx(180.0)
    assert patrol_leg_destination(C, A).pose.yaw_deg == pytest.approx(-45.0)   # 끝 다음은 처음


def test_leg_goal_keeps_the_saved_heading_when_places_overlap():
    twin = Destination(id="t", name="같은 자리", pose=Pose2D(1.0, 0.0, 77.0, "map"), calibrated=True)
    assert patrol_leg_destination(A, twin).pose.yaw_deg == 10.0


def test_patrol_legs_never_carry_a_destination_for_goal_events():
    """구간 이동에는 goal_* 가 나가지 않는다 — 로직은 active_destination 을 비워 두고
    취소에도 목적지를 싣지 않는다(노드가 goal_canceled 를 만들 재료가 없다)."""
    logic = patrolling()
    assert logic.active_destination is None
    seen = []
    seen += tick(logic, 5.0, NavStatus.SUCCEEDED)
    seen += tick(logic, 8.1, NavStatus.NONE)
    seen += logic.on_patrol_event(INTRUDER, 9.0)
    assert logic.active_destination is None
    assert not [a for a in seen if isinstance(a, GoalEvent)]
    assert [a for a in seen if isinstance(a, CancelNav)] == [CancelNav(None)]
    assert all(n.destination.id.startswith(PATROL_DESTINATION_PREFIX) for n in navigates(seen))


# ---- 도착 → 3초 정지 → 다음 지점 → 한 바퀴 ---------------------------------


def test_dwells_three_seconds_at_each_place_then_goes_on():
    logic = patrolling()
    assert tick(logic, 5.0, NavStatus.SUCCEEDED) == [SetNavSpeedLimit(0.0)]
    assert logic.state == State.PATROL_DWELL
    assert tick(logic, 7.9, NavStatus.NONE) == []
    actions = tick(logic, 8.0, NavStatus.NONE)
    assert logic.state == State.PATROLLING
    assert actions == [
        PatrolEvent("patrol_waypoint", destination=B, index=1, lap=1, count=3),
        SetNavSpeedCap(0.6),
        Navigate(patrol_leg_destination(B, C)),
    ]


def test_after_the_last_place_comes_the_first_and_the_lap_counts_up():
    logic = patrolling()
    t = 1.0
    seen = []
    for _ in range(4):                       # A·B·C 도착, 그리고 다시 A 도착
        t += 4.0
        tick(logic, t, NavStatus.SUCCEEDED)
        t += 3.0
        seen += tick(logic, t, NavStatus.NONE)
    waypoints = [(a.index, a.lap) for a in seen if isinstance(a, PatrolEvent)]
    assert waypoints == [(1, 1), (2, 1), (0, 2), (1, 2)]
    assert navigates(seen)[2].destination == patrol_leg_destination(A, B)


def test_speed_cap_is_optional():
    logic = started(patrol_speed_mps=0.0)
    actions = tick(logic, 1.0, NavStatus.NONE)
    assert not [a for a in actions if isinstance(a, SetNavSpeedCap)]
    assert navigates(actions)


# ---- 구간 실패 --------------------------------------------------------------


def test_a_failed_leg_skips_to_the_next_place_without_dwelling():
    logic = patrolling()
    actions = tick(logic, 4.0, NavStatus.FAILED)
    assert logic.state == State.PATROLLING
    assert actions == [
        SetNavSpeedLimit(0.0),
        PatrolEvent("patrol_waypoint", destination=B, index=1, lap=1, count=3),
        SetNavSpeedCap(0.6),
        Navigate(patrol_leg_destination(B, C)),
    ]


def test_three_failures_in_a_row_end_the_patrol():
    logic = patrolling()
    tick(logic, 4.0, NavStatus.FAILED)
    tick(logic, 5.0, NavStatus.CANCELED)            # 밖에서 취소된 것도 못 간 것이다
    actions = tick(logic, 6.0, NavStatus.FAILED)
    assert actions == [SetNavSpeedLimit(0.0), PatrolEvent("patrol_stopped", reason="nav_failed")]
    assert logic.state == State.IDLE and not logic.patrol_active


def test_an_arrival_resets_the_failure_count():
    logic = patrolling()
    tick(logic, 4.0, NavStatus.FAILED)
    tick(logic, 5.0, NavStatus.FAILED)
    tick(logic, 9.0, NavStatus.SUCCEEDED)           # C 도착 — 연속이 끊겼다
    tick(logic, 12.0, NavStatus.NONE)
    tick(logic, 13.0, NavStatus.FAILED)
    tick(logic, 14.0, NavStatus.FAILED)
    assert logic.state == State.PATROLLING


# ---- 경보 -------------------------------------------------------------------


def test_intruder_while_moving_stops_then_reports_then_speaks():
    logic = patrolling()
    actions = logic.on_patrol_event(INTRUDER, 3.0)
    assert logic.state == State.PATROL_ALARM
    assert actions == [
        SetNavSpeedLimit(0.0),
        CancelNav(None),
        PatrolEvent("patrol_intruder", alarm_id="20261114_203455_intruder", kind="intruder",
                    time="2026-11-14T20:34:55", clip_id="20261114_203455_intruder"),
        Say("침입자 발생", priority="emergency"),
    ]


def test_sound_while_dwelling_reports_the_level():
    logic = dwelling()
    actions = logic.on_patrol_event(SOUND, 6.0)
    assert logic.state == State.PATROL_ALARM
    assert actions == [
        PatrolEvent("patrol_sound", alarm_id="20261114_203455_sound", kind="sound",
                    time="2026-11-14T20:34:55", level_db=71.5),
        Say("침입자 발생", priority="emergency"),
    ]


def test_light_has_its_own_state_and_sentence():
    logic = dwelling()
    actions = logic.on_patrol_event(LIGHT, 6.0)
    assert logic.state == State.PATROL_LIGHT
    assert actions == [
        PatrolEvent("patrol_light", alarm_id="20261114_203455_light", kind="light",
                    time="2026-11-14T20:34:55", photo="light_203455.jpg"),
        Say("불 켜짐 감지", priority="emergency"),
    ]


@pytest.mark.parametrize("make", [
    lambda: (lambda l: l.on_patrol_event(INTRUDER, 3.0) and l)(patrolling()),
    lambda: (lambda l: l.on_patrol_event(LIGHT, 6.0) and l)(dwelling()),
    lambda: (lambda l: l.on_pause_request(3.0) and l)(patrolling()),
])
def test_further_detections_are_ignored_outside_moving_and_dwelling(make):
    logic = make()
    state = logic.state
    for event in (INTRUDER, LIGHT, SOUND):
        assert logic.on_patrol_event(event, 7.0) == []
    assert logic.state == state


# ---- 감시 항목 켜고 끄기(2026-11-14 사용자 요청) -----------------------------


def watching(watch_off, dwell=False) -> MissionLogic:
    """꺼 둔 항목이 있는 순찰 — 첫 지점으로 가는 중(t=1) 또는 그 지점에 서 있는 중(t=5~8)."""
    logic = new_logic()
    assert logic.on_patrol_start(ROUTE, BOUNDS, True, True, 0.0, watch_off)[1] == GateReason.OK
    tick(logic, 1.0, NavStatus.NONE)
    if dwell:
        tick(logic, 5.0, NavStatus.SUCCEEDED)
    return logic


@pytest.mark.parametrize("off,event", [("intruder", INTRUDER), ("light", LIGHT), ("sound", SOUND)])
def test_an_item_switched_off_raises_no_alarm_moving_or_dwelling(off, event):
    for dwell in (False, True):
        logic = watching([off], dwell)
        state = logic.state
        assert logic.on_patrol_event(event, 6.0) == []
        assert logic.state == state


def test_the_other_items_still_raise_alarms():
    logic = watching(["light"], dwell=True)
    assert logic.on_patrol_event(LIGHT, 6.0) == []
    assert events(logic.on_patrol_event(SOUND, 6.5)) == ["patrol_sound"]
    logic = watching(["intruder", "sound"])
    assert logic.on_patrol_event(INTRUDER, 2.0) == [] and logic.on_patrol_event(SOUND, 2.5) == []
    logic = watching(["intruder", "sound"], dwell=True)
    assert events(logic.on_patrol_event(LIGHT, 6.0)) == ["patrol_light"]


def test_an_item_switched_off_does_not_count_as_proof_of_readiness():
    logic = new_logic()
    logic.on_patrol_start(ROUTE, BOUNDS, True, True, 0.0, ["intruder"])
    assert logic.on_patrol_event(INTRUDER, 0.5) == []
    assert logic.state == State.PATROL_DWELL
    assert tick(logic, 0.6, NavStatus.NONE, ok=False) == []      # 아직 준비 신호가 없다 — 출발하지 않는다
    assert navigates(tick(logic, 1.0, NavStatus.NONE))           # 준비 신호가 오면 출발


def test_camera_loss_cannot_be_switched_off():
    logic = new_logic()
    logic.on_patrol_start(ROUTE, BOUNDS, True, True, 0.0, ["camera_lost", "status", "clip", "light"])
    tick(logic, 1.0, NavStatus.NONE)
    assert logic.state == State.PATROLLING
    assert events(tick(logic, 6.0, ok=False)) == ["patrol_camera_lost"]   # 3초 넘게 정상 신호 없음


def test_malformed_events_are_ignored():
    logic = patrolling()
    for bad in ({}, {"kind": "unknown"}, {"kind": 7}, {"kind": ["sound"]}, [], "intruder", None):
        assert logic.on_patrol_event(bad, 2.0) == []
    assert logic.state == State.PATROLLING
    # 숫자가 아닌 크기·글자가 아닌 파일 이름은 빈 값으로 받는다.
    actions = logic.on_patrol_event({"kind": "sound", "db": "loud"}, 2.5)
    assert actions[2] == PatrolEvent("patrol_sound", alarm_id="20261114_203455_sound",
                                     kind="sound", time="2026-11-14T20:34:55")


# ---- 음성 반복·이벤트 재전송·확인 -------------------------------------------


def _alarm_ticks(logic, start, end, step=0.2):
    """경보 상태에서 start~end 를 틱으로 돌며 (시각, 말, 재전송) 을 모은다."""
    said, resent = [], []
    t = start
    while t <= end + 1e-9:
        for a in logic.on_tick(round(t, 3), NavStatus.NONE):
            if isinstance(a, Say):
                said.append((round(t, 3), a.text, a.priority))
            elif isinstance(a, PatrolEvent):
                resent.append((round(t, 3), a.event, a.resend))
        t += step
    return said, resent


def test_intruder_sentence_repeats_every_three_seconds_until_acked():
    logic = patrolling()
    logic.on_patrol_event(INTRUDER, 10.0)
    said, _ = _alarm_ticks(logic, 10.2, 19.8)
    assert said == [(13.0, "침입자 발생", "emergency"), (16.0, "침입자 발생", "emergency"),
                    (19.0, "침입자 발생", "emergency")]
    assert logic.on_patrol_ack("", 20.0) == (
        [StopSpeech(), PatrolEvent("patrol_acked", alarm_id="20261114_203455_intruder")], GateReason.OK)
    assert _alarm_ticks(logic, 20.2, 40.0) == ([], [])
    assert logic.state == State.PATROL_ALARM            # 확인은 음성만 끈다 — 정지는 그대로


def test_light_sentence_repeats_every_ten_seconds():
    logic = dwelling()
    logic.on_patrol_event(LIGHT, 6.0)
    said, _ = _alarm_ticks(logic, 6.2, 27.0)
    assert said == [(16.0, "불 켜짐 감지", "emergency"), (26.0, "불 켜짐 감지", "emergency")]


def test_alarm_event_is_resent_every_ten_seconds_until_acked():
    logic = patrolling()
    logic.on_patrol_event(INTRUDER, 10.0)
    _, resent = _alarm_ticks(logic, 10.2, 31.0)
    assert resent == [(20.0, "patrol_intruder", True), (30.0, "patrol_intruder", True)]
    logic.on_patrol_ack("", 31.0)
    assert _alarm_ticks(logic, 31.2, 60.0)[1] == []


def test_a_late_tick_does_not_pile_up_repeats():
    logic = patrolling()
    logic.on_patrol_event(INTRUDER, 10.0)
    actions = logic.on_tick(40.0, NavStatus.NONE)      # 30초 밀린 틱
    assert says(actions) == ["침입자 발생"] and events(actions) == ["patrol_intruder"]
    assert logic.on_tick(40.2, NavStatus.NONE) == []


def test_ack_is_idempotent_and_checks_the_alarm_id():
    logic = patrolling()
    assert logic.on_patrol_ack("", 2.0) == ([], GateReason.NO_PATROL_ALARM)
    logic.on_patrol_event(INTRUDER, 3.0)
    assert logic.on_patrol_ack("20261114_200000_sound", 4.0) == ([], GateReason.PATROL_ALARM_MISMATCH)
    assert events(logic.on_patrol_ack("20261114_203455_intruder", 5.0)[0]) == ["patrol_acked"]
    assert logic.on_patrol_ack("20261114_203455_intruder", 6.0) == ([], GateReason.OK)


def test_ack_cuts_the_sentence_that_is_playing():
    """확인은 재생 중인 경보 음성도 끊는다(2026-11-16 — 경고음을 붙인 녹음이 약 3초). 두 번째 확인은 조용하다."""
    for alarm in (INTRUDER, SOUND):
        logic = patrolling()
        logic.on_patrol_event(alarm, 3.0)
        actions, reason = logic.on_patrol_ack("", 4.0)
        assert reason == GateReason.OK and actions[0] == StopSpeech()
        assert events(actions) == ["patrol_acked"]
        assert logic.on_patrol_ack("", 5.0) == ([], GateReason.OK)


def test_stop_during_an_unacked_alarm_cuts_the_sentence_first():
    logic = with_home(patrolling())
    logic.on_patrol_event(INTRUDER, 3.0)
    actions, _ = logic.on_patrol_stop(4.0)
    assert actions[0] == StopSpeech()
    acked = with_home(patrolling())
    acked.on_patrol_event(INTRUDER, 3.0)
    acked.on_patrol_ack("", 3.5)
    assert StopSpeech() not in acked.on_patrol_stop(4.0)[0]      # 이미 끊었다 — 홈 복귀 안내를 지우지 않는다


# ---- 영상 id ----------------------------------------------------------------


def test_clip_id_learned_later_is_attached_to_the_sound_alarm():
    logic = patrolling()
    logic.on_patrol_event(SOUND, 10.0)
    clip = {"src": "detector", "kind": "clip", "clip_id": "20261114_203456_sound",
            "state": "started", "alarm": "sound"}
    assert logic.on_patrol_event(clip, 10.5) == [
        PatrolRecordClip("20261114_203455_sound", "20261114_203456_sound")]
    assert logic.on_patrol_event(dict(clip, state="done", frames=75), 25.0) == []   # 이미 붙였다
    # 재전송에는 그 뒤로 영상 id 가 실린다.
    resend = [a for a in logic.on_tick(20.0, NavStatus.NONE) if isinstance(a, PatrolEvent)]
    assert resend[0].clip_id == "20261114_203456_sound" and resend[0].resend


def test_intruder_alarm_already_knows_its_clip():
    logic = patrolling()
    logic.on_patrol_event(INTRUDER, 10.0)
    clip = {"src": "detector", "kind": "clip", "clip_id": INTRUDER["clip_id"],
            "state": "started", "alarm": "intruder"}
    assert logic.on_patrol_event(clip, 10.2) == []


SOUND_CLIP = {"src": "detector", "kind": "clip", "clip_id": "20261114_203456_sound",
              "state": "started", "alarm": "sound"}


def test_camera_lost_alarm_never_gets_the_sound_clip():
    """감지 노드는 제가 내지 않은 patrol_alarm 이면 소리 장면을 찍는다 — 카메라 끊김 경보에
    그 `…_sound` 장면이 붙으면 안 된다(2026-11-14 구성요소 감사)."""
    logic = patrolling()
    tick(logic, 4.2, ok=False)                                # 카메라 끊김 경보
    assert logic.state == State.PATROL_ALARM
    assert logic.on_patrol_event(SOUND_CLIP, 4.5) == []
    assert logic.on_patrol_event(dict(SOUND_CLIP, state="done", frames=75), 20.0) == []
    resent = [a for a in logic.on_tick(14.5, NavStatus.NONE) if isinstance(a, PatrolEvent)]
    assert resent and resent[0].clip_id == ""


def test_light_alarm_never_gets_a_clip():
    logic = dwelling()
    logic.on_patrol_event(LIGHT, 6.0)
    assert logic.on_patrol_event(SOUND_CLIP, 6.5) == []
    assert logic.on_patrol_event(dict(SOUND_CLIP, alarm="intruder"), 6.5) == []


def test_clip_of_another_kind_is_not_attached_to_the_current_alarm():
    logic = patrolling()
    logic.on_patrol_event(SOUND, 10.0)
    clip = {"src": "detector", "kind": "clip", "clip_id": INTRUDER["clip_id"],
            "state": "started", "alarm": "intruder"}
    assert logic.on_patrol_event(clip, 10.2) == []
    # 같은 종류(sound)면 붙는다.
    assert logic.on_patrol_event(SOUND_CLIP, 10.3) == [
        PatrolRecordClip("20261114_203455_sound", "20261114_203456_sound")]


def test_clip_without_its_alarm_kind_is_not_attached():
    logic = patrolling()
    logic.on_patrol_event(SOUND, 10.0)
    for kind in (None, "", "light", "camera_lost", 7):
        assert logic.on_patrol_event(dict(SOUND_CLIP, alarm=kind), 10.2) == []
    no_key = {k: v for k, v in SOUND_CLIP.items() if k != "alarm"}
    assert logic.on_patrol_event(no_key, 10.2) == []


def test_clip_without_an_id_is_ignored():
    logic = patrolling()
    logic.on_patrol_event(SOUND, 10.0)
    for clip in ({"kind": "clip", "alarm": "sound"}, {"kind": "clip", "clip_id": "", "alarm": "sound"},
                 {"kind": "clip", "clip_id": 7, "alarm": "sound"}):
        assert logic.on_patrol_event(clip, 10.2) == []


def test_late_clip_of_another_kind_is_not_attached_to_the_last_alarm():
    logic = patrolling()
    logic.on_patrol_event(SOUND, 10.0)
    logic.on_patrol_event(DETECTOR_OK, 11.0)
    logic.on_resume_request(True, 11.0)
    clip = {"src": "detector", "kind": "clip", "clip_id": "x_intruder", "state": "done",
            "alarm": "intruder"}
    assert logic.on_patrol_event(clip, 11.5) == []


def test_clip_that_arrives_after_resume_goes_to_the_last_alarm():
    logic = patrolling()
    logic.on_patrol_event(SOUND, 10.0)
    logic.on_patrol_event(DETECTOR_OK, 11.0)
    assert logic.on_resume_request(True, 11.0)[1] == GateReason.OK
    clip = {"src": "detector", "kind": "clip", "clip_id": "20261114_203456_sound",
            "state": "started", "alarm": "sound"}
    assert logic.on_patrol_event(clip, 11.5) == [
        PatrolRecordClip("20261114_203455_sound", "20261114_203456_sound")]


# ---- 재개 -------------------------------------------------------------------


def test_resume_from_an_alarm_on_the_way_acks_and_goes_to_the_same_place():
    logic = patrolling()
    logic.on_patrol_event(INTRUDER, 3.0)
    logic.on_patrol_event(DETECTOR_OK, 9.0)
    actions, reason = logic.on_resume_request(True, 9.5)
    assert reason == GateReason.OK and logic.state == State.PATROLLING
    assert actions == [
        StopSpeech(),
        PatrolEvent("patrol_acked", alarm_id="20261114_203455_intruder"),
        PatrolEvent("patrol_resumed"),
        PatrolEvent("patrol_waypoint", destination=A, index=0, lap=1, count=3),
        SetNavSpeedCap(0.6),
        Navigate(patrol_leg_destination(A, B)),
    ]


def test_resume_after_ack_does_not_ack_twice():
    logic = patrolling()
    logic.on_patrol_event(INTRUDER, 3.0)
    logic.on_patrol_ack("", 4.0)
    logic.on_patrol_event(DETECTOR_OK, 9.0)
    assert events(logic.on_resume_request(True, 9.5)[0]) == ["patrol_resumed", "patrol_waypoint"]


def test_resume_from_an_alarm_at_a_place_goes_to_the_next_place():
    logic = dwelling()
    logic.on_patrol_event(LIGHT, 6.0)
    logic.on_patrol_event(DETECTOR_OK, 20.0)
    actions, reason = logic.on_resume_request(True, 20.5)
    assert reason == GateReason.OK
    assert navigates(actions) == [Navigate(patrol_leg_destination(B, C))]


def test_resume_is_refused_while_the_camera_is_down_or_nav2_is_not_ready():
    logic = patrolling()
    logic.on_patrol_event(INTRUDER, 3.0)
    # 마지막 정상 신호(t=1)에서 3초 넘게 지났다.
    assert logic.on_resume_request(True, 9.0) == ([], GateReason.DETECTOR_NOT_READY)
    assert logic.state == State.PATROL_ALARM
    logic.on_patrol_event(DETECTOR_OK, 9.5)
    assert logic.on_resume_request(False, 9.6) == ([], GateReason.NAV_NOT_READY)
    assert logic.on_resume_request(True, 9.7)[1] == GateReason.OK


def test_resume_while_moving_or_dwelling_is_refused():
    assert patrolling().on_resume_request(True, 2.0) == ([], GateReason.NOT_PAUSED)
    assert dwelling().on_resume_request(True, 6.0) == ([], GateReason.NOT_PAUSED)


# ---- 일시정지 ---------------------------------------------------------------


def test_pause_on_the_way_then_resume_to_the_same_place():
    logic = patrolling()
    actions, reason = logic.on_pause_request(3.0)
    assert reason == GateReason.OK and logic.state == State.PATROL_PAUSED
    assert actions == [SetNavSpeedLimit(0.0), CancelNav(None), PatrolEvent("patrol_paused")]
    assert not says(actions)                                   # 순찰 중에는 안내 멘트가 없다
    assert logic.on_tick(60.0, NavStatus.NONE) == []           # 재개까지 그대로(카메라 감시도 쉼)
    logic.on_patrol_event(DETECTOR_OK, 61.0)
    actions, reason = logic.on_resume_request(True, 61.5)
    assert reason == GateReason.OK
    assert actions == [
        PatrolEvent("patrol_resumed"),
        PatrolEvent("patrol_waypoint", destination=A, index=0, lap=1, count=3),
        SetNavSpeedCap(0.6),
        Navigate(patrol_leg_destination(A, B)),
    ]


def test_pause_at_a_place_then_resume_to_the_next_place():
    logic = dwelling()
    actions, reason = logic.on_pause_request(6.0)
    assert (actions, reason) == ([PatrolEvent("patrol_paused")], GateReason.OK)
    logic.on_patrol_event(DETECTOR_OK, 30.0)
    assert navigates(logic.on_resume_request(True, 30.5)[0]) == [
        Navigate(patrol_leg_destination(B, C))]


def test_pause_during_arming_then_resume_starts_at_the_first_place():
    logic = started()
    assert logic.on_pause_request(2.0)[1] == GateReason.OK
    assert logic.on_tick(30.0, NavStatus.NONE) == []           # 준비 시한(15초)도 멈춘다
    assert logic.state == State.PATROL_PAUSED
    logic.on_patrol_event(DETECTOR_OK, 31.0)
    actions, reason = logic.on_resume_request(True, 31.5)
    assert reason == GateReason.OK
    assert navigates(actions) == [Navigate(patrol_leg_destination(A, B))]


def test_pause_is_refused_during_an_alarm_or_when_already_paused():
    logic = patrolling()
    logic.on_patrol_event(INTRUDER, 3.0)
    assert logic.on_pause_request(4.0) == ([], GateReason.NOT_NAVIGATING)
    assert logic.state == State.PATROL_ALARM
    paused = patrolling()
    paused.on_pause_request(3.0)
    assert paused.on_pause_request(4.0) == ([], GateReason.NOT_NAVIGATING)


# ---- 카메라 끊김 ------------------------------------------------------------


def test_camera_lost_after_three_seconds_without_a_good_status():
    logic = patrolling()                                       # 마지막 정상 신호 t=1
    assert tick(logic, 4.0, ok=False) == []                    # 3.0초 — 아직
    actions = tick(logic, 4.2, ok=False)
    assert logic.state == State.PATROL_ALARM
    assert actions == [
        SetNavSpeedLimit(0.0),
        CancelNav(None),
        PatrolEvent("patrol_camera_lost", alarm_id="20261114_203455_camera_lost",
                    kind="camera_lost", time="2026-11-14T20:34:55"),
    ]                                                          # 말은 하지 않는다


def test_a_not_ready_status_does_not_count_as_alive():
    logic = dwelling()                                         # 마지막 정상 신호 t=5
    for t in (6.0, 7.0, 8.0):
        logic.on_patrol_event({"src": "detector", "kind": "status", "ready": False,
                               "camera_ok": True}, t)
    actions = logic.on_tick(8.1, NavStatus.NONE)
    assert events(actions) == ["patrol_camera_lost"] and logic.state == State.PATROL_ALARM


def test_camera_lost_is_resent_but_never_spoken_and_blocks_resume():
    logic = patrolling()
    tick(logic, 4.2, ok=False)
    said, resent = _alarm_ticks(logic, 4.4, 25.0)
    assert said == [] and [r[1:] for r in resent] == [("patrol_camera_lost", True)] * 2
    assert logic.on_resume_request(True, 26.0) == ([], GateReason.DETECTOR_NOT_READY)
    logic.on_patrol_event(DETECTOR_OK, 27.0)                   # 카메라가 돌아왔다
    assert logic.on_resume_request(True, 27.5)[1] == GateReason.OK


# ---- 종료 -------------------------------------------------------------------


def with_home(logic: MissionLogic) -> MissionLogic:
    logic.return_destination = HOME
    return logic


def test_stop_on_the_way_ends_the_patrol_then_returns_home():
    """관리자가 경비를 종료하면 홈으로 돌아간다(2026-11-14 사용자 결정) — 속도 상한 해제·
    patrol_stopped 뒤에 앱 홈 복귀와 똑같은 행동이 이어진다. 구간 goal 은 따로 취소하지 않는다
    (복귀 goal 이 선점한다 — 취소를 먼저 보내면 Nav2 가 복귀 goal 까지 내린다, 11-14 실기)."""
    logic = with_home(patrolling())
    actions, reason = logic.on_patrol_stop(3.0)
    assert reason == GateReason.OK and logic.state == State.RETURNING
    assert logic.patrol_stop_home_reason == GateReason.OK and not logic.patrol_active
    assert actions == [
        SetNavSpeedLimit(0.0),
        PatrolEvent("patrol_stopped", reason="admin", return_home=True),
        SetNavSpeedLimit(0.0),
        Navigate(HOME),
    ]
    # 앱의 /vica/mission/return_home 과 같은 행동·같은 복귀 종류다.
    direct = with_home(new_logic())
    accepted, _, direct_actions = direct.on_return_home_request(True, 3.0)
    assert accepted and actions[2:] == direct_actions
    assert logic._returning_home is True and direct._returning_home is True
    # 도착하면 쉰다 — 순찰은 다시 돌지 않는다.
    assert logic.on_tick(10.0, NavStatus.RUNNING) == []
    logic.on_tick(20.0, NavStatus.SUCCEEDED)
    assert logic.state == State.IDLE and logic._patrol is None


def test_stop_without_a_home_ends_in_place():
    logic = patrolling()
    actions, reason = logic.on_patrol_stop(3.0)
    assert reason == GateReason.OK and logic.state == State.IDLE
    assert logic.patrol_stop_home_reason == GateReason.NO_HOME
    assert actions == [SetNavSpeedLimit(0.0), CancelNav(None),
                       PatrolEvent("patrol_stopped", reason="admin", return_home=False)]
    assert logic.on_tick(4.0, NavStatus.NONE) == []            # 그 자리에서 쉰다


def test_stop_when_nav2_is_not_ready_ends_in_place():
    logic = with_home(patrolling())
    actions, reason = logic.on_patrol_stop(3.0, nav_ready=False)
    assert reason == GateReason.OK and logic.state == State.IDLE
    assert logic.patrol_stop_home_reason == GateReason.NAV_NOT_READY
    assert events(actions) == ["patrol_stopped"] and not navigates(actions)
    assert [a.return_home for a in actions if isinstance(a, PatrolEvent)] == [False]


@pytest.mark.parametrize("name", ["patrol_dwell", "patrol_alarm", "patrol_light", "patrol_paused"])
def test_stop_while_standing_also_returns_home(name):
    logic = with_home(PATROL_STATE_MAKERS[name]())
    actions, reason = logic.on_patrol_stop(9.0)
    assert reason == GateReason.OK and logic.state == State.RETURNING
    silence = [StopSpeech()] if name in ("patrol_alarm", "patrol_light") else []   # 확인 전 경보 음성을 끊는다
    assert actions == silence + [PatrolEvent("patrol_stopped", reason="admin", return_home=True),
                                 SetNavSpeedLimit(0.0), Navigate(HOME)]


def test_stop_during_an_alarm_silences_it():
    """경보 음성 반복·이벤트 재전송은 종료와 함께 멈춘다."""
    logic = with_home(patrolling())
    logic.on_patrol_event(INTRUDER, 3.0)
    logic.on_patrol_stop(4.0)
    for t in (7.0, 14.0, 24.0, 40.0):
        assert logic.on_tick(t, NavStatus.RUNNING) == []
    assert logic.state == State.RETURNING


def test_endings_that_are_not_the_admins_do_not_return_home():
    """구간 3연속 실패·감지 노드 준비 시한·비상정지로 끝나면 홈으로 가지 않는다."""
    failed = with_home(patrolling())
    tick(failed, 4.0, NavStatus.FAILED)
    tick(failed, 5.0, NavStatus.FAILED)
    actions = tick(failed, 6.0, NavStatus.FAILED)
    assert actions[-1] == PatrolEvent("patrol_stopped", reason="nav_failed", return_home=False)
    assert failed.state == State.IDLE and not navigates(actions)

    not_ready = with_home(started())
    actions = not_ready.on_tick(15.0, NavStatus.NONE)
    assert actions == [PatrolEvent("patrol_stopped", reason="detector_not_ready", return_home=False)]
    assert not_ready.state == State.IDLE

    estopped = with_home(patrolling())
    actions = estopped.on_estop(True, 3.0)
    assert PatrolEvent("patrol_stopped", reason="estop", return_home=False) in actions
    assert estopped.state == State.ESTOPPED and not navigates(actions)


@pytest.mark.parametrize("make", [started, dwelling,
                                  lambda: (lambda l: l.on_patrol_event(LIGHT, 6.0) and l)(dwelling())])
def test_stop_while_standing_only_reports(make):
    logic = make()
    silence = [StopSpeech()] if logic.state == State.PATROL_LIGHT else []           # 확인 전 경보 음성을 끊는다
    assert logic.on_patrol_stop(9.0) == (
        silence + [PatrolEvent("patrol_stopped", reason="admin")], GateReason.OK)
    assert logic.state == State.IDLE


def test_start_rejection_lines_match_the_approved_mockup():
    """앱이 그대로 보여 주는 문구 — 목업과 글자까지 같다. 시작 거절에만 쓰고,
    다른 순찰 서비스의 비상정지 거절 문구는 그대로다."""
    from vica_mission_manager.mission_logic import PATROL_REJECT_TEXT, PATROL_START_REJECT_TEXT

    assert PATROL_START_REJECT_TEXT == {
        GateReason.ESTOP_ACTIVE: "비상정지 중에는 경비를 시작할 수 없습니다.",
        GateReason.NO_PATROL_ROUTE: "순찰 지점이 2곳 이상이어야 경비를 시작할 수 있습니다.",
    }
    assert PATROL_REJECT_TEXT[GateReason.ESTOP_ACTIVE] == "비상정지 중입니다. 해제한 뒤 다시 시도하세요."


def test_stop_when_not_patrolling_is_refused():
    assert new_logic().on_patrol_stop(1.0) == ([], GateReason.NOT_PATROLLING)


def test_app_cancel_during_patrol_ends_the_patrol_quietly():
    logic = patrolling()
    actions, reason = logic.on_app_cancel(3.0)
    assert reason == GateReason.OK and logic.state == State.IDLE
    assert events(actions) == ["patrol_stopped"] and not says(actions)


def test_app_cancel_during_patrol_also_returns_home():
    logic = with_home(patrolling())
    actions, reason = logic.on_app_cancel(3.0)
    assert reason == GateReason.OK and logic.state == State.RETURNING
    assert actions == [SetNavSpeedLimit(0.0),
                       PatrolEvent("patrol_stopped", reason="admin", return_home=True),
                       SetNavSpeedLimit(0.0), Navigate(HOME)]
    assert not says(actions)
    # Nav2 가 준비되지 않았으면 그 자리에서 끝낸다.
    blocked = with_home(patrolling())
    blocked.on_app_cancel(3.0, nav_ready=False)
    assert blocked.state == State.IDLE and blocked.patrol_stop_home_reason == GateReason.NAV_NOT_READY


# ---- 비상정지 ---------------------------------------------------------------


def test_estop_on_the_way_keeps_the_usual_handling_and_ends_the_patrol():
    logic = patrolling()
    actions = logic.on_estop(True, 3.0)
    assert logic.state == State.ESTOPPED and logic._patrol is None
    assert actions == [
        SetNavSpeedLimit(0.0),
        CancelNav(None),
        PatrolEvent("patrol_stopped", reason="estop"),
        Say(MSG_ESTOPPED, priority="emergency"),
    ]


def test_estop_while_standing_is_silent_but_still_ends_the_patrol():
    logic = dwelling()
    assert logic.on_estop(True, 6.0) == [PatrolEvent("patrol_stopped", reason="estop")]
    assert logic.state == State.ESTOPPED


def test_after_release_the_patrol_does_not_start_again():
    logic = patrolling()
    logic.on_estop(True, 3.0)
    logic.on_estop(False, 10.0)
    actions = logic.on_tick(12.0, NavStatus.NONE)
    assert logic.state == State.IDLE
    assert not navigates(actions) and not events(actions)
    assert logic.on_tick(60.0, NavStatus.NONE) == []
    assert logic.on_resume_request(True, 61.0)[1] != GateReason.OK


def test_emergency_word_and_tip_over_also_end_the_patrol():
    spoken = patrolling()
    assert events(spoken.on_emergency("멈춰", 3.0)) == ["patrol_stopped"]
    assert spoken.state == State.ESTOPPED

    tipped = patrolling()
    actions = tipped.on_tipped(True, 3.0)
    assert [a for a in actions if isinstance(a, PatrolEvent)] == [
        PatrolEvent("patrol_stopped", reason="estop")]
    assert says(actions) == [MSG_TIPPED] and tipped.state == State.ESTOPPED


def test_estop_during_an_alarm_stops_the_repeats():
    logic = patrolling()
    logic.on_patrol_event(INTRUDER, 3.0)
    assert logic.on_estop(True, 4.0) == [PatrolEvent("patrol_stopped", reason="estop")]
    assert logic.on_tick(20.0, NavStatus.NONE) == []


# ---- 순찰 중 받지 않는 것 ---------------------------------------------------

PATROL_STATE_MAKERS = {
    "patrolling": patrolling,
    "patrol_dwell": dwelling,
    "patrol_alarm": lambda: (lambda l: l.on_patrol_event(INTRUDER, 3.0) and l)(patrolling()),
    "patrol_light": lambda: (lambda l: l.on_patrol_event(LIGHT, 6.0) and l)(dwelling()),
    "patrol_paused": lambda: (lambda l: l.on_pause_request(3.0) and l)(patrolling()),
}


@pytest.fixture(params=sorted(PATROL_STATE_MAKERS))
def in_patrol(request):
    logic = PATROL_STATE_MAKERS[request.param]()
    assert logic.state.value == request.param
    return logic


def test_wake_word_is_ignored(in_patrol):
    state = in_patrol.state
    assert in_patrol.on_wake_call(7.0) == [WakeReply(listen=False)]
    assert in_patrol.on_wake(7.0) == []                       # 창 안에서 건진 호출도
    assert in_patrol.on_wake_doa(90.0, True, 7.0) == []
    assert in_patrol.state == state


@pytest.mark.parametrize("kind", ["navigate", "wait", "finish", "cancel", "pause", "resume",
                                  "affirm", "deny", "help", "question"])
def test_voice_requests_are_ignored(in_patrol, kind):
    state = in_patrol.state
    intent = IntentData(kind, "a" if kind == "navigate" else "", False, "normal")
    assert in_patrol.on_voice_intent(intent, 7.0, lambda _id: A, BOUNDS, True) == []
    assert in_patrol.state == state


def test_approach_and_destination_requests_are_refused(in_patrol):
    state = in_patrol.state
    request = ApproachRequest(goal=Pose2D(2.0, 1.0, 0.0, "map"), track_id=7)
    assert in_patrol.on_approach_request(request, BOUNDS, True, 7.0) == (
        [], GateReason.BUSY_NAVIGATING)
    assert in_patrol.on_app_destination(B, BOUNDS, True, 7.0) == ([], GateReason.BUSY_NAVIGATING)
    assert in_patrol.on_app_destination(B, BOUNDS, True, 7.0, allow_private=True,
                                        is_delivery=True) == ([], GateReason.BUSY_NAVIGATING)
    in_patrol.return_destination = HOME
    accepted, _, actions = in_patrol.on_return_home_request(True, 7.0)
    assert not accepted and actions == []
    assert in_patrol.state == state


# ---- 순찰이 아닐 때 ---------------------------------------------------------


def test_patrol_events_do_nothing_outside_a_patrol():
    logic = new_logic()
    for event in (DETECTOR_OK, INTRUDER, LIGHT, SOUND,
                  {"kind": "clip", "clip_id": "c", "alarm": "sound"}):
        assert logic.on_patrol_event(event, 1.0) == []
    assert logic.state == State.IDLE and logic._patrol is None
    assert logic.on_patrol_ack("", 1.0) == ([], GateReason.NO_PATROL_ALARM)


def test_a_stale_patrol_state_without_context_cancels_and_ends():
    """비상정지 스레드와 틱이 겹쳐 문맥만 사라진 경우 — 순찰 상태에 갇히지 않고, 가던 구간을
    거둔다(안 거두면 래치가 풀린 뒤 Nav2 가 그 지점으로 혼자 간다)."""
    logic = patrolling()
    logic._patrol = None
    assert logic.on_tick(3.0, NavStatus.RUNNING) == [SetNavSpeedLimit(0.0), CancelNav(None)]
    assert logic.state == State.IDLE

    standing = dwelling()
    standing._patrol = None
    assert standing.on_tick(6.0, NavStatus.NONE) == []          # 서 있었으면 거둘 것이 없다
    assert standing.state == State.IDLE


def test_a_stale_patrol_state_during_an_estop_goes_back_to_estopped():
    logic = patrolling()
    logic._patrol = None
    logic.estop_active = True                                   # 래치는 걸려 있다
    assert logic.on_tick(3.0, NavStatus.RUNNING) == [SetNavSpeedLimit(0.0), CancelNav(None)]
    assert logic.state == State.ESTOPPED
