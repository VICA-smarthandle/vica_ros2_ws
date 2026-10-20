"""몸통 넘어짐(/vica/tipped) 멘트 규칙 — 2026-10-15 사용자 결정."""
from vica_mission_manager.mission_logic import (
    MSG_ESTOP_RELEASED,
    MSG_ESTOP_REJECT,
    MSG_ESTOP_WAKE,
    MSG_ESTOPPED,
    MSG_ESTOPPED_BY_APP,
    MSG_TIPPED,
    TIP_REPEAT_SEC,
    TIP_SELF_WAKE_GUARD_SEC,
    CancelNav,
    Destination,
    IntentData,
    MapBounds,
    MissionLogic,
    NavStatus,
    Pose2D,
    Say,
    State,
    WakeReply,
)

BOUNDS = MapBounds(min_x=-15.1, min_y=-8.59, max_x=10.0, max_y=8.0)


def make_dest():
    return Destination(
        id="room_407",
        name="윤지영 교수님 사무실",
        pose=Pose2D(x=3.0, y=2.0, yaw_deg=90.0, frame_id="map"),
        is_approachable=True,
        calibrated=True,
        arrival_message="윤지영 교수님 사무실 앞에 도착했습니다.",
    )


def make_intent(**kw):
    defaults = dict(
        intent="navigate",
        matched_destination_id="room_407",
        need_confirm=False,
        safety_flag="normal",
    )
    defaults.update(kw)
    return IntentData(**defaults)


def start_navigation(logic, t=0.0):
    logic.on_intent(make_intent(), make_dest(), BOUNDS, True, t)
    assert logic.state == State.NAVIGATING


def says(actions):
    return [a.text for a in actions if isinstance(a, Say)]


def tick(logic, t):
    return says(logic.on_tick(t, NavStatus.NONE))


def test_baked_text_is_unchanged():
    assert MSG_TIPPED == "비카가 넘어져 관리자를 호출했습니다. 지나가실 때 주의하세요."
    assert MSG_ESTOPPED_BY_APP == "안전을 위해 멈추겠습니다."


def test_tip_while_driving_says_only_the_tip_ment():
    logic = MissionLogic()
    start_navigation(logic)
    actions = logic.on_tipped(True, 1.0)
    assert says(actions) == [MSG_TIPPED]
    tip = [a for a in actions if isinstance(a, Say)][0]
    assert tip.priority == "emergency"
    assert any(isinstance(a, CancelNav) for a in actions)
    assert logic.state == State.ESTOPPED
    assert says(logic.on_estop(True, 1.05)) == []


def test_latch_first_then_tip_still_one_tip_ment_and_no_cancel_twice():
    logic = MissionLogic()
    assert says(logic.on_estop(True, 1.0)) == []
    assert says(logic.on_tipped(True, 1.05)) == [MSG_TIPPED]
    assert says(logic.on_estop(True, 1.1)) == []


def test_button_inside_the_tip_hold_is_cut_by_the_tip_ment():
    logic = MissionLogic()
    start_navigation(logic)
    assert says(logic.on_estop(True, 1.0)) == [MSG_ESTOPPED]
    actions = logic.on_tipped(True, 1.6)
    assert says(actions) == [MSG_TIPPED]
    assert [a.priority for a in actions if isinstance(a, Say)] == ["emergency"]
    assert says(logic.on_estop(True, 1.65)) == []


def test_tip_while_standing_still_speaks():
    logic = MissionLogic()
    assert says(logic.on_tipped(True, 1.0)) == [MSG_TIPPED]
    assert logic.state == State.ESTOPPED


def test_repeated_true_is_spoken_once():
    logic = MissionLogic()
    logic.on_tipped(True, 1.0)
    assert says(logic.on_tipped(True, 1.5)) == []
    assert says(logic.on_tipped(True, 3.0)) == []


def test_repeats_every_ten_seconds_until_stood_up():
    logic = MissionLogic()
    logic.on_tipped(True, 0.0)
    logic.on_estop(True, 0.05)
    spoken = []
    t = 0.0
    while t < 35.0:
        t = round(t + 0.2, 3)
        if tick(logic, t):
            spoken.append(t)
    assert spoken == [10.0, 20.0, 30.0]
    assert TIP_REPEAT_SEC == 10.0


def test_standing_up_stops_the_repeat_and_keeps_the_latch():
    logic = MissionLogic(estop_release_grace_sec=1.0)
    logic.on_tipped(True, 0.0)
    logic.on_estop(True, 0.05)
    assert says(logic.on_tipped(False, 5.0)) == []
    for t in (10.0, 20.0, 30.0):
        assert tick(logic, t) == []
    assert logic.state == State.ESTOPPED


def test_release_after_admin_reset_is_announced_once():
    logic = MissionLogic(estop_release_grace_sec=1.0)
    start_navigation(logic)
    logic.on_tipped(True, 1.0)
    logic.on_estop(True, 1.05)
    logic.on_tipped(False, 8.0)
    logic.on_estop(False, 12.0)
    assert tick(logic, 12.6) == []
    assert tick(logic, 13.2) == [MSG_ESTOP_RELEASED]
    assert logic.state == State.IDLE
    assert tick(logic, 14.0) == []
    assert tick(logic, 30.0) == []


def test_never_leaves_estop_while_still_tipped():
    logic = MissionLogic(estop_release_grace_sec=1.0)
    logic.on_tipped(True, 0.0)
    logic.on_estop(False, 0.05)
    spoken = []
    for i in range(1, 50):
        spoken += tick(logic, i * 0.2)
    assert MSG_ESTOP_RELEASED not in spoken
    assert logic.state == State.ESTOPPED


def test_voice_stop_while_tipped_is_silent():
    logic = MissionLogic()
    logic.on_tipped(True, 0.0)
    logic.on_estop(True, 0.05)
    assert says(logic.on_emergency("멈춰", 3.0)) == []


def test_tip_after_voice_stop_is_new_news():
    logic = MissionLogic()
    assert says(logic.on_emergency("멈춰", 0.0)) == [MSG_ESTOP_WAKE]
    assert says(logic.on_tipped(True, 2.0)) == [MSG_TIPPED]


def test_self_wake_right_after_the_tip_ment_gets_no_reply():
    logic = MissionLogic()
    logic.on_tipped(True, 0.0)
    actions = logic.on_wake_call(3.0)
    assert says(actions) == []
    assert any(isinstance(a, WakeReply) and not a.listen for a in actions)
    assert TIP_SELF_WAKE_GUARD_SEC < TIP_REPEAT_SEC


def test_real_call_while_tipped_is_answered_and_delays_the_repeat():
    logic = MissionLogic()
    logic.on_tipped(True, 0.0)
    logic.on_estop(True, 0.05)
    assert says(logic.on_wake_call(8.0)) == [MSG_ESTOP_WAKE]
    assert tick(logic, 10.0) == []
    assert tick(logic, 17.8) == []
    assert tick(logic, 18.0) == [MSG_TIPPED]


def test_destination_while_tipped_is_rejected_and_delays_the_repeat():
    logic = MissionLogic()
    logic.on_tipped(True, 0.0)
    logic.on_estop(True, 0.05)
    actions = logic.on_intent(make_intent(), make_dest(), BOUNDS, True, 9.0)
    assert says(actions) == [MSG_ESTOP_REJECT]
    assert tick(logic, 10.0) == []
    assert tick(logic, 19.0) == [MSG_TIPPED]


def test_tipping_again_after_standing_up_speaks_again():
    logic = MissionLogic()
    logic.on_tipped(True, 0.0)
    logic.on_estop(True, 0.05)
    logic.on_tipped(False, 5.0)
    assert says(logic.on_tipped(True, 6.0)) == [MSG_TIPPED]
    assert tick(logic, 15.8) == []
    assert tick(logic, 16.0) == [MSG_TIPPED]


def test_not_tipped_false_at_start_is_a_no_op():
    logic = MissionLogic()
    assert logic.on_tipped(False, 0.0) == []
    assert logic.state == State.IDLE


def test_app_estop_while_driving_does_not_say_admin_was_called():
    logic = MissionLogic()
    start_navigation(logic)
    logic.on_app_estop(True)
    assert says(logic.on_estop(True, 1.0)) == [MSG_ESTOPPED_BY_APP]


def test_app_estop_release_is_still_announced():
    logic = MissionLogic(estop_release_grace_sec=1.0)
    start_navigation(logic)
    logic.on_app_estop(True)
    logic.on_estop(True, 1.0)
    logic.on_app_estop(False)
    logic.on_estop(False, 5.0)
    assert tick(logic, 6.2) == [MSG_ESTOP_RELEASED]


def test_app_estop_while_standing_stays_silent():
    logic = MissionLogic()
    logic.on_app_estop(True)
    assert says(logic.on_estop(True, 1.0)) == []


def test_button_while_driving_keeps_the_full_ment():
    logic = MissionLogic()
    start_navigation(logic)
    logic.on_app_estop(False)
    assert says(logic.on_estop(True, 1.0)) == [MSG_ESTOPPED]


def test_mission_takes_the_app_cause_from_the_1hz_state_topic():
    from pathlib import Path
    import vica_mission_manager.mission_manager_node as node_mod
    src = Path(node_mod.__file__).read_text(encoding="utf-8")
    assert '"/app_estop_state"' in src
    assert '"/app_emergency_stop"' not in src
    assert src.index('"/vica/tipped"') < src.index('"/app_estop_state"') < src.index('"/emergency_stop"')
