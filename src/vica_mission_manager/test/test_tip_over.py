"""몸통 넘어짐(/vica/tipped) 멘트 규칙 — 2026-10-15 사용자 결정.

- 넘어지면 넘어짐 멘트 하나만 말한다("안전을 위해 멈추겠습니다…"는 내지 않는다).
  /vica/tipped 와 /emergency_stop 중 어느 쪽이 먼저 와도 한 번이다.
- 서 있다가 넘어져도 말한다(정지 중 침묵 규칙의 예외).
- 세울 때까지 10초마다 같은 멘트. 세우면 반복만 멈추고, 해제는 관리자 앱 reset 뒤
  "비상멈춤이 해제되었습니다." 한 번.
- 넘어진 동안 다른 비상정지 멘트가 연달아 붙지 않는다(사용자: "멘트가 중복해서 나오지
  않도록 주의").
- 관리자가 앱에서 직접 건 비상정지는 "관리자를 호출했습니다"를 뺀다(어긋남 3).
"""
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
    # 음성 assets/baked/manifest.json 의 mission_msg_tipped.wav 와 글자 하나까지 같아야
    # 구운 소리가 나간다(높이·빠르기를 손본 녹음이라 다시 굽지 않는다).
    assert MSG_TIPPED == "비카가 넘어져 관리자를 호출했습니다. 지나가실 때 주의하세요."
    assert MSG_ESTOPPED_BY_APP == "안전을 위해 멈추겠습니다."   # = replies.EMERGENCY_REPLY


def test_tip_while_driving_says_only_the_tip_ment():
    logic = MissionLogic()
    start_navigation(logic)
    actions = logic.on_tipped(True, 1.0)
    assert says(actions) == [MSG_TIPPED]
    tip = [a for a in actions if isinstance(a, Say)][0]
    assert tip.priority == "emergency"
    assert any(isinstance(a, CancelNav) for a in actions)
    assert logic.state == State.ESTOPPED
    # 래치가 뒤따라 와도 "안전을 위해 멈추겠습니다…"를 덧붙이지 않는다.
    assert says(logic.on_estop(True, 1.05)) == []


def test_latch_first_then_tip_still_one_tip_ment_and_no_cancel_twice():
    # 서 있을 때: 래치(통신 등)가 먼저 조용히 걸리고 넘어짐이 뒤에 와도 넘어짐 멘트 하나.
    logic = MissionLogic()
    assert says(logic.on_estop(True, 1.0)) == []
    assert says(logic.on_tipped(True, 1.05)) == [MSG_TIPPED]
    assert says(logic.on_estop(True, 1.1)) == []


def test_button_inside_the_tip_hold_is_cut_by_the_tip_ment():
    # 한계(on_tipped 문서): 넘어짐 1초 판정 전에 버튼으로 먼저 걸렸다. 그 멘트가 먼저
    # 나가고, 넘어짐 멘트가 emergency 로 끊고 들어간다. 넘어짐 멘트는 한 번이다.
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
        t = round(t + 0.2, 3)          # 미션 틱 5 Hz
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
    assert logic.state == State.ESTOPPED          # 래치가 걸려 있는 한 그대로


def test_release_after_admin_reset_is_announced_once():
    logic = MissionLogic(estop_release_grace_sec=1.0)
    start_navigation(logic)
    logic.on_tipped(True, 1.0)
    logic.on_estop(True, 1.05)
    logic.on_tipped(False, 8.0)                   # 세움
    logic.on_estop(False, 12.0)                   # 관리자 앱 reset
    assert tick(logic, 12.6) == []                # grace 1초 안
    assert tick(logic, 13.2) == [MSG_ESTOP_RELEASED]
    assert logic.state == State.IDLE
    assert tick(logic, 14.0) == []
    assert tick(logic, 30.0) == []


def test_never_leaves_estop_while_still_tipped():
    # 비상정지 노드가 tipped 를 모르는 옛 판이면 래치가 안 걸린다. 그래도 넘어진 채
    # "비상멈춤이 해제되었습니다."를 말하거나 IDLE 로 나가면 안 된다.
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
    # 이미 "멈춰"로 서 있던 로봇이 넘어졌다 — 넘어짐은 새 소식이라 말한다.
    logic = MissionLogic()
    assert says(logic.on_emergency("멈춰", 0.0)) == [MSG_ESTOP_WAKE]
    assert says(logic.on_tipped(True, 2.0)) == [MSG_TIPPED]


def test_self_wake_right_after_the_tip_ment_gets_no_reply():
    # 멘트가 "비카가…"로 시작한다. 제 목소리를 호출로 들어도 대답 멘트를 붙이지 않는다.
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
    # 10초가 된 순간에 넘어짐 멘트를 바로 붙이지 않는다 — 대답부터 다시 센다.
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


# ---- 앱 버튼(어긋남 3) -----------------------------------------------------


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
    # 10 Hz /app_emergency_stop 구독은 서 있을 때 CPU 를 약 3 %p 더 썼다(run96).
    from pathlib import Path
    import vica_mission_manager.mission_manager_node as node_mod
    src = Path(node_mod.__file__).read_text(encoding="utf-8")
    assert '"/app_estop_state"' in src
    assert '"/app_emergency_stop"' not in src
    # 같은 비상 콜백 그룹에서 /emergency_stop 보다 먼저 만든다.
    assert src.index('"/vica/tipped"') < src.index('"/app_estop_state"') < src.index('"/emergency_stop"')
