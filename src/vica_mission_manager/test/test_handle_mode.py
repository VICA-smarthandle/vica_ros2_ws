"""손잡이 터치 × 진동 — 잡기 대기·손 놓침 정지·자동 재출발 (2026-09-30).

정본: docs/superpowers/specs/2026-09-28-touch-haptic-integration-final.md 4절.
시계는 전부 주입한다. 센서 메시지는 드라이버처럼 0.1 s 간격으로 넣는다.
"""
import pytest

from vica_mission_manager.mission_logic import (
    DIALOG_GRIP_WAIT,
    DIALOG_PAUSED_HANDLE,
    GRIP_WAIT_TIMEOUT_SEC,
    HANDLE_LOST_GIVE_UP_SEC,
    HANDLE_LOST_REPEAT_SEC,
    MSG_APPROACH_ONBOARDING,
    MSG_APPROACH_ONBOARDING_SHORT,
    MSG_APPROACH_QUESTION,
    MSG_APPROACH_QUESTION_SHORT,
    MSG_CANCELED,
    MSG_HANDLE_HINT,
    MSG_HANDLE_LOST,
    MSG_HANDLE_UNAVAILABLE,
    MSG_PAUSED,
    USER_ATTACHED_SUPPRESS_SEC,
    CancelNav,
    Destination,
    GateReason,
    Haptic,
    IntentData,
    MapBounds,
    MissionLogic,
    Navigate,
    NavStatus,
    Pose2D,
    Say,
    State,
)

BOUNDS = MapBounds(min_x=-15.1, min_y=-8.59, max_x=10.0, max_y=8.0)
STEP = 0.1


def make_dest():
    return Destination(
        id="room_407", name="윤지영 교수님 사무실",
        pose=Pose2D(x=3.0, y=2.0, yaw_deg=90.0, frame_id="map"),
        is_approachable=True, calibrated=True,
        arrival_message="윤지영 교수님 사무실 앞에 도착했습니다.",
    )


def go_intent():
    return IntentData(intent="navigate", matched_destination_id="room_407",
                      need_confirm=False, safety_flag="normal")


def run(logic, t0, t1, contact, nav=NavStatus.NONE, fresh=True, nav_ready=True):
    """[t0, t1) 동안 0.1 s 마다 센서 한 건 + tick 한 번. 나온 동작을 모은다."""
    out = []
    t = t0
    while t < t1 - 1e-9:
        if contact is not None:
            logic.on_handle_state(contact, fresh, t)
        out.extend(logic.on_tick(t, nav, nav_ready=nav_ready))
        t = round(t + STEP, 6)
    return out


def says(actions):
    return [a.text for a in actions if isinstance(a, Say)]


def haptics(actions):
    return [a.pattern for a in actions if isinstance(a, Haptic)]


def accept(logic, t=2.0):
    """핸들 쪽 호출 → 수락 → 회전 없이 손잡이를 내준 순간(잡기 대기 시작)."""
    logic.on_wake_doa(175.0, True, t - 1.0)
    return logic.on_approach_answer(True, t)


def sensor_online(logic, t0=0.0, t1=1.0):
    run(logic, t0, t1, contact=False)


def engaged_logic(**kw):
    """잡기 대기를 통과한 사용자(활성 모드로 출발할 사람)."""
    logic = MissionLogic(wake_doa_sign=1.0, **kw)
    sensor_online(logic, 0.0, 2.0)
    accept(logic, 2.0)
    run(logic, 2.0, 4.5, contact=True)
    assert logic._handle_engaged
    return logic


def depart(logic, t):
    actions = logic.on_intent(go_intent(), make_dest(), BOUNDS, True, t)
    assert logic.state == State.NAVIGATING
    return actions


# ── 잡기 대기 (4.3 (가)) ─────────────────────────────────────────────────


class TestGripWait:
    def test_hint_and_vibration_open_the_wait_without_onboarding(self):
        logic = MissionLogic(wake_doa_sign=1.0)
        sensor_online(logic, 0.0, 2.0)
        actions = accept(logic, 2.0)
        assert haptics(actions) == ["long"]
        assert says(actions) == [MSG_HANDLE_HINT]
        assert logic.dialog_state == DIALOG_GRIP_WAIT
        assert logic.state == State.IDLE

    def test_two_seconds_of_grip_acks_and_onboards(self):
        logic = MissionLogic(wake_doa_sign=1.0)
        sensor_online(logic, 0.0, 2.0)
        accept(logic, 2.0)
        actions = run(logic, 2.0, 4.5, contact=True)
        assert "tick" in haptics(actions)
        assert says(actions) == [MSG_APPROACH_ONBOARDING]
        assert logic.dialog_state == "idle"
        # 잡음 확인은 2.0 s 를 채운 뒤다 — 대기 시작 전 접촉은 세지 않는다.
        assert logic._handle_engaged

    def test_grip_before_the_wait_does_not_count(self):
        """회전 중·대기 시작 전부터 쥐고 있어도 시작 뒤 2초는 지켜본다."""
        logic = MissionLogic(wake_doa_sign=1.0)
        run(logic, 0.0, 2.0, contact=True)
        accept(logic, 2.0)
        early = run(logic, 2.0, 3.5, contact=True)
        assert "tick" not in haptics(early)
        late = run(logic, 3.5, 4.5, contact=True)
        assert "tick" in haptics(late)

    def test_chattering_grip_at_80_percent_passes(self):
        logic = MissionLogic(wake_doa_sign=1.0)
        sensor_online(logic, 0.0, 2.0)
        accept(logic, 2.0)
        out = []
        t = 2.0
        for i in range(40):     # 0.1 s 단위로 5칸 중 1칸 놓음 = 80 %
            logic.on_handle_state(i % 5 != 4, True, t)
            out.extend(logic.on_tick(t, NavStatus.NONE))
            t = round(t + STEP, 6)
        out.extend(run(logic, t, t + 0.5, contact=True))
        assert "tick" in haptics(out)

    def test_half_grip_does_not_pass(self):
        logic = MissionLogic(wake_doa_sign=1.0)
        sensor_online(logic, 0.0, 2.0)
        accept(logic, 2.0)
        out = []
        t = 2.0
        for i in range(60):
            logic.on_handle_state(i % 2 == 0, True, t)
            out.extend(logic.on_tick(t, NavStatus.NONE))
            t = round(t + STEP, 6)
        assert "tick" not in haptics(out)
        assert logic.dialog_state == DIALOG_GRIP_WAIT

    def test_vibration_repeats_every_two_seconds(self):
        logic = MissionLogic(wake_doa_sign=1.0)
        sensor_online(logic, 0.0, 2.0)
        accept(logic, 2.0)
        out = run(logic, 2.0, 8.05, contact=False)
        assert haptics(out) == ["long", "long", "long"]      # 4, 6, 8 s

    def test_timeout_onboards_as_inactive(self):
        logic = MissionLogic(wake_doa_sign=1.0)
        sensor_online(logic, 0.0, 2.0)
        accept(logic, 2.0)
        out = run(logic, 2.0, 2.0 + GRIP_WAIT_TIMEOUT_SEC + 0.2, contact=False)
        assert says(out) == [MSG_APPROACH_ONBOARDING]
        assert "tick" not in haptics(out)
        assert not logic._handle_engaged
        assert logic.dialog_state == "idle"

    def test_no_sensor_onboards_at_once(self):
        """touch_enabled false·드라이버 없음: 기다릴 수단이 없다 — 09-11 흐름."""
        logic = MissionLogic(wake_doa_sign=1.0)
        actions = accept(logic, 2.0)
        assert says(actions) == [MSG_HANDLE_HINT, MSG_APPROACH_ONBOARDING]
        assert haptics(actions) == ["long"]
        assert logic.dialog_state == "idle"

    def test_sensor_drops_during_wait(self):
        logic = MissionLogic(wake_doa_sign=1.0)
        sensor_online(logic, 0.0, 2.0)
        accept(logic, 2.0)
        run(logic, 2.0, 2.5, contact=False)
        out = run(logic, 2.5, 4.0, contact=None)       # 메시지 끊김
        assert says(out) == [MSG_APPROACH_ONBOARDING]
        assert not logic._handle_engaged

    def test_destination_spoken_before_grip_ends_the_wait(self):
        logic = MissionLogic(wake_doa_sign=1.0)
        sensor_online(logic, 0.0, 2.0)
        accept(logic, 2.0)
        run(logic, 2.0, 3.0, contact=False)
        depart(logic, 3.0)
        assert logic.dialog_state == "navigating"
        assert not logic.handle_active       # 쥐고 있지 않았다
        out = run(logic, 3.0, 6.0, contact=False, nav=NavStatus.RUNNING)
        assert MSG_APPROACH_ONBOARDING not in says(out)

    def test_question_during_wait_keeps_waiting(self):
        logic = MissionLogic(wake_doa_sign=1.0)
        sensor_online(logic, 0.0, 2.0)
        accept(logic, 2.0)
        logic.on_intent(IntentData(intent="question", matched_destination_id="",
                                   need_confirm=False, safety_flag="normal"),
                        None, BOUNDS, True, 2.5)
        assert logic.dialog_state == DIALOG_GRIP_WAIT

    def test_estop_ends_the_wait(self):
        logic = MissionLogic(wake_doa_sign=1.0)
        sensor_online(logic, 0.0, 2.0)
        accept(logic, 2.0)
        logic.on_estop(True, 2.5)
        assert logic.state == State.ESTOPPED
        assert logic._grip_wait_since is None


# ── 출발 순간의 모드 (4.3 (나)) ──────────────────────────────────────────


class TestDepartureMode:
    def test_engaged_user_departs_active(self):
        logic = engaged_logic()
        depart(logic, 5.0)
        assert logic.handle_active

    def test_engaged_but_let_go_still_departs_active(self):
        """잠깐 손을 뗐어도 활성 — 0.5초 뒤 서서 다시 잡으라고 할 뿐 떠나지 않는다."""
        logic = engaged_logic()
        run(logic, 4.5, 5.0, contact=False)
        depart(logic, 5.0)
        assert logic.handle_active

    def test_unengaged_holding_departs_active(self):
        logic = MissionLogic()
        run(logic, 0.0, 1.0, contact=True)
        depart(logic, 1.0)
        assert logic.handle_active

    def test_unengaged_not_holding_departs_inactive(self):
        logic = MissionLogic()
        run(logic, 0.0, 1.0, contact=False)
        depart(logic, 1.0)
        assert not logic.handle_active

    def test_no_sensor_departs_inactive(self):
        logic = MissionLogic()
        depart(logic, 1.0)
        assert not logic.handle_active

    def test_engagement_expires_with_the_attached_guard(self):
        """떠난 뒤 다른 사람이 부른 안내에 '잡았던 사람' 표시가 번지지 않는다."""
        logic = engaged_logic()
        t = 4.5 + USER_ATTACHED_SUPPRESS_SEC + 1.0
        run(logic, t - 1.0, t, contact=False)
        depart(logic, t)
        assert not logic.handle_active

    def test_app_destination_is_always_inactive(self):
        logic = MissionLogic()
        run(logic, 0.0, 1.0, contact=True)
        _, reason = logic.on_app_destination(make_dest(), BOUNDS, True, 1.0)
        assert reason == GateReason.OK
        assert not logic.handle_active


# ── 손 놓침 정지 · 재출발 (4.3 (다)) ─────────────────────────────────────


def active_nav():
    logic = engaged_logic()
    depart(logic, 5.0)
    run(logic, 5.0, 6.0, contact=True, nav=NavStatus.RUNNING)
    return logic


class TestHandleRelease:
    def test_release_for_half_a_second_pauses(self):
        logic = active_nav()
        out = run(logic, 6.0, 6.8, contact=False, nav=NavStatus.RUNNING)
        assert logic.state == State.PAUSED
        assert logic.dialog_state == DIALOG_PAUSED_HANDLE
        assert says(out) == [MSG_HANDLE_LOST]
        assert haptics(out) == ["long"]
        assert any(isinstance(a, CancelNav) for a in out)
        assert MSG_PAUSED not in says(out)       # "말씀해 주세요" 가 섞이면 안 된다

    def test_regrip_within_grace_keeps_going(self):
        logic = active_nav()
        run(logic, 6.0, 6.3, contact=False, nav=NavStatus.RUNNING)
        run(logic, 6.3, 8.0, contact=True, nav=NavStatus.RUNNING)
        assert logic.state == State.NAVIGATING

    def test_regrip_resumes_with_tick(self):
        logic = active_nav()
        run(logic, 6.0, 7.0, contact=False, nav=NavStatus.RUNNING)
        assert logic.state == State.PAUSED
        out = run(logic, 7.0, 8.5, contact=True)
        assert logic.state == State.NAVIGATING
        assert logic.handle_active
        assert haptics(out)[0] == "tick"
        assert any(isinstance(a, Navigate) for a in out)
        assert any("다시 출발합니다" in s for s in says(out))

    def test_regrip_waits_for_nav_ready(self):
        logic = active_nav()
        run(logic, 6.0, 7.0, contact=False, nav=NavStatus.RUNNING)
        run(logic, 7.0, 8.5, contact=True, nav_ready=False)
        assert logic.state == State.PAUSED
        run(logic, 8.5, 8.7, contact=True, nav_ready=True)
        assert logic.state == State.NAVIGATING

    def test_lost_notice_repeats(self):
        logic = active_nav()
        run(logic, 6.0, 7.0, contact=False, nav=NavStatus.RUNNING)
        out = run(logic, 7.0, 7.0 + HANDLE_LOST_REPEAT_SEC, contact=False)
        assert says(out) == [MSG_HANDLE_LOST]
        assert haptics(out) == ["long"]

    def test_gives_up_after_three_minutes(self):
        logic = active_nav()
        run(logic, 6.0, 7.0, contact=False, nav=NavStatus.RUNNING)
        out = run(logic, 7.0, 7.0 + HANDLE_LOST_GIVE_UP_SEC, contact=False)
        assert logic.state == State.IDLE
        assert MSG_CANCELED in says(out)

    def test_voice_resume_without_grip_goes_inactive(self):
        logic = active_nav()
        run(logic, 6.0, 7.0, contact=False, nav=NavStatus.RUNNING)
        _, reason = logic.on_resume_request(True, 7.2)
        assert reason == GateReason.OK
        assert logic.state == State.NAVIGATING
        assert not logic.handle_active
        run(logic, 7.2, 9.0, contact=False, nav=NavStatus.RUNNING)
        assert logic.state == State.NAVIGATING     # 곧바로 다시 서지 않는다

    def test_voice_pause_while_holding_does_not_auto_resume(self):
        """쥔 채 "잠깐" — 말로 재개할 때까지 선다(설계 E 장면)."""
        logic = active_nav()
        _, reason = logic.on_pause_request(6.0)
        assert reason == GateReason.OK
        assert logic.dialog_state == "paused"
        run(logic, 6.0, 9.0, contact=True)
        assert logic.state == State.PAUSED

    def test_voice_pause_during_handle_pause_converts_it(self):
        logic = active_nav()
        run(logic, 6.0, 7.0, contact=False, nav=NavStatus.RUNNING)
        actions, reason = logic.on_pause_request(7.1)
        assert reason == GateReason.OK
        assert says(actions) == [MSG_PAUSED]
        run(logic, 7.1, 9.0, contact=True)
        assert logic.state == State.PAUSED

    def test_inactive_nav_ignores_release(self):
        logic = MissionLogic()
        run(logic, 0.0, 1.0, contact=False)
        depart(logic, 1.0)
        run(logic, 1.0, 5.0, contact=False, nav=NavStatus.RUNNING)
        assert logic.state == State.NAVIGATING

    def test_arrival_wins_over_release_on_the_same_tick(self):
        logic = active_nav()
        run(logic, 6.0, 6.5, contact=False, nav=NavStatus.RUNNING)
        assert logic.state == State.NAVIGATING   # 놓은 지 0.4 s — 아직 안 섬
        logic.on_handle_state(False, True, 6.5)
        out = logic.on_tick(6.5, NavStatus.SUCCEEDED)   # 0.5 s 째 tick 에 도착
        assert logic.state in (State.ARRIVED, State.ASKING_NEXT)
        assert not logic.handle_active
        assert MSG_HANDLE_LOST not in says(out)

    def test_estop_clears_handle_pause(self):
        logic = active_nav()
        run(logic, 6.0, 7.0, contact=False, nav=NavStatus.RUNNING)
        logic.on_estop(True, 7.1)
        assert logic.state == State.ESTOPPED
        assert not logic._handle_pause
        assert not logic.handle_active


# ── 상향 두절 (4.3 (라)) ─────────────────────────────────────────────────


class TestUplinkLoss:
    def test_loss_during_active_nav_goes_inactive_and_keeps_driving(self):
        logic = active_nav()
        out = run(logic, 6.0, 8.0, contact=None, nav=NavStatus.RUNNING)
        assert logic.state == State.NAVIGATING
        assert not logic.handle_active
        assert says(out) == [MSG_HANDLE_UNAVAILABLE]

    def test_unfresh_message_is_loss_not_release(self):
        logic = active_nav()
        out = run(logic, 6.0, 8.0, contact=False, fresh=False, nav=NavStatus.RUNNING)
        assert logic.state == State.NAVIGATING
        assert says(out) == [MSG_HANDLE_UNAVAILABLE]

    def test_loss_while_handle_paused_stays_put_and_asks_for_voice(self):
        logic = active_nav()
        run(logic, 6.0, 7.0, contact=False, nav=NavStatus.RUNNING)
        out = run(logic, 7.0, 9.0, contact=None)
        assert logic.state == State.PAUSED
        assert says(out) == [MSG_PAUSED]
        assert logic.dialog_state == "paused"
        assert not any(isinstance(a, Navigate) for a in out)


# ── 60초 자물쇠 보강 (4.4) · 상황판 (4.7) ─────────────────────────────────


class TestAttachedGuardAndDialogState:
    def test_contact_keeps_the_guard_after_sixty_seconds(self):
        logic = engaged_logic()
        t = 2.0 + USER_ATTACHED_SUPPRESS_SEC + 5.0
        run(logic, t - 1.0, t, contact=True)
        assert logic.user_attached_guard_active(t)
        run(logic, t, t + 1.0, contact=False)
        assert not logic.user_attached_guard_active(t + 1.0)

    def test_contact_without_a_session_does_not_arm_the_guard(self):
        logic = MissionLogic()
        run(logic, 0.0, 1.0, contact=True)
        assert not logic.user_attached_guard_active(1.0)

    @pytest.mark.parametrize("state", list(State))
    def test_dialog_state_defaults_to_state_value(self, state):
        logic = MissionLogic()
        logic.state = state
        assert logic.dialog_state == state.value


# ── 노드 배선 계약 ───────────────────────────────────────────────────────
from pathlib import Path  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
NODE = (PKG / "vica_mission_manager" / "mission_manager_node.py").read_text(encoding="utf-8")
LAUNCH = (PKG / "launch" / "mission_manager.launch.py").read_text(encoding="utf-8")


def test_node_feeds_handle_state_to_logic():
    assert 'SmartHandleState, "/vica/smart_handle_state"' in NODE
    assert "self.logic.on_handle_state(" in NODE
    assert "msg.user_contact, msg.uplink_fresh" in NODE


def test_node_publishes_dialog_state_not_raw_state():
    """LLM 상황판은 손잡이 두 단계를 볼 수 있어야 한다(설계 4.7)."""
    assert "self._ledger, self.logic.dialog_state," in NODE
    assert "self._ledger, self.logic.state.value," not in NODE


def test_old_hint_spoken_hook_is_gone():
    assert "on_handle_hint_spoken" not in NODE


def test_launch_exposes_the_two_field_tuned_values():
    for name in ("grip_release_grace_sec", "grip_wait_timeout_sec"):
        assert f'DeclareLaunchArgument("{name}"' in LAUNCH
        assert f'"{name}": ParameterValue(' in LAUNCH


# ── 짧은 원고 (2026-10-05 인수인계 "로컬 LLM 정비" 1번) ─────────────────────
class TestShortApproachMents:
    def test_default_keeps_long_ments(self):
        logic = MissionLogic(wake_doa_sign=1.0)
        assert logic.approach_question_msg == MSG_APPROACH_QUESTION
        assert logic.approach_onboarding_msg == MSG_APPROACH_ONBOARDING

    def test_short_question_on_handle_side_call(self):
        logic = MissionLogic(wake_doa_sign=1.0, short_approach_ments=True)
        actions = logic.on_wake_doa(175.0, True, 1.0)
        assert says(actions) == [MSG_APPROACH_QUESTION_SHORT]
        assert logic.state == State.AWAITING_USER

    def test_short_onboarding_after_grip(self):
        logic = MissionLogic(wake_doa_sign=1.0, short_approach_ments=True)
        sensor_online(logic, 0.0, 2.0)
        accept(logic, 2.0)
        actions = run(logic, 2.0, 4.5, contact=True)
        assert says(actions) == [MSG_APPROACH_ONBOARDING_SHORT]

    def test_short_texts_are_exact(self):
        """녹음은 글자 하나까지 같아야 나온다 — 인수인계 문구 그대로인지 고정."""
        assert MSG_APPROACH_QUESTION_SHORT == "안녕하세요? 시각장애인 안내로봇 비카입니다. 안내를 받으시겠어요?"
        assert MSG_APPROACH_ONBOARDING_SHORT == (
            "저에게 말을 거실 때는 '비카야'라고 불러주세요. 어디로 가고 싶으신가요?")
