"""'비카야' 호출 반응표 (2026-10-07 사용자 결정, 인수인계 문서 작업 계획 탭).

예전에는 음성 쪽이 상태와 상관없이 늘 "말 멈추기 → '네?' → 듣기"를 했다. 이제
미션이 상태를 보고 정한다(on_wake_call): 대답하는 상태는 끊기·"네?"·listen, 대답하지
않는 상태는 ignore 만, 비상 정지는 한 마디와 ignore. 음성 쪽은 호출 즉시 듣기 창을
열어 두고 WakeReply 가 올 때까지 들은 말을 쥐고 있다.
"""
from pathlib import Path

import pytest

from reaction_states import (ROOM, TOILET, asking, awaiting_user, confirming, confirming_change, intent, lookup,
                             navigating_cancel_asked, navigating_help_asked, waiting)
from vica_mission_manager.mission_logic import (
    MSG_CANCEL_CONFIRM,
    MSG_CANCELED,
    MSG_CONFIRM_PROMPT_FALLBACK,
    MSG_ESTOP_REJECT,
    MSG_ESTOP_WAKE,
    MSG_HELP_CONFIRM,
    MSG_HELP_SENT,
    MSG_LEAVING_NOTICE,
    MSG_REASK_LOUDER,
    MSG_REASK_RETRY,
    MSG_WAKE_GREETING,
    WAKE_REVIVE_SEC,
    Destination,
    IntentData,
    MapBounds,
    MissionLogic,
    Navigate,
    NavStatus,
    Pose2D,
    Say,
    State,
    StopSpeech,
    WakeReply,
    say_destination,
)

BOUNDS = MapBounds(min_x=-50, min_y=-50, max_x=50, max_y=50)

# 작업 계획 탭 '"비카야" 반응표' 그대로. 상태 이름 정본은 State 다.
LISTEN_STATES = (
    State.IDLE, State.CONFIRMING, State.PAUSED,
    State.NAVIGATING, State.ASKING_NEXT, State.ASKING_WAIT_TIME,
    State.WAITING_RELEASE, State.WAITING, State.ARRIVED, State.SEEKING,
    State.RETURNING,
)
# 접근 질문의 답을 기다리는 동안은 "비카야"를 무시한다(2026-10-09 사용자 결정) — 말을 끊지 않고 "네?"도
# 없이, 들은 말은 그 질문의 답으로 LLM 에 넘긴다(listen). 질문도 접지 않는다.
APPROACH_QUESTION_STATES = (State.AWAITING_USER,)
IGNORE_STATES = (
    State.APPROACHING, State.TURNING, State.FAILED,
    State.MOVING_TO_WAIT_SPOT, State.MOVING_BACK_TO_DEST,
    # 경비(순찰) 중에는 사용자 응대가 없다(2026-11-14).
    State.PATROLLING, State.PATROL_DWELL, State.PATROL_ALARM, State.PATROL_LIGHT,
    State.PATROL_PAUSED,
)


def _replies(actions):
    return [a for a in actions if isinstance(a, WakeReply)]


def _say(actions):
    return [a.text for a in actions if isinstance(a, Say)]


def _dest(**kw):
    d = dict(id="d1", name="화장실", pose=Pose2D(x=3, y=2, yaw_deg=90, frame_id="map"),
             calibrated=True)
    d.update(kw)
    return Destination(**d)


def _intent(**kw):
    d = dict(intent="navigate", matched_destination_id="d1", need_confirm=False,
             safety_flag="normal")
    d.update(kw)
    return IntentData(**d)


def test_table_covers_every_state():
    """새 상태가 생기면 반응표에서 빠지지 않게 — 비상 정지까지 셋이 모든 상태를 덮는다."""
    covered = set(LISTEN_STATES) | set(IGNORE_STATES) | set(APPROACH_QUESTION_STATES) | {State.ESTOPPED}
    assert covered == set(State)
    assert not set(LISTEN_STATES) & set(IGNORE_STATES)
    assert not set(APPROACH_QUESTION_STATES) & (set(LISTEN_STATES) | set(IGNORE_STATES))


@pytest.mark.parametrize("state", APPROACH_QUESTION_STATES)
def test_approach_question_ignores_the_call_but_keeps_listening(state):
    logic = MissionLogic()
    logic.state = state
    assert logic.on_wake_call(1.0) == [WakeReply(listen=True)]
    assert logic.state == state


@pytest.mark.parametrize("state", LISTEN_STATES)
def test_listen_states_greet_and_listen(state):
    logic = MissionLogic()
    logic.state = state
    if state == State.WAITING:
        # 대기 중 첫 호출은 자리 알림만 한다(2026-10-21, test_wait_spot) — 여기서는 그 뒤의 호출이다.
        logic._wait_located = True
    actions = logic.on_wake_call(1.0)
    assert _replies(actions) == [WakeReply(listen=True)]
    # 하던 말을 끊고 그 뒤에 "네?" — 같은 토픽 순서(09-01 순서 뒤집힘 사고).
    assert isinstance(actions[0], StopSpeech)
    greet = actions[1]
    assert isinstance(greet, Say) and greet.text == MSG_WAKE_GREETING
    assert greet.priority == "response"
    # 질문 예약을 걸지 않는다 — 창은 이미 열려 있고, 예약 창이 호출 창을 갈아치운다.
    assert greet.expects_reply is False
    assert _say(actions).count(MSG_WAKE_GREETING) == 1


@pytest.mark.parametrize("state", IGNORE_STATES)
def test_ignore_states_stay_silent(state):
    logic = MissionLogic()
    logic.state = state
    actions = logic.on_wake_call(1.0)
    assert actions == [WakeReply(listen=False)]
    assert logic.state == state


def test_estop_says_one_line_and_ignores():
    logic = MissionLogic()
    logic.state = State.ESTOPPED
    actions = logic.on_wake_call(1.0)
    assert actions == [Say(MSG_ESTOP_WAKE, priority="response"), WakeReply(listen=False)]


def test_estop_flag_wins_over_state():
    """래치가 걸린 직후 상태가 아직 안 바뀐 순간에도 비상 정지로 답한다."""
    logic = MissionLogic()
    logic.estop_active = True
    assert _replies(logic.on_wake_call(1.0)) == [WakeReply(listen=False)]


def test_estop_line_is_the_head_of_the_reject_message():
    assert MSG_ESTOP_REJECT.startswith(MSG_ESTOP_WAKE)
    assert MSG_WAKE_GREETING == "네?"


def test_waiting_keeps_wait_and_listens():
    """대기 중 호출 — 대기를 끝내지 않고 듣는다(남은 시간 그대로)."""
    logic = MissionLogic(arrival_dialog=True)
    logic.on_intent(_intent(), _dest(category="restroom"), BOUNDS, True, 0.0)
    logic.on_tick(1.0, NavStatus.SUCCEEDED)
    logic.on_arrival_question_spoken(2.0)
    logic.on_arrival_answer(IntentData(intent="affirm", matched_destination_id="",
                                       need_confirm=False, safety_flag="normal"), 3.0)
    assert logic.state == State.WAITING
    left = logic.wait_left_sec(10.0)
    # 첫 호출은 자리만 알리고 듣지 않는다(2026-10-21) — 두 번째부터 듣는다. 어느 쪽이든 대기는 그대로.
    assert _replies(logic.on_wake_call(10.0)) == [WakeReply(listen=False)]
    actions = logic.on_wake_call(10.0)
    assert _replies(actions) == [WakeReply(listen=True)]
    assert logic.state == State.WAITING
    assert logic.wait_left_sec(10.0) == left


def test_confirming_question_is_folded():
    logic = MissionLogic()
    logic.on_intent(_intent(need_confirm=True), _dest(), BOUNDS, True, 0.0)
    assert logic.state == State.CONFIRMING
    actions = logic.on_wake_call(1.0)
    assert logic.state == State.IDLE
    assert _replies(actions) == [WakeReply(listen=True)]


def test_rescue_path_does_state_work_without_greeting():
    """창 안에서 소리로 건진 호출 — 노드는 on_wake 만 부른다. 인사·판정이 없다."""
    logic = MissionLogic()
    logic.on_intent(_intent(need_confirm=True), _dest(), BOUNDS, True, 0.0)
    actions = logic.on_wake(1.0)
    assert logic.state == State.IDLE
    assert not _replies(actions) and not _say(actions)


class TestReviewFixes:
    """2026-10-07 독립 검토가 찾은 것들."""

    def test_verdict_goes_right_after_the_greeting(self):
        """판정이 상태 정리(재출발 등)보다 먼저 음성 쪽에 닿는다."""
        logic = MissionLogic()
        logic.on_intent(_intent(need_confirm=True), _dest(), BOUNDS, True, 0.0)
        actions = logic.on_wake_call(1.0)
        assert isinstance(actions[0], StopSpeech)
        assert isinstance(actions[1], Say) and actions[1].text == MSG_WAKE_GREETING
        assert actions[2] == WakeReply(listen=True)

    def test_approach_question_opened_by_the_same_call_is_not_cut(self):
        """호출 방향(wake_doa)이 먼저 와 접근 질문을 막 시작했다 — 그 질문이 대답이다."""
        logic = MissionLogic(wake_doa_sign=1.0)
        logic.on_wake_doa(175.0, True, 1.0)              # 손잡이 쪽 — 돌지 않고 곧장 질문
        assert logic.state == State.AWAITING_USER
        assert logic.on_wake_call(1.01) == [WakeReply(listen=True)]
        assert logic.state == State.AWAITING_USER

    def test_ignored_call_during_estop_does_not_advance_the_onboarding_ladder(self):
        logic = MissionLogic(wake_doa_sign=1.0)
        logic.on_wake_doa(175.0, True, 1.0)
        logic.on_approach_answer(True, 2.0)               # 온보딩 — 목적지 되묻기 사다리 시작
        assert logic._dest_prompt_stage == "asked"
        logic.on_estop(True, 3.0)
        assert _say(logic.on_wake_call(4.0)) == [MSG_ESTOP_WAKE]
        # 음성 쪽이 무시하며 창을 닫는다 — "잘 듣지 못했습니다…"가 나오면 안 된다.
        assert logic.on_listen_state("empty:wake-ignored", 4.1) == []
        assert logic._dest_prompt_stage == "asked"


# ---- "비카야" 뒤 처리 — 파라미터 wake_revive 가 켜졌을 때만(원칙 13 · 3, Task 3.9) ------------------------------
# 호출은 되돌리기 어려운 질문(확인 질문 · "안내를 취소할까요?" · "직원의 도움이 필요하신가요?")을 조용히 접고 하나를
# 적어 둔다. 40초 안에 승낙이 오면 실행하지 않고 그 질문을 다시 한다. 호출어만 들린 말("네?")은 되묻기 횟수에 세지 않는다.
# 꺼져 있으면(기본값) 지금 그대로다.
PKG = Path(__file__).resolve().parents[1]
TOILET_ASK = say_destination(MSG_CONFIRM_PROMPT_FALLBACK, "화장실")
WAKE_ONLY = intent("unknown", ask_text=MSG_WAKE_GREETING, ask_kind="open")
GREET = [StopSpeech(), Say(MSG_WAKE_GREETING, priority="response", kind="open"), WakeReply(listen=True)]


def _says(actions):
    return [a.text for a in actions if isinstance(a, Say)]


def _voice(logic, kind, t, **kw):
    return logic.on_voice_intent(intent(kind, **kw), t, lookup, BOUNDS, True)


# 설명서 Step 1 의 시험
def test_p13_confirm_folded_by_wake_then_yes_asks_again_and_does_not_depart():
    logic, t = confirming(wake_revive=True)
    logic.on_wake_call(t)
    acts = logic.on_voice_intent(intent("affirm"), t + 3, lookup, BOUNDS, True)
    assert _says(acts) == [say_destination(MSG_CONFIRM_PROMPT_FALLBACK, "화장실")]
    assert logic.state == State.CONFIRMING and not any(isinstance(a, Navigate) for a in acts)


def test_p13_cancel_question_folded_by_wake_then_yes_asks_again():
    logic, t = navigating_cancel_asked(wake_revive=True)
    logic.on_wake_call(t)
    assert _says(logic.on_voice_intent(intent("affirm"), t + 3, lookup, BOUNDS, True)) == [MSG_CANCEL_CONFIRM]
    assert logic.state == State.NAVIGATING


def test_p13_help_question_folded_by_wake_then_yes_asks_again():
    logic, t = navigating_help_asked(wake_revive=True)
    logic.on_wake_call(t)
    assert _says(logic.on_voice_intent(intent("affirm"), t + 3, lookup, BOUNDS, True)) == [MSG_HELP_CONFIRM]
    assert logic.help_phase == "asking"


def test_p13_revive_expires_and_other_words_clear_it():
    logic, t = confirming(wake_revive=True)
    logic.on_wake_call(t)
    assert _says(logic.on_voice_intent(intent("affirm"), t + 41, lookup, BOUNDS, True)) == []
    logic, t = confirming(wake_revive=True)
    logic.on_wake_call(t)
    logic.on_voice_intent(intent("question"), t + 2, lookup, BOUNDS, True)
    assert _says(logic.on_voice_intent(intent("affirm"), t + 4, lookup, BOUNDS, True)) == []


def test_p13_wake_only_is_not_counted_in_arrival_question():
    logic, t = asking(wake_revive=True)
    wake_only = intent("unknown", ask_text=MSG_WAKE_GREETING, ask_kind="open")
    for i in range(3):
        acts = logic.on_voice_intent(wake_only, t + i, lookup, BOUNDS, True)
        assert MSG_LEAVING_NOTICE not in _says(acts) and logic.state == State.ASKING_NEXT


def test_off_keeps_todays_wake_behaviour():
    logic, t = navigating_cancel_asked()
    logic.on_wake_call(t)
    assert logic.cancel_confirm_pending is True


# 설명서 밖 — 꺼짐 = 지금 그대로
def test_p13_revive_is_off_by_default_and_the_window_is_40s():
    assert MissionLogic().wake_revive is False
    assert WAKE_REVIVE_SEC == 40.0


def test_off_wake_keeps_todays_actions_and_leaves_nothing_folded():
    """꺼짐: 호출이 돌려주는 동작이 지금과 같다(확인 질문은 접고, 취소 확인·도움 확인은 살아 있다). _wake_folded 는 늘 None."""
    logic, t = confirming()
    assert logic.on_wake_call(t) == GREET and logic.state == State.IDLE and logic._wake_folded is None
    logic, t = confirming_change()
    acts = logic.on_wake_call(t)
    assert acts[:3] == GREET and [a.destination for a in acts if isinstance(a, Navigate)] == [ROOM]
    assert logic.state == State.NAVIGATING and logic._wake_folded is None
    logic, t = navigating_cancel_asked()
    assert logic.on_wake_call(t) == GREET and logic.cancel_confirm_pending and logic._wake_folded is None
    logic, t = navigating_help_asked()
    assert logic.on_wake_call(t) == GREET and logic.help_phase == "asking" and logic._wake_folded is None


def test_off_yes_after_the_call_still_runs_the_cancel_or_help_today():
    """꺼짐: 지금은 "네?" 뒤의 "응" 하나로 취소·도움 요청이 실행된다 — 켜짐이 고치는 것(원칙 13)."""
    logic, t = navigating_cancel_asked()
    logic.on_wake_call(t)
    assert MSG_CANCELED in _says(_voice(logic, "affirm", t + 3)) and logic.state == State.IDLE
    logic, t = navigating_help_asked()
    logic.on_wake_call(t)
    assert MSG_HELP_SENT in _says(_voice(logic, "affirm", t + 3)) and logic.state == State.PAUSED


@pytest.mark.parametrize("make", [confirming, confirming_change, navigating_cancel_asked, navigating_help_asked])
def test_p13_folding_is_silent(make):
    """켜짐도 호출이 돌려주는 동작은 꺼짐과 같다 — 접는 말은 없다(2026-09-01)."""
    off, t = make()
    on, _ = make(wake_revive=True)
    assert on.on_wake_call(t) == off.on_wake_call(t)


def test_p13_cancel_and_help_questions_are_folded_and_remembered():
    logic, t = navigating_cancel_asked(wake_revive=True)
    logic.on_wake_call(t)
    assert logic.cancel_confirm_pending is False and logic._wake_folded == ("cancel", "", t)
    assert logic.state == State.NAVIGATING
    logic, t = navigating_help_asked(wake_revive=True)
    logic.on_wake_call(t)
    assert logic.help_phase == "none" and logic._wake_folded == ("help", "", t)
    assert logic.state == State.NAVIGATING


def test_p13_rescue_call_folds_and_remembers_too():
    """창 안에서 건진 호출 — 노드는 on_wake 만 부른다. 거기서도 접고 적는다."""
    logic, t = navigating_help_asked(wake_revive=True)
    assert logic.on_wake(t) == []
    assert logic.help_phase == "none" and logic._wake_folded == ("help", "", t)


def test_p13_change_question_folded_by_wake_then_yes_asks_the_change_again():
    """주행 중 바꾸기 질문 — 호출에 지금처럼 원래 목적지로 다시 가고, "응"이면 바꾸기 질문을 다시 한다(출발·바꾸기 없음)."""
    logic, t = confirming_change(wake_revive=True)
    logic.on_wake_call(t)
    assert logic.state == State.NAVIGATING and logic.active_destination == ROOM
    assert logic._wake_folded == ("confirm", TOILET.id, t)
    acts = _voice(logic, "affirm", t + 3)
    assert _says(acts) == [TOILET_ASK]
    assert not any(isinstance(a, Navigate) for a in acts)
    assert logic.state == State.CONFIRMING and logic._change_from == ROOM and logic._wake_folded is None


def test_p13_two_open_questions_fold_together_and_the_confirm_is_remembered():
    """바꾸기 질문 + "안내를 취소할까요?"가 함께 열려 있으면 둘 다 접고 확인 질문만 적는다(확인 > 취소 > 도움)."""
    logic, t = confirming_change(wake_revive=True)
    assert _says(_voice(logic, "finish", t)) == [MSG_CANCEL_CONFIRM]
    assert logic.state == State.CONFIRMING and logic.cancel_confirm_pending
    logic.on_wake_call(t + 1)
    assert logic.cancel_confirm_pending is False and logic._wake_folded == ("confirm", TOILET.id, t + 1)
    assert _says(_voice(logic, "affirm", t + 2)) == [TOILET_ASK]


def test_p13_no_after_the_fold_revives_nothing():
    logic, t = confirming(wake_revive=True)
    logic.on_wake_call(t)
    assert _says(_voice(logic, "deny", t + 2)) == [] and logic.state == State.IDLE
    assert logic._wake_folded is None
    assert _says(_voice(logic, "affirm", t + 4)) == [] and logic.state == State.IDLE
    logic, t = navigating_cancel_asked(wake_revive=True)
    logic.on_wake_call(t)
    assert _says(_voice(logic, "deny", t + 2)) == []
    assert logic.state == State.NAVIGATING and not logic.cancel_confirm_pending and logic._wake_folded is None


def test_p13_revive_window_includes_40s():
    logic, t = navigating_cancel_asked(wake_revive=True)
    logic.on_wake_call(t)
    assert _says(_voice(logic, "affirm", t + WAKE_REVIVE_SEC)) == [MSG_CANCEL_CONFIRM]


def test_p13_wake_only_keeps_the_folded_question():
    """창 안에서 또 들린 "비카야"(호출어만 들린 unknown · rescue 호출)는 적어 둔 것을 지우지 않는다."""
    logic, t = navigating_cancel_asked(wake_revive=True)
    logic.on_wake_call(t)
    assert _says(logic.on_voice_intent(WAKE_ONLY, t + 1, lookup, BOUNDS, True)) == [MSG_WAKE_GREETING]
    logic.on_wake(t + 1)
    assert logic._wake_folded == ("cancel", "", t)
    assert _says(_voice(logic, "affirm", t + 2)) == [MSG_CANCEL_CONFIRM]


def test_p13_wake_only_does_not_use_up_the_one_reask_when_the_ladder_is_off():
    """세 단계 되묻기가 꺼져도(한 번 다시 묻기) 호출어만 들린 말은 그 한 번을 쓰지 않고 질문을 다시 말하지도 않는다."""
    logic, t = confirming(wake_revive=True)
    for i in range(3):
        assert _says(logic.on_voice_intent(WAKE_ONLY, t + i, lookup, BOUNDS, True)) == [MSG_WAKE_GREETING]
    assert logic.state == State.CONFIRMING and logic._confirm_ladder.step == 0
    assert _says(_voice(logic, "unknown", t + 4)) == [TOILET_ASK]      # 한 번 다시 묻기는 그대로 남아 있다


def test_off_wake_only_still_uses_the_one_reask():
    """꺼짐: 지금은 호출어만 들린 말도 못 알아들은 답으로 센다 — 첫 번째에 질문을 다시 말한다."""
    logic, t = confirming()
    says = [_says(logic.on_voice_intent(WAKE_ONLY, t + i, lookup, BOUNDS, True)) for i in range(3)]
    assert says == [[TOILET_ASK], [MSG_WAKE_GREETING], [MSG_WAKE_GREETING]]


@pytest.mark.parametrize("make, question", [
    (confirming, TOILET_ASK),
    (navigating_cancel_asked, MSG_CANCEL_CONFIRM),
    (navigating_help_asked, MSG_HELP_CONFIRM),
])
def test_p13_revived_question_starts_its_reask_ladder_from_zero(make, question):
    """세 단계 되묻기와 함께 켜짐 — 접었다가 되살린 질문의 사다리는 0 부터다."""
    logic, t = make(wake_revive=True, reask_ladder=True)
    assert _says(_voice(logic, "unknown", t)) == [MSG_REASK_RETRY]
    assert _says(_voice(logic, "unknown", t + 1)) == [MSG_REASK_LOUDER]
    logic.on_wake_call(t + 2)
    assert _says(_voice(logic, "affirm", t + 3)) == [question]
    assert _says(_voice(logic, "unknown", t + 4)) == [MSG_REASK_RETRY]


def test_p13_approach_question_and_first_wait_call_stay_as_today():
    """접근 질문 중(2026-10-09)과 대기 중 첫 호출(2026-10-21)은 켜져도 그대로다."""
    logic, t = awaiting_user(wake_revive=True)
    assert logic.on_wake_call(t) == [WakeReply(listen=True)] and logic.state == State.AWAITING_USER
    assert logic._wake_folded is None
    off, t = waiting()
    on, _ = waiting(wake_revive=True)
    assert on.on_wake_call(t) == off.on_wake_call(t) and on.state == off.state == State.WAITING
    assert on._wake_folded is None


def test_node_and_launch_expose_wake_revive_off_by_default():
    node = (PKG / "vica_mission_manager" / "mission_manager_node.py").read_text(encoding="utf-8")
    launch = (PKG / "launch" / "mission_manager.launch.py").read_text(encoding="utf-8")
    assert 'self.declare_parameter("wake_revive", False)' in node
    assert 'wake_revive=bool(self.get_parameter("wake_revive").value)' in node
    assert 'DeclareLaunchArgument("wake_revive", default_value="false")' in launch
    assert '"wake_revive": ParameterValue(' in launch
