"""'비카야' 호출 반응표 (2026-10-07 사용자 결정, 인수인계 문서 작업 계획 탭).

예전에는 음성 쪽이 상태와 상관없이 늘 "말 멈추기 → '네?' → 듣기"를 했다. 이제
미션이 상태를 보고 정한다(on_wake_call): 대답하는 상태는 끊기·"네?"·listen, 대답하지
않는 상태는 ignore 만, 비상 정지는 한 마디와 ignore. 음성 쪽은 호출 즉시 듣기 창을
열어 두고 WakeReply 가 올 때까지 들은 말을 쥐고 있다.
"""
import pytest

from vica_mission_manager.mission_logic import (
    MSG_ESTOP_REJECT,
    MSG_ESTOP_WAKE,
    MSG_WAKE_GREETING,
    Destination,
    IntentData,
    MapBounds,
    MissionLogic,
    NavStatus,
    Pose2D,
    Say,
    State,
    StopSpeech,
    WakeReply,
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
