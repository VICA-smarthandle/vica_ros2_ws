"""세 단계 되묻기 — 파라미터 reask_ladder 가 켜졌을 때만(원칙 8 · 9 · 21 · 31, Task 3.8)."""
from pathlib import Path

import pytest

from reaction_states import (BOUNDS, TOILET, asking, asking_end_confirm, asking_wait_time, awaiting_user, confirming,
                             confirming_change, estopped, idle, intent, lookup, navigating, navigating_cancel_asked,
                             navigating_help_asked, paused_resume_asked, waiting, waiting_asked, waiting_need_asked)
from vica_mission_manager.mission_logic import (
    _RESUME_QUESTION_CLOSERS,
    MSG_APPROACH_REASK,
    MSG_ASK_ENTRANCE,
    MSG_ASK_RESTROOM,
    MSG_ASK_WAIT_TIME,
    MSG_CANCEL_CONFIRM,
    MSG_CANCEL_KEPT,
    MSG_CANCELED,
    MSG_CONFIRM_PROMPT_FALLBACK,
    MSG_CONFIRM_TIMEOUT,
    MSG_HELP_CANCEL_CONFIRM,
    MSG_HELP_CANCELED,
    MSG_HELP_CONFIRM,
    MSG_LEAVING_NOTICE,
    MSG_REASK_LOUDER,
    MSG_REASK_RETRY,
    MSG_RESUME_CONFIRM,
    MSG_SPK_REASK,
    MSG_WAIT_FINISH_ASK,
    MSG_WAIT_NEED_ASK,
    MSG_WAKE_GREETING,
    CancelNav,
    MissionLogic,
    Navigate,
    NavStatus,
    Say,
    State,
    say_destination,
)
from vica_mission_manager.question_record import QuestionTracker

TOILET_ASK = say_destination(MSG_CONFIRM_PROMPT_FALLBACK, "화장실")
PKG = Path(__file__).resolve().parents[1]


def _says(actions):
    return [a.text for a in actions if isinstance(a, Say)]


def _question(acts):
    """동작 가운데 kind 가 있는 Say(질문 기록에 실릴 말)."""
    found = [a for a in acts if isinstance(a, Say) and a.kind]
    assert len(found) == 1, acts
    return found[0]


def _voice(logic, kind, t, **kw):
    return logic.on_voice_intent(intent(kind, **kw), t, lookup, BOUNDS, True)


def test_p8_confirm_question_goes_retry_louder_then_folds_when_not_driving():
    logic, t = confirming(reask_ladder=True)
    r1 = logic.on_voice_intent(intent("unknown"), t, lookup, BOUNDS, True)
    r2 = logic.on_voice_intent(intent("unknown"), t + 5, lookup, BOUNDS, True)
    assert _says(r1) == [MSG_REASK_RETRY] and _says(r2) == [MSG_REASK_LOUDER]
    assert _question(r1).question_text == TOILET_ASK and _question(r1).kind == "confirm"
    assert (_question(r1).step, _question(r2).step) == (1, 2)
    logic.on_voice_intent(intent("unknown"), t + 10, lookup, BOUNDS, True)
    assert logic.state != State.CONFIRMING


def test_p21_third_step_offers_staff_only_while_driving():
    logic, t = navigating(reask_ladder=True)
    says = [_says(logic.on_voice_intent(intent("unknown"), t + i, lookup, BOUNDS, True)) for i in range(3)]
    assert says == [[MSG_REASK_RETRY], [MSG_REASK_LOUDER], [MSG_HELP_CONFIRM]]
    logic, t = idle(reask_ladder=True)
    says = [_says(logic.on_voice_intent(intent("unknown"), t + i, lookup, BOUNDS, True)) for i in range(3)]
    assert says == [[MSG_REASK_RETRY], [MSG_REASK_LOUDER], []]


def test_p9_the_voice_handed_retry_sentence_is_not_said_twice():
    logic, t = idle(reask_ladder=True)
    acts = logic.on_voice_intent(intent("unknown", ask_text=MSG_REASK_RETRY, ask_kind="open"), t, lookup, BOUNDS, True)
    assert _says(acts) == [MSG_REASK_RETRY]


def test_p8_silence_and_not_understood_count_apart():
    """대답 없음은 원래 질문을 다시 말하고 사다리를 올리지 않는다 — 그 뒤의 못 알아들은 말은 ① 부터(Task N4)."""
    logic, t = confirming(reask_ladder=True)
    assert _says(logic.on_tick(t + 16, NavStatus.NONE)) == [TOILET_ASK]
    assert _says(logic.on_voice_intent(intent("unknown"), t + 20, lookup, BOUNDS, True)) == [MSG_REASK_RETRY]


def test_off_still_reasks_the_same_question_once():
    logic, t = confirming()
    ask = say_destination(MSG_CONFIRM_PROMPT_FALLBACK, "화장실")
    assert _says(logic.on_voice_intent(intent("unknown"), t, lookup, BOUNDS, True)) == [ask]


def test_off_is_the_default_and_reasks_carry_no_step_or_question_text():
    logic, t = confirming()
    assert MissionLogic().reask_ladder is False
    say = _question(_voice(logic, "unknown", t))
    assert (say.text, say.step, say.question_text) == (TOILET_ASK, 0, "")
    assert _voice(logic, "unknown", t + 1) == []
    assert logic.state == State.CONFIRMING
    assert _says(logic.on_tick(30.5, NavStatus.NONE)) == [MSG_CONFIRM_TIMEOUT]


def test_off_idle_unknown_stays_silent():
    logic, t = idle()
    assert [_voice(logic, "unknown", t + i) for i in range(3)] == [[], [], []]
    logic, t = navigating()
    assert [_voice(logic, "unknown", t + i) for i in range(3)] == [[], [], []]


def test_on_switches_all_five_question_ladders():
    on, off = MissionLogic(reask_ladder=True), MissionLogic()
    names = ("_confirm_ladder", "_cancel_ladder", "_help_ladder", "_wait_ladder", "_arrival_ladder")
    assert all(getattr(on, n).three_step for n in names)
    assert not any(getattr(off, n).three_step for n in names)


def test_p8_reask_sentences_word_for_word():
    assert MSG_REASK_RETRY == "잘 듣지 못했습니다. 다시 말씀해 주세요."
    assert MSG_REASK_LOUDER == "죄송합니다. 더 크게, 또박또박 말씀해 주세요."


def test_p8_confirm_silence_says_the_question_at_15s_then_folds_at_30s():
    logic, t = confirming(reask_ladder=True)
    assert _says(logic.on_tick(14.9, NavStatus.NONE)) == []
    first = logic.on_tick(15.0, NavStatus.NONE)
    assert _says(first) == [TOILET_ASK] and (_question(first).question_text, _question(first).step) == ("", 0)
    assert _question(first).options == (("wc", "화장실"),) and _question(first).expects_reply
    assert _says(logic.on_tick(29.9, NavStatus.NONE)) == []
    assert logic.state == State.CONFIRMING
    assert _says(logic.on_tick(30.0, NavStatus.NONE)) == [MSG_CONFIRM_TIMEOUT]
    assert logic.state == State.IDLE


def test_p8_confirm_silence_waits_while_the_user_is_speaking():
    logic, t = confirming(reask_ladder=True)
    logic.on_listen_state("speech", 14.0)
    assert _says(logic.on_tick(15.0, NavStatus.NONE)) == []
    logic.on_listen_state("empty:short-reject", 16.0)
    assert _says(logic.on_tick(16.1, NavStatus.NONE)) == [TOILET_ASK]


def test_p21_cancel_question_while_driving_goes_retry_louder_help():
    logic, t = navigating(reask_ladder=True)
    assert _says(_voice(logic, "cancel", t)) == [MSG_CANCEL_CONFIRM]
    r1, r2, r3 = (_voice(logic, "unknown", t + i) for i in (1, 2, 3))
    assert [_says(r) for r in (r1, r2, r3)] == [[MSG_REASK_RETRY], [MSG_REASK_LOUDER], [MSG_HELP_CONFIRM]]
    assert _question(r1).question_text == MSG_CANCEL_CONFIRM and _question(r1).kind == "yesno"
    assert not logic.cancel_confirm_pending and logic.help_phase == "asking"
    assert logic.state == State.NAVIGATING


def test_p21_cancel_question_silence_never_reaches_help_while_driving():
    """대답 없음은 직원 도움(③)으로 가지 않는다 — 15 s 에 같은 질문, 30 s 에 조용히 닫기(꺼짐과 같다, Task N4)."""
    logic, t = navigating(reask_ladder=True)
    _voice(logic, "cancel", t)
    assert _says(logic.on_tick(t + 15, NavStatus.NONE)) == [MSG_CANCEL_CONFIRM]
    assert _says(logic.on_tick(t + 30, NavStatus.NONE)) == [] and not logic.cancel_confirm_pending
    assert _says(logic.on_tick(t + 45, NavStatus.NONE)) == [] and logic.help_phase == "none"


def test_p8_cancel_question_not_driving_closes_quietly():
    logic, t = navigating(reask_ladder=True)
    _voice(logic, "pause", t)
    assert logic.state == State.PAUSED
    _voice(logic, "cancel", t + 1)
    assert logic.cancel_confirm_pending
    says = [_says(_voice(logic, "unknown", t + 2 + i)) for i in range(3)]
    assert says == [[MSG_REASK_RETRY], [MSG_REASK_LOUDER], []]
    assert not logic.cancel_confirm_pending and logic.help_phase == "none" and logic.state == State.PAUSED


def test_p21_help_question_goes_retry_louder_then_drops_without_a_third():
    logic, t = navigating(reask_ladder=True)
    assert _says(_voice(logic, "help", t)) == [MSG_HELP_CONFIRM]
    says = [_says(_voice(logic, "unknown", t + 1 + i)) for i in range(3)]
    assert says == [[MSG_REASK_RETRY], [MSG_REASK_LOUDER], []]
    assert logic.help_phase == "none" and logic.state == State.NAVIGATING


def test_p8_help_question_silence_asks_once_then_drops():
    logic, t = navigating(reask_ladder=True)
    _voice(logic, "help", t)
    assert _says(logic.on_tick(t + 15, NavStatus.NONE)) == [MSG_HELP_CONFIRM]
    assert _says(logic.on_tick(t + 30, NavStatus.NONE)) == []
    assert logic.help_phase == "none"


def test_p8_help_cancel_question_steps_then_keeps_the_request():
    logic, t = navigating(reask_ladder=True)
    _voice(logic, "help", t)
    _voice(logic, "affirm", t + 1)
    assert _says(_voice(logic, "cancel", t + 2)) == [MSG_HELP_CANCEL_CONFIRM]
    first = _voice(logic, "unknown", t + 3)
    assert _says(first) == [MSG_REASK_RETRY] and _question(first).question_text == MSG_HELP_CANCEL_CONFIRM
    assert _says(logic.on_tick(t + 3 + 15, NavStatus.NONE)) == [MSG_HELP_CANCEL_CONFIRM]
    assert _says(logic.on_tick(t + 3 + 30, NavStatus.NONE)) == []
    assert not logic.help_cancel_pending and logic.help_phase == "requested"


def test_p8_arrival_question_goes_retry_louder_then_leaving_notice():
    logic, t = asking(reask_ladder=True)
    r1, r2, r3 = (_voice(logic, "unknown", t + i) for i in range(3))
    assert [_says(r) for r in (r1, r2, r3)] == [[MSG_REASK_RETRY], [MSG_REASK_LOUDER], [MSG_LEAVING_NOTICE]]
    assert _question(r1).question_text == MSG_ASK_RESTROOM and _question(r1).kind == "yesno"
    assert _question(r2).step == 2


def test_p8_arrival_silence_keeps_its_8s_clock():
    """8 초 침묵 → 같은 질문 → 또 8 초 → 떠나기 예고(꺼짐과 같다, Task N4)."""
    logic, _ = asking(reask_ladder=True)
    assert _says(logic.on_tick(10.5, NavStatus.NONE)) == [MSG_ASK_RESTROOM]
    logic.on_arrival_question_spoken(11.0)
    assert _says(logic.on_tick(19.5, NavStatus.NONE)) == [MSG_LEAVING_NOTICE]


def test_p8_wait_time_yes_still_asks_the_same_question_then_waits_30_minutes():
    logic, t = asking_wait_time(reask_ladder=True)
    assert _says(_voice(logic, "affirm", t)) == [MSG_ASK_WAIT_TIME]
    _voice(logic, "affirm", t + 1)
    assert logic.state == State.WAITING and logic.wait_minutes_requested() == 30


def test_p8_wait_need_question_steps_then_keeps_waiting():
    logic, t = waiting(reask_ladder=True)
    assert _says(_voice(logic, "cancel", t)) == [MSG_WAIT_NEED_ASK]
    r1 = _voice(logic, "unknown", t + 1)
    assert _says(r1) == [MSG_REASK_RETRY] and _question(r1).kind == "yesno_inverted"
    assert _says(logic.on_tick(t + 1 + 15, NavStatus.NONE)) == [MSG_WAIT_NEED_ASK]
    assert _says(logic.on_tick(t + 1 + 30, NavStatus.NONE)) == [MSG_CANCEL_KEPT]
    assert logic.state == State.WAITING


def test_p8_wait_finish_question_silence_asks_once():
    """대답 없음은 꺼짐과 같다 — 15 s 에 같은 질문 한 번, 그 뒤는 다시 묻지 않는다(30 s 창이 닫힘, Task N4)."""
    logic, t = waiting_asked(reask_ladder=True)
    first = logic.on_tick(t + 14.5, NavStatus.NONE)
    assert _says(first) == [MSG_WAIT_FINISH_ASK] and _question(first).question_text == ""
    assert MSG_WAIT_FINISH_ASK not in _says(logic.on_tick(t + 29.5, NavStatus.NONE))
    assert MSG_WAIT_FINISH_ASK not in _says(logic.on_tick(t + 44.5, NavStatus.NONE))
    assert logic._wait_finish_asked_at == t + 14.5 and logic.state == State.WAITING


def test_p8_an_understood_word_starts_the_count_over():
    logic, t = confirming(reask_ladder=True)
    assert _says(_voice(logic, "unknown", t)) == [MSG_REASK_RETRY]
    _voice(logic, "question", t + 1)
    assert logic.state == State.CONFIRMING
    assert _says(_voice(logic, "unknown", t + 2)) == [MSG_REASK_RETRY]


def test_p8_a_dropped_answer_does_not_start_the_count_over():
    """질문 전에 시작한 "네"는 버린다(2026-10-11) — 처리하지 않은 말이라 횟수도 그대로다."""
    logic, t = confirming(reask_ladder=True)
    assert _says(_voice(logic, "unknown", t)) == [MSG_REASK_RETRY]
    logic.on_listen_state("speech", t + 0.5)
    logic.note_question_asked(MSG_REASK_RETRY, t + 1)
    assert _voice(logic, "affirm", t + 2) == [] and logic.state == State.CONFIRMING
    assert _says(_voice(logic, "unknown", t + 3)) == [MSG_REASK_LOUDER]


def test_p8_idle_count_starts_over_on_wake():
    logic, t = idle(reask_ladder=True)
    assert _says(_voice(logic, "unknown", t)) == [MSG_REASK_RETRY]
    assert _says(_voice(logic, "unknown", t + 1)) == [MSG_REASK_LOUDER]
    logic.on_wake_call(t + 2)
    assert _says(_voice(logic, "unknown", t + 3)) == [MSG_REASK_RETRY]


def test_p8_idle_count_starts_over_on_an_understood_word():
    logic, t = idle(reask_ladder=True)
    _voice(logic, "unknown", t)
    _voice(logic, "question", t + 1)
    assert _says(_voice(logic, "unknown", t + 2)) == [MSG_REASK_RETRY]


def test_p8_idle_reask_is_an_open_question():
    logic, t = idle(reask_ladder=True)
    say = _question(_voice(logic, "unknown", t))
    assert (say.kind, say.expects_reply, say.question_text, say.step) == ("open", True, "", 1)


def test_p8_no_reask_where_voice_is_not_taken():
    logic, t = estopped(reask_ladder=True)
    assert [_voice(logic, "unknown", t + i) for i in range(2)] == [[], []]


def test_p9_handed_retry_is_dropped_even_when_the_ladder_is_done():
    logic, t = idle(reask_ladder=True)
    handed = dict(ask_text=MSG_REASK_RETRY, ask_kind="open")
    _voice(logic, "unknown", t, **handed)
    _voice(logic, "unknown", t + 1, **handed)
    assert _voice(logic, "unknown", t + 2, **handed) == []


WAKE_ONLY = dict(ask_text=MSG_WAKE_GREETING, ask_kind="open")
LADDERS = ("_confirm_ladder", "_cancel_ladder", "_help_ladder", "_wait_ladder", "_arrival_ladder", "_idle_ladder")


def _steps(logic):
    return {n: getattr(logic, n).step for n in LADDERS}


def test_p9_handed_wake_greeting_keeps_todays_path():
    """"네?"(호출어만 들림, 원칙 9)는 켜져 있어도 사다리를 올리지 않고 "네?"만 말한다 — 세 번 모두."""
    logic, t = idle(reask_ladder=True)
    says = [_says(_voice(logic, "unknown", t + i, **WAKE_ONLY)) for i in range(3)]
    assert says == [[MSG_WAKE_GREETING]] * 3
    assert logic._idle_ladder.step == 0
    assert _says(_voice(logic, "unknown", t + 3)) == [MSG_REASK_RETRY]


def test_p9_wake_only_is_recognised_by_reply_or_handed_question():
    assert MissionLogic._is_wake_only(intent("unknown", ask_text=MSG_WAKE_GREETING))
    assert MissionLogic._is_wake_only(intent("unknown", reply=MSG_WAKE_GREETING))
    assert not MissionLogic._is_wake_only(intent("unknown", ask_text=MSG_REASK_RETRY))
    assert not MissionLogic._is_wake_only(intent("question", reply=MSG_WAKE_GREETING))


def test_p9_wake_only_leaves_every_question_and_count_alone():
    """질문이 열려 있어도 "네?"는 그 질문·시한·횟수를 건드리지 않는다(알아들은 말로도 치지 않는다). 확인 질문은 빠졌다 —"""
    for make, state in ((idle, State.IDLE), (navigating, State.NAVIGATING), (asking, State.ASKING_NEXT)):
        logic, t = make(reask_ladder=True)
        first = _voice(logic, "unknown", t)
        assert _says(first) == [MSG_REASK_RETRY], make.__name__
        before = (_steps(logic), logic._confirm_reask_at, logic._response_deadline, logic._asking_entered_at)
        for i in range(3):
            assert _says(_voice(logic, "unknown", t + 1 + i, **WAKE_ONLY)) == [MSG_WAKE_GREETING], make.__name__
        assert logic.state == state and logic.help_phase == "none", make.__name__
        assert (_steps(logic), logic._confirm_reask_at, logic._response_deadline,
                logic._asking_entered_at) == before, make.__name__
        assert _says(_voice(logic, "unknown", t + 5)) == [MSG_REASK_LOUDER], make.__name__


def test_p9_wake_only_leaves_the_other_question_ladders_alone_too():
    def cancel_q():
        logic, t = navigating(reask_ladder=True)
        _voice(logic, "cancel", t)
        return logic, t + 1

    def help_q():
        logic, t = navigating(reask_ladder=True)
        _voice(logic, "help", t)
        return logic, t + 1

    def help_cancel_q():
        logic, t = help_q()
        _voice(logic, "affirm", t)
        _voice(logic, "cancel", t + 1)
        return logic, t + 2

    def wait_need_q():
        logic, t = waiting(reask_ladder=True)
        _voice(logic, "cancel", t)
        return logic, t + 1

    def wait_finish_q():
        return waiting_asked(reask_ladder=True)

    for make, said in ((cancel_q, MSG_CANCEL_CONFIRM), (help_q, MSG_HELP_CONFIRM), (help_cancel_q, MSG_WAKE_GREETING),
                       (wait_need_q, MSG_WAKE_GREETING), (wait_finish_q, MSG_WAKE_GREETING)):
        logic, t = make()
        assert _says(_voice(logic, "unknown", t)) == [MSG_REASK_RETRY], make.__name__
        before = _steps(logic)
        for i in range(3):
            assert _says(_voice(logic, "unknown", t + 1 + i, **WAKE_ONLY)) == [said], make.__name__
        assert _steps(logic) == before, make.__name__
        assert _says(_voice(logic, "unknown", t + 5)) == [MSG_REASK_LOUDER], make.__name__


def _confirming_unknown_place(**kw):
    logic = MissionLogic(arrival_dialog=True, **kw)
    acts = logic.on_intent(intent(matched_destination_id="nope", need_confirm=True), None, BOUNDS, True, 0.0)
    assert acts == [] and logic.state == State.CONFIRMING and logic._confirm_prompt == ""
    return logic, 1.0


def test_p8_confirm_without_a_sentence_still_reasks_then_folds():
    logic, t = _confirming_unknown_place(reask_ladder=True)
    r1, r2, r3 = (_voice(logic, "unknown", t + i, ask_text=MSG_REASK_RETRY, ask_kind="open") for i in range(3))
    assert [_says(r) for r in (r1, r2)] == [[MSG_REASK_RETRY], [MSG_REASK_LOUDER]]
    assert (_question(r1).kind, _question(r1).question_text, _question(r1).step) == ("confirm", "", 1)
    assert _says(r3) == [MSG_CONFIRM_TIMEOUT] and logic.state == State.IDLE


def test_p8_confirm_without_a_sentence_silence_folds_like_off():
    """물을 문장이 없는 확인 질문 — 꺼짐처럼 대답 없음에 다시 묻지 않고 30 s 에 접는다(Task N4)."""
    logic, _ = _confirming_unknown_place(reask_ladder=True)
    assert _says(logic.on_tick(15.0, NavStatus.NONE)) == [] and logic.state == State.CONFIRMING
    assert _says(logic.on_tick(30.0, NavStatus.NONE)) == [MSG_CONFIRM_TIMEOUT]


def test_off_confirm_without_a_sentence_stays_as_today():
    logic, t = _confirming_unknown_place()
    acts = _voice(logic, "unknown", t, ask_text=MSG_REASK_RETRY, ask_kind="open")
    assert _says(acts) == [MSG_REASK_RETRY] and _question(acts).kind == "open"
    assert _says(logic.on_tick(15.0, NavStatus.NONE)) == []
    assert _says(logic.on_tick(30.0, NavStatus.NONE)) == [MSG_CONFIRM_TIMEOUT]


def _node_says(logic, actions, now):
    """노드 _run_actions 의 질문 표시 — 대답을 기다리는 말마다 note_question_asked(question_text or text)."""
    for a in actions:
        if isinstance(a, Say) and a.expects_reply:
            logic.note_question_asked(a.question_text or a.text, now)
    return actions


def test_p8_reask_line_keeps_the_original_question_time():
    """①이 나가도 _question_at 은 원래 질문 시각에 머문다 — 원래 질문 뒤·① 전에 시작한 "네"를 버리지 않는다."""
    logic, t = confirming(reask_ladder=True)
    logic.note_question_asked(TOILET_ASK, 0.0)
    logic.on_listen_state("speech", 5.0)
    _node_says(logic, _voice(logic, "unknown", 6.0), 6.0)
    assert logic._question_at == 0.0
    acts = _voice(logic, "affirm", 7.0)
    assert logic.voice_note != "질문 전에 시작한 말이라 대답으로 안 씀"
    assert logic.state == State.NAVIGATING and any(getattr(a, "destination", None) for a in acts)


def test_off_handed_retry_is_said_as_today():
    logic, t = idle()
    acts = _voice(logic, "unknown", t, ask_text=MSG_REASK_RETRY, ask_kind="open")
    assert _says(acts) == [MSG_REASK_RETRY] and _question(acts).step == 0


def test_p8_question_record_carries_the_original_question_and_step():
    tracker = QuestionTracker("q")
    say = Say(MSG_REASK_LOUDER, priority="response", expects_reply=True, kind="confirm",
              options=(("wc", "화장실"),), question_text=TOILET_ASK, step=2)
    _, record = tracker.open(say, "confirming", 1.0)
    assert (record.text, record.step, record.kind, record.options) == (TOILET_ASK, 2, "confirm", (("wc", "화장실"),))
    _, plain = tracker.open(Say(TOILET_ASK, expects_reply=True, kind="confirm"), "confirming", 2.0)
    assert (plain.text, plain.step) == (TOILET_ASK, 0)


def _help_cancel_asked(**kw):
    """도움 요청을 보낸 뒤(일시정지) "취소" → "도움 요청을 취소할까요?"를 물은 직후. 물은 시각은 돌려주는 시각보다 1초 앞이다."""
    logic, t = navigating_help_asked(**kw)
    _voice(logic, "affirm", t)
    _voice(logic, "cancel", t + 1)
    return logic, t + 2.0


P28_QUESTIONS = [
    (confirming, TOILET_ASK, State.CONFIRMING),
    (confirming_change, TOILET_ASK, State.CONFIRMING),
    (navigating_cancel_asked, MSG_CANCEL_CONFIRM, State.NAVIGATING),
    (navigating_help_asked, MSG_HELP_CONFIRM, State.NAVIGATING),
    (_help_cancel_asked, MSG_HELP_CANCEL_CONFIRM, State.PAUSED),
    (paused_resume_asked, MSG_RESUME_CONFIRM, State.PAUSED),
    (asking, MSG_ASK_RESTROOM, State.ASKING_NEXT),
    (asking_wait_time, MSG_ASK_WAIT_TIME, State.ASKING_WAIT_TIME),
    (asking_end_confirm, MSG_ASK_ENTRANCE, State.ASKING_NEXT),
    (waiting_need_asked, MSG_WAIT_NEED_ASK, State.WAITING),
    (waiting_asked, MSG_WAIT_FINISH_ASK, State.WAITING),
    (awaiting_user, MSG_APPROACH_REASK, State.AWAITING_USER),
]
_P28_IDS = [make.__name__ for make, _, _ in P28_QUESTIONS]


def test_p28_repeat_does_not_use_up_a_reask_step():
    logic, t = confirming(reask_ladder=True)
    logic.on_voice_intent(intent("repeat"), t, lookup, BOUNDS, True)
    assert _says(logic.on_voice_intent(intent("unknown"), t + 3, lookup, BOUNDS, True)) == [MSG_REASK_RETRY]


@pytest.mark.parametrize("ladder", [False, True], ids=["ladder_off", "ladder_on"])
@pytest.mark.parametrize("make, question, state", P28_QUESTIONS, ids=_P28_IDS)
def test_p28_repeat_says_the_open_question_again(make, question, state, ladder):
    """질문마다 같은 문장 그대로(사다리 문장 ①·② 가 아니다) — 대답을 기다리는 말이고 상태는 그대로, 주행 동작은 없다."""
    logic, t = make(reask_ladder=ladder)
    acts = _voice(logic, "repeat", t)
    assert _says(acts) == [question] and logic.state == state
    say = _question(acts)
    assert say.expects_reply and (say.step, say.question_text) == (0, "")
    assert not any(isinstance(a, (Navigate, CancelNav)) for a in acts)


P28_LADDER = [
    (confirming, TOILET_ASK), (navigating_cancel_asked, MSG_CANCEL_CONFIRM), (navigating_help_asked, MSG_HELP_CONFIRM),
    (_help_cancel_asked, MSG_HELP_CANCEL_CONFIRM), (paused_resume_asked, MSG_RESUME_CONFIRM),
    (asking, MSG_ASK_RESTROOM), (waiting_need_asked, MSG_WAIT_NEED_ASK), (waiting_asked, MSG_WAIT_FINISH_ASK),
]


@pytest.mark.parametrize("make, question", P28_LADDER, ids=[m.__name__ for m, _ in P28_LADDER])
def test_p28_repeat_keeps_the_reask_count_when_the_ladder_is_on(make, question):
    """세 단계: ① 뒤의 "다시 말해줘"는 그 질문을 다시 말할 뿐 — 다음 못 알아들은 답은 ② 다(올리지도 되돌리지도 않는다)."""
    logic, t = make(reask_ladder=True)
    assert _says(_voice(logic, "unknown", t)) == [MSG_REASK_RETRY]
    assert _says(_voice(logic, "repeat", t + 1)) == [question]
    assert _says(_voice(logic, "unknown", t + 2)) == [MSG_REASK_LOUDER]


@pytest.mark.parametrize("make, question", P28_LADDER, ids=[m.__name__ for m, _ in P28_LADDER])
def test_p28_repeat_keeps_the_one_reask_when_the_ladder_is_off(make, question):
    """꺼짐(옛 방식 — 같은 질문 한 번 다시): "다시 말해줘" 뒤에도 그 한 번이 남아 있고, 이미 썼으면 되살아나지 않는다."""
    logic, t = make()
    assert _says(_voice(logic, "repeat", t)) == [question]
    assert _says(_voice(logic, "unknown", t + 1)) == [question]
    assert _says(_voice(logic, "repeat", t + 2)) == [question]
    assert question not in _says(_voice(logic, "unknown", t + 3))


def test_p28_repeat_does_not_count_toward_the_approach_reasks():
    """접근 질문의 세 번(APPROACH_REASK_MAX)은 못 알아들은 말·대답 없음만 센다 — "다시 말해줘"로는 물러나지 않는다."""
    logic, t = awaiting_user()
    for i in range(5):
        assert _says(_voice(logic, "repeat", t + i)) == [MSG_APPROACH_REASK]
    assert logic._approach_reasks == 0 and logic.state == State.AWAITING_USER


def test_p28_repeat_with_no_mission_question_is_silent_and_keeps_the_count():
    logic, t = idle(reask_ladder=True)
    assert _says(_voice(logic, "unknown", t)) == [MSG_REASK_RETRY]
    assert _voice(logic, "repeat", t + 1) == [] and logic.state == State.IDLE
    assert _says(_voice(logic, "unknown", t + 2)) == [MSG_REASK_LOUDER]


P28_ANSWERS = [
    (confirming, "affirm", lambda lg, acts: lg.state == State.NAVIGATING and lg.active_destination == TOILET),
    (confirming_change, "affirm", lambda lg, acts: lg.state == State.NAVIGATING and lg.active_destination == TOILET),
    (navigating_cancel_asked, "affirm", lambda lg, acts: MSG_CANCELED in _says(acts) and lg.state == State.IDLE),
    (navigating_help_asked, "affirm", lambda lg, acts: lg.help_phase == "requested"),
    (_help_cancel_asked, "affirm", lambda lg, acts: MSG_HELP_CANCELED in _says(acts) and lg.help_phase == "none"),
    (paused_resume_asked, "resume", lambda lg, acts: lg.state == State.NAVIGATING),
    (asking, "affirm", lambda lg, acts: lg.state == State.WAITING),
    (asking_wait_time, "wait", lambda lg, acts: lg.state == State.WAITING and lg.wait_minutes_requested() == 5),
    (asking_end_confirm, "affirm", lambda lg, acts: lg.state == State.RETURNING),
    (waiting_need_asked, "affirm", lambda lg, acts: lg.state == State.RETURNING),
    (waiting_asked, "deny", lambda lg, acts: lg.state == State.RETURNING),
    (awaiting_user, "affirm", lambda lg, acts: lg.state == State.TURNING),
]


@pytest.mark.parametrize("make, answer, done", P28_ANSWERS, ids=[m.__name__ for m, _, _ in P28_ANSWERS])
def test_p28_the_answer_after_the_repeat_answers_that_question(make, answer, done):
    logic, t = make()
    _voice(logic, "repeat", t)
    acts = _voice(logic, answer, t + 2, wait_minutes=5 if answer == "wait" else -1)
    assert done(logic, acts)


def test_p28_repeat_keeps_the_resume_question_open():
    """"다시 출발할까요?"(Task 3.4)를 닫는 요청이 아니다 — 다시 말한 뒤의 "네"가 재출발이다."""
    assert "repeat" not in _RESUME_QUESTION_CLOSERS
    logic, t = paused_resume_asked()
    assert _says(_voice(logic, "repeat", t)) == [MSG_RESUME_CONFIRM]
    assert logic._resume_question_open() and logic.state == State.PAUSED
    acts = _voice(logic, "affirm", t + 2, answer="yes", answer_kind="yesno")
    assert logic.state == State.NAVIGATING and any(isinstance(a, Navigate) for a in acts)


@pytest.mark.parametrize("make, question", [
    (confirming, TOILET_ASK), (navigating_cancel_asked, MSG_CANCEL_CONFIRM), (navigating_help_asked, MSG_HELP_CONFIRM),
    (_help_cancel_asked, MSG_HELP_CANCEL_CONFIRM), (paused_resume_asked, MSG_RESUME_CONFIRM),
], ids=["confirm", "cancel", "help", "help_cancel", "resume"])
def test_p28_repeat_restarts_the_15s_silence_clock(make, question):
    """꺼짐: t-1 에 물은 질문은 t+14 에 한 번 다시 묻는다. t+9 에 다시 말했으면 그 15 s 는 t+24 부터다."""
    logic, t = make()
    assert _says(_voice(logic, "repeat", t + 9)) == [question]
    assert question not in _says(logic.on_tick(t + 15, NavStatus.RUNNING))
    assert _says(logic.on_tick(t + 24.5, NavStatus.RUNNING)) == [question]


def test_p28_repeat_restarts_the_confirm_deadline_off():
    """꺼짐: 0 s 에 물은 확인 질문을 10 s 에 다시 말하면 30 s 시한도 10 s 부터(25 s 에 한 번 다시 → 40.5 s 에 접힘)."""
    logic, _ = confirming()
    _voice(logic, "repeat", 10.0)
    assert _says(logic.on_tick(15.5, NavStatus.NONE)) == []
    assert _says(logic.on_tick(25.5, NavStatus.NONE)) == [TOILET_ASK]
    assert _says(logic.on_tick(39.0, NavStatus.NONE)) == [] and logic.state == State.CONFIRMING
    assert _says(logic.on_tick(41.0, NavStatus.NONE)) == [MSG_CONFIRM_TIMEOUT]


def test_p28_repeat_restarts_the_clock_but_not_the_ladder_on():
    """세 단계: ①(2 s) 뒤 10 s 에 다시 말하면 대답 없음의 다시 묻기는 25 s(원래 질문 — Task N4)이고, 다음 못 알아들은 말은 ② 다."""
    logic, _ = confirming(reask_ladder=True)
    assert _says(_voice(logic, "unknown", 2.0)) == [MSG_REASK_RETRY]
    _voice(logic, "repeat", 10.0)
    assert _says(logic.on_tick(17.5, NavStatus.NONE)) == []
    assert _says(logic.on_tick(25.5, NavStatus.NONE)) == [TOILET_ASK]
    assert _says(_voice(logic, "unknown", 27.0)) == [MSG_REASK_LOUDER]


def test_p28_repeat_after_the_leaving_notice_asks_again_and_holds_the_departure():
    """도착 질문의 떠나기 예고(3 s 유예) 뒤의 "다시 말해줘" — 질문을 다시 말하고 유예를 거둔다(질문의 시계를 다시 잰다)."""
    def leaving():
        logic, _ = asking()
        logic.on_tick(10.5, NavStatus.NONE)
        logic.on_arrival_question_spoken(11.0)
        assert _says(logic.on_tick(19.5, NavStatus.NONE)) == [MSG_LEAVING_NOTICE]
        assert _says(_voice(logic, "repeat", 20.0)) == [MSG_ASK_RESTROOM]
        assert logic.on_tick(23.5, NavStatus.NONE) == [] and logic.state == State.ASKING_NEXT
        logic.on_arrival_question_spoken(24.0)
        return logic

    logic = leaving()
    _voice(logic, "affirm", 26.0)
    assert logic.state == State.WAITING
    logic = leaving()
    assert _says(logic.on_tick(32.5, NavStatus.NONE)) == [MSG_LEAVING_NOTICE]


@pytest.mark.parametrize("make, question", [
    (confirming, TOILET_ASK), (confirming_change, TOILET_ASK), (navigating_cancel_asked, MSG_CANCEL_CONFIRM),
    (navigating_help_asked, MSG_HELP_CONFIRM), (paused_resume_asked, MSG_RESUME_CONFIRM),
], ids=["confirm", "change", "cancel", "help", "resume"])
def test_p28_repeat_after_the_call_asks_the_folded_question_again(make, question):
    """wake_revive 켜짐: 호출이 접어 둔 질문을 다시 묻는다 — 접힌 질문의 승낙과 같은 길(실행이 아니라 다시 묻기)."""
    logic, t = make(wake_revive=True)
    logic.on_wake_call(t)
    assert logic._wake_folded is not None
    acts = _voice(logic, "repeat", t + 3)
    assert _says(acts) == [question] and not any(isinstance(a, Navigate) for a in acts)
    assert logic._wake_folded is None and logic._resay_open_question(t + 3) is not None


def test_p28_the_yes_after_the_revived_question_answers_it():
    logic, t = confirming(wake_revive=True)
    logic.on_wake_call(t)
    _voice(logic, "repeat", t + 3)
    assert logic.state == State.CONFIRMING
    acts = _voice(logic, "affirm", t + 5)
    assert logic.state == State.NAVIGATING and any(isinstance(a, Navigate) for a in acts)


def test_p28_repeat_after_the_call_is_silent_when_nothing_is_folded():
    """wake_revive 꺼짐(접어 둔 것이 없음)·40 s 가 지남 — 말 없음, 상태 그대로."""
    logic, t = confirming()
    logic.on_wake_call(t)
    assert _voice(logic, "repeat", t + 3) == [] and logic.state == State.IDLE
    logic, t = navigating_cancel_asked(wake_revive=True)
    logic.on_wake_call(t)
    assert _voice(logic, "repeat", t + 41) == [] and logic.state == State.NAVIGATING
    assert not logic.cancel_confirm_pending and logic._wake_folded is None


def test_p28_repeat_is_not_judged_by_the_speaker_gate():
    """판정이 켜져 있고 목소리가 '다른 사람'이어도 다시 말한다 — 로봇을 움직이지 않는 말이다. 판정 단계도 열리지 않는다."""
    logic, t = waiting_asked(speaker_gate=True)
    acts = _voice(logic, "repeat", t, speaker_verdict="fail")
    assert _says(acts) == [MSG_WAIT_FINISH_ASK] and logic.state == State.WAITING
    assert logic._spk_gate._stage is None


def test_p28_repeat_leaves_an_open_gate_step_as_it_was():
    """판정 단계(되묻기)가 열린 채 "다시 말해줘"가 와도 그 단계에 들어가지 않는다 — 단계·저장한 명령·두 번 표시·시각 그대로라"""
    logic, t = waiting(speaker_gate=True)
    assert _says(_voice(logic, "cancel", t, speaker_verdict="fail")) == [MSG_SPK_REASK]
    gate = logic._spk_gate
    before = (gate._stage, gate.stored, gate._twice, gate._at)
    acts = _voice(logic, "repeat", t + 1, speaker_verdict="fail")
    assert (gate._stage, gate.stored, gate._twice, gate._at) == before
    assert logic.state == State.WAITING and not any(isinstance(a, (Navigate, CancelNav)) for a in acts)


def test_node_and_launch_expose_reask_ladder_off_by_default():
    node = (PKG / "vica_mission_manager" / "mission_manager_node.py").read_text(encoding="utf-8")
    launch = (PKG / "launch" / "mission_manager.launch.py").read_text(encoding="utf-8")
    assert 'self.declare_parameter("reask_ladder", False)' in node
    assert 'reask_ladder=bool(self.get_parameter("reask_ladder").value)' in node
    assert 'DeclareLaunchArgument("reask_ladder", default_value="false")' in launch
    assert '"reask_ladder": ParameterValue(' in launch
    assert "self.logic.note_question_asked(action.question_text or action.text, self._now())" in node
