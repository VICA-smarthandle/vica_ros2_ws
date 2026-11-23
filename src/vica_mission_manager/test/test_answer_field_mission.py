"""대답 칸으로 처리(원칙 6 · 3, Task 3.3) — 파라미터 answer_field 가 켜졌을 때만."""
from dataclasses import replace
from pathlib import Path

import pytest

import vica_mission_manager.mission_logic as ml
from reaction_states import (BOUNDS, ELEV, ROOM, TOILET, approaching, asking, asking_end_confirm, asking_wait_time,
                             awaiting_user, confirming, confirming_change, estopped, help_requested_confirm_asked, idle,
                             intent, lookup, moving_to_wait_spot, navigating, navigating_cancel_asked,
                             navigating_help_asked, paused, paused_resume_asked, waiting, waiting_asked,
                             waiting_need_asked, waiting_release)
from vica_mission_manager.mission_logic import (
    MSG_ASK_WAIT_TIME,
    MSG_CANCEL_CONFIRM,
    MSG_CANCEL_KEPT,
    MSG_CONFIRM_TIMEOUT,
    MSG_FINISH,
    MSG_HELP_COMING,
    MSG_PAUSE_KEPT,
    MSG_REASK_LOUDER,
    MSG_REASK_RETRY,
    MSG_RESUME_CONFIRM,
    MSG_RESUMED,
    MSG_WAIT_CONFIRM,
    MSG_WAKE_GREETING,
    HelpEvent,
    MissionLogic,
    Navigate,
    NavStatus,
    Say,
    State,
    say_destination,
)

PKG = Path(__file__).resolve().parents[1]


def _says(acts):
    return [a.text for a in acts if isinstance(a, Say)]


def _voice(logic, t, it):
    return logic.on_voice_intent(it, t, lookup, BOUNDS, True)


class _FakeUuid:
    """도움 요청 id 를 고정한다 — 두 로직의 동작을 repr 로 견주려고."""
    n = 0

    @classmethod
    def uuid4(cls):
        cls.n += 1
        return f"uuid-{cls.n}"


@pytest.fixture(autouse=True)
def fixed_uuid(monkeypatch):
    monkeypatch.setattr(ml, "uuid", _FakeUuid)


def _outcome(make, it, **kw):
    """상태를 새로 만들고 음성 요청 하나를 넣은 결과 — (상태, 동작 repr, 내부 값 몇 개)."""
    _FakeUuid.n = 0
    logic, t = make(**kw)
    acts = _voice(logic, t, it)
    return (logic.state, repr(acts), logic.cancel_confirm_pending, logic.help_phase, logic._confirming_dest_id,
            logic._wait_minutes_requested, logic._wait_need_asked_at, logic._wait_finish_asked_at)


def _open_q(make):
    """그 상태에서 미션이 쥔 열린 질문의 (종류, 후보 id) — 노드 질문 기록이 대답 칸에 싣는 값과 같다."""
    logic, t = make()
    asked = [a for a in logic._resay_open_question(t) or [] if isinstance(a, Say) and a.expects_reply]
    assert len(asked) == 1, make.__name__
    return asked[0].kind, tuple(o[0] for o in asked[0].options)


def _ans(make, kind="unknown", answer="", value="", **kw):
    """그 상태의 열린 질문에 대한 대답 칸을 실은 의도(노드가 넘기는 꼴)."""
    q_kind, q_options = _open_q(make)
    return intent(kind, answer=answer, answer_value=value, answer_kind=q_kind, answer_options=q_options, **kw)


def _then_wake(make):
    """그 상태에서 "비카야" — 미션이 "네?"(열린 질문)를 내고, 노드 질문 기록은 "네?"로 바뀐다."""
    def built(**kw):
        logic, t = make(**kw)
        logic.on_wake_call(t)
        return logic, t + 2.0
    built.__name__ = make.__name__ + "+wake"
    return built


def test_p6_confirm_yes_departs_and_no_folds():
    logic, t = confirming(answer_field=True)
    acts = _voice(logic, t, _ans(confirming, "unknown", "yes"))
    assert logic.state == State.NAVIGATING
    assert [a.destination.id for a in acts if isinstance(a, Navigate)] == [TOILET.id]
    logic, t = confirming(answer_field=True)
    assert MSG_CONFIRM_TIMEOUT in _says(_voice(logic, t, _ans(confirming, "navigate", "no")))


def test_p3_same_place_named_again_asks_again_instead_of_departing():
    logic, t = confirming(answer_field=True)
    acts = _voice(logic, t, _ans(confirming, "navigate", "destination", TOILET.id))
    assert logic.state == State.CONFIRMING and not any(isinstance(a, Navigate) for a in acts)


def test_p6_minutes_answer_is_a_wait():
    logic, t = asking_wait_time(answer_field=True)
    _voice(logic, t, _ans(asking_wait_time, "unknown", "minutes", "10"))
    assert logic.state == State.WAITING


def test_other_and_off_follow_the_intent():
    logic, t = confirming(answer_field=True)
    a = _says(_voice(logic, t, _ans(confirming, "cancel", "other")))
    logic2, t2 = confirming()
    assert a == _says(_voice(logic2, t2, intent("cancel")))
    logic3, t3 = confirming()
    _voice(logic3, t3, _ans(confirming, "unknown", "yes"))
    assert logic3.state == State.CONFIRMING


def test_p3_same_place_answer_reasks_the_same_confirm_question():
    logic, t = confirming(answer_field=True)
    prompt = logic._confirm_prompt
    acts = _voice(logic, t, _ans(confirming, "navigate", "destination", TOILET.id,
                                 matched_destination_id=TOILET.id, need_confirm=True))
    asked = [a for a in acts if isinstance(a, Say) and a.expects_reply]
    assert [a.text for a in asked] == [prompt] and asked[0].kind == "confirm"
    assert logic._confirming_dest_id == TOILET.id


def test_p3_without_answer_after_an_answer_still_asks_again():
    logic, t = confirming(answer_field=True)
    _voice(logic, t, _ans(confirming, "navigate", "destination", TOILET.id))
    acts = _voice(logic, t + 1.0, intent(matched_destination_id=TOILET.id, need_confirm=True))
    assert logic.state == State.CONFIRMING and not any(isinstance(a, Navigate) for a in acts)


def test_p3_off_same_place_answer_departs_as_now():
    logic, t = confirming()
    _voice(logic, t, _ans(confirming, "navigate", "destination", TOILET.id,
                          matched_destination_id=TOILET.id, need_confirm=True))
    assert logic.state == State.NAVIGATING


def _same_place(**kw):
    return intent(matched_destination_id=TOILET.id, need_confirm=True, **kw)


def test_p3_same_place_proposal_without_answer_asks_again():
    logic, t = confirming(answer_field=True)
    prompt = logic._confirm_prompt
    acts = _voice(logic, t, _same_place())
    assert _navs(acts) == [] and logic.state == State.CONFIRMING
    asked = [a for a in acts if isinstance(a, Say) and a.expects_reply]
    assert [a.text for a in asked] == [prompt] and asked[0].kind == "confirm"
    assert asked[0].options == ((TOILET.id, TOILET.name),)
    assert logic._confirming_dest_id == TOILET.id


def test_p3_same_place_proposal_then_yes_departs():
    logic, t = confirming(answer_field=True)
    _voice(logic, t, _same_place())
    acts = _voice(logic, t + 1.0, intent("affirm"))
    assert _navs(acts) == [TOILET.id] and logic.state == State.NAVIGATING


def test_p3_confirmed_same_place_still_departs():
    logic, t = confirming(answer_field=True)
    acts = _voice(logic, t, intent(matched_destination_id=TOILET.id, need_confirm=False))
    assert _navs(acts) == [TOILET.id] and logic.state == State.NAVIGATING


def test_p3_answer_yes_on_a_same_place_proposal_departs():
    logic, t = confirming(answer_field=True)
    acts = _voice(logic, t, _ans(confirming, "navigate", "yes", matched_destination_id=TOILET.id, need_confirm=True))
    assert _navs(acts) == [TOILET.id] and logic.state == State.NAVIGATING


def test_p3_off_same_place_proposal_departs_as_now():
    logic, t = confirming()
    acts = _voice(logic, t, _same_place())
    assert _navs(acts) == [TOILET.id] and logic.state == State.NAVIGATING


def test_p3_other_place_proposal_asks_about_that_place_as_now():
    on = _outcome(confirming, intent(matched_destination_id=ELEV.id, need_confirm=True), answer_field=True)
    assert on == _outcome(confirming, intent(matched_destination_id=ELEV.id, need_confirm=True))
    assert on[0] == State.CONFIRMING and on[4] == ELEV.id and "Navigate(" not in on[1]


def test_p3_same_place_proposal_three_times_never_departs():
    logic, t = confirming(answer_field=True)
    for i in range(3):
        acts = _voice(logic, t + i, _same_place())
        assert _navs(acts) == [] and logic.state == State.CONFIRMING
        assert [a.text for a in acts if isinstance(a, Say)] == [logic._confirm_prompt]


def test_p3_same_place_proposal_is_handled_like_a_destination_answer():
    def run(it):
        _FakeUuid.n = 0
        logic, t = confirming(answer_field=True, reask_ladder=True)
        out = [_voice(logic, t, intent("unknown"))]
        out.append(_voice(logic, t + 2.0, it))
        out.append(_voice(logic, t + 3.0, intent("unknown")))
        return (repr(out), logic.state, logic._confirm_deadline, logic._confirm_reask_at, logic._confirm_ladder.step)
    plain = run(_same_place())
    assert plain == run(_ans(confirming, "navigate", "destination", TOILET.id,
                             matched_destination_id=TOILET.id, need_confirm=True))
    assert plain[1] == State.CONFIRMING and "Navigate(" not in plain[0]


def test_p3_same_place_proposal_in_the_change_question_asks_again():
    logic, t = confirming_change(answer_field=True)
    prompt = logic._confirm_prompt
    acts = _voice(logic, t, _same_place())
    assert _navs(acts) == [] and logic.state == State.CONFIRMING
    assert _says(acts) == [prompt] and logic._change_from is ROOM
    acts = _voice(logic, t + 1.0, intent("affirm"))
    assert _navs(acts) == [TOILET.id]


def test_p13_revived_confirm_then_same_place_proposal_asks_again():
    logic, t = confirming(answer_field=True, wake_revive=True)
    logic.on_wake_call(t)
    acts = _voice(logic, t + 2.0, intent("affirm"))
    assert _navs(acts) == [] and logic.state == State.CONFIRMING and logic._confirming_dest_id == TOILET.id
    acts = _voice(logic, t + 3.0, _same_place())
    assert _navs(acts) == [] and logic.state == State.CONFIRMING
    assert _says(acts) == [logic._confirm_prompt]


def test_p6_choice_of_another_place_asks_about_that_place():
    logic, t = confirming(answer_field=True)
    acts = _voice(logic, t, _ans(confirming, "unknown", "choice", ELEV.id))
    assert logic.state == State.CONFIRMING and logic._confirming_dest_id == ELEV.id
    assert not any(isinstance(a, Navigate) for a in acts)
    assert [a.options for a in acts if isinstance(a, Say) and a.expects_reply] == [((ELEV.id, ELEV.name),)]


def test_p6_end_question_no_is_a_no_even_if_read_as_cancel():
    """"여기까지 안내를 마칠까요?"에 "아니" — 의도는 cancel 로 왔어도 거절이다(안내가 끝나지 않는다)."""
    it = _ans(asking_end_confirm, "cancel", "no")
    assert it.answer_kind == "yesno"
    off = _outcome(asking_end_confirm, it)
    assert MSG_FINISH in off[1]
    on = _outcome(asking_end_confirm, it, answer_field=True)
    assert MSG_FINISH not in on[1] and on[0] == State.WAITING
    assert on == _outcome(asking_end_confirm, intent("deny"), answer_field=True)


def test_p6_ten_minutes_read_as_destination_is_a_ten_minute_wait():
    """"몇 분쯤 걸리실까요?"에 "십 분" — 의도는 목적지 제안으로 왔어도 10분 대기다."""
    it = _ans(asking_wait_time, "navigate", "minutes", "10", matched_destination_id=ELEV.id, need_confirm=True)
    assert it.answer_kind == "minutes"
    off = _outcome(asking_wait_time, it)
    assert off[0] == State.CONFIRMING
    logic, t = asking_wait_time(answer_field=True)
    acts = _voice(logic, t, it)
    assert logic.state == State.WAITING and logic._wait_minutes_requested == 10
    assert MSG_WAIT_CONFIRM.format(minutes=10) in _says(acts)


def test_p6_inverted_question_yes_is_affirm():
    """"안내가 필요 없으신가요?"의 yes = 필요 없다 — 지금 affirm 과 같은 결과(음성도 같은 방향으로 낸다)."""
    assert _open_q(waiting_need_asked)[0] == "yesno_inverted"
    yes = _outcome(waiting_need_asked, _ans(waiting_need_asked, "unknown", "yes"), answer_field=True)
    assert yes == _outcome(waiting_need_asked, intent("affirm"), answer_field=True)
    assert MSG_FINISH in yes[1]
    no = _outcome(waiting_need_asked, _ans(waiting_need_asked, "cancel", "no"), answer_field=True)
    assert no == _outcome(waiting_need_asked, intent("deny"), answer_field=True)
    assert MSG_CANCEL_KEPT in no[1]


def test_p6_cancel_question_no_keeps_guiding():
    on = _outcome(navigating_cancel_asked, _ans(navigating_cancel_asked, "cancel", "no"), answer_field=True)
    assert on == _outcome(navigating_cancel_asked, intent("deny"), answer_field=True)
    assert on[0] == State.NAVIGATING and on[2] is False


def test_p6_approach_question_yes_is_affirm():
    assert (_outcome(awaiting_user, _ans(awaiting_user, "unknown", "yes"), answer_field=True)
            == _outcome(awaiting_user, intent("affirm"), answer_field=True))


@pytest.mark.parametrize("make", [confirming, asking_wait_time, asking, waiting_need_asked])
def test_p6_destination_without_value_is_unknown(make):
    it = _ans(make, "navigate", "destination", "", matched_destination_id=ELEV.id, need_confirm=True)
    assert _outcome(make, it, answer_field=True) == _outcome(make, intent("unknown"), answer_field=True)


@pytest.mark.parametrize("value", ["열", "0", "-3", "", "10.5"])
def test_p6_minutes_not_a_positive_number_is_unknown(value):
    it = _ans(asking_wait_time, "navigate", "minutes", value, matched_destination_id=ELEV.id, need_confirm=True)
    assert (_outcome(asking_wait_time, it, answer_field=True)
            == _outcome(asking_wait_time, intent("unknown"), answer_field=True))
    assert MSG_ASK_WAIT_TIME in _outcome(asking_wait_time, it, answer_field=True)[1]


@pytest.mark.parametrize("make", [confirming, asking_wait_time, waiting_need_asked, navigating_cancel_asked])
def test_p6_unclear_is_unknown(make):
    assert (_outcome(make, _ans(make, "affirm", "unclear"), answer_field=True)
            == _outcome(make, intent("unknown"), answer_field=True))


def test_p6_swapped_intent_still_goes_through_the_speaker_gate():
    swapped = intent("unknown", answer="minutes", answer_value="20", answer_kind="minutes",
                     speaker_verdict="fail", speaker_score=0.1)
    plain = intent("wait", wait_minutes=20, speaker_verdict="fail", speaker_score=0.1)
    gated = _outcome(waiting, swapped, answer_field=True, speaker_gate=True)
    assert gated == _outcome(waiting, plain, answer_field=True, speaker_gate=True)
    assert gated != _outcome(waiting, swapped, answer_field=True)
    assert gated[5] != 20


def test_p6_understood_answer_restarts_the_reask_ladder():
    def run(on):
        logic, t = idle(answer_field=on, reask_ladder=True)
        sent = (intent("unknown"), intent("unknown", answer="yes", answer_kind="yesno"), intent("unknown"))
        return [_says(_voice(logic, t + i, it)) for i, it in enumerate(sent)]
    first = [ml.MSG_REASK_RETRY]
    assert run(True) == [first, [], first]
    assert run(False) == [first, [ml.MSG_REASK_LOUDER], []]


def test_p13_after_wake_greeting_end_question_yes_is_not_finish():
    """"여기까지 안내를 마칠까요?" → "비카야" → "네?" → "기다려"(대답 칸 yes, 종류 open) — 끝내지 않고 지금처럼 기다린다."""
    make = _then_wake(asking_end_confirm)
    it = intent("wait", answer="yes", answer_kind="open")
    on = _outcome(make, it, answer_field=True)
    assert on == _outcome(make, it) and MSG_FINISH not in on[1] and on[0] == State.WAITING


def test_p13_after_wake_greeting_wait_need_yes_is_a_twenty_minute_wait():
    make = _then_wake(waiting_need_asked)
    it = intent("wait", wait_minutes=20, answer="yes", answer_kind="open")
    on = _outcome(make, it, answer_field=True)
    assert on == _outcome(make, it) and on[5] == 20 and MSG_FINISH not in on[1]


def test_p13_after_wake_greeting_cancel_question_yes_follows_the_intent():
    make = _then_wake(navigating_cancel_asked)
    for it in (intent("wait", answer="yes", answer_kind="open"), intent("unknown", answer="yes", answer_kind="open")):
        assert _outcome(make, it, answer_field=True) == _outcome(make, it)


def test_p13_after_wake_greeting_unclear_does_not_cover_ten_minutes():
    """"몇 분쯤 걸리실까요?" → "비카야" → "네?" → "10분"(대답 칸 unclear, 종류 open) — 10분 대기(2026-10-07 사용자 결정)."""
    make = _then_wake(asking_wait_time)
    it = intent("wait", wait_minutes=10, answer="unclear", answer_kind="open")
    on = _outcome(make, it, answer_field=True)
    assert on == _outcome(make, it) and on[0] == State.WAITING and on[5] == 10


@pytest.mark.parametrize("make", [confirming, asking_end_confirm, asking_wait_time, waiting_need_asked])
def test_p6_answer_without_kind_is_not_read(make):
    """옛 노드(질문 종류를 안 싣는다)·종류 빈 값 — 바꿔 읽지 않는다."""
    for answer, value in (("yes", ""), ("no", ""), ("minutes", "10"), ("destination", ELEV.id), ("unclear", "")):
        it = intent("unknown", answer=answer, answer_value=value)
        assert _outcome(make, it, answer_field=True) == _outcome(make, intent("unknown"), answer_field=True)


def test_p6_help_confirm_yes_departs_like_the_confirmed_destination():
    logic, t = help_requested_confirm_asked(answer_field=True)
    assert logic.state == State.PAUSED and logic.help_phase == "requested"
    it = intent("unknown", answer="yes", answer_kind="confirm", answer_options=(ELEV.id,))
    on = _outcome(help_requested_confirm_asked, it, answer_field=True)
    assert on == _outcome(help_requested_confirm_asked, intent(matched_destination_id=ELEV.id))
    assert on[0] == State.NAVIGATING and f"id='{ELEV.id}'" in on[1]


def test_p6_help_confirm_yes_without_options_stays_as_now():
    it = intent("unknown", answer="yes", answer_kind="confirm", answer_options=())
    on = _outcome(help_requested_confirm_asked, it, answer_field=True)
    assert on == _outcome(help_requested_confirm_asked, intent("affirm"), answer_field=True)
    assert on[0] == State.PAUSED and "Navigate(" not in on[1]


@pytest.mark.parametrize("make", [idle, navigating, paused, waiting])
def test_p6_confirm_kind_yes_elsewhere_is_only_affirm(make):
    it = intent("unknown", answer="yes", answer_kind="confirm", answer_options=(ELEV.id,))
    assert _outcome(make, it, answer_field=True) == _outcome(make, intent("affirm"), answer_field=True)


def test_p6_answer_is_read_before_the_wake_revive():
    logic, t = navigating_cancel_asked(answer_field=True, wake_revive=True)
    logic.on_wake_call(t)
    assert logic._wake_folded is not None
    acts = _voice(logic, t + 2.0, intent("unknown", answer="yes", answer_kind="yesno"))
    assert MSG_CANCEL_CONFIRM in _says(acts) and logic.cancel_confirm_pending


def test_p6_intent_from_answer_table():
    logic = MissionLogic(answer_field=True)
    base = intent("cancel", reply="답", ask_text="어느 화장실이요?")
    read = lambda a, v="": logic._intent_from_answer(  # noqa: E731
        intent("cancel", reply="답", ask_text="어느 화장실이요?", answer=a, answer_value=v,
               answer_kind="yesno", answer_options=(ELEV.id,)))
    assert read("yes").intent == "affirm" and read("no").intent == "deny" and read("unclear").intent == "unknown"
    nav = read("destination", ELEV.id)
    assert (nav.intent, nav.matched_destination_id, nav.need_confirm) == ("navigate", ELEV.id, True)
    assert read("choice", ELEV.id).intent == "navigate"
    assert read("destination").intent == "unknown"
    wait = read("minutes", "10")
    assert (wait.intent, wait.wait_minutes) == ("wait", 10)
    for bad in ("열", "0", "-1", ""):
        assert read("minutes", bad).intent == "unknown"
    for swapped in (read("yes"), nav, wait, read("unclear")):
        assert (swapped.answer, swapped.answer_value, swapped.answer_kind, swapped.answer_options) == ("", "", "", ())
        assert (swapped.reply, swapped.ask_text) == (base.reply, base.ask_text)
    assert read("other").intent == "cancel" and read("other").answer == "other"
    assert read("모르는값").intent == "cancel"


OFF_STATES = [idle, navigating, navigating_cancel_asked, navigating_help_asked, confirming, confirming_change, paused,
              asking, asking_wait_time, asking_end_confirm, waiting_release, moving_to_wait_spot, waiting,
              waiting_asked, waiting_need_asked, approaching, awaiting_user, estopped, help_requested_confirm_asked]
OFF_INTENTS = [intent("unknown"), intent("cancel"), intent("affirm"), intent("deny"), intent("wait", wait_minutes=5),
               intent(matched_destination_id=TOILET.id, need_confirm=True),
               intent(matched_destination_id=ELEV.id, need_confirm=True)]
OFF_ANSWERS = [("yes", ""), ("no", ""), ("destination", TOILET.id), ("choice", ELEV.id), ("minutes", "10"),
               ("minutes", "열"), ("unclear", ""), ("other", "")]
OFF_KINDS = [("", ()), ("confirm", (ELEV.id,)), ("yesno", ()), ("minutes", ()), ("open", ())]


def test_p6_answer_field_is_off_by_default():
    assert MissionLogic().answer_field is False


@pytest.mark.parametrize("make", OFF_STATES, ids=lambda f: f.__name__)
def test_p6_off_answer_changes_nothing(make):
    for it in OFF_INTENTS:
        plain = _outcome(make, it)
        for answer, value in OFF_ANSWERS:
            for kind, options in OFF_KINDS:
                filled = replace(it, answer=answer, answer_value=value, answer_kind=kind, answer_options=options)
                assert _outcome(make, filled) == plain, (it.intent, answer, value, kind)


def test_node_and_launch_expose_answer_field_off_by_default():
    node = (PKG / "vica_mission_manager" / "mission_manager_node.py").read_text(encoding="utf-8")
    launch = (PKG / "launch" / "mission_manager.launch.py").read_text(encoding="utf-8")
    assert 'self.declare_parameter("answer_field", False)' in node
    assert 'answer_field=bool(self.get_parameter("answer_field").value)' in node
    assert 'DeclareLaunchArgument("answer_field", default_value="false")' in launch
    assert '"answer_field": ParameterValue(' in launch


def test_node_carries_the_answered_question_kind_and_options_read_before_closing():
    node = (PKG / "vica_mission_manager" / "mission_manager_node.py").read_text(encoding="utf-8")
    start = node.index("    def _on_intent(")
    body = node[start: node.find("\n    def ", start + 1)]
    closing = body.index('self._close_question, "answer")')
    assert body.index("open_q.kind") < closing and body.index("open_q.options") < closing
    assert "answer_kind=answer_kind" in body and "answer_options=answer_options" in body


RESUMED_ROOM = say_destination(MSG_RESUMED, ROOM.name)


def _navs(acts):
    return [a.destination.id for a in acts if isinstance(a, Navigate)]


def _tick(logic, t):
    return logic.on_tick(t, NavStatus.NONE)


def _paused_help_requested(**kw):
    """안내 주행 중 도움 요청 → "네" → 요청을 보내고 일시정지(관리자 확인 전)."""
    logic, t = navigating_help_asked(**kw)
    logic.on_voice_intent(intent("affirm"), t, lookup, BOUNDS, True)
    return logic, t + 1.0


def test_p3_resume_is_asked_by_the_mission_and_never_starts_unasked():
    logic, t = paused(answer_field=True)
    assert _says(logic.on_voice_intent(intent("resume", need_confirm=False), t, lookup, BOUNDS, True)) == [MSG_RESUME_CONFIRM]
    assert logic.state == State.PAUSED
    logic.on_voice_intent(_ans(paused_resume_asked, "affirm", "yes"), t + 2, lookup, BOUNDS, True)
    assert logic.state == State.NAVIGATING


def test_resume_declined_stays_paused():
    logic, t = paused(answer_field=True)
    logic.on_voice_intent(intent("resume", need_confirm=True), t, lookup, BOUNDS, True)
    assert _says(logic.on_voice_intent(intent("deny"), t + 2, lookup, BOUNDS, True)) == [MSG_PAUSE_KEPT]
    assert logic.state == State.PAUSED


def test_p3_resume_question_is_the_missions_yesno_question():
    logic, t = paused(answer_field=True)
    acts = _voice(logic, t, intent("resume", need_confirm=True))
    (asked,) = [a for a in acts if isinstance(a, Say)]
    assert (asked.text, asked.kind, asked.expects_reply) == (MSG_RESUME_CONFIRM, "yesno", True)
    assert _navs(acts) == [] and logic.paused_destination.id == ROOM.id
    assert MSG_RESUME_CONFIRM == "다시 출발할까요?"


@pytest.mark.parametrize("kind", ["resume", "unknown"])
def test_p6_resume_question_answer_yes_departs(kind):
    logic, t = paused_resume_asked()
    acts = _voice(logic, t, _ans(paused_resume_asked, kind, "yes"))
    assert logic.state == State.NAVIGATING
    assert _navs(acts) == [ROOM.id] and _says(acts) == [RESUMED_ROOM]


@pytest.mark.parametrize("kind", ["unknown", "deny"])
def test_p6_resume_question_answer_no_keeps_the_pause(kind):
    logic, t = paused_resume_asked()
    assert _says(_voice(logic, t, _ans(paused_resume_asked, kind, "no"))) == [MSG_PAUSE_KEPT]
    assert logic.state == State.PAUSED and logic.paused_destination.id == ROOM.id
    assert _voice(logic, t + 1, intent("affirm")) == [] and logic.state == State.PAUSED


def test_p3_resume_said_again_while_asked():
    logic, t = paused_resume_asked()
    assert _navs(_voice(logic, t, intent("resume", need_confirm=False))) == [ROOM.id]
    logic, t = paused_resume_asked()
    assert _says(_voice(logic, t, intent("resume", need_confirm=True))) == [MSG_RESUME_CONFIRM]
    assert logic.state == State.PAUSED


def test_p8_resume_question_misheard_asks_once_more_then_closes_quietly():
    logic, t = paused_resume_asked()
    assert _says(_voice(logic, t, intent("unknown"))) == [MSG_RESUME_CONFIRM]
    assert _says(_voice(logic, t + 1, intent("unknown"))) == []
    assert _says(_tick(logic, t + 28.9)) == [] and logic._resay_open_question(t + 28.9) is not None
    assert _says(_tick(logic, t + 29.0)) == []
    assert logic._resay_open_question(t + 29.0) is None and logic.state == State.PAUSED
    assert _voice(logic, t + 30, intent("affirm")) == [] and logic.state == State.PAUSED


def test_p8_resume_question_unanswered_asks_once_more_then_closes_quietly():
    logic, t = paused_resume_asked()
    asked = t - 1.0
    assert _says(_tick(logic, asked + 14.9)) == []
    assert _says(_tick(logic, asked + 15.0)) == [MSG_RESUME_CONFIRM]
    assert _says(_tick(logic, asked + 29.9)) == [] and logic._resay_open_question(asked + 29.9) is not None
    assert _says(_tick(logic, asked + 30.0)) == []
    assert logic._resay_open_question(asked + 30.0) is None and logic.state == State.PAUSED


def test_p8_resume_question_three_steps_then_closes_without_staff_help():
    logic, t = paused_resume_asked(reask_ladder=True)
    first = [a for a in _voice(logic, t, intent("unknown")) if isinstance(a, Say)]
    assert [a.text for a in first] == [MSG_REASK_RETRY]
    assert (first[0].question_text, first[0].kind, first[0].step) == (MSG_RESUME_CONFIRM, "yesno", 1)
    assert _says(_voice(logic, t + 1, intent("unknown"))) == [MSG_REASK_LOUDER]
    assert _says(_voice(logic, t + 2, intent("unknown"))) == []
    assert logic.help_phase == "none" and logic.state == State.PAUSED
    assert logic._resay_open_question(t + 2) is None


def test_p8_resume_question_silence_asks_once_then_closes_like_off():
    """세 단계(reask_ladder)여도 대답 없음은 꺼짐과 같다 — 15 s 에 같은 질문, 30 s 에 말없이 닫기(원칙 8, Task N4)."""
    logic, t = paused_resume_asked(reask_ladder=True)
    asked = t - 1.0
    assert _says(_tick(logic, asked + 15.0)) == [MSG_RESUME_CONFIRM]
    assert _says(_tick(logic, asked + 29.9)) == [] and logic._resay_open_question(asked + 29.9) is not None
    assert _says(_tick(logic, asked + 30.0)) == []
    assert logic._resay_open_question(asked + 30.0) is None
    assert logic.help_phase == "none" and logic.state == State.PAUSED


def test_p9_wake_only_does_not_climb_the_resume_ladder():
    logic, t = paused_resume_asked(reask_ladder=True)
    wake_only = intent("unknown", reply=MSG_WAKE_GREETING)
    _voice(logic, t, wake_only)
    assert logic._resume_ladder.step == 0
    assert _says(_voice(logic, t + 1, intent("unknown"))) == [MSG_REASK_RETRY]


def test_p13_wake_folds_the_resume_question_and_yes_asks_again():
    logic, t = paused_resume_asked(wake_revive=True)
    assert _says(logic.on_wake_call(t)) == [MSG_WAKE_GREETING]
    assert logic._wake_folded == ("resume", "", t)
    assert logic._resay_open_question(t) is None
    yes = intent("affirm", answer="yes", answer_kind="open")
    acts = _voice(logic, t + 2, yes)
    assert _says(acts) == [MSG_RESUME_CONFIRM] and _navs(acts) == [] and logic.state == State.PAUSED
    yes = _ans(paused_resume_asked, "affirm", "yes")
    assert _navs(_voice(logic, t + 3, yes)) == [ROOM.id]


def test_p13_without_wake_revive_the_call_leaves_the_resume_question_open():
    logic, t = paused_resume_asked()
    logic.on_wake_call(t)
    assert logic._wake_folded is None and logic._resay_open_question(t) is not None
    acts = _voice(logic, t + 2, intent("affirm", answer="yes", answer_kind="open"))
    assert _says(acts) == [MSG_RESUME_CONFIRM] and _navs(acts) == [] and logic.state == State.PAUSED
    assert _navs(_voice(logic, t + 3, _ans(paused_resume_asked, "affirm", "yes"))) == [ROOM.id]


def test_p14_dismiss_during_the_resume_question_is_a_no():
    logic, t = paused_resume_asked()
    assert _says(_voice(logic, t, intent("dismiss"))) == [MSG_PAUSE_KEPT]
    assert logic.state == State.PAUSED
    logic, t = paused(answer_field=True)
    assert _says(_voice(logic, t, intent("dismiss"))) == [MSG_CANCEL_CONFIRM]


def test_p3_other_requests_close_the_resume_question():
    logic, t = paused_resume_asked()
    _voice(logic, t, intent(matched_destination_id=ELEV.id, need_confirm=True))
    assert logic.state == State.CONFIRMING
    assert _says(_voice(logic, t + 1, intent("deny"))) == [MSG_PAUSE_KEPT]
    assert logic.state == State.PAUSED and _voice(logic, t + 2, intent("affirm")) == []
    logic, t = paused_resume_asked()
    assert _says(_voice(logic, t, intent("cancel"))) == [MSG_CANCEL_CONFIRM]
    acts = _voice(logic, t + 1, intent("affirm"))
    assert logic.state == State.IDLE and _navs(acts) == []
    logic, t = paused_resume_asked()
    assert _voice(logic, t, intent("question")) == []
    assert _navs(_voice(logic, t + 1, _ans(paused_resume_asked, "affirm", "yes"))) == [ROOM.id]


@pytest.mark.parametrize("kind,ask", [("question", "화장실은 2층에 있어요. 안내해 드릴까요?"),
                                      ("clarify", "어느 화장실로 안내해 드릴까요?")], ids=["question", "clarify"])
def test_p6_handed_question_replaces_the_resume_question(kind, ask):
    logic, t = paused_resume_asked()
    assert _says(_voice(logic, t, intent(kind, ask_text=ask, ask_kind="open"))) == [ask]
    off, t_off = paused()
    acts = _voice(logic, t + 1, intent("affirm", answer="yes", answer_kind="open"))
    assert acts == _voice(off, t_off, intent("affirm")) == [] and logic.state == State.PAUSED
    assert MSG_RESUME_CONFIRM not in _says(_tick(logic, t + 20))
    assert _says(_voice(logic, t + 21, intent("resume", need_confirm=True))) == [MSG_RESUME_CONFIRM]


@pytest.mark.parametrize("flags", [{}, {"reask_ladder": True}], ids=["ladders_off", "reask_ladder"])
def test_p13_wake_greeting_handed_over_keeps_the_resume_question(flags):
    logic, t = paused_resume_asked(**flags)
    _voice(logic, t, intent("unknown", ask_text=MSG_WAKE_GREETING, ask_kind="open"))
    assert logic._resay_open_question(t) is not None
    assert _navs(_voice(logic, t + 1, intent("affirm", answer="yes", answer_kind="open"))) == []
    assert logic.state == State.PAUSED
    assert _navs(_voice(logic, t + 2, _ans(paused_resume_asked, "affirm", "yes"))) == [ROOM.id]


def test_p8_handed_retry_after_the_old_ladder_keeps_the_resume_question():
    retry = intent("unknown", ask_text=MSG_REASK_RETRY, ask_kind="open")
    logic, t = paused_resume_asked()
    assert _says(_voice(logic, t, retry)) == [MSG_RESUME_CONFIRM]
    assert _says(_voice(logic, t + 1, retry)) == [MSG_REASK_RETRY]
    assert logic._resay_open_question(t + 1) is not None
    assert _navs(_voice(logic, t + 2, intent("affirm", answer="yes", answer_kind="open"))) == []
    assert logic.state == State.PAUSED and logic._resay_open_question(t + 2) is not None
    assert _navs(_voice(logic, t + 3, intent("resume", need_confirm=False))) == [ROOM.id]


@pytest.mark.parametrize("flags", [{}, {"wake_revive": True}], ids=["ladders_off", "wake_revive"])
def test_p6_real_question_handed_with_unknown_closes_the_resume_question(flags):
    ask = "화장실은 2층에 있어요. 안내해 드릴까요?"
    logic, t = paused_resume_asked(**flags)
    assert _says(_voice(logic, t, intent("unknown", ask_text=ask, ask_kind="open"))) == [MSG_RESUME_CONFIRM]
    assert _says(_voice(logic, t + 1, intent("unknown", ask_text=ask, ask_kind="open"))) == [ask]
    acts = _voice(logic, t + 2, intent("affirm", answer="yes", answer_kind="open"))
    assert acts == [] and logic.state == State.PAUSED


def test_p6_real_question_handed_with_an_unclear_answer_closes_the_resume_question():
    ask = "어느 화장실이요?"
    logic, t = paused_resume_asked()
    unclear = _ans(paused_resume_asked, "clarify", "unclear", ask_text=ask, ask_kind="open")
    assert _says(_voice(logic, t, unclear)) == [MSG_RESUME_CONFIRM]
    assert _says(_voice(logic, t + 1, unclear)) == [ask]
    assert _voice(logic, t + 2, intent("affirm", answer="yes", answer_kind="open")) == []
    assert logic.state == State.PAUSED


def test_p13_wake_call_folds_first_then_handed_wake_greeting_still_asks_again():
    logic, t = paused_resume_asked(wake_revive=True)
    logic.on_wake_call(t)
    _voice(logic, t + 1, intent("unknown", ask_text=MSG_WAKE_GREETING, ask_kind="open"))
    acts = _voice(logic, t + 2, intent("affirm", answer="yes", answer_kind="open"))
    assert _says(acts) == [MSG_RESUME_CONFIRM] and _navs(acts) == [] and logic.state == State.PAUSED


def test_p6_answer_without_a_handed_question_keeps_the_resume_question():
    logic, t = paused_resume_asked()
    assert _voice(logic, t, intent("question", reply="화장실은 2층에 있어요.")) == []
    assert _navs(_voice(logic, t + 1, _ans(paused_resume_asked, "affirm", "yes"))) == [ROOM.id]


def test_p3_resume_question_replaces_an_open_cancel_question():
    logic, t = paused(answer_field=True)
    assert _says(_voice(logic, t, intent("cancel"))) == [MSG_CANCEL_CONFIRM]
    assert _says(_voice(logic, t + 1, intent("resume", need_confirm=True))) == [MSG_RESUME_CONFIRM]
    assert logic.cancel_confirm_pending is False
    assert _navs(_voice(logic, t + 2, _ans(paused_resume_asked, "affirm", "yes"))) == [ROOM.id]


def test_p3_resume_question_during_a_help_request():
    logic, t = _paused_help_requested(answer_field=True)
    assert logic.state == State.PAUSED and logic.help_phase == "requested"
    _voice(logic, t, intent("cancel"))
    assert logic.help_cancel_pending
    assert _says(_voice(logic, t + 1, intent("resume", need_confirm=True))) == [MSG_RESUME_CONFIRM]
    assert not logic.help_cancel_pending
    acts = _voice(logic, t + 2, _ans(paused_resume_asked, "affirm", "yes"))
    assert _navs(acts) == [ROOM.id] and logic.help_phase == "none"
    assert [a.event for a in acts if isinstance(a, HelpEvent)] == ["help_canceled"]


def test_p3_after_the_staff_acknowledges_the_yes_does_not_start():
    logic, t = _paused_help_requested(answer_field=True)
    _voice(logic, t, intent("resume", need_confirm=True))
    acts, ok = logic.on_help_ack(logic.help_id, t + 1)
    assert ok and _says(acts) == [MSG_HELP_COMING]
    acts = _voice(logic, t + 2, intent("affirm"))
    assert _navs(acts) == [] and logic.state == State.PAUSED
    assert _says(_voice(logic, t + 3, intent("resume", need_confirm=True))) == [MSG_HELP_COMING]


def test_p3_resume_when_nav_is_not_ready_says_why_and_asks_nothing():
    off, t_off = paused()
    reject = _says(off.on_voice_intent(intent("resume"), t_off, lookup, BOUNDS, False))
    assert reject and MSG_RESUME_CONFIRM not in reject
    logic, t = paused(answer_field=True)
    assert _says(logic.on_voice_intent(intent("resume", need_confirm=True), t, lookup, BOUNDS, False)) == reject
    assert logic._resay_open_question(t) is None
    logic, t = paused_resume_asked()
    acts = logic.on_voice_intent(_ans(paused_resume_asked, "affirm", "yes"), t, lookup, BOUNDS, False)
    assert _says(acts) == reject and _navs(acts) == [] and logic.state == State.PAUSED
    assert logic._resay_open_question(t) is None


def test_p3_estop_drops_the_resume_question():
    logic, t = paused_resume_asked()
    logic.on_estop(True, t)
    assert logic.state == State.ESTOPPED and logic._resay_open_question(t) is None
    assert MSG_RESUME_CONFIRM not in _says(_tick(logic, t + 15))


def test_p3_off_paused_resume_and_answers_are_as_now():
    logic, t = paused()
    acts = _voice(logic, t, intent("resume"))
    assert logic.state == State.NAVIGATING and _navs(acts) == [ROOM.id] and _says(acts) == [RESUMED_ROOM]
    for kind in ("affirm", "deny"):
        logic, t = paused()
        assert _voice(logic, t, intent(kind)) == [] and logic.state == State.PAUSED
    logic, t = paused()
    assert _says(_voice(logic, t, intent("dismiss"))) == [MSG_CANCEL_CONFIRM]


def test_p3_off_resume_proposal_does_not_start():
    """음성만 켬(VICA_ANSWER=on · answer_field 꺼짐) — 음성은 재개 제안을 쥐지 않고 보내는데 미션은 묻지 않는다. 지금처럼"""
    logic, t = paused()
    assert _voice(logic, t, intent("resume", need_confirm=True)) == []
    assert logic.state == State.PAUSED and logic.paused_destination.id == ROOM.id
    assert _navs(_voice(logic, t + 1, intent("resume"))) == [ROOM.id]
    logic, t = _paused_help_requested()
    assert _voice(logic, t, intent("resume", need_confirm=True)) == [] and logic.state == State.PAUSED
