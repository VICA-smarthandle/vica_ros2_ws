"""대답 칸으로 처리(원칙 6 · 3, Task 3.3) — 파라미터 answer_field 가 켜졌을 때만.

음성은 로봇이 방금 한 질문의 대답이면 대답 칸(answer·answer_value)을 함께 채운다. 미션 노드는 메시지의 question_id 가 지금
열린 질문과 같을 때만 IntentData 로 넘기고(3.2), 그 질문의 종류·후보 목적지 id(answer_kind·answer_options)를 함께 싣는다.
켜지면 미션은 뜻을 아는 종류(confirm·yesno·yesno_inverted·minutes·choice)의 대답일 때만 그 값을 의도보다 먼저 본다:
yes·no = affirm·deny, destination·choice = 그 목적지 제안(확인을 거친다), minutes = 그 분만큼 대기, unclear = unknown,
other = 의도대로. open(호출 응답 "네?"·음성이 넘긴 열린 질문)이나 종류가 비면 받은 의도대로다(원칙 13).
꺼져 있으면 대답 칸을 보지 않는다(지금 그대로).

실제로 있었던 일: "여기까지 안내를 마칠까요?"에 "아니"가 취소로 읽혀 안내가 끝남 · "몇 분쯤 걸리실까요?"에 "십 분"이
목적지로 읽힘 · 확인 질문 중 같은 목적지 이름이 다시 들리면 승낙으로 보고 출발(2026-09-01 규칙).
"""
from dataclasses import replace
from pathlib import Path

import pytest

import vica_mission_manager.mission_logic as ml
from reaction_states import (BOUNDS, ELEV, TOILET, approaching, asking, asking_end_confirm, asking_wait_time,
                             awaiting_user, confirming, confirming_change, estopped, help_requested_confirm_asked, idle,
                             intent, lookup, moving_to_wait_spot, navigating, navigating_cancel_asked,
                             navigating_help_asked, paused, waiting, waiting_asked, waiting_need_asked, waiting_release)
from vica_mission_manager.mission_logic import (
    MSG_ASK_WAIT_TIME,
    MSG_CANCEL_CONFIRM,
    MSG_CANCEL_KEPT,
    MSG_CONFIRM_TIMEOUT,
    MSG_FINISH,
    MSG_WAIT_CONFIRM,
    MissionLogic,
    Navigate,
    Say,
    State,
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


# ---- 브리프 시험 ---------------------------------------------------------------------------------------
def test_p6_confirm_yes_departs_and_no_folds():
    logic, t = confirming(answer_field=True)
    acts = _voice(logic, t, _ans(confirming, "unknown", "yes"))
    assert logic.state == State.NAVIGATING
    assert [a.destination.id for a in acts if isinstance(a, Navigate)] == [TOILET.id]
    logic, t = confirming(answer_field=True)
    assert MSG_CONFIRM_TIMEOUT in _says(_voice(logic, t, _ans(confirming, "navigate", "no")))


def test_p3_same_place_named_again_asks_again_instead_of_departing():
    logic, t = confirming(answer_field=True)          # 화장실 확인 중
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
    logic3, t3 = confirming()                          # 꺼짐 — 대답 칸을 보지 않는다
    _voice(logic3, t3, _ans(confirming, "unknown", "yes"))
    assert logic3.state == State.CONFIRMING


# ---- 확인 질문 — 같은 곳 다시 말하기(원칙 3 · 6, 09-01 규칙을 쓰지 않음) ---------------------------------------
def test_p3_same_place_answer_reasks_the_same_confirm_question():
    logic, t = confirming(answer_field=True)
    prompt = logic._confirm_prompt
    acts = _voice(logic, t, _ans(confirming, "navigate", "destination", TOILET.id,
                                 matched_destination_id=TOILET.id, need_confirm=True))
    asked = [a for a in acts if isinstance(a, Say) and a.expects_reply]
    assert [a.text for a in asked] == [prompt] and asked[0].kind == "confirm"
    assert logic._confirming_dest_id == TOILET.id


def test_p3_without_answer_the_0901_rule_still_departs():
    # 대답 칸 없이 온 같은 목적지(늦은 대답·음성이 칸을 안 채움)는 지금처럼 승낙 — 바로 앞 요청이 대답 칸이었어도.
    logic, t = confirming(answer_field=True)
    _voice(logic, t, _ans(confirming, "navigate", "destination", TOILET.id))
    acts = _voice(logic, t + 1.0, intent(matched_destination_id=TOILET.id, need_confirm=True))
    assert logic.state == State.NAVIGATING and any(isinstance(a, Navigate) for a in acts)


def test_p3_off_same_place_answer_departs_as_now():
    logic, t = confirming()
    _voice(logic, t, _ans(confirming, "navigate", "destination", TOILET.id,
                          matched_destination_id=TOILET.id, need_confirm=True))
    assert logic.state == State.NAVIGATING


def test_p6_choice_of_another_place_asks_about_that_place():
    logic, t = confirming(answer_field=True)
    acts = _voice(logic, t, _ans(confirming, "unknown", "choice", ELEV.id))
    assert logic.state == State.CONFIRMING and logic._confirming_dest_id == ELEV.id
    assert not any(isinstance(a, Navigate) for a in acts)
    assert [a.options for a in acts if isinstance(a, Say) and a.expects_reply] == [((ELEV.id, ELEV.name),)]


# ---- 실제로 있었던 일 ------------------------------------------------------------------------------------
def test_p6_end_question_no_is_a_no_even_if_read_as_cancel():
    """"여기까지 안내를 마칠까요?"에 "아니" — 의도는 cancel 로 왔어도 거절이다(안내가 끝나지 않는다)."""
    it = _ans(asking_end_confirm, "cancel", "no")
    assert it.answer_kind == "yesno"
    off = _outcome(asking_end_confirm, it)
    assert MSG_FINISH in off[1]                       # 꺼짐 — 지금처럼 취소로 읽혀 안내가 끝난다
    on = _outcome(asking_end_confirm, it, answer_field=True)
    assert MSG_FINISH not in on[1] and on[0] == State.WAITING
    assert on == _outcome(asking_end_confirm, intent("deny"), answer_field=True)


def test_p6_ten_minutes_read_as_destination_is_a_ten_minute_wait():
    """"몇 분쯤 걸리실까요?"에 "십 분" — 의도는 목적지 제안으로 왔어도 10분 대기다."""
    it = _ans(asking_wait_time, "navigate", "minutes", "10", matched_destination_id=ELEV.id, need_confirm=True)
    assert it.answer_kind == "minutes"
    off = _outcome(asking_wait_time, it)
    assert off[0] == State.CONFIRMING                 # 꺼짐 — 지금처럼 목적지 제안으로 읽힌다
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


# ---- 쓸 수 없는 값은 unknown -----------------------------------------------------------------------------
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


# ---- 바꿔 읽은 의도는 그 의도가 받는 판정을 그대로 받는다(화자 판정) ---------------------------------------------
def test_p6_swapped_intent_still_goes_through_the_speaker_gate():
    # 대기 중 "20분" — 판정이 없으면 20분을 다시 세고, 본인이 아니면(fail) 다시 말해 달라고 한다. 바꿔 읽은 wait 도 같은 판정.
    # 대기 중 질문이 열려 있지 않아 노드 기록도 없다 — 시간 질문의 대답 칸 꼴을 그대로 실어 판정 순서만 본다.
    swapped = intent("unknown", answer="minutes", answer_value="20", answer_kind="minutes",
                     speaker_verdict="fail", speaker_score=0.1)
    plain = intent("wait", wait_minutes=20, speaker_verdict="fail", speaker_score=0.1)
    gated = _outcome(waiting, swapped, answer_field=True, speaker_gate=True)
    assert gated == _outcome(waiting, plain, answer_field=True, speaker_gate=True)
    assert gated != _outcome(waiting, swapped, answer_field=True)          # 판정을 건너뛰면 20분 대기가 된다
    assert gated[5] != 20


def test_p6_understood_answer_restarts_the_reask_ladder():
    # 원칙 8 — 의도는 unknown 이어도 대답 칸이 yes 면 알아들은 말이다: 마무리(되묻기 정리)도 바꿔 읽은 의도로 해 사다리가
    # 처음부터 다시 선다. 꺼져 있으면 지금처럼 못 알아들은 말로 세어 ② 로 오른다.
    def run(on):
        logic, t = idle(answer_field=on, reask_ladder=True)
        sent = (intent("unknown"), intent("unknown", answer="yes", answer_kind="yesno"), intent("unknown"))
        return [_says(_voice(logic, t + i, it)) for i, it in enumerate(sent)]
    first = [ml.MSG_REASK_RETRY]
    assert run(True) == [first, [], first]
    assert run(False) == [first, [ml.MSG_REASK_LOUDER], []]


# ---- 수정 1차: 뜻을 아는 종류의 질문에만 — "네?"(open) 뒤의 말은 받은 의도대로(원칙 13) ----------------------------
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


# ---- 수정 1차: CONFIRMING 없이 물은 확인 질문(도움 요청 뒤)의 yes = 그 목적지 확정 ---------------------------------
def test_p6_help_confirm_yes_departs_like_the_confirmed_destination():
    logic, t = help_requested_confirm_asked(answer_field=True)
    assert logic.state == State.PAUSED and logic.help_phase == "requested"
    it = intent("unknown", answer="yes", answer_kind="confirm", answer_options=(ELEV.id,))
    on = _outcome(help_requested_confirm_asked, it, answer_field=True)
    assert on == _outcome(help_requested_confirm_asked, intent(matched_destination_id=ELEV.id))   # 꺼짐의 확정
    assert on[0] == State.NAVIGATING and f"id='{ELEV.id}'" in on[1]


def test_p6_help_confirm_yes_without_options_stays_as_now():
    it = intent("unknown", answer="yes", answer_kind="confirm", answer_options=())
    on = _outcome(help_requested_confirm_asked, it, answer_field=True)
    assert on == _outcome(help_requested_confirm_asked, intent("affirm"), answer_field=True)
    assert on[0] == State.PAUSED and "Navigate(" not in on[1]


@pytest.mark.parametrize("make", [idle, navigating, paused, waiting])
def test_p6_confirm_kind_yes_elsewhere_is_only_affirm(make):
    # 확인 질문 기록이 상태보다 늦게 닫히는 틈(노드는 다음 시계에 닫는다) — 미션이 그 질문을 기다리지 않으면 확정하지 않는다.
    it = intent("unknown", answer="yes", answer_kind="confirm", answer_options=(ELEV.id,))
    assert _outcome(make, it, answer_field=True) == _outcome(make, intent("affirm"), answer_field=True)


# ---- 수정 1차: 바꿔 읽기가 "비카야" 뒤 되살리기보다 먼저(요구 7) -----------------------------------------------
def test_p6_answer_is_read_before_the_wake_revive():
    # 호출이 "안내를 취소할까요?"를 접은 뒤 yes 대답 칸 — 바꿔 읽은 affirm 이 되살리기를 만나 그 질문을 다시 한다.
    # 되살리기가 먼저면 원래 의도(unknown)가 접은 것을 지워 아무것도 묻지 않는다.
    logic, t = navigating_cancel_asked(answer_field=True, wake_revive=True)
    logic.on_wake_call(t)
    assert logic._wake_folded is not None
    acts = _voice(logic, t + 2.0, intent("unknown", answer="yes", answer_kind="yesno"))
    assert MSG_CANCEL_CONFIRM in _says(acts) and logic.cancel_confirm_pending


# ---- 바꿔 읽는 표 ---------------------------------------------------------------------------------------------
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
        # 다시 바꿔 읽지 않게 대답 칸을 모두 비운다.
        assert (swapped.answer, swapped.answer_value, swapped.answer_kind, swapped.answer_options) == ("", "", "", ())
        assert (swapped.reply, swapped.ask_text) == (base.reply, base.ask_text)
    assert read("other").intent == "cancel" and read("other").answer == "other"
    assert read("모르는값").intent == "cancel"


# ---- 꺼짐(기본값) — 대답 칸이 채워져 와도 지금과 같다 -------------------------------------------------------------
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
