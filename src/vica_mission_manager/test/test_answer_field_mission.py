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


def test_p3_without_answer_after_an_answer_still_asks_again():
    # 대답 칸 없이 온 같은 목적지(늦은 대답·음성이 칸을 안 채움)도 승낙이 아니다(Task 3.3b) — 바로 앞 요청이 대답 칸이었어도.
    logic, t = confirming(answer_field=True)
    _voice(logic, t, _ans(confirming, "navigate", "destination", TOILET.id))
    acts = _voice(logic, t + 1.0, intent(matched_destination_id=TOILET.id, need_confirm=True))
    assert logic.state == State.CONFIRMING and not any(isinstance(a, Navigate) for a in acts)


def test_p3_off_same_place_answer_departs_as_now():
    logic, t = confirming()
    _voice(logic, t, _ans(confirming, "navigate", "destination", TOILET.id,
                          matched_destination_id=TOILET.id, need_confirm=True))
    assert logic.state == State.NAVIGATING


# ---- 확인 질문 — 대답 칸 없는 같은 곳 제안도 승낙이 아니다(원칙 3, Task 3.3b) ------------------------------------------
# 2026-11-21 17:32 실기: "화장실로 안내해드릴까요?"에 "거기가 어디야" — 음성이 같은 곳 제안(navigate, need_confirm, 대답 칸 빈
# 값)으로 읽었고 09-01 규칙이 그것을 승낙으로 받아 출발했다. 켜짐의 승낙은 affirm · 대답 칸 yes · 확정(need_confirm=False)뿐이다.
def _same_place(**kw):
    return intent(matched_destination_id=TOILET.id, need_confirm=True, **kw)


def test_p3_same_place_proposal_without_answer_asks_again():   # _navs 는 아래(재개 질문 절)에 있다
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
    # "그래" → 그 목적지의 확정(need_confirm=False) — 지금 그대로 출발(11-21 실기에서도 정상으로 여러 번).
    logic, t = confirming(answer_field=True)
    acts = _voice(logic, t, intent(matched_destination_id=TOILET.id, need_confirm=False))
    assert _navs(acts) == [TOILET.id] and logic.state == State.NAVIGATING


def test_p3_answer_yes_on_a_same_place_proposal_departs():
    # 대답 칸 yes 가 의도(같은 곳 제안)보다 먼저다 — 승낙.
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
    # 같은 함수·같은 결과 — 말·시한·되묻기 사다리 모두 대답 칸 destination 으로 온 같은 곳과 같다(사다리는 처음부터).
    def run(it):
        _FakeUuid.n = 0
        logic, t = confirming(answer_field=True, reask_ladder=True)
        out = [_voice(logic, t, intent("unknown"))]              # 사다리 한 단 오름
        out.append(_voice(logic, t + 2.0, it))
        out.append(_voice(logic, t + 3.0, intent("unknown")))    # 처음부터 다시 오르는가
        return (repr(out), logic.state, logic._confirm_deadline, logic._confirm_reask_at, logic._confirm_ladder.step)
    plain = run(_same_place())
    assert plain == run(_ans(confirming, "navigate", "destination", TOILET.id,
                             matched_destination_id=TOILET.id, need_confirm=True))
    assert plain[1] == State.CONFIRMING and "Navigate(" not in plain[0]


def test_p3_same_place_proposal_in_the_change_question_asks_again():
    # 주행 중 바꾸기 질문("화장실로 안내해드릴까요?", 409호로 가다 멈춤)도 같다 — 대답 칸 destination 이 이미 다시 묻는 갈래다.
    logic, t = confirming_change(answer_field=True)
    prompt = logic._confirm_prompt
    acts = _voice(logic, t, _same_place())
    assert _navs(acts) == [] and logic.state == State.CONFIRMING
    assert _says(acts) == [prompt] and logic._change_from is ROOM
    acts = _voice(logic, t + 1.0, intent("affirm"))
    assert _navs(acts) == [TOILET.id]


def test_p13_revived_confirm_then_same_place_proposal_asks_again():
    # 호출로 접은 확인 질문을 "응"으로 되살린 뒤(다시 묻기) 같은 곳 제안이 또 와도 다시 묻는다.
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


# ---- Task 3.4: "다시 출발할까요?"를 미션이 묻는다(원칙 3 · 6, 11-18 설계 4.4·4.5) ------------------------------------
# 켜지면 일시정지 중의 재개(제안이든 확정이든)는 먼저 묻고, 그 질문의 네 = 재출발 · 아니요 = MSG_PAUSE_KEPT(일시정지 유지).
# 되묻기·대답 없음은 취소 확인과 같은 방식(사다리 하나, ③ 직원 도움 없음). 꺼지면 지금 그대로.
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
    logic.on_voice_intent(_ans(paused_resume_asked, "affirm", "yes"), t + 2, lookup, BOUNDS, True)   # 원칙 3 — 대답 칸 yes
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
    assert MSG_RESUME_CONFIRM == "다시 출발할까요?"   # 음성 replies.RESUME_CONFIRM 과 같은 글자(음성 test_mission_phrases 가 대조)


@pytest.mark.parametrize("kind", ["resume", "unknown"])
def test_p6_resume_question_answer_yes_departs(kind):
    # 실제 꼴: 음성 승낙 단축이 확정 resume 에 대답 칸 yes(질문 종류 yesno)를 싣는다 — 3.3 보고 6절에서 재개하지 않던 자리.
    logic, t = paused_resume_asked()
    acts = _voice(logic, t, _ans(paused_resume_asked, kind, "yes"))
    assert logic.state == State.NAVIGATING
    assert _navs(acts) == [ROOM.id] and _says(acts) == [RESUMED_ROOM]


@pytest.mark.parametrize("kind", ["unknown", "deny"])
def test_p6_resume_question_answer_no_keeps_the_pause(kind):
    logic, t = paused_resume_asked()
    assert _says(_voice(logic, t, _ans(paused_resume_asked, kind, "no"))) == [MSG_PAUSE_KEPT]
    assert logic.state == State.PAUSED and logic.paused_destination.id == ROOM.id
    # 질문이 닫혔다 — 뒤늦은 "네"로 출발하지 않는다.
    assert _voice(logic, t + 1, intent("affirm")) == [] and logic.state == State.PAUSED


def test_p3_resume_said_again_while_asked():
    logic, t = paused_resume_asked()               # 확정(음성이 "다시 가자"를 그 질문의 승낙으로 읽음) = 재출발
    assert _navs(_voice(logic, t, intent("resume", need_confirm=False))) == [ROOM.id]
    logic, t = paused_resume_asked()               # 제안 = 같은 질문을 다시(확인 질문 중 "다시 가자"와 같은 방식)
    assert _says(_voice(logic, t, intent("resume", need_confirm=True))) == [MSG_RESUME_CONFIRM]
    assert logic.state == State.PAUSED


def test_p8_resume_question_misheard_asks_once_more_then_closes_quietly():
    logic, t = paused_resume_asked()               # 물은 시각 t-1 → 시한 t+29
    assert _says(_voice(logic, t, intent("unknown"))) == [MSG_RESUME_CONFIRM]
    assert _says(_voice(logic, t + 1, intent("unknown"))) == []
    assert _says(_tick(logic, t + 28.9)) == [] and logic._resay_open_question(t + 28.9) is not None
    assert _says(_tick(logic, t + 29.0)) == []     # 시한 — 말없이 닫는다
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
    assert _says(_voice(logic, t + 2, intent("unknown"))) == []      # 끝 — 일시정지 중이라 ③(직원 도움)은 없다
    assert logic.help_phase == "none" and logic.state == State.PAUSED
    assert logic._resay_open_question(t + 2) is None


def test_p8_resume_question_silence_climbs_the_three_steps():
    logic, t = paused_resume_asked(reask_ladder=True)
    asked = t - 1.0
    assert _says(_tick(logic, asked + 15.0)) == [MSG_REASK_RETRY]
    assert _says(_tick(logic, asked + 29.9)) == []
    assert _says(_tick(logic, asked + 30.0)) == [MSG_REASK_LOUDER]
    assert _says(_tick(logic, asked + 45.0)) == []
    assert logic._resay_open_question(asked + 45.0) is None
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
    assert logic._resay_open_question(t) is None             # 조용히 접었다
    yes = intent("affirm", answer="yes", answer_kind="open")  # "네?" 뒤의 "응" — 열린 질문의 대답이라 바꿔 읽지 않는다
    acts = _voice(logic, t + 2, yes)
    assert _says(acts) == [MSG_RESUME_CONFIRM] and _navs(acts) == [] and logic.state == State.PAUSED
    yes = _ans(paused_resume_asked, "affirm", "yes")
    assert _navs(_voice(logic, t + 3, yes)) == [ROOM.id]   # 다시 물은 질문의 "네"(대답 칸 yes)는 출발


def test_p13_without_wake_revive_the_call_leaves_the_resume_question_open():
    # 꺼짐 — 취소·도움 확인과 같이 호출은 재개 질문을 건드리지 않는다. "네?" 뒤의 "응"(open — 맨 affirm)은 그 질문의 승낙이 아니라
    # 다시 묻기다(원칙 3, 전체 검토 C2). 다시 물은 질문의 대답 칸 yes 가 출발이다.
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
    logic, t = paused(answer_field=True)               # 질문이 없으면 지금처럼 — 안내 중의 "됐어"는 취소 확인
    assert _says(_voice(logic, t, intent("dismiss"))) == [MSG_CANCEL_CONFIRM]


def test_p3_other_requests_close_the_resume_question():
    logic, t = paused_resume_asked()   # 다른 목적지 제안 → 확인 질문 → 아니요 → 일시정지로(옛 재개 질문은 닫혀 있다)
    _voice(logic, t, intent(matched_destination_id=ELEV.id, need_confirm=True))
    assert logic.state == State.CONFIRMING
    assert _says(_voice(logic, t + 1, intent("deny"))) == [MSG_PAUSE_KEPT]
    assert logic.state == State.PAUSED and _voice(logic, t + 2, intent("affirm")) == []
    logic, t = paused_resume_asked()   # 취소 → "안내를 취소할까요?"의 "네"는 안내 취소다(재출발 아님)
    assert _says(_voice(logic, t, intent("cancel"))) == [MSG_CANCEL_CONFIRM]
    acts = _voice(logic, t + 1, intent("affirm"))
    assert logic.state == State.IDLE and _navs(acts) == []
    logic, t = paused_resume_asked()   # 사용자 질문(대답은 음성 몫)은 질문을 닫지 않는다
    assert _voice(logic, t, intent("question")) == []
    assert _navs(_voice(logic, t + 1, _ans(paused_resume_asked, "affirm", "yes"))) == [ROOM.id]


@pytest.mark.parametrize("kind,ask", [("question", "화장실은 2층에 있어요. 안내해 드릴까요?"),
                                      ("clarify", "어느 화장실로 안내해 드릴까요?")], ids=["question", "clarify"])
def test_p6_handed_question_replaces_the_resume_question(kind, ask):
    # 원칙 6 — 질문 뒤의 말은 가장 최근 질문의 대답이다. 미션이 음성이 넘긴 질문을 말하면 "다시 출발할까요?"는 닫힌다 —
    # 남기면 그 질문에 한 "네"(질문 기록이 open 이라 바꿔 읽지 않은 affirm)가 재출발이 된다(Task 3.4 검토).
    logic, t = paused_resume_asked()
    assert _says(_voice(logic, t, intent(kind, ask_text=ask, ask_kind="open"))) == [ask]
    off, t_off = paused()                              # 질문 없는 일시정지의 "네"와 같다
    acts = _voice(logic, t + 1, intent("affirm", answer="yes", answer_kind="open"))
    assert acts == _voice(off, t_off, intent("affirm")) == [] and logic.state == State.PAUSED
    assert MSG_RESUME_CONFIRM not in _says(_tick(logic, t + 20))     # 닫힌 질문을 시계가 다시 묻지 않는다
    assert _says(_voice(logic, t + 21, intent("resume", need_confirm=True))) == [MSG_RESUME_CONFIRM]   # 다시 가자 = 다시 묻는다


@pytest.mark.parametrize("flags", [{}, {"reask_ladder": True}], ids=["ladders_off", "reask_ladder"])
def test_p13_wake_greeting_handed_over_keeps_the_resume_question(flags):
    # 원칙 9 · 13 — 창 안의 "비카야"에 넘긴 "네?"는 새 질문이 아니다(note_question_asked·_resume_reask 와 같은 뜻). 재개 질문은
    # 남는다. 다음 "응"(대답 칸 open — 바꿔 읽지 않은 맨 affirm)은 그 질문의 승낙이 아니라 다시 묻기다(원칙 3, 전체 검토 C1) —
    # 출발은 대답 칸 yes 뿐이다. wake_revive 켜짐이면 창 안 "비카야"도 호출처럼 접는다(전체 검토 C3 — test_inwindow_wake).
    logic, t = paused_resume_asked(**flags)
    _voice(logic, t, intent("unknown", ask_text=MSG_WAKE_GREETING, ask_kind="open"))
    assert logic._resay_open_question(t) is not None
    assert _navs(_voice(logic, t + 1, intent("affirm", answer="yes", answer_kind="open"))) == []
    assert logic.state == State.PAUSED
    assert _navs(_voice(logic, t + 2, _ans(paused_resume_asked, "affirm", "yes"))) == [ROOM.id]


def test_p8_handed_retry_after_the_old_ladder_keeps_the_resume_question():
    # 옛 사다리: 둘째 못 알아들은 말에는 미션이 말하지 않아 음성이 넘긴 "잘 듣지 못했습니다…"가 나간다 — 되묻는 말이지 다른 질문이 아니다.
    retry = intent("unknown", ask_text=MSG_REASK_RETRY, ask_kind="open")
    logic, t = paused_resume_asked()
    assert _says(_voice(logic, t, retry)) == [MSG_RESUME_CONFIRM]
    assert _says(_voice(logic, t + 1, retry)) == [MSG_REASK_RETRY]
    assert logic._resay_open_question(t + 1) is not None
    # 넘긴 되묻는 말(open) 뒤의 "응"은 바꿔 읽지 않은 맨 affirm — 승낙이 아니다(원칙 3, 전체 검토 C1). 확정 resume 은 출발.
    assert _navs(_voice(logic, t + 2, intent("affirm", answer="yes", answer_kind="open"))) == []
    assert logic.state == State.PAUSED and logic._resay_open_question(t + 2) is not None
    assert _navs(_voice(logic, t + 3, intent("resume", need_confirm=False))) == [ROOM.id]


@pytest.mark.parametrize("flags", [{}, {"wake_revive": True}], ids=["ladders_off", "wake_revive"])
def test_p6_real_question_handed_with_unknown_closes_the_resume_question(flags):
    # 수정 3차 — 의도가 unknown 이어도 넘긴 글자가 로봇의 "네?"·되묻는 말이 아니면 다른 질문이다(원칙 6). 옛 사다리: 첫째는
    # 미션이 같은 질문을 다시 말해 넘긴 말을 버리고, 둘째(사다리 끝)는 넘긴 질문을 말한다 — 그 뒤 "네"로 출발하지 않는다.
    ask = "화장실은 2층에 있어요. 안내해 드릴까요?"
    logic, t = paused_resume_asked(**flags)
    assert _says(_voice(logic, t, intent("unknown", ask_text=ask, ask_kind="open"))) == [MSG_RESUME_CONFIRM]
    assert _says(_voice(logic, t + 1, intent("unknown", ask_text=ask, ask_kind="open"))) == [ask]
    acts = _voice(logic, t + 2, intent("affirm", answer="yes", answer_kind="open"))
    assert acts == [] and logic.state == State.PAUSED


def test_p6_real_question_handed_with_an_unclear_answer_closes_the_resume_question():
    # 대답 칸 unclear 가 unknown 으로 바뀐 요청(무작위 검사의 꼴)이 진짜 질문을 넘겼다 — 둘째에 말해지고 재개 질문은 닫힌다.
    ask = "어느 화장실이요?"
    logic, t = paused_resume_asked()
    unclear = _ans(paused_resume_asked, "clarify", "unclear", ask_text=ask, ask_kind="open")
    assert _says(_voice(logic, t, unclear)) == [MSG_RESUME_CONFIRM]
    assert _says(_voice(logic, t + 1, unclear)) == [ask]
    assert _voice(logic, t + 2, intent("affirm", answer="yes", answer_kind="open")) == []
    assert logic.state == State.PAUSED


def test_p13_wake_call_folds_first_then_handed_wake_greeting_still_asks_again():
    logic, t = paused_resume_asked(wake_revive=True)  # 호출이 먼저 접은 경우는 그대로 — "응"은 다시 묻는다
    logic.on_wake_call(t)
    _voice(logic, t + 1, intent("unknown", ask_text=MSG_WAKE_GREETING, ask_kind="open"))
    acts = _voice(logic, t + 2, intent("affirm", answer="yes", answer_kind="open"))
    assert _says(acts) == [MSG_RESUME_CONFIRM] and _navs(acts) == [] and logic.state == State.PAUSED


def test_p6_answer_without_a_handed_question_keeps_the_resume_question():
    logic, t = paused_resume_asked()                   # 넘긴 질문 없는 사용자 질문(대답은 음성이 말함)은 재개 질문을 둔다
    assert _voice(logic, t, intent("question", reply="화장실은 2층에 있어요.")) == []
    assert _navs(_voice(logic, t + 1, _ans(paused_resume_asked, "affirm", "yes"))) == [ROOM.id]


def test_p3_resume_question_replaces_an_open_cancel_question():
    logic, t = paused(answer_field=True)
    assert _says(_voice(logic, t, intent("cancel"))) == [MSG_CANCEL_CONFIRM]
    assert _says(_voice(logic, t + 1, intent("resume", need_confirm=True))) == [MSG_RESUME_CONFIRM]
    assert logic.cancel_confirm_pending is False      # 질문은 하나만 — 다음 "네"가 안내 취소로 잡히지 않게
    assert _navs(_voice(logic, t + 2, _ans(paused_resume_asked, "affirm", "yes"))) == [ROOM.id]


def test_p3_resume_question_during_a_help_request():
    logic, t = _paused_help_requested(answer_field=True)
    assert logic.state == State.PAUSED and logic.help_phase == "requested"
    _voice(logic, t, intent("cancel"))                # "도움 요청을 취소할까요?"
    assert logic.help_cancel_pending
    assert _says(_voice(logic, t + 1, intent("resume", need_confirm=True))) == [MSG_RESUME_CONFIRM]
    assert not logic.help_cancel_pending              # 질문은 하나만 — 다음 "네"가 도움 요청 취소로 잡히지 않게
    acts = _voice(logic, t + 2, _ans(paused_resume_asked, "affirm", "yes"))
    assert _navs(acts) == [ROOM.id] and logic.help_phase == "none"
    assert [a.event for a in acts if isinstance(a, HelpEvent)] == ["help_canceled"]


def test_p3_after_the_staff_acknowledges_the_yes_does_not_start():
    logic, t = _paused_help_requested(answer_field=True)
    _voice(logic, t, intent("resume", need_confirm=True))
    acts, ok = logic.on_help_ack(logic.help_id, t + 1)
    assert ok and _says(acts) == [MSG_HELP_COMING]
    acts = _voice(logic, t + 2, intent("affirm"))      # 관리자가 오는 중 — 사용자 말로는 움직이지 않는다(R9)
    assert _navs(acts) == [] and logic.state == State.PAUSED
    assert _says(_voice(logic, t + 3, intent("resume", need_confirm=True))) == [MSG_HELP_COMING]


def test_p3_resume_when_nav_is_not_ready_says_why_and_asks_nothing():
    off, t_off = paused()
    reject = _says(off.on_voice_intent(intent("resume"), t_off, lookup, BOUNDS, False))
    assert reject and MSG_RESUME_CONFIRM not in reject
    logic, t = paused(answer_field=True)
    assert _says(logic.on_voice_intent(intent("resume", need_confirm=True), t, lookup, BOUNDS, False)) == reject
    assert logic._resay_open_question(t) is None
    logic, t = paused_resume_asked()                   # 물은 뒤 준비가 풀렸다 — 이유를 말하고 닫는다
    acts = logic.on_voice_intent(_ans(paused_resume_asked, "affirm", "yes"), t, lookup, BOUNDS, False)
    assert _says(acts) == reject and _navs(acts) == [] and logic.state == State.PAUSED
    assert logic._resay_open_question(t) is None


def test_p3_estop_drops_the_resume_question():
    logic, t = paused_resume_asked()
    logic.on_estop(True, t)
    assert logic.state == State.ESTOPPED and logic._resay_open_question(t) is None
    assert MSG_RESUME_CONFIRM not in _says(_tick(logic, t + 15))


def test_p3_off_paused_resume_and_answers_are_as_now():
    logic, t = paused()                                # 꺼짐 — 확정 resume 은 지금처럼 곧바로 재출발
    acts = _voice(logic, t, intent("resume"))
    assert logic.state == State.NAVIGATING and _navs(acts) == [ROOM.id] and _says(acts) == [RESUMED_ROOM]
    for kind in ("affirm", "deny"):
        logic, t = paused()
        assert _voice(logic, t, intent(kind)) == [] and logic.state == State.PAUSED
    logic, t = paused()
    assert _says(_voice(logic, t, intent("dismiss"))) == [MSG_CANCEL_CONFIRM]


def test_p3_off_resume_proposal_does_not_start():
    """음성만 켬(VICA_ANSWER=on · answer_field 꺼짐) — 음성은 재개 제안을 쥐지 않고 보내는데 미션은 묻지 않는다. 지금처럼
    받는 즉시 재출발하면 확인 없는 출발이다(원칙 3). 둘 다 꺼짐에서는 음성이 제안을 보내지 않아(should_forward_intent) 이 길이 없다."""
    logic, t = paused()
    assert _voice(logic, t, intent("resume", need_confirm=True)) == []
    assert logic.state == State.PAUSED and logic.paused_destination.id == ROOM.id
    assert _navs(_voice(logic, t + 1, intent("resume"))) == [ROOM.id]      # 확정은 지금처럼 곧바로 재출발
    logic, t = _paused_help_requested()                                      # 도움 요청 뒤(관리자 확인 전)도 같다
    assert _voice(logic, t, intent("resume", need_confirm=True)) == [] and logic.state == State.PAUSED
