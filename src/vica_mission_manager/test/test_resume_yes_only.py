"""재개 질문("다시 출발할까요?")의 승낙은 대답 칸 yes 를 바꿔 읽은 affirm 과 확정 resume 뿐이다(원칙 3, 전체 검토 C1·C2)."""
import pytest

from reaction_states import BOUNDS, ROOM, intent, lookup, paused_resume_asked
from vica_mission_manager.mission_logic import (
    MSG_PAUSE_KEPT,
    MSG_REASK_LOUDER,
    MSG_REASK_RETRY,
    MSG_RESUME_CONFIRM,
    MSG_RESUMED,
    MSG_WAKE_GREETING,
    CancelNav,
    Navigate,
    NavStatus,
    Say,
    State,
    say_destination,
)

YES = dict(answer="yes", answer_kind="yesno")
NO = dict(answer="no", answer_kind="yesno")
OPEN_YES = dict(answer="yes", answer_kind="open")
RESUMED_ROOM = say_destination(MSG_RESUMED, ROOM.name)
FLAGS = [{}, {"reask_ladder": True}, {"wake_revive": True}, {"reask_ladder": True, "wake_revive": True}]
FLAG_IDS = ["ladders_off", "reask_ladder", "wake_revive", "both"]


def _voice(logic, t, it):
    return logic.on_voice_intent(it, t, lookup, BOUNDS, True)


def _says(acts):
    return [a.text for a in acts if isinstance(a, Say)]


def _navs(acts):
    return [a.destination.id for a in acts if isinstance(a, Navigate)]


@pytest.mark.parametrize("flags", FLAGS, ids=FLAG_IDS)
@pytest.mark.parametrize("it", [intent("affirm"), intent("affirm", **OPEN_YES), intent("affirm", answer="other", answer_kind="yesno")],
                         ids=["no_field", "open_yes", "other"])
def test_p3_bare_affirm_asks_the_resume_question_again(flags, it):
    """글자 경로 '음'(대답 칸 없음) · "네?" 뒤의 "응"(open) · 대답 칸 other — 재출발하지 않고 같은 질문을 다시 묻는다."""
    logic, t = paused_resume_asked(**flags)
    acts = _voice(logic, t, it)
    assert _navs(acts) == [] and logic.state == State.PAUSED
    assert _says(acts) == [MSG_REASK_RETRY if flags.get("reask_ladder") else MSG_RESUME_CONFIRM]
    (asked,) = [a for a in acts if isinstance(a, Say)]
    assert asked.expects_reply and asked.kind == "yesno"
    assert logic._resume_question_open()


@pytest.mark.parametrize("flags", FLAGS, ids=FLAG_IDS)
def test_p3_bare_affirm_is_counted_like_a_misheard_answer(flags):
    """맨 affirm 과 unknown 은 재개 질문에서 같은 동작·같은 사다리 단계·같은 열림 — 세 번 이어서 견준다(세 단계는 셋째에"""
    a, t = paused_resume_asked(**flags)
    b, _ = paused_resume_asked(**flags)
    for i in range(3):
        assert _voice(a, t + i, intent("affirm")) == _voice(b, t + i, intent("unknown")), i
        assert a._resume_ladder.step == b._resume_ladder.step, i
        assert a._resume_question_open() == b._resume_question_open(), i
        assert a.state == b.state == State.PAUSED, i


@pytest.mark.parametrize("flags", FLAGS, ids=FLAG_IDS)
@pytest.mark.parametrize("it", [intent("resume", **YES), intent("affirm", **YES), intent("unknown", **YES), intent("resume")],
                         ids=["resume+yes", "affirm+yes", "unknown+yes", "confirmed_resume"])
def test_p3_yes_in_the_answer_field_or_a_confirmed_resume_departs(flags, it):
    """소리 경로 "네" = 의도 resume 확정 + 대답 칸 yes / 의도 affirm + 대답 칸 yes, 글자 경로 = 확정 resume — 출발."""
    logic, t = paused_resume_asked(**flags)
    acts = _voice(logic, t, it)
    assert logic.state == State.NAVIGATING
    assert _navs(acts) == [ROOM.id] and _says(acts) == [RESUMED_ROOM]


@pytest.mark.parametrize("reask", [False, True], ids=["ladder_off", "ladder_on"])
def test_p3_yes_after_the_wake_greeting_asks_again_without_wake_revive(reask):
    """C2 — wake_revive 꺼짐: 호출은 재개 질문을 접지 않고, "네?" 뒤의 "응"(open, 맨 affirm)은 다시 묻기다(재출발 없음)."""
    logic, t = paused_resume_asked(reask_ladder=reask)
    assert _says(logic.on_wake_call(t)) == [MSG_WAKE_GREETING]
    assert logic._wake_folded is None and logic._resume_question_open()
    acts = _voice(logic, t + 2, intent("affirm", **OPEN_YES))
    assert _navs(acts) == [] and logic.state == State.PAUSED
    assert _says(acts) == [MSG_REASK_RETRY if reask else MSG_RESUME_CONFIRM]
    acts = _voice(logic, t + 4, intent("affirm", **YES))
    assert _navs(acts) == [ROOM.id] and logic.state == State.NAVIGATING


@pytest.mark.parametrize("reask", [False, True], ids=["ladder_off", "ladder_on"])
def test_p3_yes_after_the_wake_greeting_revives_the_question_with_wake_revive(reask):
    """wake_revive 켜짐: 지금처럼 호출이 접고, "응"은 되살려 다시 묻는다(Task 3.9). 다시 물은 질문의 맨 affirm 도 다시 묻기다."""
    logic, t = paused_resume_asked(wake_revive=True, reask_ladder=reask)
    logic.on_wake_call(t)
    assert logic._wake_folded == ("resume", "", t)
    acts = _voice(logic, t + 2, intent("affirm", **OPEN_YES))
    assert _says(acts) == [MSG_RESUME_CONFIRM] and _navs(acts) == [] and logic.state == State.PAUSED
    acts = _voice(logic, t + 3, intent("affirm"))
    assert _navs(acts) == [] and logic.state == State.PAUSED
    assert _says(acts) == [MSG_REASK_RETRY if reask else MSG_RESUME_CONFIRM]
    assert _navs(_voice(logic, t + 4, intent("affirm", **YES))) == [ROOM.id]


def test_p28_repeat_revives_the_folded_resume_question_and_only_yes_departs():
    """"다시 말해줘"로 되살린 재개 질문(Task 4.3)도 같다 — 맨 affirm 은 다시 묻고, 대답 칸 yes 는 출발."""
    logic, t = paused_resume_asked(wake_revive=True)
    logic.on_wake_call(t)
    assert _says(_voice(logic, t + 2, intent("repeat"))) == [MSG_RESUME_CONFIRM]
    assert _navs(_voice(logic, t + 3, intent("affirm"))) == [] and logic.state == State.PAUSED
    assert _navs(_voice(logic, t + 4, intent("affirm", **YES))) == [ROOM.id]


@pytest.mark.parametrize("flags", FLAGS, ids=FLAG_IDS)
@pytest.mark.parametrize("it", [intent("deny"), intent("unknown", **NO), intent("affirm", **NO)],
                         ids=["deny", "unknown+no", "affirm+no"])
def test_p3_no_keeps_the_pause(flags, it):
    logic, t = paused_resume_asked(**flags)
    acts = _voice(logic, t, it)
    assert _says(acts) == [MSG_PAUSE_KEPT] and _navs(acts) == []
    assert logic.state == State.PAUSED and not logic._resume_question_open()


def _moved(acts):
    return any(isinstance(a, (Navigate, CancelNav)) for a in acts)


def test_p3_three_bare_affirms_never_depart_and_the_old_ladder_closes_at_the_deadline():
    logic, t = paused_resume_asked()
    runs = [_voice(logic, t + i, intent("affirm")) for i in range(3)]
    assert [_says(a) for a in runs] == [[MSG_RESUME_CONFIRM], [], []]
    assert not any(_moved(a) for a in runs) and logic.state == State.PAUSED
    assert logic._resume_question_open()
    assert not _moved(logic.on_tick(t + 29.0, NavStatus.NONE))
    assert not logic._resume_question_open() and logic.state == State.PAUSED


def test_p3_three_bare_affirms_never_depart_and_the_three_steps_close_the_question():
    logic, t = paused_resume_asked(reask_ladder=True)
    runs = [_voice(logic, t + i, intent("affirm")) for i in range(3)]
    assert [_says(a) for a in runs] == [[MSG_REASK_RETRY], [MSG_REASK_LOUDER], []]
    assert not any(_moved(a) for a in runs)
    assert not logic._resume_question_open() and logic.state == State.PAUSED and logic.help_phase == "none"
    assert _voice(logic, t + 3, intent("affirm", **YES)) == [] and logic.state == State.PAUSED
