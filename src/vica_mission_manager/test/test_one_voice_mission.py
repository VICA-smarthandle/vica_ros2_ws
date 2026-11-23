"""한 목소리(설계 2026-11-14 4.4, 3-가) — 음성이 넘긴 질문 문장을 미션이 말한다."""
from pathlib import Path

from reaction_states import BOUNDS, awaiting_user, idle, intent, lookup, patrolling, paused
from vica_mission_manager.mission_logic import MSG_APPROACH_REASK, Navigate, Say, State

RESUME_CONFIRM = "다시 출발할까요?"


def _says(actions):
    return [a for a in actions if isinstance(a, Say)]


def test_handed_open_question_is_spoken_with_kind():
    logic, now = idle()
    acts = logic.on_voice_intent(intent("clarify", ask_text="어떤 서류를 내러 가시나요?", ask_kind="open"),
                                 now, lookup, BOUNDS, True)
    says = _says(acts)
    assert [(s.text, s.expects_reply, s.kind) for s in says] == [("어떤 서류를 내러 가시나요?", True, "open")]
    assert logic.state == State.IDLE


def test_handed_choice_carries_option_names():
    logic, now = idle()
    acts = logic.on_voice_intent(intent("clarify", ask_text="화장실과 화장실 입구 중 어디로 갈까요?",
                                        ask_kind="choice", ask_option_ids=("wc", "d1", "없는id")),
                                 now, lookup, BOUNDS, True)
    (say,) = _says(acts)
    assert (say.kind, say.options) == ("choice", (("wc", "화장실"), ("d1", "화장실 입구")))


def test_handed_question_dropped_when_mission_speaks():
    """접근 질문 중 질문 — 미션이 "안내를 받으시겠어요?"를 다시 묻는다. 넘겨받은 질문은 버려 한 번에 한 문장."""
    logic, now = awaiting_user()
    acts = logic.on_voice_intent(intent("question", ask_text="무엇을 도와드릴까요?", ask_kind="open"),
                                 now, lookup, BOUNDS, True)
    assert [s.text for s in _says(acts)] == [MSG_APPROACH_REASK]


def test_handed_question_silent_in_patrol():
    logic, now = patrolling()
    acts = logic.on_voice_intent(intent("clarify", ask_text="어디로 가시나요?", ask_kind="open"),
                                 now, lookup, BOUNDS, True)
    assert acts == []


def test_unknown_ask_kind_falls_back_to_open():
    logic, now = idle()
    acts = logic.on_voice_intent(intent("clarify", ask_text="어디로 가시나요?", ask_kind="weird"),
                                 now, lookup, BOUNDS, True)
    assert [s.kind for s in _says(acts)] == ["open"]


def test_held_resume_question_spoken_in_paused():
    """음성이 쥔 "다시 출발할까요?"(결정 B) — 미션은 말만 하고 움직이지 않는다."""
    logic, now = paused()
    acts = logic.on_voice_intent(intent("clarify", ask_text=RESUME_CONFIRM, ask_kind="yesno"),
                                 now, lookup, BOUNDS, True)
    assert [(s.text, s.kind) for s in _says(acts)] == [(RESUME_CONFIRM, "yesno")]
    assert logic.state == State.PAUSED
    assert not [a for a in acts if isinstance(a, Navigate)]


def test_node_reads_ask_fields():
    src = (Path(__file__).resolve().parents[1] / "vica_mission_manager" / "mission_manager_node.py").read_text()
    body = src[src.index("    def _on_intent("):src.index("    def _lookup_destination(")]
    for needle in ('ask_text=str(getattr(msg, "ask_text"', 'ask_kind=str(getattr(msg, "ask_kind"',
                   'ask_option_ids=tuple(getattr(msg, "ask_option_ids"'):
        assert needle in body, needle
