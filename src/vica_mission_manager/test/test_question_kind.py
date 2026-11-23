"""질문 Say 의 종류·후보 표기(설계 2026-11-14 4.1, 2단계) — 말과 동작은 그대로, 종류만 덧붙는다."""
from __future__ import annotations

import ast
from pathlib import Path

import pytest

from vica_mission_manager import mission_logic
from vica_mission_manager.mission_logic import (
    IntentData, MissionLogic, QUESTION_KINDS, Say,
    MSG_ASK_WAIT_TIME, MSG_CANCEL_CONFIRM, MSG_WAIT_NEED_ASK, MSG_WAKE_GREETING,
)
from test_wait_spot import BOUNDS, HOME, _dest, _intent, arrive
from test_wait_speaker_flow import it, voice, waiting


def sites_without_kind(source: str) -> list[tuple[int, str]]:
    """expects_reply=True 인 Say(…) 와 _ask(…) 호출 중 kind 를 안 넘긴 자리 [(줄, 소스)]."""
    lines, out = source.splitlines(), []
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.Call):
            continue
        f = node.func
        name = f.id if isinstance(f, ast.Name) else f.attr if isinstance(f, ast.Attribute) else ""
        kws = {k.arg: k.value for k in node.keywords}
        asks = name == "Say" and isinstance(kws.get("expects_reply"), ast.Constant) and kws["expects_reply"].value is True
        if (asks and "kind" not in kws) or (name == "_ask" and len(node.args) < 2 and "kind" not in kws):
            out.append((node.lineno, lines[node.lineno - 1].strip()))
    return sorted(out)


def questions(actions) -> list[Say]:
    """질문 = 종류(kind)가 붙은 Say. 호출 응답 "네?"는 expects_reply 없이 kind 만 단다."""
    return [a for a in actions if isinstance(a, Say) and a.kind]


def test_every_question_site_declares_a_kind():
    assert sites_without_kind(Path(mission_logic.__file__).read_text(encoding="utf-8")) == []


def test_checker_catches_a_missing_kind():
    src = 'x = Say("a", priority="response", expects_reply=True)\ny = self._ask("b")\nz = self._ask("c", "open")\n'
    assert [n for n, _ in sites_without_kind(src)] == [1, 2]


def _wake():
    return MissionLogic(return_destination=HOME, arrival_dialog=True).on_wake_call(1.0)


def _need_ask():
    return voice(waiting(gate=False), it("cancel", "fail"), 20.0)


def _wait_time():
    logic, _ = arrive()
    return logic.on_arrival_answer(_intent("wait"), 3.0)


def _cancel_confirm():
    logic = MissionLogic(return_destination=HOME, arrival_dialog=True)
    logic.on_intent(_intent(), _dest(), BOUNDS, True, 0.0)
    cancel = IntentData(intent="cancel", matched_destination_id="", need_confirm=False, safety_flag="normal")
    return logic.on_voice_intent(cancel, 5.0, lambda _id: _dest(), BOUNDS, True)


FLOWS = [(_wake, MSG_WAKE_GREETING, "open"), (_need_ask, MSG_WAIT_NEED_ASK, "yesno_inverted"),
         (_wait_time, MSG_ASK_WAIT_TIME, "minutes"), (_cancel_confirm, MSG_CANCEL_CONFIRM, "yesno")]


@pytest.mark.parametrize("flow,text,kind", FLOWS)
def test_flows_emit_the_declared_kind(flow, text, kind):
    asked = questions(flow())
    assert [q.text for q in asked] == [text]
    assert asked[0].kind == kind and asked[0].options == ()


def test_confirm_question_carries_the_destination_as_option():
    logic = waiting()
    logic.on_handle_state(True, True, 19.5)
    asked = questions(voice(logic, it("navigate", "pass", need_confirm=True), 20.0))
    assert len(asked) == 1 and asked[0].kind == "confirm" and asked[0].expects_reply
    assert asked[0].options == (("d1", "화장실 입구"),)


def test_wake_greeting_is_an_open_question_without_listen_request():
    asked = questions(_wake())
    assert [(q.text, q.kind, q.expects_reply) for q in asked] == [(MSG_WAKE_GREETING, "open", False)]


def test_every_listen_request_is_a_question():
    """듣기 창을 여는 말(expects_reply)은 모두 종류를 단다 — 위 흐름들에서."""
    _logic, arrival = arrive()
    for acts in (_need_ask(), _wait_time(), _cancel_confirm(), arrival):
        for a in acts:
            assert not (isinstance(a, Say) and a.expects_reply and not a.kind)


def test_arrival_question_is_yesno():
    _logic, acts = arrive()
    asked = questions(acts)
    assert len(asked) == 1 and asked[0].kind == "yesno"


def test_kind_is_one_of_the_six():
    assert QUESTION_KINDS == ("open", "confirm", "choice", "yesno", "yesno_inverted", "minutes")
    _logic, arrival = arrive()
    for q in [*(q for flow, _t, _k in FLOWS for q in questions(flow())), *questions(arrival)]:
        assert q.kind in QUESTION_KINDS


def test_plain_speech_has_no_kind():
    assert Say("안내를 시작합니다.").kind == "" and Say("안내를 시작합니다.").options == ()


def test_proposal_question_is_confirm_kind():
    """3-가 한 목소리 — 목적지 제안의 확인 질문은 미션이 confirm_prompt 로 묻는다(결정 A)."""
    from reaction_states import BOUNDS as RB, ELEV, idle, intent as rintent, lookup as rlookup

    logic, now = idle()
    acts = logic.on_voice_intent(rintent(matched_destination_id=ELEV.id, need_confirm=True), now, rlookup, RB, True)
    (say,) = [a for a in acts if isinstance(a, mission_logic.Say)]
    assert (say.text, say.expects_reply, say.kind, say.options) == (
        "엘리베이터로 안내해드릴까요?", True, "confirm", (("elev", "엘리베이터"),))
