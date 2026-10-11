"""질문보다 먼저 시작한 예/아니요는 그 질문의 대답이 아니다(2026-10-11 사용자 결정).

run90 18:43:35 주행 중 "비카야"→"네?"에 한 "네"(말 시작 18:43:36 쯤)가, 그 사이 도착해 18:43:39.55 에 물은
"여기서 대기할까요?"의 대답으로 쓰여 곧바로 "몇 분쯤 걸리실까요?"로 넘어갔다. 노드는 대답을 청하는 말을 내보낼 때
note_question_asked 로 알리고, 귀가 말 시작을 알린 시각(speech)과 견준다. 닫힘(closed)은 받아쓰기 뒤에 나가
질문보다 늦거나 LLM 판정보다도 늦을 수 있다(아래 실측 두 시험).
"""
from vica_mission_manager.mission_logic import (
    MSG_CANCEL_CONFIRM, SAME_QUESTION_SEC, IntentData, MissionLogic, Say, State,
)


def _intent(kind, nc=False):
    return IntentData(kind, "", nc, "normal")


def _asking_cancel(t=100.0):
    """안내 중 "안내를 취소할까요?"를 물은 상태 — 노드처럼 질문 시각을 알린다."""
    logic = MissionLogic()
    logic.state = State.NAVIGATING
    acts = logic.on_voice_intent(_intent("cancel", nc=True), t - 1.0, lambda _id: None)
    asked = [a for a in acts if isinstance(a, Say) and a.expects_reply]
    assert [a.text for a in asked] == [MSG_CANCEL_CONFIRM]
    logic.note_question_asked(asked[0].text, t)
    return logic


def test_yes_spoken_before_the_question_is_not_its_answer():
    logic = _asking_cancel(t=100.0)
    logic.on_listen_state("open", 97.0)
    logic.on_listen_state("speech", 97.5)          # 질문(100.0) 전에 시작한 말
    logic.on_listen_state("closed", 101.0)         # 닫힘은 받아쓰기 뒤라 질문보다 늦을 수 있다
    acts = logic.on_voice_intent(_intent("affirm"), 101.5, lambda _id: None)
    assert acts == [] and "질문 전에" in logic.voice_note
    assert logic.state == State.NAVIGATING and logic.cancel_confirm_pending   # 질문은 그대로 기다린다


def test_yes_spoken_after_the_question_is_its_answer():
    logic = _asking_cancel(t=100.0)
    logic.on_listen_state("open", 103.0)
    logic.on_listen_state("speech", 103.3)
    logic.on_listen_state("closed", 104.0)
    acts = logic.on_voice_intent(_intent("affirm"), 105.0, lambda _id: None)
    assert acts and logic.voice_note == "" and not logic.cancel_confirm_pending


def test_no_spoken_before_the_question_is_dropped_too():
    logic = _asking_cancel(t=100.0)
    logic.on_listen_state("speech", 98.0)
    assert logic.on_voice_intent(_intent("deny"), 101.0, lambda _id: None) == []


def test_reasking_the_same_question_keeps_the_first_time():
    # 다시 묻기(같은 글자)는 첫 질문 시각을 둔다 — 첫 질문에 늦게 온 대답을 버리지 않게
    logic = _asking_cancel(t=100.0)
    logic.on_listen_state("speech", 105.5)
    logic.note_question_asked(MSG_CANCEL_CONFIRM, 115.0)
    acts = logic.on_voice_intent(_intent("affirm"), 116.0, lambda _id: None)
    assert acts and logic.voice_note == ""


def test_same_text_long_after_is_a_new_question():
    logic = _asking_cancel(t=100.0)
    logic.note_question_asked(MSG_CANCEL_CONFIRM, 100.0 + SAME_QUESTION_SEC + 1.0)
    assert logic._question_at == 100.0 + SAME_QUESTION_SEC + 1.0


def test_without_a_noted_question_nothing_changes():
    logic = MissionLogic()
    logic.state = State.NAVIGATING
    logic.on_voice_intent(_intent("cancel", nc=True), 99.0, lambda _id: None)
    logic.on_listen_state("speech", 97.5)
    assert logic.on_voice_intent(_intent("affirm"), 101.0, lambda _id: None)


def test_other_intents_are_not_held_back():
    # 목적지·취소 같은 요청은 질문 전에 한 말이어도 그대로 처리한다(예/아니요만 질문에 기대는 말이다)
    logic = _asking_cancel(t=100.0)
    logic.on_listen_state("speech", 99.0)
    logic.on_voice_intent(_intent("cancel", nc=False), 101.0, lambda _id: None)
    assert logic.voice_note == ""


def test_node_reports_every_question_it_sends():
    # 모든 말은 노드 _run_actions 한 곳을 지난다 — 대답을 청하는 말이면 거기서 질문 시각을 알린다
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / "vica_mission_manager" / "mission_manager_node.py").read_text(
        encoding="utf-8")
    body = src[src.index("    def _run_actions"):]
    body = body[:body.index("\n    def ", 10)]
    assert "self.logic.note_question_asked(action.text, self._now())" in body


# ── 닫힘(closed)은 받아쓰기 뒤에 나간다 — 말 시작(speech)으로 견준다(2026-10-11 20:1x 실측) ──────────

def test_run90_1843_answer_whose_speech_began_before_the_question_is_dropped():
    # "비카야"→"네?" 창에서 18:43:36 쯤 말을 시작한 "네"(닫힘 40.30 은 받아쓰기 뒤라 질문 39.55 보다 늦다)
    logic = _asking_cancel(t=39.55)
    logic.on_listen_state("open", 35.54)
    logic.on_listen_state("speech", 36.1)
    logic.on_listen_state("closed", 40.30)
    assert logic.on_voice_intent(_intent("affirm"), 41.14, lambda _id: None) == []


def test_1912_answer_arriving_before_its_own_closed_is_kept():
    # 질문 42.27 뒤 51.1 에 말을 시작한 "네" — Realtime 판정(54.08)이 귀의 닫힘(54.12)보다 먼저 왔다
    logic = _asking_cancel(t=42.27)
    logic.on_listen_state("open", 30.0)
    logic.on_listen_state("speech", 30.5)
    logic.on_listen_state("closed", 33.0)        # 질문 전 다른 말의 닫힘
    logic.on_listen_state("open", 48.0)
    logic.on_listen_state("speech", 51.1)
    acts = logic.on_voice_intent(_intent("affirm"), 54.08, lambda _id: None)
    assert acts and logic.voice_note == ""


def test_without_any_speech_signal_answers_are_kept():
    # 귀의 말 시작 신호를 못 받았으면 예전처럼 받는다 — 진짜 대답을 버리는 쪽으로 틀리지 않게
    logic = _asking_cancel(t=100.0)
    logic.on_listen_state("closed", 99.0)
    assert logic.on_voice_intent(_intent("affirm"), 101.0, lambda _id: None)
