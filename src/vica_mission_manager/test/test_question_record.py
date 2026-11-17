"""질문 기록의 열림·닫힘(설계 2026-11-14 4.1, 2단계). 닫는 기준에 시간을 쓰지 않는다 — 11-17 사용자."""
from __future__ import annotations

from vica_mission_manager.mission_logic import Say
from vica_mission_manager.question_record import FAULT_CLOSE_SEC, QuestionRecord, QuestionTracker


def ask(text="여기서 대기할까요?", kind="yesno", options=()):
    return Say(text, priority="response", expects_reply=True, kind=kind, options=options)


def test_open_gives_new_id_each_time():
    t = QuestionTracker("q")
    _, a = t.open(ask(), "asking_next", 10.0)
    _, b = t.open(ask(), "asking_next", 12.0)          # 같은 문장을 다시 물어도 새 id
    assert (a.id, b.id) == ("q-1", "q-2")


def test_open_fills_the_record_from_the_say():
    _, r = QuestionTracker("q").open(ask("화장실로 안내해드릴까요?", "confirm", (("d1", "화장실"),)), "confirming", 5.0)
    assert r == QuestionRecord(id="q-1", kind="confirm", text="화장실로 안내해드릴까요?", options=(("d1", "화장실"),),
                               step=0, on_silence="", opened_at=5.0, state="confirming")


def test_open_replaces_and_returns_old():
    t = QuestionTracker("q")
    _, first = t.open(ask(), "asking_next", 10.0)
    old, second = t.open(ask("몇 분쯤 걸리실까요?", "minutes"), "asking_wait_time", 14.0)
    assert old == first and t.current == second


def test_first_open_has_no_old_record():
    old, _ = QuestionTracker("q").open(ask(), "asking_next", 10.0)
    assert old is None


def test_close_returns_record_once():
    t = QuestionTracker("q")
    _, r = t.open(ask(), "asking_next", 10.0)
    assert t.close("answer") == r and t.current is None
    assert t.close("answer") is None


def test_empty_listen_state_closes():
    for state in ("empty", "empty:reject '어'", "empty:wake-ignored"):
        t = QuestionTracker("q")
        _, r = t.open(ask(), "asking_next", 10.0)
        assert t.on_listen_state(state) == r and t.current is None


def test_speech_listen_state_keeps_open():
    t = QuestionTracker("q")
    _, r = t.open(ask(), "asking_next", 10.0)
    for state in ("open", "speech", "closed", "wake-rescue '비카야'"):   # 말을 했으면 의도가 뒤따라 온다
        assert t.on_listen_state(state) is None and t.current == r


def test_listen_state_without_question_is_nothing():
    assert QuestionTracker("q").on_listen_state("empty") is None


def test_tick_closes_on_state_change():
    t = QuestionTracker("q")
    _, r = t.open(ask("화장실로 안내해드릴까요?", "confirm"), "confirming", 10.0)
    assert t.tick("navigating", 11.0) == (r, "state") and t.current is None


def test_fault_close_after_30s():
    assert FAULT_CLOSE_SEC == 30.0
    t = QuestionTracker("q")
    _, r = t.open(ask(), "asking_next", 10.0)
    assert t.tick("asking_next", 40.1) == (r, "fault")


def test_tick_keeps_open_before_30s():
    t = QuestionTracker("q")
    _, r = t.open(ask(), "asking_next", 10.0)
    assert t.tick("asking_next", 39.9) is None and t.current == r


def test_missing_or_unknown_kind_becomes_open():
    t = QuestionTracker("q")
    _, a = t.open(Say("네?", priority="response", expects_reply=True), "idle", 1.0)
    _, b = t.open(ask(kind="maybe"), "idle", 2.0)
    assert (a.kind, b.kind) == ("open", "open")
