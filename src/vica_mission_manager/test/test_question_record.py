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


# ---- 노드 연결(소스 검사 — rclpy 없이) ------------------------------------------------
from pathlib import Path  # noqa: E402

NODE = (Path(__file__).resolve().parents[1] / "vica_mission_manager" / "mission_manager_node.py").read_text(encoding="utf-8")


def _body(name: str) -> str:
    start = NODE.index(f"    def {name}(")
    end = NODE.find("\n    def ", start + 1)
    return NODE[start: end if end != -1 else len(NODE)]


def test_node_publishes_the_question_topic_latched():
    at = NODE.index('"/vica/question"')
    assert "TRANSIENT_LOCAL" in NODE[at: at + 300] and "depth=1" in NODE[at: at + 300]


def test_node_survives_an_old_interface_install():
    assert "except ImportError" in NODE[NODE.index("import VicaQuestion"): NODE.index("import VicaQuestion") + 200]
    assert "if self.pub_question is None" in _body("_publish_question")


def test_node_opens_a_question_for_every_spoken_question():
    body = _body("_run_actions")
    assert "action.kind or" in body and "self._open_question, action)" in body


def test_node_closes_the_question_before_handling_the_intent():
    body = _body("_on_intent")
    assert body.index('self._close_question, "answer")') < body.index("self.logic.on_voice_intent(")


def test_node_closes_on_silent_listen_result_before_the_reask():
    body = _body("_on_listen_state")
    assert body.index("self._listen_question") < body.index("self.logic.on_listen_state(")
    assert "self._questions.on_listen_state(" in _body("_listen_question")


def test_node_tick_closes_on_state_change_or_fault():
    assert "self._questions.tick(" in _body("_tick_question")


def test_node_log_lines_match_the_audit_tool():
    assert "[질문] 열림 id={record.id} kind={record.kind} 후보={len(record.options)} " in NODE
    assert "글자={record.text}" in NODE
    assert "[질문] 닫힘 id={record.id} 사유={reason} 열린 시간=" in NODE
    assert "[질문] kind 없음 — open 으로 냄" in NODE


# ---- 최종 검토(11-17) 반영 ------------------------------------------------------------
import ast  # noqa: E402

LOG_METHODS = {"debug", "info", "warn", "warning", "error", "fatal"}


def logger_methods_picked_by_condition(source: str) -> list[int]:
    """`x.warn if c else x.info` 처럼 로거 메서드를 조건식으로 고르는 줄 번호.

    rclpy 는 같은 호출 줄에서 심각도가 바뀌면 ValueError 를 낸다 — 그 예외가 콜백 밖으로 나가면 노드가 죽는다.
    """
    return sorted(n.lineno for n in ast.walk(ast.parse(source)) if isinstance(n, ast.IfExp)
                  and any(isinstance(b, ast.Attribute) and b.attr in LOG_METHODS for b in (n.body, n.orelse)))


def test_checker_catches_a_logger_picked_by_condition():
    src = "log = self.get_logger().warn if bad else self.get_logger().info\nlog('x')\n"
    assert logger_methods_picked_by_condition(src) == [1]


def test_node_never_picks_a_logger_method_by_condition():
    assert logger_methods_picked_by_condition(NODE) == []


def test_question_record_failure_cannot_stop_the_mission():
    """질문 기록은 기록일 뿐 — 네 접점 모두 보호 함수를 거친다."""
    assert "except Exception" in _body("_question_safely")
    assert "self._question_safely(self._open_question, action)" in _body("_run_actions")
    assert 'self._question_safely(self._close_question, "answer")' in _body("_on_intent")
    assert "self._question_safely(self._tick_question)" in _body("_tick")
    assert "self._question_safely(self._listen_question, msg.data)" in _body("_on_listen_state")


def test_node_clears_a_stale_question_at_startup():
    """미션만 다시 뜨면 음성이 옛 질문을 들고 있다 — 발행자를 만든 직후 빈 기록을 한 번 낸다."""
    at = NODE.index('"/vica/question"')
    assert "self._publish_question(None)" in NODE[at: at + 700]


def test_open_log_says_whether_a_listen_request_went_out():
    assert "요청={'y' if say.expects_reply else 'n'}" in NODE
