"""접근 질문 '안내를 받으시겠어요?' (2026-10-09 사용자 결정, run82 실기 세 장면)."""
import pytest

from vica_mission_manager.mission_logic import (
    APPROACH_REASK_MAX,
    APPROACH_RESPONSE_TIMEOUT_SEC,
    EAR_GRACE_SEC,
    EAR_HOLD_MAX_SEC,
    LLM_THINKING_TAIL_SEC,
    MSG_APPROACH_ACCEPTED,
    MSG_APPROACH_DECLINED,
    MSG_APPROACH_NO_ANSWER,
    MSG_APPROACH_QUESTION,
    MSG_APPROACH_REASK,
    MSG_APPROACH_UNANSWERED,
    MSG_WAKE_GREETING,
    ApproachRequest,
    Destination,
    IntentData,
    MapBounds,
    MissionLogic,
    NavStatus,
    Navigate,
    Pose2D,
    Say,
    State,
    StopSpeech,
    WakeReply,
)

BOUNDS = MapBounds(min_x=-15.1, min_y=-8.59, max_x=10.0, max_y=8.0)
HOME = Destination(id="standby", name="대기 위치", pose=Pose2D(-2.0, -1.0, 0.0, "map"), calibrated=True)
T = APPROACH_RESPONSE_TIMEOUT_SEC


def _asking(t=5.0):
    """다가가 질문을 마친 상태(AWAITING_USER, 질문 재생 끝 = t). 로봇 launch 처럼 접근 뒤 홈 복귀가 켜져 있다."""
    logic = MissionLogic(return_destination=HOME, auto_return_home=True)
    req = ApproachRequest(goal=Pose2D(1.0, 0.5, 30.0, "map"), track_id=7, approachable=True)
    _, reason = logic.on_approach_request(req, BOUNDS, True, 0.0)
    logic.on_tick(t - 1.0, NavStatus.SUCCEEDED)
    assert logic.state == State.AWAITING_USER
    logic.on_approach_question_spoken(t)
    return logic


def _intent(kind, reply=""):
    return IntentData(intent=kind, matched_destination_id="", need_confirm=False,
                      safety_flag="normal", reply=reply)


def _voice(logic, kind, now, reply=""):
    return logic.on_voice_intent(_intent(kind, reply), now, lambda _id: None, BOUNDS, True)


def _says(actions):
    return [a.text for a in actions if isinstance(a, Say)]


def _ticks(logic, t0, t1, step=0.5):
    out, t = [], t0
    while t <= t1 + 1e-9:
        out += logic.on_tick(t, NavStatus.RUNNING)
        t += step
    return out


def _use_up_reasks(logic, t=7.0):
    """못 알아들은 말로 다시 묻기를 다 쓴다. 매번 다시 묻기 재생이 끝난 시각을 알린다. 마지막 시각을 돌려준다."""
    for _ in range(APPROACH_REASK_MAX):
        assert _says(_voice(logic, "unknown", t)) == [MSG_APPROACH_REASK]
        t += 2.0
        logic.on_approach_question_spoken(t)
        t += 1.0
    return t


def test_reask_is_the_tail_of_the_first_question():
    """노드는 재생 끝(tts_done)에서 이 글자를 찾아 8초를 다시 센다 — 첫 질문에도 들어 있어야 한다."""
    assert MSG_APPROACH_REASK == "안내를 받으시겠어요?"
    assert MSG_APPROACH_QUESTION.endswith(MSG_APPROACH_REASK)


def test_wake_call_does_not_greet_or_cut_the_question():
    logic = _asking()
    actions = logic.on_wake_call(6.0)
    assert actions == [WakeReply(listen=True)]
    assert not any(isinstance(a, StopSpeech) for a in actions)
    assert MSG_WAKE_GREETING not in _says(actions)
    assert logic.state == State.AWAITING_USER


def test_wake_keeps_the_question_so_the_next_yes_is_accepted():
    """17:17 장면 — '비카야' 다음 "안내를 받을게요."(affirm)가 수락돼야 한다."""
    logic = _asking()
    logic.on_wake(6.0)
    assert logic.state == State.AWAITING_USER
    assert MSG_APPROACH_ACCEPTED in _says(_voice(logic, "affirm", 7.0))
    assert logic.state == State.TURNING


def test_reask_up_to_three_times():
    assert APPROACH_REASK_MAX == 3


def test_unanswered_words_point_to_the_wake_word():
    assert MSG_APPROACH_UNANSWERED == "필요하시면 '비카야'라고 불러 주세요."


def test_unclear_answer_is_reasked_once_at_once():
    logic = _asking()
    actions = _voice(logic, "unknown", 7.0)
    assert actions == [Say(MSG_APPROACH_REASK, priority="response", expects_reply=True)]
    assert logic.state == State.AWAITING_USER


@pytest.mark.parametrize("kind", ["clarify", "pause", "resume", "wait", "finish"])
def test_other_non_answers_are_reasked_too(kind):
    logic = _asking()
    assert _says(_voice(logic, kind, 7.0)) == [MSG_APPROACH_REASK]


def test_question_is_answered_first_then_reasked():
    """17:14 장면 "뭐야?" — LLM 의 답이 끝난 뒤에 다시 묻는다(순서가 바뀌지 않게)."""
    logic = _asking()
    reply = "저는 안내 로봇 비카예요."
    assert _voice(logic, "question", 7.0, reply=reply) == []
    assert logic.on_approach_reply_spoken("다른 말", 8.0) == []
    actions = logic.on_approach_reply_spoken(reply, 8.5)
    assert _says(actions) == [MSG_APPROACH_REASK]


def test_question_reask_falls_back_if_the_answer_never_finishes():
    logic = _asking()
    _voice(logic, "question", 7.0, reply="저는 안내 로봇 비카예요.")
    assert MSG_APPROACH_REASK in _says(_ticks(logic, 7.5, 7.0 + T + 0.5))


def test_silence_is_reasked_three_times():
    logic = _asking()
    t = 5.0
    for _ in range(APPROACH_REASK_MAX):
        said = _says(_ticks(logic, t + 0.5, t + T + 0.5))
        assert said == [MSG_APPROACH_REASK]
        assert logic.state == State.AWAITING_USER
        t += T + 2.0
        logic.on_approach_question_spoken(t)


def test_silence_after_three_reasks_leaves_with_the_wake_word_hint():
    logic = _asking()
    t = 5.0
    for _ in range(APPROACH_REASK_MAX):
        _ticks(logic, t + 0.5, t + T + 0.5)
        t += T + 2.0
        logic.on_approach_question_spoken(t)
    actions = _ticks(logic, t + 0.5, t + T + 0.5)
    assert _says(actions) == [MSG_APPROACH_UNANSWERED]
    assert logic.state == State.RETURNING
    assert any(isinstance(a, Navigate) and a.destination == HOME for a in actions)


def test_second_and_third_non_answers_are_reasked():
    logic = _asking()
    _voice(logic, "unknown", 7.0)
    logic.on_approach_question_spoken(9.0)
    assert _says(_voice(logic, "clarify", 10.0)) == [MSG_APPROACH_REASK]
    logic.on_approach_question_spoken(12.0)
    assert _says(_voice(logic, "unknown", 13.0)) == [MSG_APPROACH_REASK]
    assert logic.state == State.AWAITING_USER


def test_non_answer_after_three_reasks_leaves_with_the_wake_word_hint():
    logic = _asking()
    t = _use_up_reasks(logic)
    actions = _voice(logic, "clarify", t)
    assert _says(actions) == [MSG_APPROACH_UNANSWERED]
    assert logic.state == State.RETURNING


def test_question_after_three_reasks_is_answered_then_leaves():
    """18:39 장면 — 마지막 질문에도 LLM 답이 먼저, 그다음 '비카야' 안내로 물러난다."""
    logic = _asking()
    t = _use_up_reasks(logic)
    reply = "안내를 받으시겠냐고 여쭤봤어요."
    assert _voice(logic, "question", t, reply=reply) == []
    actions = logic.on_approach_reply_spoken(reply, t + 1.5)
    assert _says(actions) == [MSG_APPROACH_UNANSWERED]
    assert logic.state == State.RETURNING


def test_plain_no_keeps_the_declined_words():
    """분명한 아니요는 다시 물은 뒤에도 "알겠습니다. 이만 물러납니다."다."""
    logic = _asking()
    _voice(logic, "unknown", 7.0)
    logic.on_approach_question_spoken(9.0)
    actions = _voice(logic, "deny", 10.0)
    assert _says(actions) == [MSG_APPROACH_DECLINED]
    assert logic.state == State.RETURNING


def test_yes_after_the_reask_is_accepted():
    logic = _asking()
    _voice(logic, "unknown", 7.0)
    logic.on_approach_question_spoken(9.0)
    assert MSG_APPROACH_ACCEPTED in _says(_voice(logic, "affirm", 10.0))
    assert logic.state == State.TURNING


def test_yes_after_the_third_reask_is_accepted():
    logic = _asking()
    t = _use_up_reasks(logic)
    assert MSG_APPROACH_ACCEPTED in _says(_voice(logic, "affirm", t))
    assert logic.state == State.TURNING


def test_no_answer_message_is_no_longer_used():
    """옛 동작('실례했습니다')은 다시 묻기·아니요로 바뀌었다."""
    logic = _asking()
    said = _says(_ticks(logic, 5.5, 40.0))
    assert MSG_APPROACH_NO_ANSWER not in said


def test_each_approach_gets_its_own_reasks():
    logic = _asking()
    t = _use_up_reasks(logic)
    _voice(logic, "unknown", t)
    assert logic.state == State.RETURNING
    logic.on_tick(t + 20.0, NavStatus.SUCCEEDED)
    req = ApproachRequest(goal=Pose2D(1.0, 0.5, 30.0, "map"), track_id=9, approachable=True)
    logic.on_approach_request(req, BOUNDS, True, t + 30.0)
    logic.on_tick(t + 35.0, NavStatus.SUCCEEDED)
    assert logic.state == State.AWAITING_USER
    logic.on_approach_question_spoken(t + 36.0)
    _use_up_reasks(logic, t + 37.0)


def test_clock_waits_while_the_person_is_speaking():
    logic = _asking()
    logic.on_listen_state("open", 6.0)
    logic.on_listen_state("speech", 9.0)
    assert _says(_ticks(logic, 5.5, 5.0 + T + 1.0)) == []
    assert logic.state == State.AWAITING_USER


def test_clock_waits_while_the_words_are_being_understood():
    """17:19 장면 — 말이 끝나(closed) LLM 이 알아듣는 동안은 기다리고, 그 답으로 정한다."""
    logic = _asking()
    logic.on_listen_state("open", 6.0)
    logic.on_listen_state("closed", 12.5)
    assert _says(_ticks(logic, 13.0, 15.0)) == []
    assert _says(_voice(logic, "clarify", 15.2)) == [MSG_APPROACH_REASK]


def test_ear_grace_is_eight_seconds():
    """10-10 사용자 결정(2-가): 6 -> 8초. LLM 이 늦으면 미션이 먼저 다시 묻고 곧이어 수락했다."""
    assert EAR_GRACE_SEC == 8.0


def test_clock_waits_eight_seconds_for_a_late_llm():
    """10-10 12:13 장면 — "그래."가 STT 를 지나(closed) Realtime 이 9초 만에 실패, 글자 경로의 affirm 은"""
    logic = _asking()
    logic.on_listen_state("open", 5.2)
    logic.on_listen_state("speech", 5.9)
    logic.on_listen_state("closed", 7.5)
    assert _says(_ticks(logic, 8.0, 15.0)) == []
    assert MSG_APPROACH_ACCEPTED in _says(_voice(logic, "affirm", 14.9))


def test_clock_still_ends_after_the_grace():
    logic = _asking()
    logic.on_listen_state("open", 5.2)
    logic.on_listen_state("closed", 7.5)
    assert _says(_ticks(logic, 8.0, 15.0)) == []
    assert _says(_ticks(logic, 15.5, 16.0)) == [MSG_APPROACH_REASK]


def test_clock_waits_while_the_llm_is_thinking():
    """10-10 12:13 장면을 시간과 무관하게 막는다 — Realtime 이 9초 걸려도 생각 중이면 기다린다."""
    logic = _asking()
    logic.on_listen_state("open", 5.2)
    logic.on_listen_state("closed", 7.5)
    logic.on_llm_thinking(True, 5.8)
    assert _says(_ticks(logic, 8.0, 18.0)) == []
    logic.on_llm_thinking(False, 18.2)
    assert MSG_APPROACH_ACCEPTED in _says(_voice(logic, "affirm", 18.21))


def test_short_tail_after_thinking_ends():
    """생각이 끝난 뒤 의도가 미션에 닿기까지 잠깐(LLM_THINKING_TAIL_SEC)은 더 기다린다."""
    logic = _asking()
    logic.on_llm_thinking(True, 6.0)
    logic.on_llm_thinking(False, 14.0)
    assert _says(_ticks(logic, 14.0, 14.0 + LLM_THINKING_TAIL_SEC - 0.1)) == []
    assert _says(_ticks(logic, 14.0 + LLM_THINKING_TAIL_SEC + 0.1, 15.5)) == [MSG_APPROACH_REASK]


def test_thinking_that_never_ends_is_capped():
    """LLM 노드가 생각 중 표시를 켠 채 죽어도 상한(EAR_HOLD_MAX_SEC) 뒤엔 다시 묻는다."""
    logic = _asking()
    logic.on_llm_thinking(True, 6.0)
    assert _says(_ticks(logic, 6.5, 6.0 + EAR_HOLD_MAX_SEC - 0.5)) == []
    assert _says(_ticks(logic, 6.0 + EAR_HOLD_MAX_SEC + 0.5, 6.0 + EAR_HOLD_MAX_SEC + 1.0)) == [MSG_APPROACH_REASK]


def test_thinking_true_twice_keeps_the_first_start():
    """글자 경로가 다시 켜도 상한은 처음 켠 때부터 센다(무한 연장 방지)."""
    logic = _asking()
    logic.on_llm_thinking(True, 6.0)
    logic.on_llm_thinking(True, 20.0)
    assert MSG_APPROACH_REASK in _says(_ticks(logic, 6.0 + EAR_HOLD_MAX_SEC + 0.5, 6.0 + EAR_HOLD_MAX_SEC + 1.0))


def _node_src():
    from pathlib import Path
    return (Path(__file__).resolve().parents[1] / "vica_mission_manager"
            / "mission_manager_node.py").read_text(encoding="utf-8")


def test_node_feeds_the_llm_thinking_signal_to_the_logic():
    src = _node_src()
    assert '"/vica/thinking"' in src
    assert "self.logic.on_llm_thinking(msg.data, self._now())" in src


def test_node_passes_the_llm_reply_to_the_logic():
    assert "reply=msg.reply," in _node_src()


def test_node_restarts_the_clock_on_either_question_and_reports_spoken_replies():
    src = _node_src()
    done = src[src.index("    def _on_tts_done"):src.index("    def _on_wake(self")]
    assert "if MSG_APPROACH_REASK in msg.data:" in done
    assert "self.logic.on_approach_reply_spoken(msg.data, self._now())" in done
