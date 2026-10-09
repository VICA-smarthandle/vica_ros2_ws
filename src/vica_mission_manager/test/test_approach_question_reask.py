"""접근 질문 '안내를 받으시겠어요?' (2026-10-09 사용자 결정, run82 실기 세 장면).

1. 이 질문의 답을 기다리는 동안 '비카야'는 무시한다 — 말을 끊지 않고 '네?'도 하지 않으며 질문을 접지 않는다
   (17:14·17:17 장면: '비카야'에 질문이 접혀 이어진 "그래"·"안내를 받을게요"가 버려졌다).
2. 예·아니요가 아닌 말(질문·못 알아들음·잠깐 등)이나 8초 침묵에는 '안내를 받으시겠어요?'로 다시 묻는다
   — 세 번까지(18:12 실기 뒤 사용자 결정, 처음엔 한 번이었다. '비카야'만 들려도 한 번을 써 버렸다).
   질문이었으면 LLM 의 짧은 답이 끝난 뒤 묻는다.
3. 세 번 다시 물은 뒤에도 예·아니요가 아니면(침묵 포함) 물러난다(홈 복귀). 이때 마지막 말은
   "필요하시면 '비카야'라고 불러 주세요."다 — 대답을 못 들었는데 "알겠습니다"는 맞지 않는다(18:39 장면,
   사용자 결정 '나'). 분명한 아니요는 그대로 "알겠습니다. 이만 물러납니다."
4. 사람이 말하는 중이거나 방금 한 말을 처리하는 중이면 8초가 지나도 떠나지 않는다
   (17:19 장면: "나 골랐어"를 처리하는 사이 "실례했습니다"로 떠났다).
"""
import pytest

from vica_mission_manager.mission_logic import (
    APPROACH_REASK_MAX,
    APPROACH_RESPONSE_TIMEOUT_SEC,
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


# ---- 1. '비카야' 무시 ------------------------------------------------------------------

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
    logic.on_wake(6.0)                                  # 창 안 구제(노드가 on_wake 만 부르는 길)
    assert logic.state == State.AWAITING_USER
    assert MSG_APPROACH_ACCEPTED in _says(_voice(logic, "affirm", 7.0))
    assert logic.state == State.TURNING


# ---- 2·3. 세 번까지 다시 묻기, 그다음 물러나기 ----------------------------------------------

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
        logic.on_approach_question_spoken(t)             # 다시 묻기 재생 끝 — 8초를 다시 센다


def test_silence_after_three_reasks_leaves_with_the_wake_word_hint():
    logic = _asking()
    t = 5.0
    for _ in range(APPROACH_REASK_MAX):
        _ticks(logic, t + 0.5, t + T + 0.5)              # 다시 묻기
        t += T + 2.0
        logic.on_approach_question_spoken(t)             # 다시 묻기 재생 끝
    actions = _ticks(logic, t + 0.5, t + T + 0.5)
    assert _says(actions) == [MSG_APPROACH_UNANSWERED]
    assert logic.state == State.RETURNING
    assert any(isinstance(a, Navigate) and a.destination == HOME for a in actions)   # 홈으로 간다


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
    t = _use_up_reasks(logic)                             # 이번 접근의 다시 묻기를 다 씀
    _voice(logic, "unknown", t)                           # 물러남
    assert logic.state == State.RETURNING
    logic.on_tick(t + 20.0, NavStatus.SUCCEEDED)          # 복귀 끝
    req = ApproachRequest(goal=Pose2D(1.0, 0.5, 30.0, "map"), track_id=9, approachable=True)
    logic.on_approach_request(req, BOUNDS, True, t + 30.0)
    logic.on_tick(t + 35.0, NavStatus.SUCCEEDED)
    assert logic.state == State.AWAITING_USER
    logic.on_approach_question_spoken(t + 36.0)
    _use_up_reasks(logic, t + 37.0)                       # 새 접근은 다시 세 번


# ---- 4. 말하는 중에는 떠나지 않는다 -------------------------------------------------------

def test_clock_waits_while_the_person_is_speaking():
    logic = _asking()
    logic.on_listen_state("open", 6.0)
    logic.on_listen_state("speech", 9.0)                 # 8초 시계가 끝나는 13초에도 말하는 중
    assert _says(_ticks(logic, 5.5, 5.0 + T + 1.0)) == []
    assert logic.state == State.AWAITING_USER


def test_clock_waits_while_the_words_are_being_understood():
    """17:19 장면 — 말이 끝나(closed) LLM 이 알아듣는 동안은 기다리고, 그 답으로 정한다."""
    logic = _asking()
    logic.on_listen_state("open", 6.0)
    logic.on_listen_state("closed", 12.5)                # 13초에 시계 끝 — 유예 중
    assert _says(_ticks(logic, 13.0, 15.0)) == []
    assert _says(_voice(logic, "clarify", 15.2)) == [MSG_APPROACH_REASK]


# ---- 노드 배선 (rclpy 없이 소스 글자로, test_handle_mode 방식) -------------------------------

def _node_src():
    from pathlib import Path
    return (Path(__file__).resolve().parents[1] / "vica_mission_manager"
            / "mission_manager_node.py").read_text(encoding="utf-8")


def test_node_passes_the_llm_reply_to_the_logic():
    assert "reply=msg.reply," in _node_src()


def test_node_restarts_the_clock_on_either_question_and_reports_spoken_replies():
    src = _node_src()
    done = src[src.index("    def _on_tts_done"):src.index("    def _on_wake(self")]
    assert "if MSG_APPROACH_REASK in msg.data:" in done          # 첫 질문의 끝도 이 글자다
    assert "self.logic.on_approach_reply_spoken(msg.data, self._now())" in done
