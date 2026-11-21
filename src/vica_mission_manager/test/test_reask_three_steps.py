"""세 단계 되묻기 — 파라미터 reask_ladder 가 켜졌을 때만(원칙 8 · 9 · 21 · 31, Task 3.8).

켜지면 미션 질문이 열린 채 못 알아들었거나(unknown) 대답이 없을 때 ① "잘 듣지 못했습니다. 다시 말씀해 주세요."
② "죄송합니다. 더 크게, 또박또박 말씀해 주세요." ③ 안내 주행 중에만 "직원의 도움이 필요하신가요?" 그다음은 그 질문이
끝까지 대답을 못 받았을 때 하던 일. ①② 는 질문을 다시 말하지 않고 그 문장만 말한다 — 질문의 종류·후보는 그대로,
질문 기록에는 원래 질문(question_text)과 단계(step). 질문이 없는 상태의 못 알아들은 말도 같은 순서로 되묻는다.
꺼져 있으면(기본값) 지금 그대로 — 같은 질문을 한 번 다시.
"""
from pathlib import Path

from reaction_states import (BOUNDS, asking, asking_wait_time, confirming, estopped, idle, intent, lookup,
                             navigating, waiting, waiting_asked)
from vica_mission_manager.mission_logic import (
    MSG_ASK_RESTROOM,
    MSG_ASK_WAIT_TIME,
    MSG_CANCEL_CONFIRM,
    MSG_CANCEL_KEPT,
    MSG_CONFIRM_PROMPT_FALLBACK,
    MSG_CONFIRM_TIMEOUT,
    MSG_HELP_CANCEL_CONFIRM,
    MSG_HELP_CONFIRM,
    MSG_LEAVING_NOTICE,
    MSG_REASK_LOUDER,
    MSG_REASK_RETRY,
    MSG_WAIT_FINISH_ASK,
    MSG_WAIT_NEED_ASK,
    MSG_WAKE_GREETING,
    MissionLogic,
    NavStatus,
    Say,
    State,
    say_destination,
)
from vica_mission_manager.question_record import QuestionTracker

TOILET_ASK = say_destination(MSG_CONFIRM_PROMPT_FALLBACK, "화장실")
PKG = Path(__file__).resolve().parents[1]


def _says(actions):
    return [a.text for a in actions if isinstance(a, Say)]


def _question(acts):
    """동작 가운데 kind 가 있는 Say(질문 기록에 실릴 말)."""
    found = [a for a in acts if isinstance(a, Say) and a.kind]
    assert len(found) == 1, acts
    return found[0]


def _voice(logic, kind, t, **kw):
    return logic.on_voice_intent(intent(kind, **kw), t, lookup, BOUNDS, True)


# ---- 설명서 Step 1 의 시험 ---------------------------------------------------------
def test_p8_confirm_question_goes_retry_louder_then_folds_when_not_driving():
    logic, t = confirming(reask_ladder=True)
    r1 = logic.on_voice_intent(intent("unknown"), t, lookup, BOUNDS, True)
    r2 = logic.on_voice_intent(intent("unknown"), t + 5, lookup, BOUNDS, True)
    assert _says(r1) == [MSG_REASK_RETRY] and _says(r2) == [MSG_REASK_LOUDER]
    assert _question(r1).question_text == TOILET_ASK and _question(r1).kind == "confirm"
    assert (_question(r1).step, _question(r2).step) == (1, 2)
    logic.on_voice_intent(intent("unknown"), t + 10, lookup, BOUNDS, True)
    assert logic.state != State.CONFIRMING


def test_p21_third_step_offers_staff_only_while_driving():
    logic, t = navigating(reask_ladder=True)
    says = [_says(logic.on_voice_intent(intent("unknown"), t + i, lookup, BOUNDS, True)) for i in range(3)]
    assert says == [[MSG_REASK_RETRY], [MSG_REASK_LOUDER], [MSG_HELP_CONFIRM]]
    logic, t = idle(reask_ladder=True)
    says = [_says(logic.on_voice_intent(intent("unknown"), t + i, lookup, BOUNDS, True)) for i in range(3)]
    assert says == [[MSG_REASK_RETRY], [MSG_REASK_LOUDER], []]


def test_p9_the_voice_handed_retry_sentence_is_not_said_twice():
    logic, t = idle(reask_ladder=True)
    acts = logic.on_voice_intent(intent("unknown", ask_text=MSG_REASK_RETRY, ask_kind="open"), t, lookup, BOUNDS, True)
    assert _says(acts) == [MSG_REASK_RETRY]


def test_p8_silence_and_not_understood_share_one_count():
    logic, t = confirming(reask_ladder=True)
    assert _says(logic.on_tick(t + 16, NavStatus.NONE)) == [MSG_REASK_RETRY]            # 대답 없음
    assert _says(logic.on_voice_intent(intent("unknown"), t + 20, lookup, BOUNDS, True)) == [MSG_REASK_LOUDER]


def test_off_still_reasks_the_same_question_once():
    logic, t = confirming()
    ask = say_destination(MSG_CONFIRM_PROMPT_FALLBACK, "화장실")
    assert _says(logic.on_voice_intent(intent("unknown"), t, lookup, BOUNDS, True)) == [ask]


# ---- 꺼짐 = 지금 그대로 -------------------------------------------------------------
def test_off_is_the_default_and_reasks_carry_no_step_or_question_text():
    logic, t = confirming()
    assert MissionLogic().reask_ladder is False
    say = _question(_voice(logic, "unknown", t))
    assert (say.text, say.step, say.question_text) == (TOILET_ASK, 0, "")
    assert _voice(logic, "unknown", t + 1) == []                       # 두 번째는 흘려보낸다(옛 방식)
    assert logic.state == State.CONFIRMING                             # 시한(30 s)에 접힌다
    assert _says(logic.on_tick(30.5, NavStatus.NONE)) == [MSG_CONFIRM_TIMEOUT]


def test_off_idle_unknown_stays_silent():
    logic, t = idle()
    assert [_voice(logic, "unknown", t + i) for i in range(3)] == [[], [], []]
    logic, t = navigating()
    assert [_voice(logic, "unknown", t + i) for i in range(3)] == [[], [], []]


def test_on_switches_all_five_question_ladders():
    on, off = MissionLogic(reask_ladder=True), MissionLogic()
    names = ("_confirm_ladder", "_cancel_ladder", "_help_ladder", "_wait_ladder", "_arrival_ladder")
    assert all(getattr(on, n).three_step for n in names)
    assert not any(getattr(off, n).three_step for n in names)


# ---- 문장 --------------------------------------------------------------------------
def test_p8_reask_sentences_word_for_word():
    assert MSG_REASK_RETRY == "잘 듣지 못했습니다. 다시 말씀해 주세요."
    assert MSG_REASK_LOUDER == "죄송합니다. 더 크게, 또박또박 말씀해 주세요."


# ---- 확인 질문: 대답 없음만으로 ① → ② → 끝 ------------------------------------------------
def test_p8_confirm_silence_steps_every_15s_then_folds():
    logic, t = confirming(reask_ladder=True)          # 0 초에 물음
    assert _says(logic.on_tick(14.9, NavStatus.NONE)) == []
    first = logic.on_tick(15.0, NavStatus.NONE)
    assert _says(first) == [MSG_REASK_RETRY] and _question(first).question_text == TOILET_ASK
    assert _question(first).options == (("wc", "화장실"),) and _question(first).expects_reply
    assert _says(logic.on_tick(29.9, NavStatus.NONE)) == []             # 옛 30 s 시한은 접지 않는다
    assert logic.state == State.CONFIRMING
    second = logic.on_tick(30.0, NavStatus.NONE)
    assert _says(second) == [MSG_REASK_LOUDER] and _question(second).step == 2
    assert _says(logic.on_tick(44.9, NavStatus.NONE)) == []
    assert _says(logic.on_tick(45.0, NavStatus.NONE)) == [MSG_CONFIRM_TIMEOUT]
    assert logic.state == State.IDLE


def test_p8_confirm_silence_waits_while_the_user_is_speaking():
    logic, t = confirming(reask_ladder=True)
    logic.on_listen_state("speech", 14.0)
    assert _says(logic.on_tick(15.0, NavStatus.NONE)) == []
    logic.on_listen_state("empty:short-reject", 16.0)
    assert _says(logic.on_tick(16.1, NavStatus.NONE)) == [MSG_REASK_RETRY]


# ---- 안내 취소 확인(주행 중): ① ② ③ 직원 도움 --------------------------------------------
def test_p21_cancel_question_while_driving_goes_retry_louder_help():
    logic, t = navigating(reask_ladder=True)
    assert _says(_voice(logic, "cancel", t)) == [MSG_CANCEL_CONFIRM]
    r1, r2, r3 = (_voice(logic, "unknown", t + i) for i in (1, 2, 3))
    assert [_says(r) for r in (r1, r2, r3)] == [[MSG_REASK_RETRY], [MSG_REASK_LOUDER], [MSG_HELP_CONFIRM]]
    assert _question(r1).question_text == MSG_CANCEL_CONFIRM and _question(r1).kind == "yesno"
    assert not logic.cancel_confirm_pending and logic.help_phase == "asking"   # 도움 요청 길 그대로
    assert logic.state == State.NAVIGATING


def test_p21_cancel_question_silence_also_reaches_help_while_driving():
    logic, t = navigating(reask_ladder=True)
    _voice(logic, "cancel", t)                                       # t 에 물음
    assert _says(logic.on_tick(t + 15, NavStatus.NONE)) == [MSG_REASK_RETRY]
    assert _says(logic.on_tick(t + 30, NavStatus.NONE)) == [MSG_REASK_LOUDER]
    assert _says(logic.on_tick(t + 45, NavStatus.NONE)) == [MSG_HELP_CONFIRM]


def test_p8_cancel_question_not_driving_closes_quietly():
    logic, t = navigating(reask_ladder=True)
    _voice(logic, "pause", t)
    assert logic.state == State.PAUSED
    _voice(logic, "cancel", t + 1)
    assert logic.cancel_confirm_pending
    says = [_says(_voice(logic, "unknown", t + 2 + i)) for i in range(3)]
    assert says == [[MSG_REASK_RETRY], [MSG_REASK_LOUDER], []]
    assert not logic.cancel_confirm_pending and logic.help_phase == "none" and logic.state == State.PAUSED


# ---- 도움 질문: ③ 없음 -----------------------------------------------------------------
def test_p21_help_question_goes_retry_louder_then_drops_without_a_third():
    logic, t = navigating(reask_ladder=True)
    assert _says(_voice(logic, "help", t)) == [MSG_HELP_CONFIRM]
    says = [_says(_voice(logic, "unknown", t + 1 + i)) for i in range(3)]
    assert says == [[MSG_REASK_RETRY], [MSG_REASK_LOUDER], []]
    assert logic.help_phase == "none" and logic.state == State.NAVIGATING


def test_p8_help_question_silence_steps_then_drops():
    logic, t = navigating(reask_ladder=True)
    _voice(logic, "help", t)
    assert _says(logic.on_tick(t + 15, NavStatus.NONE)) == [MSG_REASK_RETRY]
    assert _says(logic.on_tick(t + 30, NavStatus.NONE)) == [MSG_REASK_LOUDER]
    assert _says(logic.on_tick(t + 45, NavStatus.NONE)) == []
    assert logic.help_phase == "none"


def test_p8_help_cancel_question_steps_then_keeps_the_request():
    logic, t = navigating(reask_ladder=True)
    _voice(logic, "help", t)
    _voice(logic, "affirm", t + 1)                                   # 도움 요청 보냄(일시정지)
    assert _says(_voice(logic, "cancel", t + 2)) == [MSG_HELP_CANCEL_CONFIRM]
    first = _voice(logic, "unknown", t + 3)
    assert _says(first) == [MSG_REASK_RETRY] and _question(first).question_text == MSG_HELP_CANCEL_CONFIRM
    assert _says(logic.on_tick(t + 3 + 15, NavStatus.NONE)) == [MSG_REASK_LOUDER]
    assert _says(logic.on_tick(t + 3 + 30, NavStatus.NONE)) == []
    assert not logic.help_cancel_pending and logic.help_phase == "requested"


# ---- 도착 질문: ① ② 떠나기 예고 -----------------------------------------------------------
def test_p8_arrival_question_goes_retry_louder_then_leaving_notice():
    logic, t = asking(reask_ladder=True)
    r1, r2, r3 = (_voice(logic, "unknown", t + i) for i in range(3))
    assert [_says(r) for r in (r1, r2, r3)] == [[MSG_REASK_RETRY], [MSG_REASK_LOUDER], [MSG_LEAVING_NOTICE]]
    assert _question(r1).question_text == MSG_ASK_RESTROOM and _question(r1).kind == "yesno"
    assert _question(r2).step == 2


def test_p8_arrival_silence_keeps_its_8s_clock():
    logic, _ = asking(reask_ladder=True)                             # 2 초에 질문 재생 끝 → 10 초 마감
    assert _says(logic.on_tick(10.5, NavStatus.NONE)) == [MSG_REASK_RETRY]
    logic.on_arrival_question_spoken(11.0)
    assert _says(logic.on_tick(19.5, NavStatus.NONE)) == [MSG_REASK_LOUDER]
    logic.on_arrival_question_spoken(20.0)
    assert _says(logic.on_tick(28.5, NavStatus.NONE)) == [MSG_LEAVING_NOTICE]


def test_p8_wait_time_yes_still_asks_the_same_question_then_waits_30_minutes():
    logic, t = asking_wait_time(reask_ladder=True)
    assert _says(_voice(logic, "affirm", t)) == [MSG_ASK_WAIT_TIME]       # 사다리 값이 ① 이어도 같은 질문
    _voice(logic, "affirm", t + 1)
    assert logic.state == State.WAITING and logic.wait_minutes_requested() == 30


# ---- 대기 질문 ---------------------------------------------------------------------
def test_p8_wait_need_question_steps_then_keeps_waiting():
    logic, t = waiting(reask_ladder=True)
    assert _says(_voice(logic, "cancel", t)) == [MSG_WAIT_NEED_ASK]
    r1 = _voice(logic, "unknown", t + 1)
    assert _says(r1) == [MSG_REASK_RETRY] and _question(r1).kind == "yesno_inverted"
    assert _says(logic.on_tick(t + 1 + 15, NavStatus.NONE)) == [MSG_REASK_LOUDER]
    assert _says(logic.on_tick(t + 1 + 30, NavStatus.NONE)) == [MSG_CANCEL_KEPT]
    assert logic.state == State.WAITING


def test_p8_wait_finish_question_silence_steps_then_closes():
    logic, t = waiting_asked(reask_ladder=True)                      # t-1 에 물음
    first = logic.on_tick(t + 14.5, NavStatus.NONE)
    assert _says(first) == [MSG_REASK_RETRY] and _question(first).question_text == MSG_WAIT_FINISH_ASK
    assert _says(logic.on_tick(t + 29.5, NavStatus.NONE)) == [MSG_REASK_LOUDER]
    assert MSG_REASK_LOUDER not in _says(logic.on_tick(t + 44.5, NavStatus.NONE))
    assert logic._wait_finish_asked_at is None and logic.state == State.WAITING


# ---- 알아들은 말 · 호출 = 처음부터 -----------------------------------------------------------
def test_p8_an_understood_word_starts_the_count_over():
    logic, t = confirming(reask_ladder=True)
    assert _says(_voice(logic, "unknown", t)) == [MSG_REASK_RETRY]
    _voice(logic, "question", t + 1)                                  # 질문과 상관없는 정보 질문(LLM 이 답함)
    assert logic.state == State.CONFIRMING
    assert _says(_voice(logic, "unknown", t + 2)) == [MSG_REASK_RETRY]


def test_p8_a_dropped_answer_does_not_start_the_count_over():
    """질문 전에 시작한 "네"는 버린다(2026-10-11) — 처리하지 않은 말이라 횟수도 그대로다."""
    logic, t = confirming(reask_ladder=True)
    assert _says(_voice(logic, "unknown", t)) == [MSG_REASK_RETRY]
    logic.on_listen_state("speech", t + 0.5)
    logic.note_question_asked(MSG_REASK_RETRY, t + 1)               # 되물음이 나간 시각 — 말은 그보다 먼저 시작
    assert _voice(logic, "affirm", t + 2) == [] and logic.state == State.CONFIRMING
    assert _says(_voice(logic, "unknown", t + 3)) == [MSG_REASK_LOUDER]


def test_p8_idle_count_starts_over_on_wake():
    logic, t = idle(reask_ladder=True)
    assert _says(_voice(logic, "unknown", t)) == [MSG_REASK_RETRY]
    assert _says(_voice(logic, "unknown", t + 1)) == [MSG_REASK_LOUDER]
    logic.on_wake_call(t + 2)
    assert _says(_voice(logic, "unknown", t + 3)) == [MSG_REASK_RETRY]


def test_p8_idle_count_starts_over_on_an_understood_word():
    logic, t = idle(reask_ladder=True)
    _voice(logic, "unknown", t)
    _voice(logic, "question", t + 1)
    assert _says(_voice(logic, "unknown", t + 2)) == [MSG_REASK_RETRY]


def test_p8_idle_reask_is_an_open_question():
    logic, t = idle(reask_ladder=True)
    say = _question(_voice(logic, "unknown", t))
    assert (say.kind, say.expects_reply, say.question_text, say.step) == ("open", True, "", 1)


def test_p8_no_reask_where_voice_is_not_taken():
    logic, t = estopped(reask_ladder=True)
    assert [_voice(logic, "unknown", t + i) for i in range(2)] == [[], []]


# ---- 원칙 9: 음성이 넘긴 되묻는 문장 --------------------------------------------------------
def test_p9_handed_retry_is_dropped_even_when_the_ladder_is_done():
    logic, t = idle(reask_ladder=True)
    handed = dict(ask_text=MSG_REASK_RETRY, ask_kind="open")
    _voice(logic, "unknown", t, **handed)
    _voice(logic, "unknown", t + 1, **handed)
    assert _voice(logic, "unknown", t + 2, **handed) == []           # 사다리 끝 — 음성 문장도 말하지 않는다


def test_p9_handed_wake_greeting_keeps_todays_path():
    """"네?"(호출어만 들림)는 지금처럼 넘겨받은 질문 길로 간다 — 미션이 이미 말했으면 버리고, 아니면 말한다(3.9 가 다룬다)."""
    logic, t = idle(reask_ladder=True)
    for i in range(2):
        _voice(logic, "unknown", t + i)
    acts = _voice(logic, "unknown", t + 2, ask_text=MSG_WAKE_GREETING, ask_kind="open")
    assert _says(acts) == [MSG_WAKE_GREETING]


def test_off_handed_retry_is_said_as_today():
    logic, t = idle()
    acts = _voice(logic, "unknown", t, ask_text=MSG_REASK_RETRY, ask_kind="open")
    assert _says(acts) == [MSG_REASK_RETRY] and _question(acts).step == 0


# ---- 질문 기록 · 노드 · launch ------------------------------------------------------------
def test_p8_question_record_carries_the_original_question_and_step():
    tracker = QuestionTracker("q")
    say = Say(MSG_REASK_LOUDER, priority="response", expects_reply=True, kind="confirm",
              options=(("wc", "화장실"),), question_text=TOILET_ASK, step=2)
    _, record = tracker.open(say, "confirming", 1.0)
    assert (record.text, record.step, record.kind, record.options) == (TOILET_ASK, 2, "confirm", (("wc", "화장실"),))
    _, plain = tracker.open(Say(TOILET_ASK, expects_reply=True, kind="confirm"), "confirming", 2.0)
    assert (plain.text, plain.step) == (TOILET_ASK, 0)


def test_node_and_launch_expose_reask_ladder_off_by_default():
    node = (PKG / "vica_mission_manager" / "mission_manager_node.py").read_text(encoding="utf-8")
    launch = (PKG / "launch" / "mission_manager.launch.py").read_text(encoding="utf-8")
    assert 'self.declare_parameter("reask_ladder", False)' in node
    assert 'reask_ladder=bool(self.get_parameter("reask_ladder").value)' in node
    assert 'DeclareLaunchArgument("reask_ladder", default_value="false")' in launch
    assert '"reask_ladder": ParameterValue(' in launch
