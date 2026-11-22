"""대답이 없을 때는 원래 질문을 다시 말한다 — reask_ladder 켜짐에서도(원칙 8 · 6 · 31, Task N4, 2026-11-23 사용자 결정 "가안").

2026-11-22 20:10 실기(reask_ladder 켬): 확인 질문 뒤 곁질문에 답한 다음 사용자가 말이 없자 질문한 지 15 초에 "잘 듣지 못했습니다.
다시 말씀해 주세요."가 나왔다 — 말을 안 했는데 못 들었다고 하고, 무엇을 물었는지도 다시 말하지 않았다("여기까지 안내를 마칠까요?" 뒤
조용 10 초도 같았다). 원칙 8: 대답이 없는 것은 못 알아들은 것이 아니다.
- 대답 없음(시계) 갈래는 꺼짐과 같다 — 다시 묻는 문장·시각·횟수·끝 동작과 그 시각. 시계로는 ①·②·③(직원 도움)이 나오지 않는다.
- 못 알아들은 말(unknown)의 사다리는 그대로다 — ① → ② → (주행 중) ③ → 끝.
- 섞일 때: ①·② 를 말하면 그때부터 그 질문의 시계와 끝 시한을 다시 잰다. 대답 없음으로 다시 말한 횟수와 사다리 단수는 따로 센다.
"""
import re

import pytest

from reaction_states import (BOUNDS, ELEV, asking, asking_end_confirm, asking_wait_time, awaiting_user, confirming,
                             confirming_change, dest, help_requested_confirm_asked, intent, lookup,
                             navigating_cancel_asked, navigating_help_asked, paused, paused_resume_asked, waiting,
                             waiting_asked, waiting_need_asked)
from vica_mission_manager.mission_logic import (
    MSG_APPROACH_REASK,
    MSG_ASK_ENTRANCE,
    MSG_ASK_RESTROOM,
    MSG_CANCEL_CONFIRM,
    MSG_CANCEL_KEPT,
    MSG_CONFIRM_PROMPT_FALLBACK,
    MSG_CONFIRM_TIMEOUT,
    MSG_HELP_CANCEL_CONFIRM,
    MSG_HELP_CONFIRM,
    MSG_LEAVING_NOTICE,
    MSG_REASK_LOUDER,
    MSG_REASK_RETRY,
    MSG_RESUME_CONFIRM,
    MSG_WAIT_BEACON,
    MSG_WAIT_FINISH_ASK,
    MSG_WAIT_NEED_ASK,
    MissionLogic,
    NavStatus,
    Say,
    State,
    say_destination,
)
from vica_mission_manager.reask_ladder import END, LOUDER, RETRY, SAME, ReaskLadder

TOILET_ASK = say_destination(MSG_CONFIRM_PROMPT_FALLBACK, "화장실")
NONE = NavStatus.NONE
UUID = re.compile(r"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}")


def _says(actions):
    return [a.text for a in actions if isinstance(a, Say)]


def _q(actions):
    """대기 중 10 초 알림(M3, ambient)은 빼고 본 말."""
    return [s for s in _says(actions) if s != MSG_WAIT_BEACON]


def _voice(logic, kind, t, **kw):
    return logic.on_voice_intent(intent(kind, **kw), t, lookup, BOUNDS, True)


# ---- 장면 — 미션이 쥔 질문마다(시계 갈래가 있는 것 전부 + 없는 것 둘) --------------------------------------------
def _help_cancel_asked(**kw):
    """도움 요청을 보낸 뒤(일시정지) "취소" → "도움 요청을 취소할까요?". 물은 시각은 돌려주는 시각보다 1초 앞이다."""
    logic, t = navigating_help_asked(**kw)
    _voice(logic, "affirm", t)
    _voice(logic, "cancel", t + 1)
    assert logic.help_cancel_pending
    return logic, t + 2.0


def _paused_cancel_asked(**kw):
    """일시정지 중 "취소" → "안내를 취소할까요?"(안내 주행이 아니라 ③ 이 없는 자리)."""
    logic, t = paused(**kw)
    _voice(logic, "cancel", t)
    assert logic.cancel_confirm_pending and logic.state == State.PAUSED
    return logic, t + 1.0


def _paused_proposal(**kw):
    """일시정지 중 다른 곳 제안 → 확인 질문(답이 없으면 일시정지 그대로 MSG_PAUSE_KEPT)."""
    logic, t = paused(**kw)
    _voice(logic, "navigate", t, matched_destination_id=ELEV.id, need_confirm=True)
    assert logic.state == State.CONFIRMING and logic._pause_hold
    return logic, t + 1.0


def _waiting_proposal(**kw):
    """대기 중 다른 곳 제안 → 확인 질문(답이 없으면 하던 대기로)."""
    logic, t = waiting(**kw)
    _voice(logic, "navigate", t, matched_destination_id=ELEV.id, need_confirm=True)
    assert logic.state == State.CONFIRMING
    return logic, t + 1.0


def _confirming_unknown_place(**kw):
    """미션이 모르는 목적지 제안 — 물을 문장이 없는 확인 질문(꺼짐은 대답 없음에 다시 묻지 않는다)."""
    logic = MissionLogic(arrival_dialog=True, **kw)
    logic.on_intent(intent(matched_destination_id="nope", need_confirm=True), None, BOUNDS, True, 0.0)
    assert logic.state == State.CONFIRMING and logic._confirm_prompt == ""
    return logic, 1.0


def _asking_entrance(**kw):
    """도착 질문(종료형) "여기까지 안내를 마칠까요?"."""
    return asking(dest("entrance", id="gate", name="정문"), **kw)


def _asking_generic(**kw):
    """도착 질문(그 외 유형)."""
    return asking(dest("reception", id="desk", name="안내데스크"), **kw)


FLAGS =[{}, {"wake_revive": True}, {"answer_field": True}, {"wake_revive": True, "answer_field": True}]
FLAG_IDS = ["plain", "wr", "af", "wr+af"]


def _end_confirm_spoken(**kw):
    """"여기까지 안내를 마칠까요?"(asking_end_confirm)의 재생이 끝난 뒤 — 노드는 재생 끝(tts_done)마다 8 초를 다시 센다."""
    logic, t = asking_end_confirm(**kw)
    logic.on_arrival_question_spoken(t)
    return logic, t + 1.0


SCENES = [confirming, confirming_change, _paused_proposal, _waiting_proposal, _confirming_unknown_place,
          navigating_cancel_asked, _paused_cancel_asked, navigating_help_asked, _help_cancel_asked, paused_resume_asked,
          asking, _asking_entrance, _asking_generic, asking_wait_time, _end_confirm_spoken,
          waiting_need_asked, waiting_asked, awaiting_user, help_requested_confirm_asked]
SCENE_IDS = [m.__name__.strip("_") for m in SCENES]


def _snapshot(logic, t):
    return (logic.state, logic.cancel_confirm_pending, logic.help_phase, logic.help_cancel_pending,
            logic._resume_question_open(), logic._open_question_name(t))


def _tts_done(logic, text, t):
    """노드 _on_tts_done 과 같은 순서 — 접근 질문 글자면 8 초, 접근 중 LLM 답의 끝, 도착 질문 8 초(글자 대조 없음), M2."""
    if MSG_APPROACH_REASK in text:
        logic.on_approach_question_spoken(t)
    acts = list(logic.on_approach_reply_spoken(text, t))
    logic.on_arrival_question_spoken(t)
    return acts + list(logic.on_wait_speech_spoken(text, t))


def _run(logic, t0, events=None, until=130.0):
    """노드처럼 0.5 s 마다 tick — 말(미션의 말·음성이 한 LLM 답)은 1 s 뒤 재생이 끝난다(_tts_done).
    events 는 {t0 부터의 초: 의도} — 그 시각에 음성 의도를 넣는다. 동작은 repr 로(질문 기록의 step·question_text 까지) 견준다."""
    events = dict(events or {})
    log, done = [], []
    for k in range(int(until / 0.5) + 1):
        t = t0 + k * 0.5
        acts = []
        for item in [d for d in done if d[0] <= t]:
            done.remove(item)
            acts += _tts_done(logic, item[1], t)
        event = events.get(k * 0.5)
        if event is not None:
            acts += logic.on_voice_intent(event, t, lookup, BOUNDS, True)
            if event.reply:
                done.append((t + 1.0, event.reply))
        acts += logic.on_tick(t, NONE)
        done += [(t + 1.0, a.text) for a in acts if isinstance(a, Say)]
        log.append((t, [UUID.sub("<id>", repr(a)) for a in acts], _snapshot(logic, t)))
    return log


# ---- R1: 대답 없음(시계) = 꺼짐과 같다 -----------------------------------------------------------------------------
# 알아들은 말(곁질문)·"다시 말해줘"가 끼어도 시계는 꺼짐과 같다 — 곁질문은 대답 없음 횟수를 처음으로 돌리지 않는다.
SIDE = intent("question", reply="화장실은 이 복도 끝 오른쪽에 있습니다.")    # 곁질문 — 음성이 LLM 답을 말한다
EVENTS = [None, {2.0: SIDE}, {2.0: SIDE, 16.0: SIDE}, {5.0: intent("repeat")}, {20.0: intent("repeat")}]
EVENT_IDS = ["silence", "side_question", "side_question_twice", "repeat_early", "repeat_late"]


@pytest.mark.parametrize("events", EVENTS, ids=EVENT_IDS)
@pytest.mark.parametrize("flags", FLAGS, ids=FLAG_IDS)
@pytest.mark.parametrize("make", SCENES, ids=SCENE_IDS)
def test_p8_silence_on_matches_off_word_for_word_and_second_for_second(make, flags, events):
    on, t = make(reask_ladder=True, **flags)
    off, t_off = make(**flags)
    assert t == t_off
    assert _run(on, t, events) == _run(off, t, events)


@pytest.mark.parametrize("flags", FLAGS, ids=FLAG_IDS)
@pytest.mark.parametrize("make", SCENES, ids=SCENE_IDS)
def test_p8_silence_never_says_the_ladder_lines_or_offers_staff(make, flags):
    """켜짐에서 시계로는 ①·② 문장이 나오지 않고 직원 도움 질문(③)으로 가지 않는다 — 도움 질문이 아니던 자리에서 도움 질문이 열리지 않는다."""
    logic, t = make(reask_ladder=True, **flags)
    help_before = logic.help_phase
    lines = [s for _, acts, _ in _run(logic, t) for s in acts]
    assert not any(MSG_REASK_RETRY in s or MSG_REASK_LOUDER in s for s in lines)
    if help_before != "asking":
        assert not any(f"'{MSG_HELP_CONFIRM}'" in s for s in lines)
        assert logic.help_phase == help_before or logic.help_phase == "none"


def test_p8_silence_on_the_driving_cancel_question_ends_quietly_without_staff_help():
    logic, t = navigating_cancel_asked(reask_ladder=True)         # t-1 에 물음
    asked = t - 1.0
    assert _says(logic.on_tick(asked + 15.0, NONE)) == [MSG_CANCEL_CONFIRM]
    assert _says(logic.on_tick(asked + 30.0, NONE)) == []
    assert not logic.cancel_confirm_pending and logic.help_phase == "none" and logic.state == State.NAVIGATING
    assert _says(logic.on_tick(asked + 45.0, NONE)) == [] and logic.help_phase == "none"


# ---- R2: 못 알아들은 말의 사다리는 그대로 -------------------------------------------------------------------------
UNKNOWN_ENDS = [
    (confirming, lambda lg, acts: _says(acts) == [MSG_CONFIRM_TIMEOUT] and lg.state == State.IDLE),
    (navigating_cancel_asked, lambda lg, acts: _says(acts) == [MSG_HELP_CONFIRM] and lg.help_phase == "asking"),
    (_paused_cancel_asked, lambda lg, acts: _says(acts) == [] and not lg.cancel_confirm_pending),
    (navigating_help_asked, lambda lg, acts: _says(acts) == [] and lg.help_phase == "none"),
    (_help_cancel_asked, lambda lg, acts: _says(acts) == [] and not lg.help_cancel_pending),
    (paused_resume_asked, lambda lg, acts: _says(acts) == [] and not lg._resume_question_open()),
    (asking, lambda lg, acts: _says(acts) == [MSG_LEAVING_NOTICE]),
    (waiting_need_asked, lambda lg, acts: _says(acts) == [MSG_CANCEL_KEPT] and lg.state == State.WAITING),
    (waiting_asked, lambda lg, acts: _says(acts) == [] and lg._wait_finish_asked_at is None),
]


@pytest.mark.parametrize("make, ended", UNKNOWN_ENDS, ids=[m.__name__.strip("_") for m, _ in UNKNOWN_ENDS])
def test_p8_misheard_answers_still_climb_retry_louder_then_the_end(make, ended):
    logic, t = make(reask_ladder=True)
    assert _says(_voice(logic, "unknown", t)) == [MSG_REASK_RETRY]
    assert _says(_voice(logic, "unknown", t + 1)) == [MSG_REASK_LOUDER]
    assert ended(logic, _voice(logic, "unknown", t + 2))


# ---- R3: 섞일 때 -------------------------------------------------------------------------------------------------
# 15 초 시계의 질문(물은 시각 = 돌려주는 시각 - 1) — 원래 문장, 끝내 대답이 없을 때의 말.
FIFTEEN = [
    (confirming, TOILET_ASK, [MSG_CONFIRM_TIMEOUT]),
    (navigating_cancel_asked, MSG_CANCEL_CONFIRM, []),
    (navigating_help_asked, MSG_HELP_CONFIRM, []),
    (_help_cancel_asked, MSG_HELP_CANCEL_CONFIRM, []),
    (paused_resume_asked, MSG_RESUME_CONFIRM, []),
]
FIFTEEN_IDS = ["confirm", "cancel", "help", "help_cancel", "resume"]


def _open(logic, t):
    return logic._open_question_name(t) is not None


@pytest.mark.parametrize("make, question, end", FIFTEEN, ids=FIFTEEN_IDS)
def test_p8_misheard_then_silence_says_the_question_then_ends_like_off(make, question, end):
    """(가) 못 알아들은 말 → ① → 15 초 조용 → 원래 질문 → 조용 → 꺼짐과 같은 끝(① 부터 30 초)."""
    logic, t = make(reask_ladder=True)
    u = t + 3.0
    assert _says(_voice(logic, "unknown", u)) == [MSG_REASK_RETRY]
    assert _says(logic.on_tick(u + 14.5, NONE)) == []
    acts = logic.on_tick(u + 15.0, NONE)
    assert _says(acts) == [question]
    (asked,) = [a for a in acts if isinstance(a, Say)]
    assert (asked.step, asked.question_text, asked.expects_reply) == (0, "", True)
    assert _says(logic.on_tick(u + 29.5, NONE)) == [] and _open(logic, u + 29.5)
    assert _says(logic.on_tick(u + 30.0, NONE)) == end and not _open(logic, u + 30.0)


@pytest.mark.parametrize("make, question, end", FIFTEEN, ids=FIFTEEN_IDS)
def test_p8_silence_then_misheard_starts_the_ladder_at_retry(make, question, end):
    """(나) 15 초 조용 → 원래 질문 → 못 알아들은 말 → ①. 대답 없음으로 다시 말하기는 이미 썼다(꺼짐이 허용하는 한 번) — 그 뒤 조용하면
    ① 부터 다시 잰 끝 시한(30 초)에 꺼짐과 같은 끝."""
    logic, t = make(reask_ladder=True)
    asked = t - 1.0
    assert _says(logic.on_tick(asked + 15.0, NONE)) == [question]
    assert _says(_voice(logic, "unknown", asked + 20.0)) == [MSG_REASK_RETRY]
    assert _says(logic.on_tick(asked + 35.0, NONE)) == [] and _open(logic, asked + 35.0)
    assert _says(logic.on_tick(asked + 49.5, NONE)) == [] and _open(logic, asked + 49.5)
    assert _says(logic.on_tick(asked + 50.0, NONE)) == end and not _open(logic, asked + 50.0)


@pytest.mark.parametrize("make, question, end", FIFTEEN, ids=FIFTEEN_IDS)
def test_p8_retry_restarts_the_deadline_so_the_question_does_not_fold_right_after(make, question, end):
    """(다) 시한 1 초 전의 ① — 다시 말해 달라고 했으니 꺼짐에서 질문을 새로 말했을 때와 같은 시간(15 초 다시 묻기·30 초 끝)을 준다."""
    logic, t = make(reask_ladder=True)
    asked = t - 1.0
    u = asked + 29.0
    assert _says(_voice(logic, "unknown", u)) == [MSG_REASK_RETRY]
    assert _says(logic.on_tick(asked + 30.5, NONE)) == [] and _open(logic, asked + 30.5)
    assert _says(logic.on_tick(u + 15.0, NONE)) == [question]
    assert _says(logic.on_tick(u + 29.5, NONE)) == [] and _open(logic, u + 29.5)
    assert _says(logic.on_tick(u + 30.0, NONE)) == end


def test_p8_louder_restarts_the_deadline_too():
    logic, _ = confirming(reask_ladder=True)                       # 0 s 에 물음
    _voice(logic, "unknown", 10.0)
    assert _says(_voice(logic, "unknown", 28.0)) == [MSG_REASK_LOUDER]
    assert _says(logic.on_tick(30.5, NONE)) == [] and logic.state == State.CONFIRMING
    assert _says(logic.on_tick(43.0, NONE)) == [TOILET_ASK]          # ② 부터 15 s — 대답 없음으로는 아직 안 썼다
    assert _says(logic.on_tick(58.0, NONE)) == [MSG_CONFIRM_TIMEOUT]


# 도착 질문(재생이 끝난 때부터 8 초) — asking(): 2 s 에 재생 끝 → 10 s 마감.
def test_p8_arrival_misheard_then_silence_says_the_question_then_leaves_like_off():
    logic, t = asking(reask_ladder=True)
    assert _says(_voice(logic, "unknown", t)) == [MSG_REASK_RETRY]
    logic.on_arrival_question_spoken(t + 1.0)
    assert _says(logic.on_tick(t + 8.5, NONE)) == []
    assert _says(logic.on_tick(t + 9.0, NONE)) == [MSG_ASK_RESTROOM]
    logic.on_arrival_question_spoken(t + 10.0)
    assert _says(logic.on_tick(t + 18.0, NONE)) == [MSG_LEAVING_NOTICE]
    logic.on_tick(t + 21.0, NONE)
    assert logic.state == State.RETURNING


def test_p8_arrival_silence_then_misheard_starts_at_retry():
    logic, _ = asking(reask_ladder=True)
    assert _says(logic.on_tick(10.5, NONE)) == [MSG_ASK_RESTROOM]
    logic.on_arrival_question_spoken(11.0)
    assert _says(_voice(logic, "unknown", 12.0)) == [MSG_REASK_RETRY]
    logic.on_arrival_question_spoken(13.0)
    assert _says(logic.on_tick(20.5, NONE)) == []
    assert _says(logic.on_tick(21.0, NONE)) == [MSG_LEAVING_NOTICE]


def test_p8_arrival_retry_during_the_leaving_notice_holds_the_departure():
    """떠나기 예고(3 s 유예) 중의 못 알아들은 말 → ① — 유예를 거둔다(① 을 말하고 곧바로 떠나지 않는다). 그 뒤 조용하면 다시 예고."""
    logic, _ = asking(reask_ladder=True)
    logic.on_tick(10.5, NONE)
    logic.on_arrival_question_spoken(11.0)
    assert _says(logic.on_tick(19.5, NONE)) == [MSG_LEAVING_NOTICE]
    assert _says(_voice(logic, "unknown", 20.0)) == [MSG_REASK_RETRY]
    logic.on_arrival_question_spoken(21.0)
    assert logic.on_tick(23.0, NONE) == [] and logic.state == State.ASKING_NEXT
    assert _says(logic.on_tick(28.5, NONE)) == []
    assert _says(logic.on_tick(29.0, NONE)) == [MSG_LEAVING_NOTICE]


# 대기 중 질문 — waiting_need_asked()·waiting_asked(): 11 s 에 물음, 12 s 를 돌려준다.
def test_p8_wait_need_misheard_then_silence_says_the_question_then_keeps_waiting():
    logic, t = waiting_need_asked(reask_ladder=True)
    assert _says(_voice(logic, "unknown", t)) == [MSG_REASK_RETRY]
    assert _q(logic.on_tick(t + 14.5, NONE)) == []
    assert _q(logic.on_tick(t + 15.0, NONE)) == [MSG_WAIT_NEED_ASK]
    assert _q(logic.on_tick(t + 29.5, NONE)) == []
    assert _q(logic.on_tick(t + 30.0, NONE)) == [MSG_CANCEL_KEPT]
    assert logic.state == State.WAITING and logic._wait_need_asked_at is None


def test_p8_wait_need_silence_then_misheard_starts_at_retry():
    logic, t = waiting_need_asked(reask_ladder=True)
    asked = t - 1.0
    assert _q(logic.on_tick(asked + 15.0, NONE)) == [MSG_WAIT_NEED_ASK]
    assert _says(_voice(logic, "unknown", asked + 19.0)) == [MSG_REASK_RETRY]
    assert _q(logic.on_tick(asked + 33.5, NONE)) == []
    assert _q(logic.on_tick(asked + 34.0, NONE)) == [MSG_CANCEL_KEPT]


def test_p8_wait_finish_misheard_then_silence_says_the_question_once():
    logic, t = waiting_asked(reask_ladder=True)
    assert _says(_voice(logic, "unknown", t)) == [MSG_REASK_RETRY]
    assert _q(logic.on_tick(t + 15.0, NONE)) == [MSG_WAIT_FINISH_ASK]
    later = [s for k in range(31, 120) for s in _q(logic.on_tick(t + k * 0.5 + 15.0, NONE))]
    assert later == []


def test_p8_wait_finish_silence_then_misheard_starts_at_retry():
    logic, t = waiting_asked(reask_ladder=True)
    asked = t - 1.0
    assert _q(logic.on_tick(asked + 15.0, NONE)) == [MSG_WAIT_FINISH_ASK]
    assert _says(_voice(logic, "unknown", asked + 19.0)) == [MSG_REASK_RETRY]
    later = [s for k in range(0, 100) for s in _q(logic.on_tick(asked + 19.5 + k * 0.5, NONE))]
    assert later == []


# ---- 2026-11-22 20:10 실기의 두 장면 --------------------------------------------------------------------------------
ON_SETS = [{"reask_ladder": True}, {"reask_ladder": True, "wake_revive": True, "answer_field": True}]
ON_IDS = ["ladder", "all_three"]


@pytest.mark.parametrize("flags", ON_SETS, ids=ON_IDS)
def test_p8_1122_side_question_then_silence_says_the_toilet_question_at_15s(flags):
    """"화장실로 안내해드릴까요?" → 사용자가 다른 것을 물어 답함(question) → 말 없음 → 질문한 지 15 초에 같은 질문."""
    assert TOILET_ASK == "화장실로 안내해드릴까요?"
    logic, _ = confirming(**flags)                                  # 0 s 에 물음
    _voice(logic, "question", 4.0, reply="화장실은 이 복도 끝 오른쪽에 있습니다.")
    assert logic.state == State.CONFIRMING
    assert _says(logic.on_tick(14.5, NONE)) == []
    acts = logic.on_tick(15.0, NONE)
    assert _says(acts) == ["화장실로 안내해드릴까요?"]
    (asked,) = [a for a in acts if isinstance(a, Say)]
    assert (asked.kind, asked.step, asked.question_text, asked.expects_reply) == ("confirm", 0, "", True)
    assert _says(logic.on_tick(29.5, NONE)) == [] and logic.state == State.CONFIRMING
    assert _says(logic.on_tick(30.0, NONE)) == [MSG_CONFIRM_TIMEOUT]   # 꺼짐과 같은 30 s 끝


@pytest.mark.parametrize("flags", ON_SETS, ids=ON_IDS)
def test_p8_1122_end_question_silence_says_it_again(flags):
    """"여기까지 안내를 마칠까요?" 뒤 조용(재생 끝부터 8 초) → 같은 질문 다시, 또 조용하면 떠나기 예고(꺼짐과 같다)."""
    assert MSG_ASK_ENTRANCE == "여기까지 안내를 마칠까요?"
    logic, t = asking_end_confirm(**flags)
    logic.on_arrival_question_spoken(t)
    assert _says(logic.on_tick(t + 7.5, NONE)) == []
    acts = logic.on_tick(t + 8.0, NONE)
    assert _says(acts) == ["여기까지 안내를 마칠까요?"]
    (asked,) = [a for a in acts if isinstance(a, Say)]
    assert (asked.step, asked.question_text) == (0, "")
    logic.on_arrival_question_spoken(t + 10.0)
    assert _says(logic.on_tick(t + 18.0, NONE)) == [MSG_LEAVING_NOTICE]


# ---- 사다리 — 대답 없음 횟수(순수 로직) ------------------------------------------------------------------------------
def test_p8_ladder_counts_silence_apart_from_misheard_words_when_on():
    ladder = ReaskLadder(three_step=True)
    assert ladder.next(driving=False) == RETRY
    assert ladder.quiet_count == 0
    assert ladder.next_quiet() == SAME and ladder.quiet_count == 1
    assert ladder.next(driving=False) == LOUDER                     # 못 알아들은 말의 단수는 그대로 이어진다
    assert ladder.next_quiet() == END
    ladder.restart_heard()                                          # 알아들은 말 — 못 알아들은 말의 단수만 처음으로
    assert (ladder.step, ladder.quiet_count) == (0, 2)
    ladder.reset()                                                  # 질문을 새로 열면 둘 다 처음으로
    assert (ladder.step, ladder.quiet_count) == (0, 0)


def test_off_ladder_silence_and_misheard_share_one_mark():
    ladder = ReaskLadder()
    assert ladder.next_quiet() == SAME and (ladder.step, ladder.quiet_count) == (1, 1)
    assert ladder.next(driving=True) == END and ladder.next_quiet() == END
    other = ReaskLadder()
    assert other.next(driving=False) == SAME and other.quiet_count == 1   # 못 알아들은 말이 그 한 번을 썼다
