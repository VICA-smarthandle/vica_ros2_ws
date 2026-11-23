"""대답이 없을 때는 원래 질문을 다시 말한다 — reask_ladder 켜짐에서도(원칙 8 · 6 · 31, Task N4, 2026-11-23 사용자 결정 "가안")."""
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
    MSG_RESUME_CONFIRM,
    MSG_WAIT_BEACON,
    MSG_WAIT_FINISH_ASK,
    MSG_WAIT_NEED_ASK,
    CancelNav,
    MissionLogic,
    Navigate,
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


def _drive(logic, t0, voice=None, until=130.0):
    """노드처럼 0.5 s 마다 tick — 말(미션의 말·음성이 한 LLM 답)은 1 s 뒤 재생이 끝난다(_tts_done). (시각, 동작 목록)을 낸다."""
    pick = voice if callable(voice) else (lambda t, ev=dict(voice or {}): ev.get(round(t - t0, 1)))
    done = []
    for k in range(int(until / 0.5) + 1):
        t = t0 + k * 0.5
        acts = []
        for item in [d for d in done if d[0] <= t]:
            done.remove(item)
            acts += _tts_done(logic, item[1], t)
        event = pick(t)
        if event is not None:
            acts += logic.on_voice_intent(event, t, lookup, BOUNDS, True)
            if event.reply:
                done.append((t + 1.0, event.reply))
        acts += logic.on_tick(t, NONE)
        done += [(t + 1.0, a.text) for a in acts if isinstance(a, Say)]
        yield t, acts


def _run(logic, t0, events=None, until=130.0):
    """_drive 의 기록 — 동작은 repr 로(질문 기록의 step·question_text 까지) 견준다."""
    return [(t, [UUID.sub("<id>", repr(a)) for a in acts], _snapshot(logic, t))
            for t, acts in _drive(logic, t0, events, until)]


def _timeline(logic, t0, events=None, until=130.0):
    """_drive 의 말(대기 알림 ambient 빼고)·주행 동작 — (시각, 글자 또는 동작 이름). 도움 요청 재전송(벽시계 10 s)은 뺀다."""
    out = []
    for t, acts in _drive(logic, t0, events, until):
        for a in acts:
            if isinstance(a, Say) and a.priority != "ambient":
                out.append((t, a.text))
            elif isinstance(a, (Navigate, CancelNav)):
                out.append((t, type(a).__name__))
    return out


def _first_resay(make, flags):
    """꺼짐 조용 장면에서 시계가 처음 질문을 다시 말한 시각(t0 부터의 초). 없으면 None."""
    logic, t0 = make(**flags)
    for t, acts in _drive(logic, t0):
        if any(isinstance(a, Say) and a.expects_reply for a in acts):
            return round(t - t0, 1)
    return None


SIDE = intent("question", reply="화장실은 이 복도 끝 오른쪽에 있습니다.")
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
    logic, t = navigating_cancel_asked(reask_ladder=True)
    asked = t - 1.0
    assert _says(logic.on_tick(asked + 15.0, NONE)) == [MSG_CANCEL_CONFIRM]
    assert _says(logic.on_tick(asked + 30.0, NONE)) == []
    assert not logic.cancel_confirm_pending and logic.help_phase == "none" and logic.state == State.NAVIGATING
    assert _says(logic.on_tick(asked + 45.0, NONE)) == [] and logic.help_phase == "none"


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
    """(나) 15 초 조용 → 원래 질문 → 못 알아들은 말 → ①. ① 뒤는 꺼짐에서 그 질문을 ① 의 시각에 새로 물었을 때와 같다(검토 1) —"""
    logic, t = make(reask_ladder=True)
    asked = t - 1.0
    assert _says(logic.on_tick(asked + 15.0, NONE)) == [question]
    assert _says(_voice(logic, "unknown", asked + 20.0)) == [MSG_REASK_RETRY]
    assert _says(logic.on_tick(asked + 34.5, NONE)) == [] and _open(logic, asked + 34.5)
    assert _says(logic.on_tick(asked + 35.0, NONE)) == [question]
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
    logic, _ = confirming(reask_ladder=True)
    _voice(logic, "unknown", 10.0)
    assert _says(_voice(logic, "unknown", 28.0)) == [MSG_REASK_LOUDER]
    assert _says(logic.on_tick(30.5, NONE)) == [] and logic.state == State.CONFIRMING
    assert _says(logic.on_tick(43.0, NONE)) == [TOILET_ASK]
    assert _says(logic.on_tick(58.0, NONE)) == [MSG_CONFIRM_TIMEOUT]


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
    """① 뒤는 새로 물었을 때와 같다(검토 1) — 재생 끝 8 초에 원래 질문, 또 8 초에 떠나기 예고."""
    logic, _ = asking(reask_ladder=True)
    assert _says(logic.on_tick(10.5, NONE)) == [MSG_ASK_RESTROOM]
    logic.on_arrival_question_spoken(11.0)
    assert _says(_voice(logic, "unknown", 12.0)) == [MSG_REASK_RETRY]
    logic.on_arrival_question_spoken(13.0)
    assert _says(logic.on_tick(20.5, NONE)) == []
    assert _says(logic.on_tick(21.0, NONE)) == [MSG_ASK_RESTROOM]
    logic.on_arrival_question_spoken(22.0)
    assert _says(logic.on_tick(29.5, NONE)) == []
    assert _says(logic.on_tick(30.0, NONE)) == [MSG_LEAVING_NOTICE]


def test_p8_arrival_retry_during_the_leaving_notice_holds_the_departure():
    """떠나기 예고(3 s 유예) 중의 못 알아들은 말 → ① — 유예를 거둔다(① 을 말하고 곧바로 떠나지 않는다). 그 뒤는 새로 물었을 때와"""
    logic, _ = asking(reask_ladder=True)
    logic.on_tick(10.5, NONE)
    logic.on_arrival_question_spoken(11.0)
    assert _says(logic.on_tick(19.5, NONE)) == [MSG_LEAVING_NOTICE]
    assert _says(_voice(logic, "unknown", 20.0)) == [MSG_REASK_RETRY]
    logic.on_arrival_question_spoken(21.0)
    assert logic.on_tick(23.0, NONE) == [] and logic.state == State.ASKING_NEXT
    assert _says(logic.on_tick(28.5, NONE)) == []
    assert _says(logic.on_tick(29.0, NONE)) == [MSG_ASK_RESTROOM]
    logic.on_arrival_question_spoken(30.0)
    assert _says(logic.on_tick(38.0, NONE)) == [MSG_LEAVING_NOTICE]


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
    assert _q(logic.on_tick(asked + 34.0, NONE)) == [MSG_WAIT_NEED_ASK]
    assert _q(logic.on_tick(asked + 48.5, NONE)) == []
    assert _q(logic.on_tick(asked + 49.0, NONE)) == [MSG_CANCEL_KEPT]


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
    assert _q(logic.on_tick(asked + 33.5, NONE)) == []
    assert _q(logic.on_tick(asked + 34.0, NONE)) == [MSG_WAIT_FINISH_ASK]
    later = [s for k in range(1, 100) for s in _q(logic.on_tick(asked + 34.0 + k * 0.5, NONE))]
    assert later == []


AFTER_RETRY = [confirming, confirming_change, _paused_proposal, _waiting_proposal, _confirming_unknown_place,
               navigating_cancel_asked, _paused_cancel_asked, navigating_help_asked, _help_cancel_asked, paused_resume_asked,
               waiting_asked, waiting_need_asked,
               asking, _asking_entrance, _asking_generic, asking_wait_time, _end_confirm_spoken]
ARRIVAL = (asking, _asking_entrance, _asking_generic, asking_wait_time, _end_confirm_spoken)


@pytest.mark.parametrize("flags", FLAGS, ids=FLAG_IDS)
@pytest.mark.parametrize("make", AFTER_RETRY, ids=[m.__name__.strip("_") for m in AFTER_RETRY])
def test_p8_after_retry_runs_like_the_question_newly_asked_when_off(make, flags):
    """(나) 조용 → 원래 질문 → 못 알아들은 말 → ① — ① 뒤의 말·시각·끝 동작이 꺼짐에서 그 질문을 ① 의 시각에 새로 물었을 때와 같다"""
    first = _first_resay(make, flags)
    at = (first if first is not None else 15.0) + 3.0
    on, t0 = make(reask_ladder=True, **flags)
    timeline = _timeline(on, t0, {at: intent("unknown")}, until=at + 100.0)
    assert [t for t, s in timeline if s == MSG_REASK_RETRY] == [t0 + at]
    anchor = t0 + at + (1.0 if make in ARRIVAL else 0.0)
    after_on = [(round(t - anchor, 1), s) for t, s in timeline if t > t0 + at and t - anchor <= 95.0]
    off, t_off = make(**flags)
    after_off = [(round(t - (t_off - 1.0), 1), s) for t, s in _timeline(off, t_off, until=100.0)
                 if t - (t_off - 1.0) <= 95.0]
    assert after_on == after_off


CLIMB = [
    (confirming, TOILET_ASK, [MSG_CONFIRM_TIMEOUT]),
    (navigating_cancel_asked, MSG_CANCEL_CONFIRM, [MSG_HELP_CONFIRM]),
    (_paused_cancel_asked, MSG_CANCEL_CONFIRM, []),
    (navigating_help_asked, MSG_HELP_CONFIRM, []),
    (_help_cancel_asked, MSG_HELP_CANCEL_CONFIRM, []),
    (paused_resume_asked, MSG_RESUME_CONFIRM, []),
    (asking, MSG_ASK_RESTROOM, [MSG_LEAVING_NOTICE]),
    (_end_confirm_spoken, MSG_ASK_ENTRANCE, [MSG_LEAVING_NOTICE]),
    (waiting_need_asked, MSG_WAIT_NEED_ASK, [MSG_CANCEL_KEPT]),
    (waiting_asked, MSG_WAIT_FINISH_ASK, []),
]


@pytest.mark.parametrize("make, question, end", CLIMB, ids=[m.__name__.strip("_") for m, _, _ in CLIMB])
def test_p8_misheard_words_between_silences_still_end_at_the_third(make, question, end):
    """못 알아들은 말 → ① → 조용 → 원래 질문 → 못 알아들은 말 → ② → 조용 → 원래 질문 → 못 알아들은 말 → 끝. 대답 없음 횟수가 ①·② 마다"""
    logic, t0 = make(reask_ladder=True)
    plan = {"at": t0 + 2.0, "heard": 0}

    def voice(t):
        if plan["at"] is not None and t >= plan["at"]:
            plan["at"] = None
            plan["heard"] += 1
            return intent("unknown")
        return None

    said = []
    for t, acts in _drive(logic, t0, voice, until=200.0):
        lines = [a.text for a in acts if isinstance(a, Say) and a.priority != "ambient"]
        said += lines
        if plan["heard"] == 3:
            break
        if question in lines:
            plan["at"] = t + 2.0
    assert said == [MSG_REASK_RETRY, question, MSG_REASK_LOUDER, question, *end]


AFTER_RESAY = [intent("affirm"), intent("deny"), intent("cancel"), intent("finish"), intent("help"),
               intent("wait", wait_minutes=-1), intent("wait", wait_minutes=5), intent("pause"),
               intent("resume", need_confirm=True), intent("resume"),
               intent(matched_destination_id=ELEV.id, need_confirm=True), intent(matched_destination_id=ELEV.id),
               intent(matched_destination_id="nope"), SIDE, intent("clarify", reply="어느 화장실이요?"), intent("repeat"),
               intent("dismiss"), intent("affirm", answer="yes", answer_kind="yesno"),
               intent("affirm", answer="yes", answer_kind="open"), intent("deny", answer="no", answer_kind="yesno"),
               intent("wait", answer="minutes", answer_value="10", answer_kind="minutes")]


@pytest.mark.parametrize("flags", FLAGS, ids=FLAG_IDS)
@pytest.mark.parametrize("make", SCENES, ids=SCENE_IDS)
def test_p8_words_after_the_silence_resay_are_handled_like_off(make, flags):
    """조용 → 원래 질문 다시 → 의도 하나(21종) → 조용 60 s — 꺼짐과 켜짐의 동작 목록이 같다. 꺼짐은 대답 없음의 다시 묻기를 사다리"""
    first = _first_resay(make, flags)
    at = (first if first is not None else 15.0) + 2.0
    for it in AFTER_RESAY:
        on, t0 = make(reask_ladder=True, **flags)
        off, _ = make(**flags)
        assert _run(on, t0, {at: it}, until=at + 60.0) == _run(off, t0, {at: it}, until=at + 60.0), it


@pytest.mark.parametrize("flags", FLAGS, ids=FLAG_IDS)
def test_p8_where_question_yes_after_the_silence_resay_is_handled_like_off(flags):
    """도착 질문 중 "다시 가자" → "네, 어디로 모실까요?" → 8 초 조용 → 같은 질문 → "네"(어디인지 없음) — 꺼짐과 같다(떠나기 예고)."""
    events = {2.0: intent("resume"), 14.0: intent("affirm")}
    on, t0 = asking(reask_ladder=True, **flags)
    off, _ = asking(**flags)
    log_on, log_off = _run(on, t0, events, until=40.0), _run(off, t0, events, until=40.0)
    assert any(f"'{MSG_WAIT_FINISH_ASK}'" in a for _, acts, _ in log_on[:25] for a in acts)
    assert log_on == log_off


def test_p8_minutes_question_yes_after_the_silence_resay_waits_30_minutes():
    """검토 2 의 장면 — "몇 분쯤 걸리실까요?" → 8 초 조용 → 같은 질문 → "네"(분 없음) = 꺼짐과 같이 기본 30 분 대기(세 번째로 묻지 않는다)."""
    for flags in ({}, {"reask_ladder": True}):
        logic, t = asking_wait_time(**flags)
        assert _says(logic.on_tick(t + 7.0, NONE)) == [MSG_ASK_WAIT_TIME]
        logic.on_arrival_question_spoken(t + 9.0)
        acts = _voice(logic, "affirm", t + 11.0)
        assert MSG_ASK_WAIT_TIME not in _says(acts) and logic.wait_minutes_requested() == 30, flags


ON_SETS = [{"reask_ladder": True}, {"reask_ladder": True, "wake_revive": True, "answer_field": True}]
ON_IDS = ["ladder", "all_three"]


@pytest.mark.parametrize("flags", ON_SETS, ids=ON_IDS)
def test_p8_1122_side_question_then_silence_says_the_toilet_question_at_15s(flags):
    """"화장실로 안내해드릴까요?" → 사용자가 다른 것을 물어 답함(question) → 말 없음 → 질문한 지 15 초에 같은 질문."""
    assert TOILET_ASK == "화장실로 안내해드릴까요?"
    logic, _ = confirming(**flags)
    _voice(logic, "question", 4.0, reply="화장실은 이 복도 끝 오른쪽에 있습니다.")
    assert logic.state == State.CONFIRMING
    assert _says(logic.on_tick(14.5, NONE)) == []
    acts = logic.on_tick(15.0, NONE)
    assert _says(acts) == ["화장실로 안내해드릴까요?"]
    (asked,) = [a for a in acts if isinstance(a, Say)]
    assert (asked.kind, asked.step, asked.question_text, asked.expects_reply) == ("confirm", 0, "", True)
    assert _says(logic.on_tick(29.5, NONE)) == [] and logic.state == State.CONFIRMING
    assert _says(logic.on_tick(30.0, NONE)) == [MSG_CONFIRM_TIMEOUT]


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


def test_p8_ladder_counts_silence_apart_from_misheard_words_when_on():
    ladder = ReaskLadder(three_step=True)
    assert ladder.next(driving=False) == RETRY
    assert ladder.quiet_count == 0
    assert ladder.next_quiet() == SAME and ladder.quiet_count == 1
    assert ladder.next(driving=False) == LOUDER
    assert ladder.next_quiet() == END
    ladder.restart_heard()
    assert (ladder.step, ladder.quiet_count) == (0, 2)
    ladder.restart_quiet()
    assert ladder.quiet_count == 0 and ladder.next_quiet() == SAME
    ladder.reset()
    assert (ladder.step, ladder.quiet_count) == (0, 0)


def test_off_ladder_silence_and_misheard_share_one_mark():
    ladder = ReaskLadder()
    assert ladder.next_quiet() == SAME and (ladder.step, ladder.quiet_count) == (1, 1)
    assert ladder.next(driving=True) == END and ladder.next_quiet() == END
    other = ReaskLadder()
    assert other.next(driving=False) == SAME and other.quiet_count == 1
