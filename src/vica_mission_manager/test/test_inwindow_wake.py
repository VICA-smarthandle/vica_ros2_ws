"""듣기 창 안에서 음성 의도로 온 "비카야"(원칙 13 · 9 · 3, 전체 검토 C3).

음성은 듣기 창이 열린 동안 들린 "비카야"를 unknown + 넘긴 "네?"로 낸다(_is_wake_only) — on_wake_call·on_wake 를 거치지 않는다.
- wake_revive 켜짐: on_wake 와 같은 접기(_wake_fold — 확인 질문은 _fold_confirming(announce=False) + _wake_fold_questions(그 목적지),
  그 밖은 _wake_fold_questions(None))를 태운다. "네?"는 지금처럼 음성이 넘긴 말로 미션이 한 번 말한다. 그 뒤의 "응"은 되살리기(다시 묻기).
- wake_revive 꺼짐 + reask_ladder 켜짐: 옛 방식(둘 다 꺼짐)처럼 그 자리에서 그 질문을 같은 문장으로 다시 말한다("네?" 대신) —
  사다리 횟수는 올리지 않는다(시계는 "다시 말해줘"처럼 지금부터 다시 잰다).
- 둘 다 꺼짐: 지금 그대로.
도착 질문·대기 시간 질문처럼 호출이 접지 않는 질문은 바뀌지 않는다 — "네?"만 말하고 횟수에 세지 않는다(11-21 실기: 도착 질문 중
"비카야" 세 번 → "네?" 세 번 → "아니" 정상).
"""
import inspect
import random

import pytest

from reaction_states import (BOUNDS, ELEV, ROWS, asking, asking_end_confirm, asking_wait_time, confirming,
                             confirming_change, help_requested_confirm_asked, intent, lookup, navigating_cancel_asked,
                             navigating_help_asked, paused_resume_asked, waiting_asked, waiting_need_asked)
from vica_mission_manager.mission_logic import (
    MSG_CANCEL_CONFIRM,
    MSG_CANCELED,
    MSG_CONFIRM_PROMPT_FALLBACK,
    MSG_FINISH,
    MSG_HELP_CONFIRM,
    MSG_REASK_LOUDER,
    MSG_REASK_RETRY,
    MSG_RESUME_CONFIRM,
    MSG_WAIT_FINISH_ASK,
    MSG_WAIT_NEED_ASK,
    MSG_WAKE_GREETING,
    QUESTION_REASK_SEC,
    CancelNav,
    HelpEvent,
    Navigate,
    NavStatus,
    Say,
    State,
    say_destination,
)

TOILET_ASK = say_destination(MSG_CONFIRM_PROMPT_FALLBACK, "화장실")
WAKE_ONLY = intent("unknown", ask_text=MSG_WAKE_GREETING, ask_kind="open")   # 창 안 "비카야" — 음성 지름길·Realtime 이 같은 꼴
OPEN_YES = intent("affirm", answer="yes", answer_kind="open")               # "네?"(open) 뒤의 "응" — 바꿔 읽지 않는다
LADDERS = ("_confirm_ladder", "_cancel_ladder", "_resume_ladder", "_help_ladder", "_wait_ladder", "_arrival_ladder",
           "_idle_ladder")
FOLDED_NAMES = ("confirm", "cancel", "help_confirm", "resume")   # 호출이 접는 질문(_open_question_name 의 이름)

# 호출이 접는(wake_revive) 되돌리기 어려운 질문 — (만드는 함수, 질문 문장, "응"으로 되살린 뒤의 상태)
FOLDED = [
    (confirming, TOILET_ASK, State.CONFIRMING),
    (confirming_change, TOILET_ASK, State.CONFIRMING),
    (navigating_cancel_asked, MSG_CANCEL_CONFIRM, State.NAVIGATING),
    (navigating_help_asked, MSG_HELP_CONFIRM, State.NAVIGATING),
    (paused_resume_asked, MSG_RESUME_CONFIRM, State.PAUSED),
]
FOLDED_IDS = [m.__name__ for m, _, _ in FOLDED]
REVIVE_FLAGS = [{"wake_revive": True}, {"wake_revive": True, "reask_ladder": True},
                {"wake_revive": True, "reask_ladder": True, "answer_field": True}]
REVIVE_IDS = ["wr", "wr+rl", "all"]


def _voice(logic, t, it):
    return logic.on_voice_intent(it, t, lookup, BOUNDS, True)


def _says(acts):
    return [a.text for a in acts if isinstance(a, Say)]


def _executed(acts):
    """출발·재출발(Navigate)·안내 취소(MSG_CANCELED)·도움 요청 — "응" 하나로 일어나면 안 되는 일. 멈추기만 하는 CancelNav 는
    뺀다 — 바꾸기 질문을 되살리면 처음 물을 때처럼 멈추고 묻는다(_navigating_destination_request)."""
    return (any(isinstance(a, Navigate) for a in acts) or MSG_CANCELED in _says(acts)
            or any(isinstance(a, HelpEvent) and a.event == "help_requested" for a in acts))


def _outcome(acts):
    """실행한 일의 요약 — 목적지까지 견준다."""
    return (tuple(a.destination.id for a in acts if isinstance(a, Navigate)),
            sum(isinstance(a, CancelNav) for a in acts), MSG_CANCELED in _says(acts),
            tuple(a.event for a in acts if isinstance(a, HelpEvent)))


def _steps(logic):
    return tuple(getattr(logic, n).step for n in LADDERS)


# ---- wake_revive 켜짐 — 접고, "응"은 다시 묻기 ------------------------------------------------------
@pytest.mark.parametrize("flags", REVIVE_FLAGS, ids=REVIVE_IDS)
@pytest.mark.parametrize("make, question, revived", FOLDED, ids=FOLDED_IDS)
def test_p13_inwindow_wake_folds_the_question_and_yes_asks_again(make, question, revived, flags):
    logic, t = make(**flags)
    acts = _voice(logic, t, WAKE_ONLY)
    assert _says(acts) == [MSG_WAKE_GREETING]                          # 음성이 넘긴 "네?" 한 번(두 번 말하지 않는다)
    assert logic._wake_folded is not None and logic._resay_open_question(t) is None
    acts = _voice(logic, t + 3, OPEN_YES)
    assert _says(acts) == [question] and not _executed(acts)
    assert logic.state == revived and logic._wake_folded is None


@pytest.mark.parametrize("flags", REVIVE_FLAGS, ids=REVIVE_IDS)
@pytest.mark.parametrize("make, question, revived", FOLDED, ids=FOLDED_IDS)
def test_p13_inwindow_wake_folds_exactly_like_on_wake(make, question, revived, flags):
    """창 안 호출의 접기 = on_wake 의 접기 — 상태·적어 둔 것·열린 질문·되돌린 주행(바꾸기 질문은 원래 목적지로 다시)이 같다."""
    a, t = make(**flags)
    b, _ = make(**flags)
    acts_a = _voice(a, t, WAKE_ONLY)
    acts_b = b.on_wake(t)
    assert [x for x in acts_a if not isinstance(x, Say)] == acts_b
    for lg in (a, b):
        lg.voice_note = ""
    snap = (lambda lg: (lg.state, lg._wake_folded, lg.cancel_confirm_pending, lg.help_phase, lg._resume_question_open(),
                        lg.active_destination, lg.paused_destination, lg._confirming_dest_id, lg._change_from,
                        lg._wake_consumed_at))
    assert snap(a) == snap(b)
    assert _voice(a, t + 3, OPEN_YES) == _voice(b, t + 3, OPEN_YES)    # 뒤의 "응"(되살리기)도 같다
    assert a.state == b.state == revived


def _build(make, **flags):
    """반응표의 모든 행을 같은 파라미터로 — 파라미터를 받지 않는 행은 만든 뒤 켠다(이 세 값은 기동 때 쓰이지 않는다)."""
    takes_kw = any(p.kind == p.VAR_KEYWORD for p in inspect.signature(make).parameters.values())
    if takes_kw:
        return make(**flags)
    logic, t = make()
    for k, v in flags.items():
        setattr(logic, k, v)
    return logic, t


ALL_ROWS = {**ROWS, "NAV_CANCEL_ASKED": navigating_cancel_asked, "NAV_HELP_ASKED": navigating_help_asked,
            "PAUSED_RESUME_ASKED": paused_resume_asked, "HELP_REQUESTED_CONFIRM": help_requested_confirm_asked,
            "ASKING_END_CONFIRM": asking_end_confirm, "WAITING_NEED_ASKED": waiting_need_asked}


@pytest.mark.parametrize("flags", REVIVE_FLAGS, ids=REVIVE_IDS)
@pytest.mark.parametrize("row", list(ALL_ROWS))
def test_p13_inwindow_wake_folds_where_on_wake_folds_in_every_state(row, flags):
    """어긋남 감시 — 반응표의 모든 상태에서 창 안 호출과 on_wake 가 같은 질문을 접고 같은 것을 적어 둔다(on_wake 가 접지 않는
    상태 — 대기·도착 질문·접근 질문·순찰 등 — 는 창 안 호출도 접지 않는다)."""
    a, t = _build(ALL_ROWS[row], **flags)
    b, _ = _build(ALL_ROWS[row], **flags)
    _voice(a, t, WAKE_ONLY)
    b.on_wake(t)
    snap = (lambda lg: (lg.state, lg._wake_folded, lg.cancel_confirm_pending, lg.help_phase, lg._resume_question_open(),
                        lg._confirming_dest_id, lg._change_from))
    assert snap(a) == snap(b), row


@pytest.mark.parametrize("make, question, revived", FOLDED, ids=FOLDED_IDS)
def test_p13_inwindow_wake_twice_keeps_one_folded_question(make, question, revived):
    logic, t = make(wake_revive=True)
    _voice(logic, t, WAKE_ONLY)
    folded = logic._wake_folded
    assert _says(_voice(logic, t + 1, WAKE_ONLY)) == [MSG_WAKE_GREETING]
    assert logic._wake_folded == folded
    acts = _voice(logic, t + 2, intent("affirm"))
    assert _says(acts) == [question] and not _executed(acts)


def test_p13_inwindow_wake_after_the_call_does_not_fold_again():
    """창 밖 호출이 이미 접었으면 창 안 "비카야"는 적어 둔 것을 그대로 둔다(다시 접을 것이 없다)."""
    logic, t = navigating_cancel_asked(wake_revive=True)
    logic.on_wake_call(t)
    assert _says(_voice(logic, t + 1, WAKE_ONLY)) == [MSG_WAKE_GREETING]
    assert logic._wake_folded == ("cancel", "", t)
    assert _says(_voice(logic, t + 2, intent("affirm"))) == [MSG_CANCEL_CONFIRM]


# ---- wake_revive 꺼짐 + reask_ladder 켜짐 — 그 질문을 다시 말한다(횟수 그대로) ---------------------
@pytest.mark.parametrize("af", [False, True], ids=["af_off", "af_on"])
@pytest.mark.parametrize("make, question, revived", FOLDED, ids=FOLDED_IDS)
def test_p9_inwindow_wake_resays_the_question_without_climbing_when_only_the_ladder_is_on(make, question, revived, af):
    if make is paused_resume_asked and not af:
        pytest.skip("재개 질문은 answer_field 켜짐에서만 열린다")
    logic, t = make(reask_ladder=True, answer_field=af)
    state = logic.state
    assert _says(_voice(logic, t, intent("unknown"))) == [MSG_REASK_RETRY]      # 진짜 못 알아들은 말 — ①
    before = _steps(logic)
    for i in range(3):
        acts = _voice(logic, t + 1 + i, WAKE_ONLY)
        assert _says(acts) == [question], i                                   # "네?" 대신 그 질문 그대로(한 문장)
        (asked,) = [a for a in acts if isinstance(a, Say)]
        assert asked.expects_reply and asked.step == 0 and not _executed(acts)
        assert _steps(logic) == before and logic.state == state and logic._resay_open_question(t + 1 + i) is not None
    assert _says(_voice(logic, t + 5, intent("unknown"))) == [MSG_REASK_LOUDER]  # 다음 못 알아들은 말은 ② — 횟수 그대로


def test_p9_inwindow_wake_resay_restarts_the_silence_clock():
    """다시 말한 질문의 15 s 시계는 그때부터 — 다시 말하자마자 사다리 문장이 이어지지 않는다("다시 말해줘"와 같다)."""
    logic, t = confirming(reask_ladder=True)                 # t-1 에 물음 → t+14 에 ①
    _voice(logic, t + 12, WAKE_ONLY)
    assert logic._confirm_reask_at == t + 12 + QUESTION_REASK_SEC
    assert _says(logic.on_tick(t + 14.5, NavStatus.NONE)) == []
    assert _says(logic.on_tick(t + 27.5, NavStatus.NONE)) == [MSG_REASK_RETRY]


# ---- 호출이 접지 않는 질문은 그대로 ---------------------------------------------------------------
KEEP = [(asking, State.ASKING_NEXT), (asking_wait_time, State.ASKING_WAIT_TIME), (asking_end_confirm, State.ASKING_NEXT),
        (waiting_need_asked, State.WAITING), (waiting_asked, State.WAITING)]
ON_FLAGS = [{"reask_ladder": True}, {"wake_revive": True}, {"reask_ladder": True, "wake_revive": True},
            {"reask_ladder": True, "wake_revive": True, "answer_field": True}]


@pytest.mark.parametrize("flags", ON_FLAGS, ids=["rl", "wr", "rl+wr", "all"])
@pytest.mark.parametrize("make, state", KEEP, ids=[m.__name__ for m, _ in KEEP])
def test_p13_inwindow_wake_keeps_arrival_and_wait_questions(make, state, flags):
    logic, t = make(**flags)
    name = logic._open_question_name(t)
    before = _steps(logic)
    for i in range(3):
        assert _says(_voice(logic, t + i, WAKE_ONLY)) == [MSG_WAKE_GREETING], i
    assert logic.state == state and _steps(logic) == before and logic._open_question_name(t + 2) == name
    assert logic._wake_folded is None


# ---- 둘 다 꺼짐 = 지금 그대로 ------------------------------------------------------------------------
def test_off_inwindow_wake_still_uses_the_one_reask_and_folds_nothing():
    logic, t = navigating_cancel_asked()
    says = [_says(_voice(logic, t + i, WAKE_ONLY)) for i in range(2)]
    assert says == [[MSG_CANCEL_CONFIRM], [MSG_WAKE_GREETING]]
    assert logic.cancel_confirm_pending and logic._wake_folded is None


# ---- 대기 질문보다 나중에 접은 확인 질문이 먼저 되살아난다(원칙 6, 검토 I-1 — wake_revive 켜짐) -------------
ELEV_ASK = say_destination(MSG_CONFIRM_PROMPT_FALLBACK, ELEV.name)
PROPOSE_ELEV = intent(matched_destination_id=ELEV.id, need_confirm=True)
CONFIRM_YES = intent("affirm", answer="yes", answer_kind="confirm")        # 다시 물은 확인 질문의 "네"(answer_field 켜짐이면 대답 칸)
WAIT_ASKED = [(waiting_need_asked, MSG_WAIT_NEED_ASK), (waiting_asked, MSG_WAIT_FINISH_ASK)]
WAIT_IDS = ["wait_need", "wait_finish"]


def _navs(acts):
    return [a.destination.id for a in acts if isinstance(a, Navigate)]


def _call(logic, how, t):
    """호출 세 길 — 창 밖 호출 · 창 안에서 소리로 건진 호출(노드가 on_wake 만 부름) · 창 안 음성 의도."""
    if how == "call":
        return logic.on_wake_call(t)
    if how == "rescue":
        return logic.on_wake(t)
    return _voice(logic, t, WAKE_ONLY)


@pytest.mark.parametrize("flags", REVIVE_FLAGS, ids=REVIVE_IDS)
@pytest.mark.parametrize("how", ["call", "rescue", "inwindow"])
@pytest.mark.parametrize("make, wait_q", WAIT_ASKED, ids=WAIT_IDS)
def test_p6_confirm_folded_after_an_older_wait_question_is_revived_first(make, wait_q, how, flags):
    """대기 질문 → 목적지 제안(확인 질문) → 호출이 확인 질문을 접음 → "응" = 확인 질문을 다시 묻기(그보다 오래된 대기 질문의 대답이
    아니다 — "안내를 종료합니다." + 홈 없음) → 다시 "응" = 그 목적지로 출발."""
    logic, t = make(**flags)
    assert _says(_voice(logic, t, PROPOSE_ELEV)) == [ELEV_ASK]
    _call(logic, how, t + 1)
    assert logic.state == State.WAITING and logic._open_question_name(t + 1) in WAIT_IDS   # 옛 대기 질문은 열린 채
    acts = _voice(logic, t + 2, OPEN_YES)
    assert _says(acts) == [ELEV_ASK] and _navs(acts) == [] and MSG_FINISH not in _says(acts)
    assert logic.state == State.CONFIRMING
    acts = _voice(logic, t + 3, CONFIRM_YES)
    assert _navs(acts) == [ELEV.id] and logic.state == State.NAVIGATING


@pytest.mark.parametrize("make, wait_q", WAIT_ASKED, ids=WAIT_IDS)
def test_p6_wait_question_asked_again_after_the_fold_is_answered_first(make, wait_q):
    """접은 뒤에 대기 질문을 새로 물었으면(15 s 다시 묻기 — 접은 시각 < 물은 시각) 지금처럼 "응"은 그 대기 질문의 대답이다 —
    접어 두지 않는 wake_revive 꺼짐과 같은 결과. (노드는 다시 물을 때 note_question_asked 로 적어 둔 것을 지운다 — 여기서는
    부르지 않아 시각 비교만으로도 그렇게 됨을 본다.)"""
    a, t = make(wake_revive=True)
    b, _ = make()
    for lg in (a, b):
        _voice(lg, t, PROPOSE_ELEV)
        lg.on_wake_call(t + 1)
        resaid = [s for k in range(2, 16) for s in _says(lg.on_tick(t + k, NavStatus.NONE))]
        assert wait_q in resaid
    assert a._wake_folded is not None and a._wake_folded[2] == t + 1
    acts_a, acts_b = _voice(a, t + 16, intent("affirm")), _voice(b, t + 16, intent("affirm"))
    assert acts_a == acts_b and a.state == b.state


@pytest.mark.parametrize("how", ["call", "rescue"])
def test_off_older_wait_question_still_takes_the_yes(how):
    """wake_revive 꺼짐은 지금 그대로 — 접은 확인 질문을 적어 두지 않아 "응"은 열린 대기 질문의 대답이다(원래 있던 동작, 바꿀지는
    사용자 결정 사항)."""
    logic, t = waiting_need_asked()
    _voice(logic, t, PROPOSE_ELEV)
    _call(logic, how, t + 1)
    acts = _voice(logic, t + 2, intent("affirm"))
    assert MSG_FINISH in _says(acts) and logic.state == State.RETURNING


# ---- 무작위 장면 — 불변식 ------------------------------------------------------------------------------
# 앞선 말·시계(장면을 여러 갈래로 데려간다). 실행하는 승낙(대답 칸 yes)도 넣어 상태가 바뀌게 둔다.
PREFIX = [
    ("voice", intent("unknown")), ("voice", WAKE_ONLY), ("voice", intent("deny")), ("voice", intent("cancel")),
    ("voice", intent("help")), ("voice", intent("resume", need_confirm=True)), ("voice", intent("pause")),
    ("voice", intent(matched_destination_id=ELEV.id, need_confirm=True)), ("voice", intent("repeat")),
    ("voice", intent("question")), ("voice", intent("affirm", answer="yes", answer_kind="yesno")),
    ("call", None), ("tick", 5.0), ("tick", 16.0),
]
SCENES = [confirming, confirming_change, navigating_cancel_asked, navigating_help_asked, paused_resume_asked,
          help_requested_confirm_asked, asking, asking_end_confirm, waiting_need_asked, waiting_asked]
BARE_YES = [intent("affirm"), OPEN_YES]                                     # 대답 칸 없는 승낙


def _run_prefix(logic, t, events):
    for (kind, arg), gap in events:
        if kind == "voice":
            _voice(logic, t, arg)
        elif kind == "call":
            logic.on_wake_call(t)
        else:
            t += arg
            logic.on_tick(t, NavStatus.NONE)
        t += gap
    return t


def test_p13_random_scenes_inwindow_wake_then_bare_yes_never_executes_with_wake_revive():
    """wake_revive 켜짐 — 호출이 접는 질문이 열린 채 창 안 "비카야" → 대답 칸 없는 "응"만으로 출발·취소·재출발·도움 요청·종료가 0건.

    같은 장면을 창 밖 호출(on_wake_call)로도 돌려 "응"의 결과가 같음을 함께 본다(창 안 호출 = 창 밖 호출). 접은 뒤에도 그보다 먼저
    물은 대기 질문("안내가 필요 없으신가요?"·"어디로 모실까요?")이 열려 있으면 접은 확인 질문이 먼저 되살아난다(원칙 6, 검토 I-1) —
    그 장면도 종료·홈 0건으로 센다. 그 밖의 질문이 남는 장면은 없다."""
    rng = random.Random(20261122)
    checked = older_wait = other = 0
    for _ in range(4000):
        flags = {"wake_revive": True, "reask_ladder": rng.random() < 0.5, "answer_field": rng.random() < 0.5}
        make = rng.choice(SCENES)
        events = [(rng.choice(PREFIX), rng.choice((0.5, 1.0, 2.0))) for _ in range(rng.randrange(0, 4))]
        a, t0 = make(**flags)
        b, _ = make(**flags)
        t = _run_prefix(a, t0, events)
        _run_prefix(b, t0, events)
        name = a._open_question_name(t)
        if name not in FOLDED_NAMES:
            continue
        _voice(a, t, WAKE_ONLY)
        b.on_wake_call(t)
        left = a._open_question_name(t)
        yes, at = rng.choice(BARE_YES), t + rng.choice((0.5, 2.0, 10.0))
        acts_a, acts_b = _voice(a, at, yes), _voice(b, at, yes)
        where = (make.__name__, flags, events, name, _says(acts_a))
        assert _outcome(acts_a) == _outcome(acts_b) and a.state == b.state, where
        if left is not None and left not in ("wait_need", "wait_finish"):
            other += 1
            continue
        assert not _executed(acts_a) and MSG_FINISH not in _says(acts_a), where
        if left is None:
            checked += 1
        else:
            older_wait += 1
    assert checked > 500 and older_wait > 30 and other == 0, (checked, older_wait, other)


def test_p9_random_scenes_ladder_only_matches_the_old_way():
    """reask_ladder 만 켜짐 — 창 안 "비카야"에 그 질문을 다시 말하므로 뒤의 "응"은 다시 들은 질문의 대답이다: 실행한 일과 상태가
    옛 방식(reask_ladder·wake_revive 꺼짐 — 그 자리에서 같은 질문을 다시 말하던 때)과 같다."""
    rng = random.Random(20261123)
    checked = 0
    for _ in range(1500):
        af = rng.random() < 0.5
        make = rng.choice([m for m, _, _ in FOLDED])
        if make is paused_resume_asked:
            af = True
        old, t = make(answer_field=af)
        new, _ = make(answer_field=af, reask_ladder=True)
        pre = rng.choice((0, 0, 1))                                          # 앞선 못 알아들은 말(옛 방식은 한 번 다시를 쓴다)
        wakes = rng.choice((1, 1, 2, 3))
        yes = rng.choice(BARE_YES + [intent("affirm", answer="yes", answer_kind="yesno")])
        for lg in (old, new):
            for i in range(pre):
                _voice(lg, t + i, intent("unknown"))
            for i in range(wakes):
                _voice(lg, t + pre + i, WAKE_ONLY)
        at = t + pre + wakes + rng.choice((0.5, 2.0))
        a_old, a_new = _voice(old, at, yes), _voice(new, at, yes)
        assert _outcome(a_old) == _outcome(a_new), (make.__name__, af, pre, wakes, yes)
        assert old.state == new.state, (make.__name__, af, pre, wakes, yes)
        checked += 1
    assert checked == 1500
