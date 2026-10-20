"""미션 요청 반응표 최종 검토 수리 (2026-10-09, 사용자 승인) — 지난 질문이 남아 엉뚱한 때 나오지 않는다."""
from reaction_states import (BOUNDS, TOILET, awaiting_user, go, intent, lookup, navigating,
                             paused, waiting, waiting_asked)
from vica_mission_manager.mission_logic import (
    MSG_APPROACH_ONBOARDING,
    MSG_CANCEL_CONFIRM,
    MSG_CONFIRM_TIMEOUT,
    MSG_FINISH,
    MSG_WAIT_DEFAULT,
    MSG_WAIT_FINISH_ASK,
    MSG_WAIT_NEED_ASK,
    NavStatus,
    Say,
    State,
)


def _says(actions):
    return [a.text for a in actions if isinstance(a, Say)]


def _voice(logic, kind, t, **kw):
    return logic.on_voice_intent(intent(kind, **kw), t, lookup, BOUNDS, True)


def _ticks(logic, t0, t1):
    """t0~t1 을 1초마다 틱하며 나온 말을 모은다."""
    said, t = [], t0
    while t <= t1:
        said += _says(logic.on_tick(t, NavStatus.NONE))
        t += 1.0
    return said


def test_wait_longer_closes_the_where_question():
    logic, t = waiting_asked()
    _voice(logic, "wait", t, wait_minutes=10)
    assert MSG_WAIT_FINISH_ASK not in _ticks(logic, t + 1, t + 20)
    acts = _voice(logic, "deny", t + 22)
    assert MSG_FINISH not in _says(acts)
    assert logic.state == State.WAITING


def test_wait_longer_closes_the_need_question():
    logic, t = waiting()
    _voice(logic, "cancel", t)
    _voice(logic, "wait", t + 1, wait_minutes=20)
    assert MSG_WAIT_NEED_ASK not in _ticks(logic, t + 2, t + 22)


def test_estop_forgets_the_wait_question():
    logic, t = waiting()
    _voice(logic, "cancel", t)
    logic.on_estop(True, t + 1)
    logic.on_estop(False, t + 2)
    for k in range(3, 30):
        logic.on_tick(t + k, NavStatus.NONE)
        if logic.state != State.ESTOPPED:
            break
    t = t + k
    logic.on_intent(go(TOILET), TOILET, BOUNDS, True, t + 1)
    logic.on_tick(t + 20, NavStatus.SUCCEEDED)
    logic.on_arrival_question_spoken(t + 21)
    logic.on_arrival_answer(intent("affirm"), t + 22)
    assert MSG_WAIT_NEED_ASK not in _ticks(logic, t + 23, t + 60)


def test_arrival_closes_the_cancel_question():
    logic, t = navigating(TOILET)
    _voice(logic, "cancel", t, need_confirm=True)
    logic.on_tick(t + 3, NavStatus.SUCCEEDED)
    logic.on_arrival_question_spoken(t + 4)
    acts = _voice(logic, "affirm", t + 5)
    assert _says(acts) == [MSG_WAIT_DEFAULT]
    assert MSG_CANCEL_CONFIRM not in _ticks(logic, t + 6, t + 40)


def test_resume_closes_the_cancel_question():
    logic, t = paused()
    _voice(logic, "cancel", t, need_confirm=True)
    _voice(logic, "resume", t + 1)
    assert logic.state == State.NAVIGATING
    assert MSG_CANCEL_CONFIRM not in _ticks(logic, t + 2, t + 40)


def _approach_confirm():
    """접근 질문에 "화장실 가고 싶어" → 돌아선 뒤 "화장실로 안내해드릴까요?"를 묻는 중."""
    logic, t = awaiting_user()
    _voice(logic, "navigate", t, matched_destination_id=TOILET.id, need_confirm=True)
    logic.on_tick(t + 4, NavStatus.SUCCEEDED)
    assert logic.state == State.CONFIRMING
    return logic, t + 5


def test_no_to_the_approach_confirm_asks_where_instead():
    logic, t = _approach_confirm()
    acts = _voice(logic, "deny", t)
    asks = [a for a in acts if isinstance(a, Say)]
    assert [a.text for a in asks] == [MSG_APPROACH_ONBOARDING]
    assert asks[0].expects_reply
    assert logic.state == State.IDLE


def test_silence_to_the_approach_confirm_asks_where_instead():
    logic, t = _approach_confirm()
    said = _ticks(logic, t, t + 35)
    assert MSG_CONFIRM_TIMEOUT not in said
    assert MSG_APPROACH_ONBOARDING in said


def test_cancel_to_the_approach_confirm_still_ends():
    """"취소"·"다 됐어"는 그만두는 말이다 — 다시 묻지 않고 지금처럼 접는다."""
    for kind in ("cancel", "finish"):
        logic, t = _approach_confirm()
        assert _says(_voice(logic, kind, t)) == [MSG_CONFIRM_TIMEOUT], kind
        assert logic.state == State.IDLE, kind
