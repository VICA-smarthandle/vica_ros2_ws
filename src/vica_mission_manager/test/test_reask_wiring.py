"""질문마다 따로 있던 "한 번 다시" 표시를 되묻기 사다리(ReaskLadder)로 옮긴 배선 — 원칙 8, 동작 변화 없음(Task 3.7).

질문 하나의 "한 번 다시"는 test_reaction_rules.py 의 '다시 묻기' 시험이 못 박는다. 여기는 두 가지를 본다.
- 두 질문이 함께 열려 있어도 각자 한 번씩 다시 묻는다 — 사다리 하나를 여러 질문이 나눠 쓰면 깨진다.
- _resay_open_question: 지금 미션이 쥔 열린 질문을 다시 말하는 동작(뒤 과제의 세 단계 되묻기·"다시 말해줘"가 쓴다).
"""
from reaction_states import (BOUNDS, TOILET, asking, awaiting_user, confirming, confirming_change, idle, intent,
                             lookup, navigating, paused, waiting, waiting_asked)
from vica_mission_manager.mission_logic import (
    MSG_APPROACH_REASK,
    MSG_ASK_RESTROOM,
    MSG_CANCEL_CONFIRM,
    MSG_CONFIRM_PROMPT_FALLBACK,
    MSG_HELP_CANCEL_CONFIRM,
    MSG_HELP_CONFIRM,
    MSG_WAIT_FINISH_ASK,
    MSG_WAIT_NEED_ASK,
    NavStatus,
    Say,
    State,
    say_destination,
)

TOILET_ASK = say_destination(MSG_CONFIRM_PROMPT_FALLBACK, "화장실")


def _says(actions):
    return [a.text for a in actions if isinstance(a, Say)]


# ---- 겹쳐 열린 질문은 각자 한 번 ------------------------------------------------
def test_p8_change_question_and_cancel_question_each_reask_once():
    """주행 중 바꾸기 질문(CONFIRMING)에 "다 됐어" → "안내를 취소할까요?"가 함께 열린다. 둘 다 15초 침묵에 한 번씩."""
    logic, t = confirming_change()                       # t-1 에 바꾸기 질문 → t+14 에 다시 묻기
    assert _says(logic.on_voice_intent(intent("finish"), t + 4, lookup, BOUNDS, True)) == [MSG_CANCEL_CONFIRM]
    assert logic.state == State.CONFIRMING and logic.cancel_confirm_pending
    assert _says(logic.on_tick(t + 14.5, NavStatus.NONE)) == [TOILET_ASK]
    assert _says(logic.on_tick(t + 19.5, NavStatus.NONE)) == [MSG_CANCEL_CONFIRM]


def test_p8_wait_question_reask_is_not_renewed_by_a_confirm_detour():
    """대기 중 "어디로 모실까요?"를 이미 다시 물었으면, 목적지 제안(확인 질문)을 거절해 대기로 돌아와도 또 묻지 않는다."""
    logic, t = waiting_asked()                           # t-1 에 물음
    assert _says(logic.on_tick(t + 14.5, NavStatus.NONE)) == [MSG_WAIT_FINISH_ASK]
    logic.on_voice_intent(intent(matched_destination_id=TOILET.id, need_confirm=True), t + 15, lookup, BOUNDS, True)
    assert logic.state == State.CONFIRMING
    logic.on_voice_intent(intent("deny"), t + 16, lookup, BOUNDS, True)
    assert logic.state == State.WAITING
    assert MSG_WAIT_FINISH_ASK not in _says(logic.on_tick(t + 40, NavStatus.NONE))


# ---- _resay_open_question ---------------------------------------------------------
def test_p8_resay_confirm_question():
    logic, t = confirming()
    acts = logic._resay_open_question(t)
    assert _says(acts) == [TOILET_ASK]
    assert acts[0].expects_reply and acts[0].kind == "confirm"


def test_p8_resay_cancel_question():
    logic, t = navigating()
    logic.on_voice_intent(intent("cancel"), t, lookup, BOUNDS, True)
    assert _says(logic._resay_open_question(t + 1)) == [MSG_CANCEL_CONFIRM]


def test_p8_resay_help_questions():
    logic, t = navigating()
    logic.on_voice_intent(intent("help"), t, lookup, BOUNDS, True)
    assert _says(logic._resay_open_question(t + 1)) == [MSG_HELP_CONFIRM]
    logic.on_voice_intent(intent("affirm"), t + 2, lookup, BOUNDS, True)      # 도움 요청 보냄(일시정지)
    logic.on_voice_intent(intent("cancel"), t + 3, lookup, BOUNDS, True)      # 도움 요청 취소 확인
    assert _says(logic._resay_open_question(t + 4)) == [MSG_HELP_CANCEL_CONFIRM]


def test_p8_resay_arrival_question():
    logic, t = asking()                                  # TOILET(restroom) — 대기형 도착 질문
    assert _says(logic._resay_open_question(t)) == [MSG_ASK_RESTROOM]
    assert logic.state == State.ASKING_NEXT


def test_p8_resay_wait_questions():
    logic, t = waiting()
    logic.on_voice_intent(intent("cancel"), t, lookup, BOUNDS, True)
    assert _says(logic._resay_open_question(t + 1)) == [MSG_WAIT_NEED_ASK]
    logic, t = waiting_asked()
    assert _says(logic._resay_open_question(t)) == [MSG_WAIT_FINISH_ASK]


def test_p8_resay_approach_question_keeps_its_own_count():
    logic, t = awaiting_user()
    assert _says(logic._resay_open_question(t)) == [MSG_APPROACH_REASK]
    assert logic._approach_reasks == 0                   # 접근 질문의 세 번 횟수는 따로 센다(그대로)


def test_p8_resay_nothing_open_is_none():
    for make in (idle, navigating, paused):
        logic, t = make()
        assert logic._resay_open_question(t) is None


def test_p8_resay_does_not_use_up_the_reask():
    """다시 말하기는 되묻기 기회를 쓰지 않는다 — 그 뒤 못 알아들은 답에도 한 번 다시 묻는다."""
    logic, t = confirming()
    logic._resay_open_question(t)
    assert _says(logic.on_voice_intent(intent("unknown"), t + 1, lookup, BOUNDS, True)) == [TOILET_ASK]
