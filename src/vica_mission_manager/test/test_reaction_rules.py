"""반응표 칸 밖의 세부 — 시간·문장 대조·이어지는 동작 (미션 요청 반응표, 2026-10-08).

칸 하나의 첫 반응은 test_reaction_table.py 가 못 박는다. 여기는 그 뒤에 이어지는 일을 본다.
"""
from reaction_states import (BOUNDS, ELEV, asking, intent, lookup, navigating, waiting,
                             waiting_release)
from vica_mission_manager.mission_logic import (
    MSG_ALREADY_GOING,
    MSG_CANCEL_CONFIRM,
    MSG_CANCELED,
    MSG_FINISH,
    MSG_START,
    MSG_WAIT_FINISH_ASK,
    MSG_WAIT_SPOT_CONFIRM,
    NavStatus,
    Say,
    State,
    say_destination,
)


def _says(actions):
    return [a.text for a in actions if isinstance(a, Say)]


# ---- Task 4: 기다려 ------------------------------------------------------------
def test_waiting_wait_with_minutes_restarts_the_clock():
    logic, t = waiting()
    acts = logic.on_voice_intent(intent("wait", wait_minutes=20), t, lookup, BOUNDS, True)
    assert _says(acts) == [MSG_WAIT_SPOT_CONFIRM.format(minutes=20, place="입구 오른쪽")]
    assert logic.state == State.WAITING
    assert logic._wait_until == t + 20 * 60.0


def test_waiting_wait_caps_at_30_minutes():
    logic, t = waiting()
    acts = logic.on_voice_intent(intent("wait", wait_minutes=45), t, lookup, BOUNDS, True)
    assert _says(acts) == [MSG_WAIT_SPOT_CONFIRM.format(minutes=30, place="입구 오른쪽")]
    assert logic._wait_until == t + 30 * 60.0


def test_release_waits_for_the_new_wait_sentence():
    """손 놓기 기다림에서 시간을 바꾸면 새 안내를 다 말한 뒤에야 대기 장소로 떠난다."""
    logic, t = waiting_release()
    old = logic._release_text
    acts = logic.on_voice_intent(intent("wait", wait_minutes=20), t, lookup, BOUNDS, True)
    new = _says(acts)[0]
    logic.on_wait_speech_spoken(old, t + 1.0)
    assert logic.on_tick(t + 1.1, NavStatus.NONE) == []
    assert logic.state == State.WAITING_RELEASE
    logic.on_wait_speech_spoken(new, t + 6.0)
    logic.on_tick(t + 6.1, NavStatus.NONE)
    assert logic.state == State.MOVING_TO_WAIT_SPOT


# ---- Task 5: 다 됐어 -----------------------------------------------------------
def test_navigating_finish_twice_cancels_like_cancel_twice():
    """안내 주행 중 "다 됐어"는 "취소"와 같은 길 — 되물은 뒤 한 번 더 말하면 취소한다."""
    logic, t = navigating()
    first = logic.on_voice_intent(intent("finish"), t, lookup, BOUNDS, True)
    assert _says(first) == [MSG_CANCEL_CONFIRM]
    second = logic.on_voice_intent(intent("finish"), t + 2.0, lookup, BOUNDS, True)
    assert MSG_CANCELED in _says(second)
    assert logic.state == State.IDLE


# ---- Task 6: 다시 가자 ---------------------------------------------------------
def test_already_going_matches_the_voice_sentence():
    """음성 replies.ALREADY_GOING("지금 {cur}{cur_josa} 가는 중이에요.")과 같은 글자."""
    assert say_destination(MSG_ALREADY_GOING, "409호") == "지금 409호로 가는 중이에요."
    assert say_destination(MSG_ALREADY_GOING, "식당") == "지금 식당으로 가는 중이에요."


def test_asking_where_answers():
    """도착 뒤 "다시 가자" → "네, 어디로 모실까요?" 다음의 답: 목적지 = 출발, 아니요 = 종료."""
    logic, t = asking()
    assert _says(logic.on_voice_intent(intent("resume"), t, lookup, BOUNDS, True)) == [
        MSG_WAIT_FINISH_ASK]
    acts = logic.on_voice_intent(intent(matched_destination_id=ELEV.id), t + 2, lookup,
                                 BOUNDS, True)
    assert _says(acts) == [say_destination(MSG_START, "엘리베이터")]
    assert logic.state == State.NAVIGATING

    logic, t = asking()
    logic.on_voice_intent(intent("resume"), t, lookup, BOUNDS, True)
    acts = logic.on_voice_intent(intent("deny"), t + 2, lookup, BOUNDS, True)
    assert _says(acts) == [MSG_FINISH]
    assert logic.state == State.RETURNING


def test_waiting_resume_twice_never_ends_the_guidance():
    """대기 중 "다시 가자"는 몇 번이든 묻기만 한다 — 끝내기는 "다 됐어" 두 번뿐이다."""
    logic, t = waiting()
    for k in range(2):
        acts = logic.on_voice_intent(intent("resume"), t + k, lookup, BOUNDS, True)
        assert _says(acts) == [MSG_WAIT_FINISH_ASK]
    assert logic.state == State.WAITING
