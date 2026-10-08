"""반응표 칸 밖의 세부 — 시간·문장 대조·이어지는 동작 (미션 요청 반응표, 2026-10-08).

칸 하나의 첫 반응은 test_reaction_table.py 가 못 박는다. 여기는 그 뒤에 이어지는 일을 본다.
"""
from reaction_states import BOUNDS, intent, lookup, waiting, waiting_release
from vica_mission_manager.mission_logic import (
    MSG_WAIT_SPOT_CONFIRM,
    NavStatus,
    Say,
    State,
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
