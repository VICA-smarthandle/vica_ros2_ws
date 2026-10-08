"""반응표 칸 밖의 세부 — 시간·문장 대조·이어지는 동작 (미션 요청 반응표, 2026-10-08).

칸 하나의 첫 반응은 test_reaction_table.py 가 못 박는다. 여기는 그 뒤에 이어지는 일을 본다.
"""
from reaction_states import (BOUNDS, ELEV, ROOM, SPOT_DEST, asking, asking_wait_time,
                             confirming, idle_braked, intent, lookup, navigating, returning,
                             returning_late, waiting, waiting_asked, waiting_release)
from vica_mission_manager.mission_logic import (
    MSG_ALREADY_GOING,
    MSG_ASK_ENTRANCE,
    MSG_ASK_WAIT_TIME,
    MSG_CONFIRM_SWITCH,
    MSG_CANCEL_KEPT,
    MSG_CANCEL_CONFIRM,
    MSG_CANCELED,
    MSG_FINISH,
    MSG_PRIVATE_DEST,
    MSG_START,
    MSG_WAIT_DEFAULT,
    MSG_WAIT_FINISH_ASK,
    MSG_WAIT_NEED_ASK,
    MSG_WAIT_SPOT_CONFIRM,
    MSG_WAIT_SPOT_DEFAULT,
    CancelNav,
    Navigate,
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


# ---- Task 7: 아니요·취소 -------------------------------------------------------
def test_release_no_then_answers():
    """M2 직후 "아니(기다리지 마)" → "마칠까요?" → 네 = 종료·홈, 아니요 = 하던 대기 그대로."""
    logic, t = waiting_release(minutes=10)
    assert _says(logic.on_voice_intent(intent("deny"), t, lookup, BOUNDS, True)) == [
        MSG_ASK_ENTRANCE]
    acts = logic.on_voice_intent(intent("affirm"), t + 2, lookup, BOUNDS, True)
    assert _says(acts) == [MSG_FINISH]
    assert logic.state == State.RETURNING

    logic, t = waiting_release(minutes=10)
    logic.on_voice_intent(intent("deny"), t, lookup, BOUNDS, True)
    acts = logic.on_voice_intent(intent("deny"), t + 2, lookup, BOUNDS, True)
    assert _says(acts) == [MSG_WAIT_SPOT_CONFIRM.format(minutes=10, place="입구 오른쪽")]
    assert logic.state == State.WAITING_RELEASE


def test_wait_time_no_then_answers():
    logic, t = asking_wait_time()
    assert _says(logic.on_voice_intent(intent("deny"), t, lookup, BOUNDS, True)) == [
        MSG_ASK_ENTRANCE]
    acts = logic.on_voice_intent(intent("deny"), t + 2, lookup, BOUNDS, True)
    assert _says(acts) == [MSG_WAIT_DEFAULT]
    assert logic.state == State.WAITING


def test_cancel_question_yes_and_no():
    """"안내를 취소할까요?"의 네·아니요 — 예전엔 둘 다 버려졌다."""
    logic, t = navigating()
    logic.on_voice_intent(intent("cancel"), t, lookup, BOUNDS, True)
    acts = logic.on_voice_intent(intent("deny"), t + 2, lookup, BOUNDS, True)
    assert _says(acts) == [MSG_CANCEL_KEPT]
    assert logic.state == State.NAVIGATING and not logic.cancel_confirm_pending

    logic, t = navigating()
    logic.on_voice_intent(intent("cancel"), t, lookup, BOUNDS, True)
    acts = logic.on_voice_intent(intent("affirm"), t + 2, lookup, BOUNDS, True)
    assert MSG_CANCELED in _says(acts)
    assert logic.state == State.IDLE


def test_waiting_no_after_the_window_is_not_an_answer():
    logic, t = waiting_asked()
    acts = logic.on_voice_intent(intent("deny"), t + 31.0, lookup, BOUNDS, True)
    assert _says(acts) == []
    assert logic.state == State.WAITING


# ---- Task 8: 결정 3 — 대기 중 취소 ---------------------------------------------
def test_need_question_is_the_users_sentence():
    assert MSG_WAIT_NEED_ASK == "안내가 필요 없으신가요?"


def test_need_question_answers():
    """부정 질문: 네(필요 없다)·다 됐어·두 번째 취소 = 종료·홈, 아니요(필요하다) = 계속 대기."""
    for answer in ("affirm", "finish", "cancel"):
        logic, t = waiting()
        assert _says(logic.on_voice_intent(intent("cancel"), t, lookup, BOUNDS, True)) == [
            MSG_WAIT_NEED_ASK]
        acts = logic.on_voice_intent(intent(answer), t + 2, lookup, BOUNDS, True)
        assert _says(acts) == [MSG_FINISH], answer
        assert logic.state == State.RETURNING, answer

    logic, t = waiting()
    logic.on_voice_intent(intent("cancel"), t, lookup, BOUNDS, True)
    acts = logic.on_voice_intent(intent("deny"), t + 2, lookup, BOUNDS, True)
    assert _says(acts) == [MSG_CANCEL_KEPT]
    assert logic.state == State.WAITING


# ---- Task 9: 결정 1 — 홈 가는 중의 "기다려" ------------------------------------
def test_returning_wait_goes_back_to_the_wait_spot():
    """대기 장소가 있는 목적지에서 안내를 마치고 홈 가는 중 "기다려" → 복귀를 멈추고 M2′ →
    말이 끝나면 대기 장소로 떠난다."""
    logic, t = returning(SPOT_DEST)
    acts = logic.on_voice_intent(intent("wait"), t, lookup, BOUNDS, True)
    assert any(isinstance(a, CancelNav) for a in acts)
    assert _says(acts) == [MSG_WAIT_SPOT_DEFAULT.format(place="입구 오른쪽")]
    assert logic.state == State.WAITING_RELEASE
    logic.on_wait_speech_spoken(_says(acts)[0], t + 5.0)
    acts = logic.on_tick(t + 5.1, NavStatus.NONE)
    assert logic.state == State.MOVING_TO_WAIT_SPOT
    assert [a.destination.id for a in acts if isinstance(a, Navigate)] == ["wait_spot:d1"]


def test_returning_wait_without_spot_goes_to_the_entrance():
    logic, t = returning()
    acts = logic.on_voice_intent(intent("wait", wait_minutes=10), t, lookup, BOUNDS, True)
    assert _says(acts) == [MSG_WAIT_SPOT_CONFIRM.format(minutes=10, place="입구 앞")]
    assert [a.destination.id for a in acts if isinstance(a, Navigate)] == ["wait_back:wc"]
    assert logic.state == State.MOVING_BACK_TO_DEST
    logic.on_tick(t + 20.0, NavStatus.SUCCEEDED)
    assert logic.state == State.WAITING
    assert logic.wait_place == "입구 앞"


def test_braked_idle_wait_stops_the_return_ladder():
    logic, t = idle_braked()
    logic.on_voice_intent(intent("wait"), t, lookup, BOUNDS, True)
    assert not logic.return_interrupted
    logic.on_tick(t + 30.0, NavStatus.SUCCEEDED)        # 입구 앞 도착
    assert logic.state == State.WAITING


def test_after_home_wait_is_plain_idle_again():
    """홈에 도착하면 직전 목적지를 잊는다 — 그 뒤의 "기다려"는 그냥 쉬는 중의 말이다."""
    logic, t = returning()
    logic.on_tick(t + 20.0, NavStatus.SUCCEEDED)        # 홈 도착
    assert logic.state == State.IDLE
    acts = logic.on_voice_intent(intent("wait"), t + 21.0, lookup, BOUNDS, True)
    assert _says(acts) == ["지금은 안내 중이 아닙니다."]


def test_late_answer_meaning_follows_the_question():
    """대기형 질문("기다릴까요?")에 답이 없어 떠난 뒤: 네 = 대기, 아니요 = 그대로 홈."""
    logic, t = returning_late()
    acts = logic.on_voice_intent(intent("deny"), t, lookup, BOUNDS, True)
    assert _says(acts) == [MSG_FINISH]
    assert logic.state == State.RETURNING
    # 한 번 답했으면 그다음 "네"는 늦은 답이 아니다.
    assert logic.on_voice_intent(intent("affirm"), t + 1, lookup, BOUNDS, True) == []


# ---- Task 10: 결정 2·4 ----------------------------------------------------------
def test_wait_time_yes_twice_waits_30_minutes():
    logic, t = asking_wait_time()
    assert _says(logic.on_voice_intent(intent("affirm"), t, lookup, BOUNDS, True)) == [
        MSG_ASK_WAIT_TIME]
    acts = logic.on_voice_intent(intent("affirm"), t + 3, lookup, BOUNDS, True)
    assert _says(acts) == [MSG_WAIT_DEFAULT]
    assert logic.state == State.WAITING


def test_switch_question_is_the_users_sentence():
    assert MSG_CONFIRM_SWITCH.format(prompt="엘리베이터로 안내해드릴까요?") == (
        "네, 엘리베이터로 안내해드릴까요?")


def test_switch_then_yes_goes_to_the_new_place():
    logic, t = confirming()
    logic.on_voice_intent(intent(matched_destination_id=ELEV.id), t, lookup, BOUNDS, True)
    assert logic.confirming_dest_id == ELEV.id
    acts = logic.on_voice_intent(intent("affirm"), t + 2, lookup, BOUNDS, True)
    assert _says(acts) == [say_destination(MSG_START, "엘리베이터")]
    assert logic.state == State.NAVIGATING


def test_switch_to_a_closed_place_keeps_the_question():
    closed = ROOM.__class__(**{**ROOM.__dict__, "id": "vip", "name": "원장실",
                               "authorization": "private"})
    logic, t = confirming()
    acts = logic.on_voice_intent(intent(matched_destination_id="vip"), t,
                                 lambda i: closed if i == "vip" else lookup(i), BOUNDS, True)
    assert _says(acts) == [MSG_PRIVATE_DEST]
    assert logic.state == State.CONFIRMING and logic.confirming_dest_id == "wc"
