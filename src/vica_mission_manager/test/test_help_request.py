"""관리자 호출(도움 요청) — 순수 로직 시험 (설계 2026-10-15, 10-16 사용자 결정 D8~D10).

설계서: Claude Docs "비카 관리자 호출 설계서". 시험 번호 T1~T17 은 설계서 9.1절, D8~D10 은 10-16 결정이다.
안내 주행 중에만 받고 → 미션이 확인 질문 → "네"면 일시정지·앱 알림(10초마다 같은 help_id) → 앱 확인.
"""
import uuid

import pytest

from reaction_states import (BOUNDS, ROOM, TOILET, asking, idle, intent, lookup, navigating,
                             paused, returning, waiting)
from vica_mission_manager.mission_logic import (
    HELP_RESEND_SEC,
    MSG_CANCEL_CONFIRM,
    MSG_HELP_CANCEL_CONFIRM,
    MSG_HELP_CANCELED,
    MSG_HELP_COMING,
    MSG_HELP_CONFIRM,
    MSG_HELP_SENT,
    MSG_HELP_UNAVAILABLE,
    CancelNav,
    GateReason,
    HelpEvent,
    Navigate,
    NavStatus,
    Say,
    State,
)


def _says(actions):
    return [a.text for a in actions if isinstance(a, Say)]


def _helps(actions):
    return [(a.event, a.help_id) for a in actions if isinstance(a, HelpEvent)]


def _voice(logic, kind, t, **kw):
    return logic.on_voice_intent(intent(kind, **kw), t, lookup, BOUNDS, True)


def asking_help():
    """T2 — 안내 주행 중 help → 확인 질문을 하고 답을 기다리는 중."""
    logic, t = navigating()
    _voice(logic, "help", t)
    return logic, t + 1.0


def requested():
    """T3 — 확인 질문에 "네" → 일시정지·도움 요청 보냄(관리자 확인 전)."""
    logic, t = asking_help()
    _voice(logic, "affirm", t)
    return logic, t + 1.0


def acknowledged():
    """T7 — 관리자가 앱에서 확인함."""
    logic, t = requested()
    logic.on_help_ack(logic.help_id, t)
    return logic, t + 1.0


# ---- T1·T2: 받는 상태 ------------------------------------------------------------
@pytest.mark.parametrize("make", [idle, waiting, returning, paused, asking])
def test_t1_help_outside_navigation_is_refused(make):
    logic, t = make()
    before = logic.state
    acts = _voice(logic, "help", t)
    assert acts == [Say(MSG_HELP_UNAVAILABLE, priority="response")]
    assert logic.state == before and logic.help_phase == "none"


def test_t2_help_while_navigating_asks_and_keeps_driving():
    logic, t = navigating()
    acts = _voice(logic, "help", t)
    assert acts == [Say(MSG_HELP_CONFIRM, priority="response", expects_reply=True)]
    assert logic.state == State.NAVIGATING and logic.help_phase == "asking"
    assert not any(isinstance(a, CancelNav) for a in acts)


def test_help_closes_a_pending_guide_cancel_question():
    """질문은 하나만 살린다 — "안내를 취소할까요?"를 묻던 중의 help 는 그 질문을 닫는다."""
    logic, t = navigating()
    assert _says(_voice(logic, "cancel", t)) == [MSG_CANCEL_CONFIRM]
    acts = _voice(logic, "help", t + 1.0)
    assert _says(acts) == [MSG_HELP_CONFIRM]
    assert logic.cancel_confirm_pending is False


# ---- T3·T4·T5: 확인 질문의 답 ------------------------------------------------------
def test_t3_yes_pauses_says_sent_and_sends_request():
    logic, t = asking_help()
    acts = _voice(logic, "affirm", t)
    assert CancelNav(ROOM, event="goal_paused") in acts
    assert logic.state == State.PAUSED and logic.paused_destination == ROOM
    assert _says(acts) == [MSG_HELP_SENT]
    [(event, help_id)] = _helps(acts)
    assert event == "help_requested" and help_id == logic.help_id
    assert uuid.UUID(help_id).version == 4
    assert logic.help_phase == "requested"


def test_t4_no_keeps_driving_silently():
    logic, t = asking_help()
    assert _voice(logic, "deny", t) == []
    assert logic.state == State.NAVIGATING and logic.help_phase == "none"


def test_t5_silence_reasks_once_then_drops():
    logic, t0 = navigating()
    _voice(logic, "help", t0)
    assert MSG_HELP_CONFIRM not in _says(logic.on_tick(t0 + 14.9, NavStatus.NONE))
    assert _says(logic.on_tick(t0 + 15.0, NavStatus.NONE)) == [MSG_HELP_CONFIRM]
    assert MSG_HELP_CONFIRM not in _says(logic.on_tick(t0 + 20.0, NavStatus.NONE))
    acts = logic.on_tick(t0 + 30.0, NavStatus.NONE)
    assert MSG_HELP_CONFIRM not in _says(acts) and _helps(acts) == []
    assert logic.help_phase == "none" and logic.state == State.NAVIGATING


def test_finish_during_question_drops_it_before_the_cancel_question():
    """질문은 하나만 살린다 — "다 됐어"에 "안내를 취소할까요?"가 나가면 도움 질문은 접힌다. 안 접으면
    그 취소 질문의 "네"가 도움 요청으로 잡혀 로봇이 서고 관리자를 부른다(10-16 검토 1)."""
    logic, t = asking_help()
    assert _says(_voice(logic, "finish", t)) == [MSG_CANCEL_CONFIRM]
    assert logic.help_phase == "none"
    acts = _voice(logic, "affirm", t + 1.0)
    assert _helps(acts) == [] and logic.state == State.IDLE


def test_unknown_answer_reasks_once_then_lets_go():
    logic, t = asking_help()
    assert _voice(logic, "unknown", t) == [Say(MSG_HELP_CONFIRM, priority="response",
                                                expects_reply=True)]
    assert _voice(logic, "unknown", t + 1.0) == []
    assert logic.help_phase == "asking"


# ---- T6·T7·T8: 재전송과 관리자 확인 ------------------------------------------------
def test_t6_resends_same_id_every_10_seconds():
    logic, t = asking_help()
    _voice(logic, "affirm", t)
    help_id = logic.help_id
    assert _helps(logic.on_tick(t + 9.9, NavStatus.NONE)) == []
    for k in (1, 2, 3):
        assert _helps(logic.on_tick(t + HELP_RESEND_SEC * k, NavStatus.NONE)) == [
            ("help_requested", help_id)]


def test_t6_late_tick_sends_once_not_the_backlog():
    logic, t = asking_help()
    _voice(logic, "affirm", t)
    assert len(_helps(logic.on_tick(t + 35.0, NavStatus.NONE))) == 1
    assert _helps(logic.on_tick(t + 44.0, NavStatus.NONE)) == []


def test_t7_ack_says_coming_and_stops_resend():
    logic, t = requested()
    help_id = logic.help_id
    acts, ok = logic.on_help_ack(help_id, t)
    assert ok is True
    assert _says(acts) == [MSG_HELP_COMING]
    assert _helps(acts) == [("help_acknowledged", help_id)]
    assert logic.help_phase == "acknowledged"
    assert _helps(logic.on_tick(t + 30.0, NavStatus.NONE)) == []


def test_t7_second_ack_is_quietly_accepted():
    logic, t = acknowledged()
    assert logic.on_help_ack(logic.help_id, t) == ([], True)


def test_t8_ack_without_request_or_with_other_id_is_refused():
    logic, t = navigating()
    assert logic.on_help_ack(str(uuid.uuid4()), t) == ([], False)
    logic, t = requested()
    assert logic.on_help_ack(str(uuid.uuid4()), t) == ([], False)
    assert logic.help_phase == "requested"


# ---- T9~T11: 다시 출발 --------------------------------------------------------------
def test_t9_after_ack_voice_resume_does_not_move():
    logic, t = acknowledged()
    acts = _voice(logic, "resume", t)
    assert acts == [Say(MSG_HELP_COMING, priority="response")]
    assert logic.state == State.PAUSED
    assert not any(isinstance(a, Navigate) for a in acts)


def test_t10_after_ack_app_resume_departs_without_event():
    logic, t = acknowledged()
    acts, reason = logic.on_resume_request(True, t)
    assert reason == GateReason.OK
    assert any(isinstance(a, Navigate) for a in acts)
    assert _helps(acts) == [] and logic.help_phase == "none" and logic.help_id is None


def test_t11_before_ack_voice_resume_departs_and_cancels_request():
    logic, t = requested()
    help_id = logic.help_id
    acts = _voice(logic, "resume", t)
    assert any(isinstance(a, Navigate) for a in acts)
    assert _helps(acts) == [("help_canceled", help_id)]
    assert logic.help_phase == "none" and logic.help_id is None
    assert _helps(logic.on_tick(t + 30.0, NavStatus.NONE)) == []


def test_before_ack_app_resume_departs_and_cancels_request():
    logic, t = requested()
    help_id = logic.help_id
    acts, reason = logic.on_resume_request(True, t)
    assert reason == GateReason.OK and any(isinstance(a, Navigate) for a in acts)
    assert _helps(acts) == [("help_canceled", help_id)]


# ---- T12~T14: 도움 요청 취소 --------------------------------------------------------
def test_t12_cancel_asks_to_cancel_the_help_request():
    logic, t = requested()
    acts = _voice(logic, "cancel", t)
    assert acts == [Say(MSG_HELP_CANCEL_CONFIRM, priority="response", expects_reply=True)]
    assert MSG_CANCEL_CONFIRM not in _says(acts)
    assert logic.help_cancel_pending is True and logic.cancel_confirm_pending is False


@pytest.mark.parametrize("answer", ["affirm", "cancel", "finish"])
def test_t13_yes_cancels_and_stays_paused(answer):
    logic, t = requested()
    help_id = logic.help_id
    _voice(logic, "cancel", t)
    acts = _voice(logic, answer, t + 1.0)
    assert _helps(acts) == [("help_canceled", help_id)]
    assert _says(acts) == [MSG_HELP_CANCELED]
    assert logic.state == State.PAUSED and logic.paused_destination == ROOM
    assert logic.help_phase == "none" and logic.help_cancel_pending is False


def test_t13_after_cancel_resume_departs():
    logic, t = requested()
    _voice(logic, "cancel", t)
    _voice(logic, "affirm", t + 1.0)
    acts = _voice(logic, "resume", t + 2.0)
    assert any(isinstance(a, Navigate) for a in acts) and _helps(acts) == []


def test_t14_no_keeps_the_request_and_resend():
    logic, t = requested()
    help_id = logic.help_id
    _voice(logic, "cancel", t)
    assert _voice(logic, "deny", t + 1.0) == []
    assert logic.help_phase == "requested" and logic.help_cancel_pending is False
    assert _helps(logic.on_tick(t + 10.0, NavStatus.NONE)) == [("help_requested", help_id)]


def test_cancel_question_silence_reasks_then_keeps_request():
    logic, t = requested()
    _voice(logic, "cancel", t)
    assert MSG_HELP_CANCEL_CONFIRM in _says(logic.on_tick(t + 15.0, NavStatus.NONE))
    logic.on_tick(t + 30.0, NavStatus.NONE)
    assert logic.help_cancel_pending is False and logic.help_phase == "requested"


def test_cancel_in_acknowledged_also_asks():
    logic, t = acknowledged()
    assert _says(_voice(logic, "cancel", t)) == [MSG_HELP_CANCEL_CONFIRM]


def test_user_cancel_after_ack_still_tells_the_app():
    """설계 5.6절 "사용자 취소 확정 — 확인 뒤도 같음": 오던 관리자가 알아야 한다."""
    logic, t = acknowledged()
    help_id = logic.help_id
    _voice(logic, "cancel", t)
    acts = _voice(logic, "affirm", t + 1.0)
    assert _helps(acts) == [("help_canceled", help_id)] and _says(acts) == [MSG_HELP_CANCELED]


# ---- T15·T16·T17: 질문을 버리는 때, 앱, 비상 정지 --------------------------------
def test_t15_arrival_during_question_drops_it():
    logic, t = asking_help()
    logic.on_tick(t, NavStatus.SUCCEEDED)
    acts = _voice(logic, "affirm", t + 1.0)
    assert _helps(acts) == [] and logic.help_phase == "none"


def test_t16_app_cancel_cancels_request():
    logic, t = requested()
    help_id = logic.help_id
    acts, reason = logic.on_app_cancel(t)
    assert reason == GateReason.OK
    assert ("help_canceled", help_id) in _helps(acts)
    assert logic.help_phase == "none" and logic.help_id is None


def test_t17_estop_keeps_resending():
    logic, t = requested()
    help_id = logic.help_id
    logic.on_estop(True, t)
    assert _helps(logic.on_tick(t + 10.0, NavStatus.NONE)) == [("help_requested", help_id)]


# ---- 같은 말이 또 올 때 (D1) ---------------------------------------------------------
def test_help_again_while_asking_is_ignored():
    logic, t = asking_help()
    assert _voice(logic, "help", t) == []


def test_help_again_after_sending_repeats_sent():
    logic, t = requested()
    help_id = logic.help_id
    acts = _voice(logic, "help", t)
    assert acts == [Say(MSG_HELP_SENT, priority="response")]
    assert logic.help_id == help_id


def test_help_again_after_ack_repeats_coming():
    logic, t = acknowledged()
    assert _voice(logic, "help", t) == [Say(MSG_HELP_COMING, priority="response")]


# ---- D3·D8: 도움 요청 중 다른 목적지 (10-16 결정 1) --------------------------------
@pytest.mark.parametrize("make", [requested, acknowledged])
def test_d8_destination_proposal_keeps_pause(make):
    """음성이 "화장실로 안내해드릴까요?"를 물어도 미션은 확인 상태로 넘어가지 않는다 — "아니요"·무응답이면
    원래 목적지와 도움 요청이 그대로 남아 관리자 앱의 "다시 출발"이 듣는다."""
    logic, t = make()
    phase = logic.help_phase
    acts = _voice(logic, "navigate", t, matched_destination_id=TOILET.id, need_confirm=True)
    assert acts == []
    assert logic.state == State.PAUSED and logic.paused_destination == ROOM
    _voice(logic, "deny", t + 1.0)
    logic.on_tick(t + 40.0, NavStatus.NONE)
    assert logic.state == State.PAUSED and logic.paused_destination == ROOM
    assert logic.help_phase == phase
    _, reason = logic.on_resume_request(True, t + 41.0)
    assert reason == GateReason.OK


def test_d3_confirmed_destination_before_ack_departs_and_cancels():
    logic, t = requested()
    help_id = logic.help_id
    acts = _voice(logic, "navigate", t, matched_destination_id=TOILET.id)
    assert Navigate in {type(a) for a in acts}
    assert logic.state == State.NAVIGATING and logic.active_destination == TOILET
    assert _helps(acts) == [("help_canceled", help_id)]
    assert logic.help_phase == "none"


def test_d3_refused_destination_keeps_pause_and_request():
    logic, t = requested()
    acts = _voice(logic, "navigate", t, matched_destination_id="nowhere")
    assert _helps(acts) == [] and _says(acts)          # 못 가는 이유만 말한다
    assert logic.state == State.PAUSED and logic.paused_destination == ROOM
    assert logic.help_phase == "requested"


def test_confirmed_destination_after_ack_does_not_move():
    logic, t = acknowledged()
    acts = _voice(logic, "navigate", t, matched_destination_id=TOILET.id)
    assert acts == [Say(MSG_HELP_COMING, priority="response")]
    assert logic.state == State.PAUSED


# ---- D4·D10: 앱 선점·앱 취소 (10-16 결정 3) -----------------------------------------
def test_d4_app_destination_before_ack_cancels_request():
    logic, t = requested()
    help_id = logic.help_id
    acts, reason = logic.on_app_destination(TOILET, BOUNDS, True, t)
    assert reason == GateReason.OK
    assert ("help_canceled", help_id) in _helps(acts) and logic.help_phase == "none"


def test_d4_app_destination_after_ack_ends_quietly():
    logic, t = acknowledged()
    acts, reason = logic.on_app_destination(TOILET, BOUNDS, True, t)
    assert reason == GateReason.OK and _helps(acts) == [] and logic.help_phase == "none"


def test_d4_app_cancel_after_ack_ends_quietly():
    logic, t = acknowledged()
    acts, _ = logic.on_app_cancel(t)
    assert _helps(acts) == [] and logic.help_phase == "none"


def test_d10_app_cancel_ends_request_even_during_estop():
    """비상 정지에 들어가면 보관한 목적지가 지워져 "다시 출발"이 안 듣는다 — 앱 "안내 취소"가
    로봇 상태와 관계없이 도움 요청을 끝내는 출구다(취소 자체는 비상 정지라 거절된다)."""
    logic, t = requested()
    help_id = logic.help_id
    logic.on_estop(True, t)
    acts, reason = logic.on_app_cancel(t + 1.0)
    assert reason == GateReason.ESTOP_ACTIVE
    assert _helps(acts) == [("help_canceled", help_id)]
    assert logic.help_phase == "none"
    assert _helps(logic.on_tick(t + 20.0, NavStatus.NONE)) == []


def test_d10_app_cancel_ends_request_when_idle():
    logic, t = requested()
    help_id = logic.help_id
    logic._to_idle()                       # 안내가 없어진 뒤(예: 비상 정지 해제 뒤)
    assert logic.help_phase == "requested"   # _to_idle 은 도움 요청을 끝내지 않는다(5.6절)
    acts, reason = logic.on_app_cancel(t)
    assert reason == GateReason.OK
    assert _helps(acts) == [("help_canceled", help_id)] and logic.help_phase == "none"
