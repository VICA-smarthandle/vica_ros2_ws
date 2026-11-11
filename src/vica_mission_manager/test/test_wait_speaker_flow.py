"""대기 중 본인 확인 흐름(2026-11-11 설계 speaker-follow 4.5·B1~B5) — 미션 로직에 연결된 모양."""
from __future__ import annotations

from vica_mission_manager.mission_logic import (
    Haptic, IntentData, MSG_FINISH, MSG_HANDLE_HINT, MSG_WAIT_LOCATE, MSG_WAIT_NEED_ASK, NavStatus, Say, State,
)
from test_wait_spot import BOUNDS, _dest, arrive, released

IN_USE = "지금은 대기중입니다. 사용자님을 기다리고 있어요"
REASK = "확인을 위해 한 번 더 말씀해 주세요"
IDENTITY = "아까 화장실 입구에 가자고 하신 사용자님이신가요?"
REJECT = "죄송합니다. 다른 분을 안내 중이라 지금은 이용하실 수 없어요."


def waiting(gate: bool = True):
    logic, _ = arrive(speaker_gate=gate)
    released(logic, now=3.0)
    logic.on_tick(10.0, NavStatus.SUCCEEDED)
    assert logic.state == State.WAITING
    return logic


def grip(logic, now, contact=True, fresh=True):
    logic.on_handle_state(contact, fresh, now)


def it(kind, verdict="", joined=-1.0, need_confirm=False, dest="d1"):
    return IntentData(intent=kind, matched_destination_id=dest if kind == "navigate" else "",
                      need_confirm=need_confirm, safety_flag="normal",
                      speaker_verdict=verdict, speaker_joined=joined)


def voice(logic, intent, now):
    return logic.on_voice_intent(intent, now, lambda _id: _dest(), BOUNDS, True)


def said(actions):
    return [a.text for a in actions if isinstance(a, Say)]


def test_gate_off_keeps_current_behavior():
    logic = waiting(gate=False)
    assert said(voice(logic, it("cancel", "fail"), 20.0)) == [MSG_WAIT_NEED_ASK]


def test_wait_first_wake_in_use_ment():
    assert IN_USE in said(waiting().on_wake_call(20.0))
    assert MSG_WAIT_LOCATE in said(waiting(gate=False).on_wake_call(20.0))


def test_no_grip_handle_hint():
    logic = waiting()
    grip(logic, 19.5, contact=False)
    acts = voice(logic, it("cancel", "pass"), 20.0)
    assert said(acts) == [MSG_HANDLE_HINT] and any(isinstance(a, Haptic) for a in acts)
    assert logic.state == State.WAITING


def test_sensor_stale_skips_grip():
    logic = waiting()
    grip(logic, 10.0, contact=False)                       # 마지막 메시지가 10 s 전 — 보드 끊김
    assert said(voice(logic, it("cancel", "pass"), 20.0)) == [MSG_WAIT_NEED_ASK]


def test_pass_navigate_asks_confirm_by_mission():
    logic = waiting()
    grip(logic, 19.5)
    acts = voice(logic, it("navigate", "pass", need_confirm=True), 20.0)
    assert logic.state == State.CONFIRMING
    assert len(said(acts)) == 1 and said(acts)[0].endswith("?")     # 미션이 확인 질문을 말한다(B5)


def test_reask_then_same_cancel_finishes_without_need_ask():
    logic = waiting()
    grip(logic, 19.5)
    assert said(voice(logic, it("cancel", "fail"), 20.0)) == [REASK]
    acts = voice(logic, it("cancel", "fail", joined=0.61), 24.0)
    assert said(acts)[0] == MSG_FINISH and MSG_WAIT_NEED_ASK not in said(acts)
    assert logic.state == State.RETURNING


def test_identity_question_uses_destination_name():
    logic = waiting()
    grip(logic, 19.5)
    voice(logic, it("cancel", "fail"), 20.0)
    assert said(voice(logic, it("cancel", "fail", joined=0.3), 24.0)) == [IDENTITY]


def test_identity_affirm_departs_with_note():
    logic = waiting()
    grip(logic, 19.5)
    voice(logic, it("finish", "fail"), 20.0)
    voice(logic, it("finish", "fail", joined=0.3), 24.0)
    acts = voice(logic, it("affirm"), 28.0)
    assert said(acts)[0] == MSG_FINISH and logic.state == State.RETURNING
    assert "[본인질문] 출발" in logic.voice_note


def test_identity_deny_rejects_and_keeps_waiting():
    logic = waiting()
    grip(logic, 19.5)
    voice(logic, it("cancel", ""), 20.0)                    # 판정 없음 → 바로 본인 질문
    assert said(voice(logic, it("deny"), 23.0)) == [REJECT]
    assert logic.state == State.WAITING


def test_missing_verdict_goes_to_identity():
    logic = waiting()
    grip(logic, 19.5)
    assert said(voice(logic, it("navigate", "", need_confirm=True), 20.0)) == [IDENTITY]
    assert logic.state == State.WAITING


def test_estop_clears_gate():
    logic = waiting()
    grip(logic, 19.5)
    voice(logic, it("cancel", ""), 20.0)                    # 본인 질문 중
    logic.on_estop(True, 21.0)
    assert logic.state == State.ESTOPPED
    assert logic._spk_gate.stored is None
