"""대기 중 본인 확인 문(2026-11-11 설계 speaker-follow 4.5·B1~B4) — 순수 판정만."""
from __future__ import annotations

from vica_mission_manager.mission_logic import IntentData
from vica_mission_manager.speaker_gate import GATED_KINDS, WaitSpeakerGate


def it(kind: str, verdict: str = "", joined: float = -1.0, dest: str = "") -> IntentData:
    return IntentData(intent=kind, matched_destination_id=dest, need_confirm=kind == "navigate",
                      safety_flag="normal", speaker_verdict=verdict, speaker_joined=joined)


def decide(g, intent, now=0.0, grip=True, sensor=True, followup=False):
    return g.decide(intent, grip_recent=grip, sensor_ok=sensor, followup_open=followup, now=now)


def test_gated_kinds():
    assert GATED_KINDS == ("navigate", "cancel", "finish", "resume", "wait")


def test_not_gated_kind_passes():
    assert decide(WaitSpeakerGate(), it("question", "fail")).action == "pass"


def test_followup_open_passes():
    # 미션 자신의 대기 질문("안내가 필요 없으신가요?")에 대한 두 번째 취소 — 앞 명령의 판정을 물려받는다(B4).
    assert decide(WaitSpeakerGate(), it("cancel", "fail"), followup=True).action == "pass"


def test_no_grip_hint():
    assert decide(WaitSpeakerGate(), it("cancel", "pass"), grip=False).action == "hint"


def test_sensor_stale_skips_grip():
    assert decide(WaitSpeakerGate(), it("cancel", "pass"), grip=False, sensor=False).action == "pass"


def test_pass_verdict_passes():
    out = decide(WaitSpeakerGate(), it("cancel", "pass"))
    assert (out.action, out.skip_confirm) == ("pass", False)


def test_missing_verdict_goes_to_identity():
    g = WaitSpeakerGate()
    assert decide(g, it("cancel", "")).action == "identity"
    assert decide(WaitSpeakerGate(), it("cancel", "no_target")).action == "identity"


def test_fail_then_same_command_skips_confirm():
    g = WaitSpeakerGate()
    assert decide(g, it("cancel", "fail")).action == "reask"
    out = decide(g, it("cancel", "fail", joined=0.61), now=3.0)
    assert (out.action, out.intent.intent, out.skip_confirm) == ("pass", "cancel", True)


def test_fail_then_question_replays_stored():
    g = WaitSpeakerGate()
    decide(g, it("navigate", "fail", dest="restroom"))
    out = decide(g, it("question", "pass", joined=0.6), now=3.0)
    assert (out.action, out.intent.intent, out.intent.matched_destination_id, out.skip_confirm) \
        == ("pass", "navigate", "restroom", False)


def test_fail_then_other_command_keeps_confirm():
    g = WaitSpeakerGate()
    decide(g, it("cancel", "fail"))
    out = decide(g, it("navigate", "fail", joined=0.6, dest="office"), now=3.0)
    assert (out.action, out.intent.intent, out.skip_confirm) == ("pass", "navigate", False)


def test_reask_deny_drops():
    g = WaitSpeakerGate()
    decide(g, it("cancel", "fail"))
    assert decide(g, it("deny", "fail", joined=0.6), now=2.0).action == "drop"
    assert decide(g, it("question"), now=3.0).action == "pass"      # 단계가 지워졌다


def test_fail_twice_goes_identity():
    g = WaitSpeakerGate()
    decide(g, it("cancel", "fail"))
    assert decide(g, it("cancel", "fail", joined=0.3), now=3.0).action == "identity"


def test_identity_affirm_passes_with_note():
    g = WaitSpeakerGate()
    decide(g, it("cancel", "fail"))
    decide(g, it("cancel", "fail", joined=0.3), now=3.0)
    out = decide(g, it("affirm"), now=6.0)
    assert (out.action, out.intent.intent, out.skip_confirm, out.note) == ("pass", "cancel", True, "[본인질문] 출발")


def test_identity_affirm_after_different_commands_keeps_confirm():
    g = WaitSpeakerGate()
    decide(g, it("cancel", "fail"))
    decide(g, it("navigate", "fail", joined=0.3, dest="office"), now=3.0)
    out = decide(g, it("affirm"), now=6.0)
    assert (out.intent.intent, out.skip_confirm) == ("navigate", False)


def test_identity_deny_rejects():
    g = WaitSpeakerGate()
    decide(g, it("finish", ""))
    assert decide(g, it("deny"), now=2.0).action == "reject"


def test_identity_other_answer_drops():
    g = WaitSpeakerGate()
    decide(g, it("finish", ""))
    assert decide(g, it("question"), now=2.0).action == "drop"


def test_identity_silence_resets_on_tick():
    g = WaitSpeakerGate()
    decide(g, it("finish", ""))
    g.tick(16.0)
    assert g.stored is None
    assert decide(g, it("affirm"), now=17.0).action == "pass"          # 열린 단계 없음 — 묻지 않은 '네'
