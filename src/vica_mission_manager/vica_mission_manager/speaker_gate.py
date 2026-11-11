"""대기 중 본인 확인 문 — 손잡이·목소리 판정으로 대기를 끝내는 명령을 받을지 정한다(순수 로직, ROS 모름).

설계: vica-voice-llm/docs/superpowers/specs/2026-11-11-speaker-follow-design.md 4.5·9절 B1~B4.
대기 중 A(안내받던 사람)가 화장실 등에 간 사이 다른 사람이 로봇을 데려가지 못하게 하되, A 는 늘 받아줄 길이
있다: 손잡이 안내 → 되묻기(두 말을 이어 붙인 점수) → 본인 질문("아까 {목적지}에 가자고 하신 사용자님이신가요?").
멘트는 미션 로직이 말한다 — 여기서는 동작 이름만 정한다.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

GATED_KINDS = ("navigate", "cancel", "finish", "resume", "wait")   # 대기를 끝내는 쪽으로 이어지는 의도(B4)
GATE_WINDOW_SEC = 15.0      # 되묻기·본인 질문의 대답을 기다리는 시간
GRIP_RECENT_SEC = 10.0      # 손잡이 '잡힘' = 이만큼 안에 닿은 적 있음(TTP223 이 쥔 채 약 8 s 뒤 풀림, B2)
NOTE_IDENTITY = "[본인질문] 출발"


@dataclass(frozen=True)
class GateOut:
    action: str                       # pass · hint · reask · identity · reject · drop
    intent: Optional[object] = None   # pass 일 때 이어서 처리할 의도
    use: str = "current"              # current · stored (기록용)
    skip_confirm: bool = False        # 같은 명령을 두 번 들었다 — 확인 질문 생략(D6)
    note: str = ""


def same_command(a, b) -> bool:  # noqa: ANN001
    """확인 질문을 건너뛸 '같은 명령'(D6) — 종류가 같고, navigate 면 목적지, wait 면 대기 시간까지 같다(검토 I1).

    되묻기 답을 잘못 알아들어 다른 곳으로 확인 없이 출발하는 일을 막는다.
    """
    if a is None or b is None or a.intent != b.intent:
        return False
    if a.intent == "navigate":
        return bool(a.matched_destination_id) and a.matched_destination_id == b.matched_destination_id
    if a.intent == "wait":
        return a.wait_minutes == b.wait_minutes
    return True


class WaitSpeakerGate:
    def __init__(self, joined_threshold: float = 0.50):
        self.joined_threshold = joined_threshold
        self._stage: Optional[str] = None     # None · "reask" · "identity"
        self._stored = None
        self._twice = False
        self._at = 0.0

    @property
    def stored(self):
        return self._stored

    def reset(self) -> None:
        self._stage, self._stored, self._twice = None, None, False

    def tick(self, now: float) -> None:
        if self._stage is not None and now - self._at > GATE_WINDOW_SEC:
            self.reset()

    def _open(self, stage: str, intent, now: float, twice: bool = False) -> GateOut:
        self._stage, self._stored, self._twice, self._at = stage, intent, twice, now
        return GateOut(stage)

    def decide(self, intent, grip_recent: bool, sensor_ok: bool, followup_open: bool, now: float) -> GateOut:
        self.tick(now)
        kind = intent.intent
        if self._stage == "reask":
            stored = self._stored
            if kind == "deny":
                self.reset()
                return GateOut("drop")
            if intent.speaker_joined >= self.joined_threshold:
                self.reset()
                if same_command(intent, stored):
                    return GateOut("pass", intent, "current", skip_confirm=True)
                if kind not in GATED_KINDS:
                    return GateOut("pass", stored, "stored")
                return GateOut("pass", intent, "current")
            keep = intent if kind in GATED_KINDS else stored
            return self._open("identity", keep, now, twice=same_command(intent, stored))
        if self._stage == "identity":
            stored, twice = self._stored, self._twice
            self.reset()
            if kind == "affirm":
                return GateOut("pass", stored, "stored", skip_confirm=twice, note=NOTE_IDENTITY)
            if kind in ("deny", "cancel"):
                return GateOut("reject")
            return GateOut("drop")
        if kind not in GATED_KINDS or followup_open:
            return GateOut("pass", intent)
        if sensor_ok and not grip_recent:
            return GateOut("hint")
        if intent.speaker_verdict == "pass":
            return GateOut("pass", intent)
        if intent.speaker_verdict in ("", "no_target"):
            return self._open("identity", intent, now)
        return self._open("reask", intent, now)
