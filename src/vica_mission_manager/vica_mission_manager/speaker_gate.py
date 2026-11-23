"""대기 중 본인 확인 문 — 손잡이·목소리 판정으로 대기를 끝내는 명령을 받을지 정한다(순수 로직, ROS 모름)."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

GATED_KINDS = ("navigate", "cancel", "finish", "resume", "wait")
GATE_WINDOW_SEC = 15.0
GRIP_RECENT_SEC = 10.0
NOTE_IDENTITY = "[본인질문] 출발"


@dataclass(frozen=True)
class GateOut:
    action: str
    intent: Optional[object] = None
    use: str = "current"
    skip_confirm: bool = False
    note: str = ""


def same_command(a, b) -> bool:  # noqa: ANN001
    """확인 질문을 건너뛸 '같은 명령'(D6) — 종류가 같고, navigate 면 목적지, wait 면 대기 시간까지 같다(검토 I1)."""
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
        self._stage: Optional[str] = None
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
