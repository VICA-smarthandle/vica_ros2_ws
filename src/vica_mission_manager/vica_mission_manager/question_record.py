"""질문 기록의 열림·닫힘 (설계 2026-11-14 4.1, 2단계) — 순수 파이썬, rclpy 없음.

미션이 대답을 기다리는 말(kind 가 붙은 Say)을 낼 때 열고, 아래 중 하나에 닫는다.
  answer   음성 의도가 왔다          silence  음성이 듣기를 빈손으로 끝냈다(/vica/listen_state 의 empty…)
  state    미션 상태가 바뀌었다      replaced 새 질문이 나왔다
  fault    위 신호가 하나도 안 왔다(음성 노드가 죽은 경우) — FAULT_CLOSE_SEC 뒤
닫는 기준에 시간을 쓰지 않는다(2026-11-17 사용자). 실기 1,660건에서 질문 재생 시작 → 대답은 중앙 7.2 s · 최대 23.6 s.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .mission_logic import QUESTION_KINDS

# 닫는 신호가 안 올 때만 쓴다 = 질문 재생 6 + 말 시작 기다림 6 + 최대 듣기 15 + 처리 3 (s)
FAULT_CLOSE_SEC = 30.0
SILENCE_PREFIX = "empty"      # 음성 듣기 결과가 이 글자로 시작하면 대답 없이 끝난 것


@dataclass(frozen=True)
class QuestionRecord:
    id: str
    kind: str
    text: str
    options: tuple            # ((목적지 id, 이름), …)
    step: int                 # 되묻기 단계 — 되묻기 구현 전에는 0
    on_silence: str           # 무응답 기본 동작 — 되묻기 구현 전에는 ""
    opened_at: float
    state: str                # 열 때의 미션 상태


class QuestionTracker:
    def __init__(self, prefix: str, fault_close_sec: float = FAULT_CLOSE_SEC):
        self._prefix, self._fault_close_sec = prefix, fault_close_sec
        self._count = 0
        self.current: Optional[QuestionRecord] = None

    def open(self, say, state: str, now: float) -> tuple[Optional[QuestionRecord], QuestionRecord]:
        """새 질문을 연다. (닫힌 옛 기록 또는 None, 새 기록)."""
        old = self.close("replaced")
        self._count += 1
        kind = say.kind if say.kind in QUESTION_KINDS else "open"
        self.current = QuestionRecord(id=f"{self._prefix}-{self._count}", kind=kind, text=say.text,
                                      options=tuple(say.options), step=0, on_silence="", opened_at=now, state=state)
        return old, self.current

    def close(self, reason: str) -> Optional[QuestionRecord]:  # noqa: ARG002 — 사유는 부르는 쪽이 로그에 쓴다
        record, self.current = self.current, None
        return record

    def on_listen_state(self, state: str) -> Optional[QuestionRecord]:
        """음성의 듣기 결과. 빈손으로 끝났으면 닫는다. 말을 했으면 의도가 뒤따라 오므로 그대로 둔다."""
        return self.close("silence") if state.startswith(SILENCE_PREFIX) else None

    def tick(self, state: str, now: float) -> Optional[tuple[QuestionRecord, str]]:
        """상태가 바뀌었거나, 닫는 신호 없이 너무 오래 열려 있으면 닫는다. (기록, 사유)."""
        if self.current is None:
            return None
        reason = ("state" if state != self.current.state
                  else "fault" if now - self.current.opened_at > self._fault_close_sec else "")
        return (self.close(reason), reason) if reason else None
