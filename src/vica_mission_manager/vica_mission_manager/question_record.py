"""질문 기록의 열림·닫힘 (설계 2026-11-14 4.1, 2단계) — 순수 파이썬, rclpy 없음."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .mission_logic import QUESTION_KINDS

NO_SIGNAL_CLOSE_SEC = 45.0
SILENCE_PREFIX = "empty"


@dataclass(frozen=True)
class QuestionRecord:
    id: str
    kind: str
    text: str
    options: tuple
    step: int
    on_silence: str
    opened_at: float
    state: str


def answer_for(open_id: str, msg_question_id: str, answer: str) -> str:
    """대답 칸이 지금 열린 질문의 것일 때만 answer 를 돌려준다. 늦게 온 대답·id 없음은 ""(지금처럼 의도로 처리)."""
    return answer if open_id and msg_question_id and open_id == msg_question_id else ""


class QuestionTracker:
    def __init__(self, prefix: str, no_signal_close_sec: float = NO_SIGNAL_CLOSE_SEC):
        self._prefix, self._no_signal_close_sec = prefix, no_signal_close_sec
        self._count = 0
        self._last_signal_at = 0.0
        self.current: Optional[QuestionRecord] = None

    def open(self, say, state: str, now: float) -> tuple[Optional[QuestionRecord], QuestionRecord]:
        """새 질문을 연다. (닫힌 옛 기록 또는 None, 새 기록)."""
        old = self.close("replaced")
        self._count += 1
        kind = say.kind if say.kind in QUESTION_KINDS else "open"
        text = getattr(say, "question_text", "") or say.text
        self.current = QuestionRecord(id=f"{self._prefix}-{self._count}", kind=kind, text=text,
                                      options=tuple(say.options), step=int(getattr(say, "step", 0)), on_silence="",
                                      opened_at=now, state=state)
        self._last_signal_at = now
        return old, self.current

    def close(self, reason: str) -> Optional[QuestionRecord]:  # noqa: ARG002 — 사유는 부르는 쪽이 로그에 쓴다
        record, self.current = self.current, None
        return record

    def on_listen_state(self, state: str, now: float) -> Optional[QuestionRecord]:
        """음성의 듣기 결과. 빈손으로 끝났으면 닫는다. 말을 했으면 의도가 뒤따라 오므로 그대로 둔다."""
        self._last_signal_at = now
        return self.close("silence") if state.startswith(SILENCE_PREFIX) else None

    def tick(self, state: str, now: float) -> Optional[tuple[QuestionRecord, str]]:
        """상태가 바뀌었거나, 음성에서 아무 신호 없이 너무 오래 열려 있으면 닫는다. (기록, 사유)."""
        if self.current is None:
            return None
        reason = ("state" if state != self.current.state
                  else "no_signal" if now - self._last_signal_at > self._no_signal_close_sec else "")
        return (self.close(reason), reason) if reason else None
