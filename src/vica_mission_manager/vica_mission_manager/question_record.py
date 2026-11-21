"""질문 기록의 열림·닫힘 (설계 2026-11-14 4.1, 2단계) — 순수 파이썬, rclpy 없음.

미션이 대답을 기다리는 말(kind 가 붙은 Say)을 낼 때 열고, 아래 중 하나에 닫는다.
  answer   음성 의도가 왔다          silence  음성이 듣기를 빈손으로 끝냈다(/vica/listen_state 의 empty…)
  state    미션 상태가 바뀌었다      replaced 새 질문이 나왔다
  no_signal 음성에서 아무 신호도 안 온다(노드가 죽었거나 듣기가 조용히 접힘) — 마지막 신호 뒤 NO_SIGNAL_CLOSE_SEC
닫는 기준에 시간을 쓰지 않는다(2026-11-17 사용자). 실기 1,660건에서 질문 재생 시작 → 대답은 중앙 7.2 s · 최대 23.6 s.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .mission_logic import QUESTION_KINDS

# 음성의 마지막 신호(없으면 질문을 연 때)부터 이만큼 조용하면 닫는다. 음성이 살아 있으면 질문 재생이 끝날 때
# open 을, 듣기가 끝날 때 empty…·closed 를 보낸다. 가장 긴 정상 간격은 질문 뒤 듣기 30 s(음성 CONFIRM_WINDOW_SEC)라
# 여유 15 s 를 더했다. 연 때부터 30 s 로 세던 것은 평소 무응답에서 먼저 터졌다(11-17 최종 검토).
NO_SIGNAL_CLOSE_SEC = 45.0
SILENCE_PREFIX = "empty"      # 음성 듣기 결과가 이 글자로 시작하면 대답 없이 끝난 것


@dataclass(frozen=True)
class QuestionRecord:
    id: str
    kind: str
    text: str
    options: tuple            # ((목적지 id, 이름), …)
    step: int                 # 되묻기 단계 — 세 단계 되묻기(reask_ladder)의 ①·② 만 1·2, 그 밖은 0
    on_silence: str           # 무응답 기본 동작 — 되묻기 구현 전에는 ""
    opened_at: float
    state: str                # 열 때의 미션 상태


class QuestionTracker:
    def __init__(self, prefix: str, no_signal_close_sec: float = NO_SIGNAL_CLOSE_SEC):
        self._prefix, self._no_signal_close_sec = prefix, no_signal_close_sec
        self._count = 0
        self._last_signal_at = 0.0     # 음성의 마지막 듣기 신호 시각(질문을 열 때는 연 시각)
        self.current: Optional[QuestionRecord] = None

    def open(self, say, state: str, now: float) -> tuple[Optional[QuestionRecord], QuestionRecord]:
        """새 질문을 연다. (닫힌 옛 기록 또는 None, 새 기록)."""
        old = self.close("replaced")
        self._count += 1
        kind = say.kind if say.kind in QUESTION_KINDS else "open"
        # 세 단계 되묻기(원칙 8)의 ①·② 는 질문 대신 되묻는 문장을 말한다 — 기록에는 원래 질문 문장과 단계를 적는다.
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
