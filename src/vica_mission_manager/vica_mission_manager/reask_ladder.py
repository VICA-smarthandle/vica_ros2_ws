"""되묻기 사다리 — 못 알아들었을 때 몇 번째로 무엇을 할지만 정한다(순수 로직, ROS·시계·문장 모름)."""
from __future__ import annotations

SAME = "same"
RETRY = "retry"
LOUDER = "louder"
HELP = "help"
END = "end"

MAX_STEP = 3


class ReaskLadder:
    """step 은 next() 가 불린 횟수(0~3 에서 멈춘다). quiet 은 세 단계에서 대답 없음(시계)으로 원래 질문을 다시 말한 횟수다 —"""

    def __init__(self, three_step: bool = False) -> None:
        self.three_step = three_step
        self.step = 0
        self.quiet = 0

    def reset(self) -> None:
        """새 질문 · 질문 닫기 · 호출 — 처음부터(두 횟수 모두)."""
        self.step = 0
        self.quiet = 0

    def restart_heard(self) -> None:
        """알아들은 말 — 못 알아들은 말의 단수만 처음으로(원칙 8). 대답 없음 횟수는 그 질문을 새로 물을 때까지 둔다 — 옛 방식은"""
        self.step = 0

    def restart_quiet(self) -> None:
        """사다리 문장(①·②)을 말했다 — 대답 없음 횟수만 처음으로(Task N4 검토 1). 다시 말해 달라고 했으니 그 뒤는 질문을 새로"""
        self.quiet = 0

    @property
    def quiet_count(self) -> int:
        """대답 없음으로 원래 질문을 다시 말한 횟수 — 옛 방식은 못 알아들은 말과 같은 표시(step)다."""
        return self.quiet if self.three_step else self.step

    def next_quiet(self) -> str:
        """대답 없음(시계) 한 단 — 세 단계여도 옛 방식 그대로: 원래 질문을 한 번 다시(SAME), 그 뒤는 그 질문의 끝(END)."""
        if not self.three_step:
            return self.next(driving=False)
        prev = self.quiet
        self.quiet = min(prev + 1, MAX_STEP)
        return SAME if prev == 0 else END

    def next(self, driving: bool, can_help: bool = True) -> str:
        prev = self.step
        self.step = min(prev + 1, MAX_STEP)
        if not self.three_step:
            return SAME if prev == 0 else END
        if prev == 0:
            return RETRY
        if prev == 1:
            return LOUDER
        if prev == 2 and driving and can_help:
            return HELP
        return END
