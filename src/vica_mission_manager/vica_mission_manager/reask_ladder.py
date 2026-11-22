"""되묻기 사다리 — 못 알아들었을 때 몇 번째로 무엇을 할지만 정한다(순수 로직, ROS·시계·문장 모름).

설계: vica-voice-llm/docs/superpowers/specs/2026-11-20-voice-principles-intent-recut-design.md 6.8절, 원칙 8.
멘트는 미션 로직이 말한다 — 여기서는 동작 이름만 돌려준다.
"""
from __future__ import annotations

SAME = "same"      # 옛 방식: 같은 질문을 한 번 더
RETRY = "retry"    # 원칙 8 첫 번째: 다시 말씀해 주세요
LOUDER = "louder"  # 원칙 8 두 번째: 더 크게 또박또박
HELP = "help"      # 원칙 8 세 번째: 직원 도움(주행 중에만)
END = "end"        # 더 묻지 않고 끝낸다

MAX_STEP = 3


class ReaskLadder:
    """step 은 next() 가 불린 횟수(0~3 에서 멈춘다). quiet 은 세 단계에서 대답 없음(시계)으로 원래 질문을 다시 말한 횟수다 —
    대답이 없는 것은 못 알아들은 것이 아니라 사다리 단수(step)와 따로 센다(원칙 8, Task N4). 옛 방식은 둘을 한 표시(step)로 센다."""

    def __init__(self, three_step: bool = False) -> None:
        self.three_step = three_step
        self.step = 0
        self.quiet = 0

    def reset(self) -> None:
        """새 질문 · 질문 닫기 · 호출 — 처음부터(두 횟수 모두)."""
        self.step = 0
        self.quiet = 0

    def restart_heard(self) -> None:
        """알아들은 말 — 못 알아들은 말의 단수만 처음으로(원칙 8). 대답 없음 횟수는 그 질문을 새로 물을 때까지 둔다 — 옛 방식은
        알아들은 말로 아무것도 돌리지 않아, 곁질문 뒤의 침묵에도 같은 질문은 한 번만 다시 묻는다(Task N4)."""
        self.step = 0

    @property
    def quiet_count(self) -> int:
        """대답 없음으로 원래 질문을 다시 말한 횟수 — 옛 방식은 못 알아들은 말과 같은 표시(step)다."""
        return self.quiet if self.three_step else self.step

    def next_quiet(self) -> str:
        """대답 없음(시계) 한 단 — 세 단계여도 옛 방식 그대로: 원래 질문을 한 번 다시(SAME), 그 뒤는 그 질문의 끝(END).
        ①·②·③ 은 말이 들렸는데 못 알아들었을 때만이다(원칙 8, Task N4). 옛 방식은 next() 와 같다(한 표시를 올린다)."""
        if not self.three_step:
            return self.next(driving=False)
        prev = self.quiet
        self.quiet = min(prev + 1, MAX_STEP)
        return SAME if prev == 0 else END

    def next(self, driving: bool, can_help: bool = True) -> str:
        prev = self.step
        self.step = min(prev + 1, MAX_STEP)
        if not self.three_step:
            return SAME if prev == 0 else END           # driving·can_help 는 옛 방식에 영향 없음
        if prev == 0:
            return RETRY                                 # 원칙 8
        if prev == 1:
            return LOUDER
        if prev == 2 and driving and can_help:
            return HELP
        return END
