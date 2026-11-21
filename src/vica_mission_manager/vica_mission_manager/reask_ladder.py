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
    """step 은 next() 가 불린 횟수(0~3 에서 멈춘다)."""

    def __init__(self, three_step: bool = False) -> None:
        self.three_step = three_step
        self.step = 0

    def reset(self) -> None:
        """알아들은 말 · 새 질문 · 호출 — 처음부터."""
        self.step = 0

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
