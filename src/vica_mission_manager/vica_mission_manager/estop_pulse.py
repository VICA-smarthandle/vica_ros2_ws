"""E-stop 펄스 상태 (rclpy 비의존 순수 로직 — unit test 대상).

emergency_estop_bridge 가 사용한다. 펄스인 이유는 keyboard_knob 의
/estop_reset 이 /emergency_stop 이 아직 true 면 해제를 거부하기 때문 —
입력은 방아쇠로만 쓰고 래치 유지는 keyboard_knob 이 담당한다.
"""
from __future__ import annotations

from typing import Optional


class EstopPulse:
    """트리거 후 pulse_sec 동안 active. 재트리거 시 연장."""

    def __init__(self, pulse_sec: float = 3.0) -> None:
        self.pulse_sec = pulse_sec
        self._until: Optional[float] = None

    def trigger(self, now: float) -> None:
        self._until = now + self.pulse_sec

    def active(self, now: float) -> bool:
        return self._until is not None and now < self._until
