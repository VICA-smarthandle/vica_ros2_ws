"""E-stop 펄스 상태 (rclpy 비의존 순수 로직 — unit test 대상)."""
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
