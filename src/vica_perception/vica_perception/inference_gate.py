"""주행 중 YOLO 추론 차단 판정 (순수 로직, rclpy 비의존)."""
from __future__ import annotations

from enum import Enum
from typing import Optional

DEFAULT_STATE_TIMEOUT_S = 3.0


def sec_to_ns(seconds: float) -> int:
    """초 단위 시간을 정수 나노초로 바꾼다."""
    return int(seconds * 1_000_000_000)


class InferenceReason(str, Enum):
    """추론을 했는가 / 안 했는가, 그리고 **왜** 인가."""

    OK = "ok"
    NO_STATE = "no_state"
    STATE_STALE = "state_stale"
    DISABLED = "disabled"
    MOVING = "moving"
    PAUSED = "paused"


_BLOCKING = (InferenceReason.MOVING, InferenceReason.PAUSED)


class InferenceGate:
    """`/vica/robot_state` 를 보고 이번 프레임에 모델을 부를지 정한다."""

    def __init__(
        self,
        state_timeout_s: float = DEFAULT_STATE_TIMEOUT_S,
        enabled: bool = True,
    ) -> None:
        if not state_timeout_s > 0.0:
            raise ValueError(
                f"상태 timeout 은 0보다 커야 한다: {state_timeout_s}"
            )
        self._timeout_ns = sec_to_ns(state_timeout_s)
        self._enabled = enabled
        self._last_ns: Optional[int] = None
        self._is_moving = False
        self._is_paused = False

    @property
    def enabled(self) -> bool:
        return self._enabled

    def observe_state(
        self,
        now_ns: int,
        is_moving: bool,
        is_paused: bool,
    ) -> None:
        """`RobotState` 한 건을 넣는다. 수신 시각은 호출자의 STEADY clock 이다."""
        self._last_ns = now_ns
        self._is_moving = bool(is_moving)
        self._is_paused = bool(is_paused)

    def reason(self, now_ns: int) -> InferenceReason:
        """지금 추론을 하는가 / 안 하는가의 사유."""
        if not self._enabled:
            return InferenceReason.DISABLED
        if self._last_ns is None:
            return InferenceReason.NO_STATE

        age_ns = now_ns - self._last_ns
        if age_ns > self._timeout_ns or age_ns < 0:
            return InferenceReason.STATE_STALE

        if self._is_moving:
            return InferenceReason.MOVING
        if self._is_paused:
            return InferenceReason.PAUSED
        return InferenceReason.OK

    def should_infer(self, now_ns: int) -> bool:
        """이번 프레임에 모델을 부를 것인가."""
        return self.reason(now_ns) not in _BLOCKING
