"""EKF /odom yaw 변화량 기반 회전 판정 (순수 로직)."""

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

from .timebase import is_fresh_ns

DIRECTION_NONE = 0
DIRECTION_LEFT = 1
DIRECTION_RIGHT = 2

PHASE_IDLE = 0
PHASE_PREPARE = 1
PHASE_NOW = 2
PHASE_COMPLETE = 3
PHASE_CANCELED = 4


def normalize_angle(radians: float) -> float:
    """각도를 (-pi, pi]로 정규화한다."""
    return math.atan2(math.sin(radians), math.cos(radians))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """quaternion에서 yaw만 추출한다."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass(frozen=True)
class TurnDecision:
    """한 tick의 판정 결과. 노드는 이 값을 TurnGuide.msg로 옮기기만 한다."""

    direction: int
    phase: int
    turn_angle_deg: float
    sequence_id: int
    source_stale: bool


class TurnDetector:
    """yaw 변화량을 누적해 좌·우 회전을 판정한다."""

    def __init__(
        self,
        window_ns: int,
        enter_threshold_rad: float,
        exit_threshold_rad: float,
        min_duration_ns: int,
        odom_timeout_ns: int,
    ) -> None:
        self.window_ns = window_ns
        self.enter_threshold_rad = enter_threshold_rad
        self.exit_threshold_rad = exit_threshold_rad
        self.min_duration_ns = min_duration_ns
        self.odom_timeout_ns = odom_timeout_ns

        self.samples: Deque[Tuple[int, float]] = deque()
        self.last_yaw_rad: Optional[float] = None
        self.last_odom_ns: Optional[int] = None
        self.unwrapped_rad: float = 0.0

        self.active_direction: int = DIRECTION_NONE
        self.sequence_id: int = 0

    def add_odom(self, yaw_rad: float, now_ns: int) -> None:
        """/odom 콜백에서 호출한다. 샘플 하나를 누적한다."""
        if self.last_yaw_rad is None:
            delta = 0.0
        else:
            delta = normalize_angle(yaw_rad - self.last_yaw_rad)

        self.last_yaw_rad = yaw_rad
        self.last_odom_ns = now_ns
        self.unwrapped_rad += delta
        self.samples.append((now_ns, self.unwrapped_rad))
        self._prune(now_ns)

    def evaluate(self, now_ns: int) -> TurnDecision:
        """현재 판정을 반환한다. 부작용은 내부 상태 갱신뿐이다."""
        if not is_fresh_ns(self.last_odom_ns, now_ns, self.odom_timeout_ns):
            return self._handle_stale()

        self._prune(now_ns)
        accum_rad = self._window_accumulation()
        sign = self._sign_of(accum_rad)
        magnitude = abs(accum_rad)

        phase = self._transition(sign, magnitude)

        return TurnDecision(
            direction=self.active_direction,
            phase=phase,
            turn_angle_deg=math.degrees(accum_rad),
            sequence_id=self.sequence_id,
            source_stale=False,
        )

    def _prune(self, now_ns: int) -> None:
        """윈도우 밖 샘플을 버린다. 기준점이 필요하므로 최소 1개는 남긴다."""
        while len(self.samples) > 1 and (now_ns - self.samples[0][0]) > self.window_ns:
            self.samples.popleft()

    def _window_accumulation(self) -> float:
        if len(self.samples) < 2:
            return 0.0
        return self.samples[-1][1] - self.samples[0][1]

    @staticmethod
    def _sign_of(value: float) -> int:
        if value > 0:
            return 1
        if value < 0:
            return -1
        return 0

    def _held_ns(self, sign: int) -> int:
        """같은 부호의 회전이 얼마나 지속됐는지 샘플에서 직접 계산한다."""
        if sign == 0 or len(self.samples) < 2:
            return 0

        newest_ns, newest_rad = self.samples[-1]
        boundary_ns = newest_ns
        for sample_ns, sample_rad in reversed(self.samples):
            if self._sign_of(newest_rad - sample_rad) in (sign, 0):
                boundary_ns = sample_ns
            else:
                break
        return newest_ns - boundary_ns

    def _transition(self, sign: int, magnitude: float) -> int:
        """hysteresis + 최소 지속시간으로 상태를 전이시킨다."""
        if self.active_direction == DIRECTION_NONE:
            entered = (
                magnitude >= self.enter_threshold_rad
                and self._held_ns(sign) >= self.min_duration_ns
                and sign != 0
            )
            if entered:
                self.active_direction = (
                    DIRECTION_LEFT if sign > 0 else DIRECTION_RIGHT
                )
                self.sequence_id += 1
                return PHASE_NOW
            return PHASE_IDLE

        if magnitude <= self.exit_threshold_rad:
            self.active_direction = DIRECTION_NONE
            return PHASE_COMPLETE

        current = DIRECTION_LEFT if sign > 0 else DIRECTION_RIGHT
        if sign != 0 and current != self.active_direction:
            self.active_direction = DIRECTION_NONE
            return PHASE_COMPLETE

        return PHASE_NOW

    def _handle_stale(self) -> TurnDecision:
        """stale 시 누적을 폐기한다. 이것이 stale 처리의 진짜 목적이다."""
        self._reset_accumulation()
        self.active_direction = DIRECTION_NONE
        return TurnDecision(
            direction=DIRECTION_NONE,
            phase=PHASE_IDLE,
            turn_angle_deg=float("nan"),
            sequence_id=self.sequence_id,
            source_stale=True,
        )

    def _reset_accumulation(self) -> None:
        self.samples.clear()
        self.last_yaw_rad = None
        self.unwrapped_rad = 0.0
