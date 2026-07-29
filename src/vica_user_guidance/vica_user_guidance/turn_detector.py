"""EKF /odom yaw 변화량 기반 회전 판정 (순수 로직).

rclpy에 의존하지 않는다. 모든 시각 인자는 호출자(ROS 노드)가 소유한 단일
STEADY_TIME clock의 정수 나노초다. 이 계층은 어떤 구동 명령도 만들지 않는다.

raw ``/cmd_vel.angular.z``는 쓰지 않는다. 명령값은 로봇이 실제로 그렇게 움직였는지
보장하지 않기 때문이다(guideline/vica_architecture.md 12장).
"""

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

from .timebase import is_fresh_ns

DIRECTION_NONE = 0
DIRECTION_LEFT = 1
DIRECTION_RIGHT = 2

PHASE_IDLE = 0
PHASE_PREPARE = 1   # 2단계(path look-ahead) 전용. 1단계는 발행하지 않는다
PHASE_NOW = 2
PHASE_COMPLETE = 3
PHASE_CANCELED = 4


def normalize_angle(radians: float) -> float:
    """각도를 (-pi, pi]로 정규화한다.

    atan2(sin, cos)를 쓰면 +pi/-pi 경계의 불연속이 사라진다.
    """
    return math.atan2(math.sin(radians), math.cos(radians))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """quaternion에서 yaw만 추출한다.

    EKF가 two_d_mode로 동작하므로 roll/pitch는 0이며 무시해도 된다.
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass(frozen=True)
class TurnDecision:
    """한 tick의 판정 결과. 노드는 이 값을 TurnGuide.msg로 옮기기만 한다."""

    direction: int          # DIRECTION_*
    phase: int              # PHASE_*
    turn_angle_deg: float   # 윈도우 누적. 부호: CCW(+) = LEFT. stale이면 NaN
    sequence_id: int
    source_stale: bool


class TurnDetector:
    """yaw 변화량을 누적해 좌·우 회전을 판정한다.

    ``add_odom``(수신)과 ``evaluate``(판정)를 분리한다. /odom은 30Hz, 발행은 20Hz로
    주기가 다르고, 분리해야 테스트에서 시각을 완전히 통제할 수 있다.
    """

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

        # (시각_ns, 펼친_yaw_rad). 펼친 yaw는 ±pi 경계 없이 단조 누적된다.
        self.samples: Deque[Tuple[int, float]] = deque()
        self.last_yaw_rad: Optional[float] = None
        self.last_odom_ns: Optional[int] = None
        self.unwrapped_rad: float = 0.0

        self.active_direction: int = DIRECTION_NONE
        self.sequence_id: int = 0

    # ── 입력 ───────────────────────────────────────────

    def add_odom(self, yaw_rad: float, now_ns: int) -> None:
        """/odom 콜백에서 호출한다. 샘플 하나를 누적한다.

        ±pi 처리의 핵심은 **연속 샘플 차이를 정규화해 누적**하는 것이다.
        윈도우 시작 yaw와 현재 yaw를 직접 빼서 정규화하면, 윈도우 안에서 180도를
        넘는 회전에서 방향이 반대로 뒤집힌다.

        전제: 샘플 간 실제 회전이 pi 미만이어야 한다. /odom 30Hz(33ms)에서 pi를
        넘으려면 5400도/s가 필요해 이 로봇에서는 물리적으로 불가능하다.
        """
        if self.last_yaw_rad is None:
            delta = 0.0
        else:
            delta = normalize_angle(yaw_rad - self.last_yaw_rad)

        self.last_yaw_rad = yaw_rad
        self.last_odom_ns = now_ns
        self.unwrapped_rad += delta
        self.samples.append((now_ns, self.unwrapped_rad))
        self._prune(now_ns)

    # ── 판정 ───────────────────────────────────────────

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

    # ── 내부 ───────────────────────────────────────────

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
        """같은 부호의 회전이 얼마나 지속됐는지 샘플에서 직접 계산한다.

        [주의] evaluate() 호출 시각을 기준으로 타이머를 돌리면 판정이 호출 빈도에
        의존하게 된다. evaluate()를 한 번만 부르면 지속시간이 항상 0이 되어 영원히
        진입하지 못한다. 지속시간은 반드시 샘플 데이터에서 유도해야 한다.

        뒤에서부터 훑어 부호가 유지되는 구간의 시간 폭을 잰다.
        """
        if sign == 0 or len(self.samples) < 2:
            return 0

        newest_ns, newest_rad = self.samples[-1]
        boundary_ns = newest_ns
        for sample_ns, sample_rad in reversed(self.samples):
            # 이 샘플부터 현재까지의 변화가 같은 부호로 유지되는가
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

        # 유지 중. 최소 지속시간은 진입에만 적용한다 — 종료에도 걸면 회전이 끝난 뒤
        # 서보가 불필요하게 더 오래 기울어져 있게 된다.
        if magnitude <= self.exit_threshold_rad:
            self.active_direction = DIRECTION_NONE
            return PHASE_COMPLETE

        current = DIRECTION_LEFT if sign > 0 else DIRECTION_RIGHT
        if sign != 0 and current != self.active_direction:
            # S자 코너: 반대 방향으로 넘어갔다. 먼저 닫고 다음 tick에 새로 진입한다.
            self.active_direction = DIRECTION_NONE
            return PHASE_COMPLETE

        return PHASE_NOW

    def _handle_stale(self) -> TurnDecision:
        """stale 시 누적을 폐기한다. 이것이 stale 처리의 진짜 목적이다.

        /odom이 끊긴 사이 로봇이 회전했다면 복구 첫 델타가 크게 튄다. 누적을 남겨두면
        즉시 오탐한다. IDLE 발행은 부수 효과일 뿐이다.
        """
        self._reset_accumulation()
        self.active_direction = DIRECTION_NONE
        return TurnDecision(
            direction=DIRECTION_NONE,
            phase=PHASE_IDLE,   # stale에서는 COMPLETE를 만들지 않는다
            turn_angle_deg=float("nan"),
            sequence_id=self.sequence_id,
            source_stale=True,
        )

    def _reset_accumulation(self) -> None:
        self.samples.clear()
        self.last_yaw_rad = None
        self.unwrapped_rad = 0.0
