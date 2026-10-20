"""사람 접근 중간 재측정·같은 사람 잇기 (순수 로직, rclpy 비의존)."""
from __future__ import annotations

import math
import statistics
from enum import Enum
from typing import TYPE_CHECKING, List, Optional

if TYPE_CHECKING:
    from .mission_logic import Pose2D

SAME_PERSON_RADIUS_M = 0.8
SUPPRESS_RADIUS_M = 1.0
RECHECK_MIN_START_M = 3.0
RECHECK_AT_MAX_M = 3.5
RECHECK_AT_MIN_M = 2.5
RECHECK_GIVE_UP_M = 2.0
RECHECK_SAMPLES = 5
RECHECK_MIN_CONFIDENCE = 0.6
RECHECK_RESEND_M = 0.12


def same_person(a: Optional[Pose2D], b: Optional[Pose2D], radius_m: float) -> bool:
    """두 탐지 위치가 같은 사람인가 — 반경 안이면 같다. 모르면(None·NaN·다른 frame) 다르다."""
    if a is None or b is None or a.frame_id != b.frame_id:
        return False
    d = math.hypot(a.x - b.x, a.y - b.y)
    return math.isfinite(d) and d <= radius_m


class RecheckPhase(str, Enum):
    SKIPPED = "skipped"
    WAITING = "waiting"
    COLLECTING = "collecting"
    DONE = "done"
    GAVE_UP = "gave_up"


_FINISHED = (RecheckPhase.DONE, RecheckPhase.GAVE_UP)


class MidcourseRecheck:
    """접근 한 번에 한 번만 다시 잰다. start() 로 새 접근마다 처음부터."""

    def __init__(
        self,
        min_start_m: float = RECHECK_MIN_START_M,
        at_min_m: float = RECHECK_AT_MIN_M,
        at_max_m: float = RECHECK_AT_MAX_M,
        give_up_m: float = RECHECK_GIVE_UP_M,
        samples: int = RECHECK_SAMPLES,
        min_confidence: float = RECHECK_MIN_CONFIDENCE,
        radius_m: float = SAME_PERSON_RADIUS_M,
    ) -> None:
        if not give_up_m < at_min_m <= at_max_m:
            raise ValueError(
                f"포기선 < 시점 하한 <= 상한 이어야 한다: {give_up_m}, {at_min_m}, {at_max_m}")
        if samples < 1:
            raise ValueError(f"표본 수는 1 이상: {samples}")
        self._min_start_m = min_start_m
        self._at_min_m = at_min_m
        self._at_max_m = at_max_m
        self._give_up_m = give_up_m
        self._samples_needed = samples
        self._min_confidence = min_confidence
        self._radius_m = radius_m
        self._person: Optional[Pose2D] = None
        self._samples: List[Pose2D] = []
        self.phase = RecheckPhase.SKIPPED
        self.trigger_m = 0.0
        self.start_distance_m = 0.0

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    @property
    def finished(self) -> bool:
        """다 모았거나 포기했다 — 이번 접근의 목표는 더 바꾸지 않는다(고정)."""
        return self.phase in _FINISHED

    def start(self, person: Optional[Pose2D], robot: Optional[Pose2D]) -> None:
        self._person = person
        self._samples = []
        d0 = _distance(robot, person)
        self.start_distance_m = d0 if d0 is not None else 0.0
        if d0 is None or d0 < self._min_start_m:
            self.phase = RecheckPhase.SKIPPED
            self.trigger_m = 0.0
            return
        self.trigger_m = min(max(d0 / 2.0, self._at_min_m), self._at_max_m)
        self.phase = RecheckPhase.WAITING

    def reset(self) -> None:
        self._person = None
        self._samples = []
        self.phase = RecheckPhase.SKIPPED
        self.trigger_m = 0.0
        self.start_distance_m = 0.0

    def move_person(self, person: Optional[Pose2D]) -> None:
        """재측정 전에 사람이 실제로 옮겨 섰다(0.5 m 갱신) — 기준 자리를 따라간다."""
        if person is not None:
            self._person = person

    def update_robot(self, robot: Optional[Pose2D]) -> None:
        """로봇 위치만으로 단계를 넘긴다(시점 도달·포기). 검출이 안 와도 포기는 나야 한다."""
        if self.phase not in (RecheckPhase.WAITING, RecheckPhase.COLLECTING):
            return
        d = _distance(robot, self._person)
        if d is None:
            return
        if self.phase == RecheckPhase.WAITING and d <= self.trigger_m:
            self.phase = RecheckPhase.COLLECTING
        if self.phase == RecheckPhase.COLLECTING and d < self._give_up_m:
            self.phase = RecheckPhase.GAVE_UP

    def observe(
        self, robot: Optional[Pose2D], detection: Pose2D, confidence: float
    ) -> Optional[Pose2D]:
        """검출 하나. 다 모인 그 순간에만 새 사람 위치(x·y 중앙값)를 돌려준다."""
        self.update_robot(robot)
        if self.phase != RecheckPhase.COLLECTING:
            return None
        if not (math.isfinite(confidence) and confidence >= self._min_confidence):
            return None
        if not same_person(detection, self._person, self._radius_m):
            return None
        self._samples.append(detection)
        if len(self._samples) < self._samples_needed:
            return None
        self.phase = RecheckPhase.DONE
        from .mission_logic import Pose2D
        return Pose2D(
            x=statistics.median(s.x for s in self._samples),
            y=statistics.median(s.y for s in self._samples),
            yaw_deg=0.0,
            frame_id=self._samples[0].frame_id,
        )


def _distance(a: Optional[Pose2D], b: Optional[Pose2D]) -> Optional[float]:
    if a is None or b is None or a.frame_id != b.frame_id:
        return None
    d = math.hypot(a.x - b.x, a.y - b.y)
    return d if math.isfinite(d) else None
