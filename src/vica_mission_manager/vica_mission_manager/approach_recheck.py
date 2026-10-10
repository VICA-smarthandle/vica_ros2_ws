"""사람 접근 중간 재측정·같은 사람 잇기 (순수 로직, rclpy 비의존).

설계 정본: 루트 저장소 docs/superpowers/specs/2026-10-10-approach-midcourse-recheck-design.md

왜 필요한가 (run84, 2026-10-10):

    6 m 밖에서 처음 잰 사람 위치가 실제 다리보다 0.25~0.45 m 멀었다. 라이다로 재도 6 m 에서는
    +0.4 m 였다 — 멀리서 한 번 재고 끝내면 어느 센서든 틀린다. 2.5~3.5 m 에서는 깊이(사람 감지)
    오차가 ±0.06 m 였다. 그런데 도중에 추적기가 같은 사람에게 새 번호를 붙여(12→16, 24→26)
    가까이서 잰 위치가 "다른 사람"으로 거절됐고, 로봇은 다리 0.63~0.69 m 에 섰다. 그러면 "네" 뒤
    180° 회전의 손잡이 꼬리 원(padding 포함 0.62 m)에 사용자 다리가 들어가 회전이 멈춘다.

    그래서 (1) 사람은 번호가 아니라 **자리**로 알아보고, (2) 접근 거리의 절반쯤(2.5~3.5 m)에서
    **한 번** 다시 재서 목표를 고친 뒤 고정한다. 비유하면 6 m 밖에서 눈대중으로 목적지를 찍고
    출발했다가, 절반쯤 와서 한 번 더 보고 고치는 것이다.

이 모듈이 하지 않는 것:

    목표(사람 앞 안전거리 자세) 계산은 approach_geometry, 다시 보낼지(12 cm)와 상태 전이는 mission_logic
    몫이다. 여기는 "언제 재고, 어느 검출을 믿고, 어떤 위치를 내놓나" 만 정한다.
"""
from __future__ import annotations

import math
import statistics
from enum import Enum
from typing import TYPE_CHECKING, List, Optional

if TYPE_CHECKING:  # mission_logic 이 이 모듈을 불러오므로 실행 때는 함수 안에서 늦게 부른다(순환 방지)
    from .mission_logic import Pose2D

# 같은 사람으로 보는 반경(접근 중·재측정). A 의 6 m 추정과 3 m 추정이 0.47 m 달랐다 — 0.7 이면
# 여유 0.23 m 뿐이라 0.8. run84 E 의 뒤쪽 오인식 상자(1.2~1.8 m)는 여전히 밖이다.
SAME_PERSON_RADIUS_M = 0.8
# 재접근 억제 반경. 접근이 끝나고 홈(약 6 m)에서 같은 사람을 다시 보면 추정이 +0.4~0.6 m 틀린다.
SUPPRESS_RADIUS_M = 1.0
# 처음 거리가 이보다 짧으면 다시 재지 않는다 — 이미 정확하다(run84 B 2.8 m 에서 오차 +0.06).
RECHECK_MIN_START_M = 3.0
# 다시 재는 시점 = 처음 거리의 절반을 이 범위로 맞춘 값. 위: 4 m 는 깊이 오차 +0.12~0.15 m.
# 아래: 포기선(RECHECK_GIVE_UP_M)과 0.5 m 는 떨어져야 모을 틈이 있다(0.4 m/s·5 Hz 로 약 6개).
RECHECK_AT_MAX_M = 3.5
RECHECK_AT_MIN_M = 2.5
# 이 안까지 다 못 모으면 고치지 않는다(지금 동작 그대로). 남은 길이 짧아 고쳐도 부드럽게 못 간다.
RECHECK_GIVE_UP_M = 2.0
# 모을 검출 수·최소 신뢰도. 중앙값이라 겹친 상자 1개(run84 E)가 섞여도 끌려가지 않는다.
RECHECK_SAMPLES = 5
RECHECK_MIN_CONFIDENCE = 0.6
# 다시 보내는 기준(목표가 이만큼 이상 옮겨질 때만). VCC 멈춤 반경 = 판정 원 0.15 − arrive_margin 0.03.
# 이보다 작게 옮겨 봐야 서는 자리가 같고, 3 m 측정 흔들림(±5~6 cm)에 괜히 다시 보내지 않는다.
RECHECK_RESEND_M = 0.12


def same_person(a: Optional[Pose2D], b: Optional[Pose2D], radius_m: float) -> bool:
    """두 탐지 위치가 같은 사람인가 — 반경 안이면 같다. 모르면(None·NaN·다른 frame) 다르다."""
    if a is None or b is None or a.frame_id != b.frame_id:
        return False
    d = math.hypot(a.x - b.x, a.y - b.y)
    return math.isfinite(d) and d <= radius_m


class RecheckPhase(str, Enum):
    SKIPPED = "skipped"        # 처음부터 가까웠거나 로봇 위치를 몰라 하지 않음
    WAITING = "waiting"        # 시점(trigger_m)까지 아직 멀다
    COLLECTING = "collecting"  # 모으는 중
    DONE = "done"              # 다 모아 위치를 냈다
    GAVE_UP = "gave_up"        # 포기선 안까지 다 못 모음


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
