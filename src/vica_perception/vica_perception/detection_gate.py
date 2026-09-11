"""사람 접근 출발 조건 판정 (순수 로직, rclpy 비의존)."""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Deque, Dict, Iterable, Optional, Sequence, Tuple


DEFAULT_MIN_CONFIDENCE = 0.6
DEFAULT_STABLE_DURATION_S = 1.0
DEFAULT_STILL_WINDOW_S = 3.0
DEFAULT_MAX_DISPLACEMENT_M = 0.3
DEFAULT_MIN_DISTANCE_M = 1.5
DEFAULT_MAX_DISTANCE_M = 8.0

DEFAULT_DETECTION_GAP_S = 0.6
DEFAULT_TELEPORT_SPEED_MPS = 2.0
DEFAULT_TRACK_EXPIRY_S = 5.0
DEFAULT_MAX_SAMPLES_PER_TRACK = 64
DEFAULT_MAX_TRACKS = 32


def sec_to_ns(seconds: float) -> int:
    """초 단위 시간을 정수 나노초로 바꾼다."""
    return int(seconds * 1_000_000_000)


class GateReason(str, Enum):
    """`approachable` 이 아니라면 **왜** 아닌가."""

    OK = "ok"
    INVALID_SAMPLE = "invalid_sample"
    LOW_CONFIDENCE = "low_confidence"
    DETECTION_LOST = "detection_lost"
    TIME_REVERSED = "time_reversed"
    TRACK_JUMP = "track_jump"
    NOT_STABLE = "not_stable"
    INSUFFICIENT_HISTORY = "insufficient_history"
    SAMPLE_OVERFLOW = "sample_overflow"
    MOVING = "moving"
    TOO_NEAR = "too_near"
    TOO_FAR = "too_far"


@dataclass(frozen=True)
class Point2D:
    """map 좌표 한 점. 사람에게는 yaw 가 의미 없어 x·y 만 둔다."""

    x: float
    y: float
    frame_id: str = "map"

    def distance_to(self, other: "Point2D") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def is_finite(self) -> bool:
        return math.isfinite(self.x) and math.isfinite(self.y)


@dataclass(frozen=True)
class DetectionSample:
    """한 프레임에서 `track_id` 하나에 대한 관측."""

    stamp_ns: int
    track_id: int
    confidence: float
    position: Point2D
    distance_m: float

    def is_finite(self) -> bool:
        return (
            math.isfinite(self.confidence)
            and math.isfinite(self.distance_m)
            and self.position.is_finite()
        )


@dataclass(frozen=True)
class GateThresholds:
    """판정 임계값 묶음."""

    min_confidence: float = DEFAULT_MIN_CONFIDENCE
    stable_duration_s: float = DEFAULT_STABLE_DURATION_S
    still_window_s: float = DEFAULT_STILL_WINDOW_S
    max_displacement_m: float = DEFAULT_MAX_DISPLACEMENT_M
    min_distance_m: float = DEFAULT_MIN_DISTANCE_M
    max_distance_m: float = DEFAULT_MAX_DISTANCE_M
    detection_gap_s: float = DEFAULT_DETECTION_GAP_S
    teleport_speed_mps: float = DEFAULT_TELEPORT_SPEED_MPS
    track_expiry_s: float = DEFAULT_TRACK_EXPIRY_S
    max_samples_per_track: int = DEFAULT_MAX_SAMPLES_PER_TRACK
    max_tracks: int = DEFAULT_MAX_TRACKS

    def __post_init__(self) -> None:
        if not 0.0 < self.min_confidence <= 1.0:
            raise ValueError(
                f"신뢰도 임계는 (0, 1] 범위여야 한다: {self.min_confidence}"
            )
        for name in ("stable_duration_s", "still_window_s", "detection_gap_s"):
            value = getattr(self, name)
            if not value > 0.0:
                raise ValueError(f"{name} 는 0보다 커야 한다: {value}")
        if self.still_window_s < self.stable_duration_s:
            raise ValueError(
                "정지 판정 창이 연속 판정보다 짧다: "
                f"{self.still_window_s}s < {self.stable_duration_s}s"
            )
        if not self.max_displacement_m > 0.0:
            raise ValueError(
                f"이동 허용치는 0보다 커야 한다: {self.max_displacement_m}"
            )
        if not 0.0 <= self.min_distance_m < self.max_distance_m:
            raise ValueError(
                "거리 범위는 0 <= 하한 < 상한 이어야 한다: "
                f"{self.min_distance_m} ~ {self.max_distance_m}"
            )
        if not self.teleport_speed_mps > 0.0:
            raise ValueError(
                f"순간이동 임계 속도는 0보다 커야 한다: {self.teleport_speed_mps}"
            )
        if not self.track_expiry_s > 0.0:
            raise ValueError(f"track 만료는 0보다 커야 한다: {self.track_expiry_s}")
        if self.max_samples_per_track < 2:
            raise ValueError(
                f"표본 상한은 2 이상이어야 한다: {self.max_samples_per_track}"
            )
        if self.max_tracks < 1:
            raise ValueError(f"track 상한은 1 이상이어야 한다: {self.max_tracks}")

    @property
    def stable_duration_ns(self) -> int:
        return sec_to_ns(self.stable_duration_s)

    @property
    def still_window_ns(self) -> int:
        return sec_to_ns(self.still_window_s)

    @property
    def detection_gap_ns(self) -> int:
        return sec_to_ns(self.detection_gap_s)

    @property
    def track_expiry_ns(self) -> int:
        return sec_to_ns(self.track_expiry_s)


@dataclass(frozen=True)
class TrackVerdict:
    """한 `track_id` 에 대한 판정 결과. 판정은 기록이므로 얼려서 돌려준다."""

    track_id: int
    stable: bool
    approachable: bool
    reason: GateReason
    stamp_ns: int
    confidence: float
    position: Optional[Point2D]
    distance_m: Optional[float]
    stable_for_ns: int = 0
    displacement_m: float = 0.0
    sample_count: int = 0


@dataclass
class _TrackHistory:
    """`track_id` 하나의 이력. 채택된(고신뢰) 표본만 들어온다."""

    samples: Deque[DetectionSample] = field(default_factory=deque)
    streak_start_ns: int = 0
    last_seen_ns: int = 0
    overflowed: bool = False

    def restart(self, sample: DetectionSample) -> None:
        self.samples.clear()
        self.samples.append(sample)
        self.streak_start_ns = sample.stamp_ns
        self.last_seen_ns = sample.stamp_ns
        self.overflowed = False


class DetectionGate:
    """시계열 탐지 결과에서 `stable` 과 `approachable` 을 판정한다."""

    def __init__(self, thresholds: Optional[GateThresholds] = None) -> None:
        self._thresholds = thresholds if thresholds is not None else GateThresholds()
        self._tracks: Dict[int, _TrackHistory] = {}

    @property
    def thresholds(self) -> GateThresholds:
        return self._thresholds

    def track_ids(self) -> Tuple[int, ...]:
        """지금 이력을 들고 있는 track 목록 (최근에 본 순서가 아니라 삽입 순)."""
        return tuple(self._tracks.keys())

    def sample_count(self, track_id: int) -> int:
        history = self._tracks.get(track_id)
        return len(history.samples) if history else 0

    def observe(self, sample: DetectionSample) -> TrackVerdict:
        """표본 하나를 넣고 그 track 의 판정을 돌려준다."""
        if not sample.is_finite():
            return self._rejected(sample, GateReason.INVALID_SAMPLE)

        if sample.confidence < self._thresholds.min_confidence:
            return self._rejected(sample, GateReason.LOW_CONFIDENCE)

        history = self._tracks.get(sample.track_id)
        if history is None:
            self._make_room()
            history = _TrackHistory()
            self._tracks[sample.track_id] = history
            history.restart(sample)
            return self._verdict(sample.track_id, history, sample.stamp_ns)

        fault = self._continuity_fault(history, sample)
        if fault is not None:
            history.restart(sample)
        else:
            self._append(history, sample)

        verdict = self._verdict(sample.track_id, history, sample.stamp_ns)
        if fault is not None and verdict.reason is GateReason.NOT_STABLE:
            verdict = _replace_reason(verdict, fault)
        return verdict

    def update(
        self,
        samples: Iterable[DetectionSample],
        now_ns: int,
    ) -> Tuple[TrackVerdict, ...]:
        """한 프레임(표본 여러 개)을 넣고 살아 있는 모든 track 의 판정을 준다."""
        for sample in samples:
            self.observe(sample)
        self.prune(now_ns)
        verdicts = [
            self.evaluate(track_id, now_ns) for track_id in tuple(self._tracks)
        ]
        return _sorted_verdicts(v for v in verdicts if v is not None)

    def evaluate(self, track_id: int, now_ns: int) -> Optional[TrackVerdict]:
        """새 표본 없이 현재 시각 기준으로 판정한다. 모르는 track 이면 None."""
        history = self._tracks.get(track_id)
        if history is None or not history.samples:
            return None
        return self._verdict(track_id, history, now_ns)

    def approachable_tracks(self, now_ns: int) -> Tuple[TrackVerdict, ...]:
        """지금 접근 자격을 갖춘 후보 전부. **고르는 것은 여기 책임이 아니다.**"""
        verdicts = []
        for track_id in tuple(self._tracks):
            verdict = self.evaluate(track_id, now_ns)
            if verdict is not None and verdict.approachable:
                verdicts.append(verdict)
        return _sorted_verdicts(verdicts)

    def prune(self, now_ns: int) -> None:
        """오래 안 보인 track 을 버린다. 이력이 무한히 쌓이지 않게 하는 축이다."""
        expiry_ns = self._thresholds.track_expiry_ns
        for track_id in tuple(self._tracks):
            age_ns = now_ns - self._tracks[track_id].last_seen_ns
            if age_ns > expiry_ns or age_ns < 0:
                del self._tracks[track_id]

    def forget(self, track_id: int) -> None:
        """track 하나를 잊는다. 접근을 마친 뒤 Mission 이 부를 수 있다."""
        self._tracks.pop(track_id, None)

    def reset(self) -> None:
        """전체 초기화 (E-stop·미션 취소 등)."""
        self._tracks.clear()

    def _make_room(self) -> None:
        """track 상한을 지킨다. 가장 오래 안 보인 것부터 버린다."""
        while len(self._tracks) >= self._thresholds.max_tracks:
            oldest = min(self._tracks, key=lambda k: self._tracks[k].last_seen_ns)
            del self._tracks[oldest]

    def _continuity_fault(
        self,
        history: _TrackHistory,
        sample: DetectionSample,
    ) -> Optional[GateReason]:
        """연속성이 깨졌으면 그 사유. 멀쩡하면 None."""
        previous = history.samples[-1]
        dt_ns = sample.stamp_ns - previous.stamp_ns

        if dt_ns < 0:
            return GateReason.TIME_REVERSED
        if dt_ns > self._thresholds.detection_gap_ns:
            return GateReason.DETECTION_LOST

        moved_m = previous.position.distance_to(sample.position)
        if dt_ns == 0:
            return GateReason.TRACK_JUMP if moved_m > 0.0 else None
        speed_mps = moved_m / (dt_ns / 1_000_000_000.0)
        if speed_mps > self._thresholds.teleport_speed_mps:
            return GateReason.TRACK_JUMP
        return None

    def _append(self, history: _TrackHistory, sample: DetectionSample) -> None:
        window_ns = self._thresholds.still_window_ns
        samples = history.samples

        while samples and sample.stamp_ns - samples[0].stamp_ns > window_ns:
            samples.popleft()

        if len(samples) >= self._thresholds.max_samples_per_track:
            history.overflowed = True
            samples.popleft()

        samples.append(sample)
        history.last_seen_ns = sample.stamp_ns

    def _verdict(
        self,
        track_id: int,
        history: _TrackHistory,
        now_ns: int,
    ) -> TrackVerdict:
        thresholds = self._thresholds
        latest = history.samples[-1]
        age_ns = now_ns - history.last_seen_ns

        def build(stable: bool, approachable: bool, reason: GateReason,
                  stable_for_ns: int, displacement_m: float) -> TrackVerdict:
            return TrackVerdict(
                track_id=track_id,
                stable=stable,
                approachable=approachable,
                reason=reason,
                stamp_ns=now_ns,
                confidence=latest.confidence,
                position=latest.position,
                distance_m=latest.distance_m,
                stable_for_ns=stable_for_ns,
                displacement_m=displacement_m,
                sample_count=len(history.samples),
            )

        if age_ns > thresholds.detection_gap_ns or age_ns < 0:
            return build(False, False, GateReason.DETECTION_LOST, 0, 0.0)

        observed_ns = history.last_seen_ns - history.streak_start_ns
        stable = observed_ns >= thresholds.stable_duration_ns
        displacement_m = _max_pairwise_distance(history.samples)

        if not stable:
            return build(False, False, GateReason.NOT_STABLE,
                         observed_ns, displacement_m)
        if observed_ns < thresholds.still_window_ns:
            return build(True, False, GateReason.INSUFFICIENT_HISTORY,
                         observed_ns, displacement_m)
        if history.overflowed:
            return build(True, False, GateReason.SAMPLE_OVERFLOW,
                         observed_ns, displacement_m)
        if displacement_m >= thresholds.max_displacement_m:
            return build(True, False, GateReason.MOVING,
                         observed_ns, displacement_m)
        if latest.distance_m < thresholds.min_distance_m:
            return build(True, False, GateReason.TOO_NEAR,
                         observed_ns, displacement_m)
        if latest.distance_m > thresholds.max_distance_m:
            return build(True, False, GateReason.TOO_FAR,
                         observed_ns, displacement_m)
        return build(True, True, GateReason.OK, observed_ns, displacement_m)

    def _rejected(
        self,
        sample: DetectionSample,
        reason: GateReason,
    ) -> TrackVerdict:
        """이력에 넣지 않은 표본에 대한 판정 — 이 프레임은 증거가 아니다."""
        position = sample.position if sample.position.is_finite() else None
        distance_m = sample.distance_m if math.isfinite(sample.distance_m) else None
        confidence = sample.confidence if math.isfinite(sample.confidence) else 0.0
        return TrackVerdict(
            track_id=sample.track_id,
            stable=False,
            approachable=False,
            reason=reason,
            stamp_ns=sample.stamp_ns,
            confidence=confidence,
            position=position,
            distance_m=distance_m,
            sample_count=self.sample_count(sample.track_id),
        )


def _replace_reason(verdict: TrackVerdict, reason: GateReason) -> TrackVerdict:
    return TrackVerdict(
        track_id=verdict.track_id,
        stable=verdict.stable,
        approachable=verdict.approachable,
        reason=reason,
        stamp_ns=verdict.stamp_ns,
        confidence=verdict.confidence,
        position=verdict.position,
        distance_m=verdict.distance_m,
        stable_for_ns=verdict.stable_for_ns,
        displacement_m=verdict.displacement_m,
        sample_count=verdict.sample_count,
    )


def _max_pairwise_distance(samples: Sequence[DetectionSample]) -> float:
    """창 안 표본들 사이의 최대 거리."""
    points = [s.position for s in samples]
    worst = 0.0
    for i, a in enumerate(points):
        for b in points[i + 1:]:
            distance = a.distance_to(b)
            if distance > worst:
                worst = distance
    return worst


def _sorted_verdicts(verdicts: Iterable[TrackVerdict]) -> Tuple[TrackVerdict, ...]:
    """승인된 후보 먼저, 그 안에서 가까운 사람 먼저, 동률이면 track_id 순."""
    return tuple(
        sorted(
            verdicts,
            key=lambda v: (
                not v.approachable,
                v.distance_m if v.distance_m is not None else math.inf,
                v.track_id,
            ),
        )
    )
