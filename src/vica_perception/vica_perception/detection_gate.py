"""사람 접근 출발 조건 판정 (순수 로직, rclpy 비의존).

정본은 `devlog/2026-08-23-사람접근-구현설계.md` §6.1 이다.

왜 판정이 둘인가:

    `stable`(신뢰도 0.6 이상이 1.0초 연속)은 **"진짜 사람인가"** 의 판정이고,
    `approachable`(3초간 이동 0.3 m 미만 + 거리 1.5~4.0 m)은 **"다가갈 가치가
    있는가"** 의 판정이다. 둘은 거르는 대상이 다르다 — 앞은 오탐을, 뒤는
    지나가는 사람을 거른다.

    근거는 속도다. 로봇 주행 상한은 0.5 m/s 인데 보행은 1.2 m/s 다. 3 m 앞
    사람에게 접근 속도 0.3 m/s 로 가면 6.3초가 걸리는데 그동안 걷는 사람은
    7.6 m 를 간다 — 접근 포기 기준(2.0 m 이탈)을 4배 넘긴다. **걷는 사람은
    절대 따라잡지 못하므로 멈춰 있는 사람만 대상으로 한다.** `stable` 만으로
    출발하면 출발하자마자 포기하고 돌아오는 동작만 반복하며, 그 자체가 통행
    방해가 된다.

    도움이 필요한 사람은 대개 멈춰 있다 — 길을 찾거나, 방향을 잃었거나,
    무언가를 기다리는 중이다. 이 제약은 시나리오와도 맞는다.

시간을 인자로 받는 이유:

    이 모듈은 `time.time()` 을 부르지 않는다. 모든 시각은 호출자(ROS 노드)가
    소유한 단일 STEADY_TIME clock 의 **정수 나노초**로 들어온다. 계약 정본은
    `vica_safety/freshness.py` 이며, 미수신은 `None`(0.0 sentinel 금지),
    시간 역전은 안전한 쪽(stale)으로 넘어진다. 안전 계층에 대한 역방향 의존을
    만들지 않으려고 `sec_to_ns` 만 여기 복제한다 —
    `vica_user_guidance/timebase.py` 와 같은 방식이다.

이 모듈이 하지 않는 것:

    **접근 대상을 고르지 않는다.** 자격을 갖춘 후보를 전부 돌려줄 뿐이다.
    설계 §3 대로 탐지 결과는 *요청*이고 goal 권한은 Mission Manager 가
    독점한다. 재탐지 억제 60초·현재 미션 상태는 Mission 쪽 정보이므로 선택도
    그쪽에서 한다. 다만 순서는 결정론적으로(가까운 사람 먼저, 동률이면
    `track_id` 오름차순) 고정한다 — 같은 입력에 같은 결과가 나와야 시험과
    재현이 된다.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Deque, Dict, Iterable, Optional, Sequence, Tuple

# ---- 기본 임계값 -------------------------------------------------------------
#
# 실기 조정은 launch parameter 로 GateThresholds 를 만들어 넣고, 이 상수는
# 건드리지 않는다.

# 설계 §12 에서 `[미검증]`. 모델 실측 후 확정한다.
DEFAULT_MIN_CONFIDENCE = 0.6
# "진짜 사람인가" — 이 시간만큼 연속으로 보여야 stable.
DEFAULT_STABLE_DURATION_S = 1.0
# "멈춰 있는가" — 이 시간만큼의 이력을 보고 이동량을 잰다.
DEFAULT_STILL_WINDOW_S = 3.0
# 창 안에서 이만큼 미만으로 움직여야 '정지'다.
DEFAULT_MAX_DISPLACEMENT_M = 0.3
# 하한: 이보다 가까우면 접근 goal(1.1 m)이 이미 지나간 자리다.
DEFAULT_MIN_DISTANCE_M = 1.5
# 상한: 이보다 멀면 도착까지 10초를 넘겨 도착 시점에 상황이 이미 달라진다.
DEFAULT_MAX_DISTANCE_M = 4.0

# 이 간격을 넘겨 끊기면 '연속'이 깨진 것으로 본다. 추론 주기 5 Hz(0.2초)
# 기준으로 3프레임 결손에 해당한다.
DEFAULT_DETECTION_GAP_S = 0.6
# 이 속도를 넘는 이동은 사람이 아니라 추적기의 ID 오배정(또는 depth 튐)으로
# 본다. 보행 1.2 m/s, 달리기 시작이 3 m/s 안팎이다. `[미검증]`
DEFAULT_TELEPORT_SPEED_MPS = 2.0
# 이 시간 동안 안 보이면 track 자체를 버린다.
DEFAULT_TRACK_EXPIRY_S = 5.0
# track 하나가 들고 있을 표본 수 상한. **추론 주기 x still_window_s 보다 커야
# 한다** — 5 Hz x 3초 = 16개이므로 64는 약 21 Hz 까지 여유가 있다. 모자라면
# 이동량을 믿을 수 없으므로 SAMPLE_OVERFLOW 로 승인을 거부한다(fail-safe).
DEFAULT_MAX_SAMPLES_PER_TRACK = 64
# 동시에 들고 있을 track 수 상한. 추적기가 ID 를 폭주 발급해도 메모리가
# 무한히 늘지 않게 한다.
DEFAULT_MAX_TRACKS = 32


def sec_to_ns(seconds: float) -> int:
    """초 단위 시간을 정수 나노초로 바꾼다.

    계약 정본은 `vica_safety/freshness.py`. 안전 계층에 대한 역방향 의존을
    만들지 않기 위한 의도적 복제다.
    """
    return int(seconds * 1_000_000_000)


class GateReason(str, Enum):
    """`approachable` 이 아니라면 **왜** 아닌가.

    로그와 진단에 그대로 쓴다. "안 간다"만 남으면 실기에서 원인을 못 찾는다.
    """

    OK = "ok"
    # 표본 자체를 못 믿는다.
    INVALID_SAMPLE = "invalid_sample"      # NaN/inf (depth 구멍 등)
    LOW_CONFIDENCE = "low_confidence"      # 신뢰도 임계 미달
    # 연속성이 깨졌다.
    DETECTION_LOST = "detection_lost"      # 결손이 임계를 넘었다
    TIME_REVERSED = "time_reversed"        # 시각이 뒤로 갔다
    TRACK_JUMP = "track_jump"              # 같은 ID 인데 순간이동
    # 이력이 모자라다.
    NOT_STABLE = "not_stable"              # 연속 관측 1.0초 미달
    INSUFFICIENT_HISTORY = "insufficient_history"  # 이력 3.0초 미달
    SAMPLE_OVERFLOW = "sample_overflow"    # 창 안 표본을 버렸다 → 이동량 불신
    # 조건 불충족.
    MOVING = "moving"                      # 지나가는 사람
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
    """한 프레임에서 `track_id` 하나에 대한 관측.

    `stamp_ns` 는 호출자가 소유한 STEADY_TIME clock 의 정수 나노초다.
    `distance_m` 은 로봇-사람 거리(설계 §12: 마스크 전체가 아니라 몸통 depth
    중앙값을 써야 한다), `position` 은 map 으로 변환을 마친 좌표다.
    """

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
    """판정 임계값 묶음.

    잘못된 조합은 조용히 굴러가는 것보다 기동 시점에 죽는 편이 안전하다.
    승인이 헐거워지면 걷는 사람에게 출발하고, 빡빡해지면 멈춰 있는 사람에게도
    영영 다가가지 못한다.
    """

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
            # 3초 창이 1초 연속보다 짧으면 "출발까지 최소 3초"가 무너진다.
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
            # 이동량을 재려면 최소 두 점이 필요하다.
            raise ValueError(
                f"표본 상한은 2 이상이어야 한다: {self.max_samples_per_track}"
            )
        if self.max_tracks < 1:
            raise ValueError(f"track 상한은 1 이상이어야 한다: {self.max_tracks}")

    # ROS 파라미터는 초로 받고, 판정은 전부 정수 나노초로 한다.
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
    """한 `track_id` 에 대한 판정 결과. 판정은 기록이므로 얼려서 돌려준다.

    `stable`·`approachable` 은 `PersonDetection.msg` 의 같은 이름 필드로 그대로
    나가고, 나머지는 진단(로그·bag 분석)용이다.
    """

    track_id: int
    stable: bool
    approachable: bool
    reason: GateReason
    stamp_ns: int
    confidence: float
    position: Optional[Point2D]
    distance_m: Optional[float]
    # 지금 연속 구간을 얼마나 관측했는가. **관측하지 않은 시간은 증거가 아니므로**
    # 마지막 채택 표본까지만 센다.
    stable_for_ns: int = 0
    # 창 안 표본들 사이의 최대 거리. 시작-끝만 재면 갔다 온 사람이 0 이 된다.
    displacement_m: float = 0.0
    sample_count: int = 0


@dataclass
class _TrackHistory:
    """`track_id` 하나의 이력. 채택된(고신뢰) 표본만 들어온다."""

    samples: Deque[DetectionSample] = field(default_factory=deque)
    # 지금 연속 구간이 시작된 시각. prune 과 무관하게 유지되므로 "3초어치
    # 쌓였는가"를 창 밖 표본을 남기지 않고도 판정할 수 있다.
    streak_start_ns: int = 0
    last_seen_ns: int = 0
    # 창 안 표본을 상한 때문에 버린 적이 있는가 → 이동량을 믿을 수 없다.
    overflowed: bool = False

    def restart(self, sample: DetectionSample) -> None:
        self.samples.clear()
        self.samples.append(sample)
        self.streak_start_ns = sample.stamp_ns
        self.last_seen_ns = sample.stamp_ns
        self.overflowed = False


class DetectionGate:
    """시계열 탐지 결과에서 `stable` 과 `approachable` 을 판정한다.

    `track_id` 별로 이력을 따로 쌓는다 — 여러 사람이 동시에 보일 수 있다.
    """

    def __init__(self, thresholds: Optional[GateThresholds] = None) -> None:
        self._thresholds = thresholds if thresholds is not None else GateThresholds()
        self._tracks: Dict[int, _TrackHistory] = {}

    @property
    def thresholds(self) -> GateThresholds:
        return self._thresholds

    # ---- 조회 ---------------------------------------------------------------

    def track_ids(self) -> Tuple[int, ...]:
        """지금 이력을 들고 있는 track 목록 (최근에 본 순서가 아니라 삽입 순)."""
        return tuple(self._tracks.keys())

    def sample_count(self, track_id: int) -> int:
        history = self._tracks.get(track_id)
        return len(history.samples) if history else 0

    # ---- 입력 ---------------------------------------------------------------

    def observe(self, sample: DetectionSample) -> TrackVerdict:
        """표본 하나를 넣고 그 track 의 판정을 돌려준다."""
        if not sample.is_finite():
            # depth 구멍은 NaN 으로 온다. 이력에 섞이면 이후 비교가 전부
            # False 가 되어 '정지'로 오판할 수 있다.
            return self._rejected(sample, GateReason.INVALID_SAMPLE)

        if sample.confidence < self._thresholds.min_confidence:
            # 이 프레임은 증거가 아니다 — 이력을 건드리지 않고 버린다.
            # 한 프레임 튀었다고 3초를 다시 쌓게 하지 않으려는 것이고,
            # 저신뢰가 결손 임계보다 오래 이어지면 아래 gap 규칙이 끊는다.
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
            # 리셋 직후는 당연히 NOT_STABLE 이다. 무엇이 끊었는지를 남긴다.
            verdict = _replace_reason(verdict, fault)
        return verdict

    def update(
        self,
        samples: Iterable[DetectionSample],
        now_ns: int,
    ) -> Tuple[TrackVerdict, ...]:
        """한 프레임(표본 여러 개)을 넣고 살아 있는 모든 track 의 판정을 준다.

        승인된 후보가 앞에, 그 안에서는 가까운 사람이 앞에 온다.
        """
        for sample in samples:
            self.observe(sample)
        self.prune(now_ns)
        verdicts = [
            self.evaluate(track_id, now_ns) for track_id in tuple(self._tracks)
        ]
        return _sorted_verdicts(v for v in verdicts if v is not None)

    # ---- 판정 ---------------------------------------------------------------

    def evaluate(self, track_id: int, now_ns: int) -> Optional[TrackVerdict]:
        """새 표본 없이 현재 시각 기준으로 판정한다. 모르는 track 이면 None."""
        history = self._tracks.get(track_id)
        if history is None or not history.samples:
            return None
        return self._verdict(track_id, history, now_ns)

    def approachable_tracks(self, now_ns: int) -> Tuple[TrackVerdict, ...]:
        """지금 접근 자격을 갖춘 후보 전부. **고르는 것은 여기 책임이 아니다.**

        재탐지 억제 60초와 미션 상태를 아는 것은 Mission Manager 이므로 선택도
        그쪽에서 한다. 여기서는 순서만 결정론적으로 고정한다.
        """
        verdicts = []
        for track_id in tuple(self._tracks):
            verdict = self.evaluate(track_id, now_ns)
            if verdict is not None and verdict.approachable:
                verdicts.append(verdict)
        return _sorted_verdicts(verdicts)

    # ---- 이력 관리 -----------------------------------------------------------

    def prune(self, now_ns: int) -> None:
        """오래 안 보인 track 을 버린다. 이력이 무한히 쌓이지 않게 하는 축이다."""
        expiry_ns = self._thresholds.track_expiry_ns
        for track_id in tuple(self._tracks):
            age_ns = now_ns - self._tracks[track_id].last_seen_ns
            if age_ns > expiry_ns or age_ns < 0:
                # 시간 역전(age < 0)도 버린다 — fail-safe.
                del self._tracks[track_id]

    def forget(self, track_id: int) -> None:
        """track 하나를 잊는다. 접근을 마친 뒤 Mission 이 부를 수 있다."""
        self._tracks.pop(track_id, None)

    def reset(self) -> None:
        """전체 초기화 (E-stop·미션 취소 등)."""
        self._tracks.clear()

    # ---- 내부 ---------------------------------------------------------------

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
            # 같은 시각에 두 위치는 있을 수 없다.
            return GateReason.TRACK_JUMP if moved_m > 0.0 else None
        speed_mps = moved_m / (dt_ns / 1_000_000_000.0)
        if speed_mps > self._thresholds.teleport_speed_mps:
            # 같은 ID 인데 사람이 낼 수 없는 속도 → 추적기가 ID 를 엉뚱한
            # 사람에게 붙였다고 본다. 이력을 버리고 3초를 다시 센다.
            return GateReason.TRACK_JUMP
        return None

    def _append(self, history: _TrackHistory, sample: DetectionSample) -> None:
        window_ns = self._thresholds.still_window_ns
        samples = history.samples

        # 창 밖 표본을 버린다 — 창은 '최근 3초'다.
        while samples and sample.stamp_ns - samples[0].stamp_ns > window_ns:
            samples.popleft()

        if len(samples) >= self._thresholds.max_samples_per_track:
            # 창 **안** 표본을 버려야 한다. 이동량이 실제보다 작게 나올 수
            # 있으므로 이 연속 구간 동안은 승인하지 않는다(fail-safe).
            # 상한을 추론 주기 x still_window_s 이상으로 올려야 한다는 신호다.
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

        # 관측이 끊긴 채 시간만 흘렀다면 stable 을 유지하면 안 된다.
        # 시간 역전(age < 0)도 stale 로 본다 — vica_safety/freshness.py 계약.
        if age_ns > thresholds.detection_gap_ns or age_ns < 0:
            return build(False, False, GateReason.DETECTION_LOST, 0, 0.0)

        observed_ns = history.last_seen_ns - history.streak_start_ns
        stable = observed_ns >= thresholds.stable_duration_ns
        displacement_m = _max_pairwise_distance(history.samples)

        if not stable:
            return build(False, False, GateReason.NOT_STABLE,
                         observed_ns, displacement_m)
        if observed_ns < thresholds.still_window_ns:
            # 아직 3초어치가 없다 — 모르면 안 간다.
            return build(True, False, GateReason.INSUFFICIENT_HISTORY,
                         observed_ns, displacement_m)
        if history.overflowed:
            return build(True, False, GateReason.SAMPLE_OVERFLOW,
                         observed_ns, displacement_m)
        if displacement_m >= thresholds.max_displacement_m:
            # '0.3 m 미만'이므로 경계값은 실패다.
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
    """창 안 표본들 사이의 최대 거리.

    시작-끝만 비교하면 0.4 m 를 갔다 온 사람이 '이동 0 m'로 보인다. 표본 수는
    still_window_s x 추론 주기(5 Hz 기준 16개)라 O(n^2) 로도 충분히 싸다.
    """
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
