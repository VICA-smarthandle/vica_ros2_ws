"""목적지 접근 다단계 감속 사다리 (순수 로직, rclpy 비의존)."""
from __future__ import annotations

from typing import Optional, Sequence, Tuple

DEFAULT_APPROACH_STAGES: Tuple[Tuple[float, float], ...] = (
    (1.0, 80.0),
    (0.5, 60.0),
)

NO_SPEED_LIMIT = 0.0

StageList = Tuple[Tuple[float, float], ...]


def normalize_stages(stages) -> StageList:
    """단계 목록을 검증하고 먼 거리부터 정렬해 돌려준다."""
    parsed = []
    for stage in stages:
        try:
            distance, percent = stage
        except (TypeError, ValueError):
            raise ValueError(
                f"단계는 (거리 m, 비율 %) 쌍이어야 한다: {stage!r}"
            ) from None
        distance = float(distance)
        percent = float(percent)
        if not distance > 0.0:
            raise ValueError(f"단계 거리는 0보다 커야 한다: {distance}")
        if not 0.0 < percent <= 100.0:
            raise ValueError(f"단계 비율은 (0, 100] 범위여야 한다: {percent}")
        parsed.append((distance, percent))

    parsed.sort(key=lambda s: s[0], reverse=True)

    for (far_d, far_p), (near_d, near_p) in zip(parsed, parsed[1:]):
        if near_d == far_d:
            raise ValueError(f"단계 거리가 중복된다: {far_d}")
        if near_p >= far_p:
            raise ValueError(
                "가까운 단계일수록 비율이 낮아야 한다: "
                f"{far_d}m={far_p}% 다음에 {near_d}m={near_p}%"
            )

    return tuple(parsed)


def stages_from_lists(distances, percents) -> StageList:
    """ROS parameter 두 배열(거리·비율)을 단계 목록으로 합친다."""
    distance_list = [float(d) for d in distances]
    percent_list = [float(p) for p in percents]
    if len(distance_list) != len(percent_list):
        raise ValueError(
            "거리와 비율의 개수가 다르다: "
            f"거리 {len(distance_list)}개, 비율 {len(percent_list)}개"
        )
    return normalize_stages(zip(distance_list, percent_list))


class ApproachSpeedLadder:
    """잔여거리에 따라 한 방향으로만 내려가는 최대속도 제한 사다리."""

    def __init__(self, stages: Optional[Sequence] = None) -> None:
        """단계 목록을 받는다. None 이면 기본 단계, 빈 목록이면 기능을 끈다."""
        self._stages = normalize_stages(
            DEFAULT_APPROACH_STAGES if stages is None else stages
        )
        self._index = -1

    @property
    def stages(self) -> StageList:
        """검증·정렬을 마친 단계 목록 (먼 거리부터)."""
        return self._stages

    @property
    def index(self) -> int:
        """지금 들어가 있는 단계 번호. 제한 전이면 -1."""
        return self._index

    @property
    def percent(self) -> float:
        """지금 걸려 있는 제한율. 아직 제한 전이면 0.0(해제)."""
        return NO_SPEED_LIMIT if self._index < 0 else self._stages[self._index][1]

    def reset(self) -> None:
        """새 Goal 용 초기화. 다음 접근에서 다시 처음 단계부터 내려간다."""
        self._index = -1

    def update(self, distance_remaining: Optional[float]) -> Optional[float]:
        """이번 tick 에 새로 진입한 단계의 제한율. 바뀐 것이 없으면 None."""
        target = self._stage_index_for(distance_remaining)
        if target <= self._index:
            return None
        self._index = target
        return self._stages[target][1]

    def _stage_index_for(self, distance_remaining: Optional[float]) -> int:
        """이 거리에 해당하는 가장 깊은 단계 번호. 판단 불가면 현재 단계."""
        if distance_remaining is None or distance_remaining <= 0.0:
            return self._index

        index = -1
        for i, (threshold, _) in enumerate(self._stages):
            if distance_remaining <= threshold:
                index = i
        return index
