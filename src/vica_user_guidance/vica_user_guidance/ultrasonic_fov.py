"""채널별 Range.field_of_view(costmap 이 칠하는 부채꼴 폭) 결정 — 순수 함수.

RangeSensorLayer 는 메시지마다 실린 field_of_view 로 호를 그린다. 그래서 채널마다 다른
폭을 줄 수 있다. 2026-09-24: 바퀴 옆 두 채널만 물리 빔을 60°로 넓히면서 칠하는 폭도
60°로 맞췄다(사용자 결정). 나머지는 공통값(30°, 09-02 A/B 확정).
"""
from typing import List, Sequence


def resolve_channel_fov(
    common: float, per_channel: Sequence[float], channels: int,
    name: str = "ultrasonic_fov_rad_per_channel",
) -> List[float]:
    """per_channel 의 양수 칸은 그 값, 0 이하 칸은 common. 길이가 channels 가 아니면 오류."""
    if len(per_channel) != channels:
        raise ValueError(
            f"{name} 은 {channels}칸이어야 합니다: {len(per_channel)}"
        )
    return [float(v) if v > 0 else float(common) for v in per_channel]
