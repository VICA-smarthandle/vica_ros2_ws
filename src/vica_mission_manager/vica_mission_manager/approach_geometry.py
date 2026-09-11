"""사람 접근 goal 계산 (순수 로직, rclpy 비의존)."""
from __future__ import annotations

import math
from typing import Optional

from .mission_logic import Pose2D

CIRCUMSCRIBED_RADIUS_M = 0.4962

PERSON_BODY_RADIUS_M = 0.25

APPROACH_GOAL_TOLERANCE_M = 0.10
DRIVING_GOAL_TOLERANCE_M = 0.25

DEFAULT_APPROACH_DISTANCE_M = 1.1

DEGENERATE_SEPARATION_M = 1e-3


def normalize_yaw_deg(yaw_deg: float) -> float:
    """각도를 (-180, 180] 한 바퀴 안으로 접는다."""
    wrapped = math.fmod(float(yaw_deg), 360.0)
    if wrapped > 180.0:
        wrapped -= 360.0
    elif wrapped <= -180.0:
        wrapped += 360.0
    return wrapped + 0.0


def approach_goal(
    person: Pose2D,
    robot: Pose2D,
    safety_distance_m: float = DEFAULT_APPROACH_DISTANCE_M,
) -> Optional[Pose2D]:
    """사람 앞 `safety_distance_m` 지점을 바라보는 goal pose 를 만든다."""
    safety_distance_m = float(safety_distance_m)
    if not safety_distance_m > 0.0:
        raise ValueError(f"안전거리는 0보다 커야 한다: {safety_distance_m}")
    if person.frame_id != robot.frame_id:
        raise ValueError(
            "사람과 로봇 좌표의 frame 이 다르다: "
            f"person={person.frame_id!r}, robot={robot.frame_id!r}"
        )

    dx = robot.x - person.x
    dy = robot.y - person.y
    separation = math.hypot(dx, dy)

    if separation < DEGENERATE_SEPARATION_M:
        return None

    yaw_deg = normalize_yaw_deg(math.degrees(math.atan2(-dy, -dx)))

    if separation <= safety_distance_m:
        return Pose2D(
            x=robot.x, y=robot.y, yaw_deg=yaw_deg, frame_id=person.frame_id
        )

    scale = safety_distance_m / separation
    return Pose2D(
        x=person.x + dx * scale,
        y=person.y + dy * scale,
        yaw_deg=yaw_deg,
        frame_id=person.frame_id,
    )
