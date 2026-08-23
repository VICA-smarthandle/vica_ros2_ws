"""사람 접근 goal 계산 (순수 로직, rclpy 비의존).

왜 필요한가:

    YOLO 세그멘테이션이 시각장애인을 찾아 주는 것은 **사람이 서 있는 좌표**뿐이다.
    그 좌표를 그대로 Nav2 goal 로 보내면 로봇은 사람이 서 있는 자리로 들어가려
    한다. 사람은 장애물 지도에 없으므로 Nav2 는 멈출 이유를 모른다. 실제로
    멈추는 것은 `collision_monitor` 의 `PolygonStop`(0.60 m) 이고, 그때는 이미
    코앞이다.

    그래서 Mission Manager 가 goal 을 만들 때 **사람에게서 로봇 쪽으로 안전거리만큼
    물러난 지점**으로 옮긴다(2026-08-23 설계 5절). 이것이 안전 3겹 중 첫 겹이며,
    유일하게 사람을 사람으로 알고 동작하는 겹이다.

        goal.position = P + (R - P) / |R - P| * 안전거리
        goal.yaw      = 사람을 바라보는 방향

    Nav2 에는 좌표 하나만 간다. **Nav2 는 그것이 사람인지 모른다.** 등록된
    목적지로 갈 때와 완전히 같은 `NavigateToPose` 다.

안전거리 1.1 m 의 근거 (설계 6.3 절):

        안전거리 > circumscribed(0.4962) + 사람 몸 반경(0.25) + goal 오차

    | goal tolerance      | 하한   | 1.1 m 일 때 여유 |
    | 0.25 (일반 주행)     | 1.00 m | +10.4 cm        |
    | 0.10 (접근 전용)     | 0.85 m | +25.4 cm        |

    회전 반경으로 잡는 이유는, 도착한 뒤 제자리에서 사람 쪽으로 도는 동작이 다음
    사이클에 들어오기 때문이다. 미리 만족시켜 두지 않으면 그때 접근 거리를 다시
    바꿔야 한다. circumscribed 0.4962 는 현재 `nav2_params.yaml` footprint 값이고
    (`17f6820` 로 꼬리 0.595 -> 0.495), 사람 몸 반경 0.25 는 어깨 폭 기준이다.

    1.1 m 는 `collision_monitor` 의 `PolygonSlow`(x 0.36 ~ 1.10) 와 일부러 겹친다.
    마지막 구간에서 0.3 -> 0.12 m/s 로 느려지는 것은 설정 오류가 아니라 의도다
    (설계 6.4 절).

무엇을 하지 않는가:

    이 모듈은 **좌표 하나만 계산한다.** 접근할지 말지(`stable` / `approachable`),
    goal 을 갱신할지(0.5 m 임계), 언제 포기할지(2.0 m / 3초)는 판단하지 않는다.
    시계열을 보는 판정은 탐지 이력을 가진 `person_detector_node` 몫이고,
    상태 전이는 `mission_logic.py` 몫이다.
"""
from __future__ import annotations

import math
from typing import Optional

from .mission_logic import Pose2D

# 현재 footprint 의 외접원 반경. nav2_params.yaml 과 같은 값이어야 한다.
CIRCUMSCRIBED_RADIUS_M = 0.4962

# 사람 몸 반경(어깨 폭 기준). 사람은 costmap 에 없으므로 이 수치는 계산으로만 지킨다.
PERSON_BODY_RADIUS_M = 0.25

# 접근 전용 goal tolerance. 일반 주행 0.25 는 그대로 두고 접근 구간에만 건다.
APPROACH_GOAL_TOLERANCE_M = 0.10
DRIVING_GOAL_TOLERANCE_M = 0.25

# 사람 앞 정지 거리. 근거는 모듈 docstring 참조.
DEFAULT_APPROACH_DISTANCE_M = 1.1

# 이 거리 미만이면 사람-로봇 방향이 잡음이라 계산을 포기한다.
# D455 depth 정밀도는 cm 단위이므로 1 mm 미만의 간격은 실측이 아니라 고장이다.
DEGENERATE_SEPARATION_M = 1e-3


def normalize_yaw_deg(yaw_deg: float) -> float:
    """각도를 (-180, 180] 한 바퀴 안으로 접는다.

    `atan2` 결과는 이미 이 범위지만, 두 각도를 빼거나 보정을 더하면 금방
    벗어난다. 같은 방향에 179.99 와 -180.01 두 표기가 생기면 "얼마나 돌아야
    하는가"를 빼기로 구하는 코드가 359.99도를 돌라고 말한다.

    -180 은 +180 으로 모은다. 정서쪽 한 방향에 표기가 둘이면 goal 비교가 흔들린다.
    """
    wrapped = math.fmod(float(yaw_deg), 360.0)
    if wrapped > 180.0:
        wrapped -= 360.0
    elif wrapped <= -180.0:
        wrapped += 360.0
    # fmod 는 -0.0 을 만들 수 있다. 표기를 하나로 고정한다.
    return wrapped + 0.0


def approach_goal(
    person: Pose2D,
    robot: Pose2D,
    safety_distance_m: float = DEFAULT_APPROACH_DISTANCE_M,
) -> Optional[Pose2D]:
    """사람 앞 `safety_distance_m` 지점을 바라보는 goal pose 를 만든다.

    사람에게서 **로봇이 있는 쪽으로** 물러난다. 로봇이 어디에 있든 goal 은
    사람-로봇 선분 위에 놓이므로, 로봇은 자기가 온 방향으로 곧장 다가가면 된다.
    yaw 는 goal 에서 사람을 향하는 방향이며 로봇->사람 방향과 같다.

    돌려주는 값:

        Pose2D  정상. frame_id 는 입력을 그대로 물려준다.
        None    사람과 로봇 위치가 사실상 같아 방향을 정할 수 없다.
                호출부는 `RequestApproach` 를 거부해야 한다(`accepted=false`).

    경계 처리 두 가지와 그 근거:

    1. `|R - P|` 가 0 에 가까울 때 -> `None`

       로봇이 사람 위에 서 있는 일은 물리적으로 불가능하다. 이 입력은 depth 0
       이나 TF 항등 변환 같은 **탐지 고장**이다. 방향이 정의되지 않으므로 아무
       방향이나 고르면 사람 쪽으로 밀고 들어갈 수 있다. 예외를 던지지 않는 이유는
       이것이 기동 파라미터가 아니라 5 Hz 로 흘러드는 센서 값이기 때문이다.
       `check_gate` 가 거부 사유를 값으로 돌려주는 것과 같은 결이다.

    2. 사람이 안전거리보다 가까울 때 -> 로봇 자리에 그대로 선다

       수식대로면 goal 이 **로봇 뒤**에 생긴다. 후진은 이 로봇에서 닫힌 축이다
       (`docs/nav2_backlog.md` 9절 `BackUp`) — 핸들 뒤에 사람이 서기 때문에
       2026-07-30 에 실제로 30 cm 후진해 문제가 됐다. 그리고 이미 1.1 m 안쪽이면
       말을 걸 수 있는 거리라 물러날 이유도 없다. 그래서 위치는 로봇 그대로 두고
       yaw 만 사람 쪽으로 돌린다. 정확히 안전거리일 때 두 갈래가 같은 점을 내므로
       경계에 불연속이 없다.

    잘못된 입력은 예외로 터뜨린다:

        safety_distance_m <= 0  안전거리를 없애는 값이다. 파라미터 오타는
                                실주행 전에 죽는 편이 안전하다.
        frame_id 불일치          map 과 odom 을 섞으면 엉뚱한 좌표로 간다.
                                센서 잡음이 아니라 배선 실수다.
    """
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
        # 방향이 없다. goal 을 만들지 않는다 (경계 1).
        return None

    # 로봇에서 사람을 보는 방향. goal 은 이 선 위에 있으므로 goal 에서 본 방향과 같다.
    yaw_deg = normalize_yaw_deg(math.degrees(math.atan2(-dy, -dx)))

    if separation <= safety_distance_m:
        # 이미 안전거리 안이다. 뒤로 물러나지 않고 보는 방향만 맞춘다 (경계 2).
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
