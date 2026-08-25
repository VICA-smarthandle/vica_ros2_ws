"""사람 접근 goal 계산 순수 로직 검증 (ROS 없이 실행된다).

이 계산이 틀리면 로봇이 시각장애인에게 **너무 가까이 붙거나 뒤로 물러난다.**
둘 다 실주행에서 사람을 다치게 하는 방향이라 경계 조건을 여기서 전부 막는다.

    1. 사람과 로봇이 같은 좌표  -> 0 으로 나눈다 (방향이 정의되지 않는다)
    2. 사람이 안전거리보다 가까움 -> goal 이 로봇 뒤로 간다 (후진 금지)
    3. yaw 가 +-180도 경계를 넘음 -> 같은 방향인데 값이 360도 튄다
    4. 정상 케이스               -> 손으로 검산되는 값이어야 한다
"""
import math

import pytest

from vica_mission_manager.approach_geometry import (
    APPROACH_GOAL_TOLERANCE_M,
    CIRCUMSCRIBED_RADIUS_M,
    DEFAULT_APPROACH_DISTANCE_M,
    DRIVING_GOAL_TOLERANCE_M,
    PERSON_BODY_RADIUS_M,
    approach_goal,
    normalize_yaw_deg,
)
from vica_mission_manager.mission_logic import Pose2D


def pose(x, y, yaw_deg=0.0, frame_id="map"):
    return Pose2D(x=x, y=y, yaw_deg=yaw_deg, frame_id=frame_id)


def distance(a: Pose2D, b: Pose2D) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def angular_diff_deg(a: float, b: float) -> float:
    """두 각도의 최단 차이. +-180도 경계를 넘어도 작은 값이 나온다."""
    return abs(normalize_yaw_deg(a - b))


# ---- 안전거리 근거 -----------------------------------------------------------


def test_default_distance_is_the_agreed_1_1_m():
    """2026-08-23 설계 6.2 절 확정값."""
    assert DEFAULT_APPROACH_DISTANCE_M == 1.1


def test_default_distance_clears_geometry_lower_bound():
    """1.1 m 는 회전 반경 + 사람 몸 반경 + goal 오차를 넘어야 한다.

    설계 6.3 절: 0.4962 + 0.25 + 0.10 = 0.8462 m 가 하한이다.
    """
    lower_bound = (
        CIRCUMSCRIBED_RADIUS_M + PERSON_BODY_RADIUS_M + APPROACH_GOAL_TOLERANCE_M
    )
    assert DEFAULT_APPROACH_DISTANCE_M > lower_bound
    assert DEFAULT_APPROACH_DISTANCE_M - lower_bound == pytest.approx(0.2538)


def test_default_distance_also_clears_driving_tolerance():
    """접근 전용 tolerance 를 못 걸어도(0.25 유지) 여유 10.4 cm 는 남는다."""
    lower_bound = (
        CIRCUMSCRIBED_RADIUS_M + PERSON_BODY_RADIUS_M + DRIVING_GOAL_TOLERANCE_M
    )
    assert DEFAULT_APPROACH_DISTANCE_M - lower_bound == pytest.approx(0.1038)


# ---- 정상 케이스 (손으로 검산되는 값) ----------------------------------------


def test_robot_due_east_of_person():
    """P(0,0) 동쪽 3 m 에 로봇 -> goal 은 (1.1, 0), 서쪽(180도)을 본다."""
    goal = approach_goal(person=pose(0.0, 0.0), robot=pose(3.0, 0.0))
    assert goal.x == pytest.approx(1.1)
    assert goal.y == pytest.approx(0.0)
    assert goal.yaw_deg == pytest.approx(180.0)


def test_robot_due_north_of_person():
    """P(0,0) 북쪽 3 m 에 로봇 -> goal 은 (0, 1.1), 남쪽(-90도)을 본다."""
    goal = approach_goal(person=pose(0.0, 0.0), robot=pose(0.0, 3.0))
    assert goal.x == pytest.approx(0.0)
    assert goal.y == pytest.approx(1.1)
    assert goal.yaw_deg == pytest.approx(-90.0)


def test_three_four_five_triangle():
    """P(1,1) - R(4,5) 는 3-4-5 삼각형이라 |R-P| = 5, 단위벡터 (0.6, 0.8) 이다.

    goal = (1 + 0.6*1.1, 1 + 0.8*1.1) = (1.66, 1.88)
    yaw  = atan2(1-1.88, 1-1.66) = atan2(-0.88, -0.66) = -126.87도
    """
    goal = approach_goal(person=pose(1.0, 1.0), robot=pose(4.0, 5.0))
    assert goal.x == pytest.approx(1.66)
    assert goal.y == pytest.approx(1.88)
    assert goal.yaw_deg == pytest.approx(-126.8699, abs=1e-3)


def test_custom_safety_distance():
    """안전거리는 인자로 바꿀 수 있다 (좁은 통로·큰 짐 등 현장 판단)."""
    goal = approach_goal(
        person=pose(0.0, 0.0), robot=pose(4.0, 0.0), safety_distance_m=1.5
    )
    assert goal.x == pytest.approx(1.5)
    assert goal.y == pytest.approx(0.0)


@pytest.mark.parametrize(
    "px,py,rx,ry",
    [
        (0.0, 0.0, 3.0, 0.0),
        (0.0, 0.0, 0.0, 3.0),
        (1.0, 1.0, 4.0, 5.0),
        (-2.5, 3.5, -7.0, -1.0),
        (2.0, -1.0, 2.0, -4.2),
    ],
)
def test_goal_keeps_exactly_the_safety_distance(px, py, rx, ry):
    """어느 방향이든 goal 과 사람 사이는 정확히 안전거리다."""
    person = pose(px, py)
    goal = approach_goal(person=person, robot=pose(rx, ry))
    assert distance(goal, person) == pytest.approx(DEFAULT_APPROACH_DISTANCE_M)


@pytest.mark.parametrize(
    "px,py,rx,ry",
    [
        (0.0, 0.0, 3.0, 0.0),
        (1.0, 1.0, 4.0, 5.0),
        (-2.5, 3.5, -7.0, -1.0),
    ],
)
def test_goal_lies_between_person_and_robot(px, py, rx, ry):
    """goal 은 사람-로봇을 잇는 선분 위에 있다 = 로봇은 앞으로만 간다."""
    person, robot = pose(px, py), pose(rx, ry)
    goal = approach_goal(person=person, robot=robot)
    assert distance(goal, person) + distance(goal, robot) == pytest.approx(
        distance(person, robot)
    )


@pytest.mark.parametrize(
    "px,py,rx,ry",
    [
        (0.0, 0.0, 3.0, 0.0),
        (1.0, 1.0, 4.0, 5.0),
        (-2.5, 3.5, -7.0, -1.0),
        (2.0, -1.0, 2.0, -4.2),
    ],
)
def test_goal_yaw_looks_at_the_person(px, py, rx, ry):
    """goal 에 서면 사람을 정면으로 본다 = 로봇->사람 방향과 같다."""
    person, robot = pose(px, py), pose(rx, ry)
    goal = approach_goal(person=person, robot=robot)
    expected = math.degrees(math.atan2(person.y - robot.y, person.x - robot.x))
    assert angular_diff_deg(goal.yaw_deg, expected) == pytest.approx(0.0, abs=1e-9)


def test_goal_keeps_the_map_frame():
    """Nav2 goal 은 map frame 이어야 한다 - 입력 frame 을 그대로 물려준다."""
    goal = approach_goal(person=pose(0.0, 0.0), robot=pose(3.0, 0.0))
    assert goal.frame_id == "map"


def test_person_yaw_is_ignored():
    """사람이 어느 쪽을 보든 goal 은 로봇 쪽으로 물러난 지점 하나뿐이다."""
    facing_away = approach_goal(person=pose(0, 0, yaw_deg=0.0), robot=pose(3.0, 0.0))
    facing_robot = approach_goal(person=pose(0, 0, yaw_deg=170.0), robot=pose(3.0, 0.0))
    assert facing_away == facing_robot


# ---- 경계 1: 같은 위치 (0 으로 나눈다) ---------------------------------------


def test_same_position_returns_none():
    """|R-P| = 0 이면 물러날 방향이 없다 -> goal 을 만들지 않는다.

    로봇이 사람 위에 서 있는 일은 물리적으로 불가능하다. 이 입력은 depth 0 이나
    TF 항등 변환 같은 **탐지 고장**이므로, 아무 방향이나 골라 goal 을 내보내면
    사람 쪽으로 밀고 들어갈 수 있다. 거부가 유일하게 안전한 답이다.
    """
    assert approach_goal(person=pose(2.0, 2.0), robot=pose(2.0, 2.0)) is None


def test_sub_millimeter_separation_returns_none():
    """1 mm 미만도 거부한다. D455 depth 정밀도는 cm 단위라 방향이 잡음이다."""
    assert approach_goal(person=pose(0.0, 0.0), robot=pose(0.0, 5e-4)) is None


def test_one_centimeter_separation_still_computes():
    """거부는 고장 판정에만 쓴다. 1 cm 는 방향이 살아 있으므로 계산한다."""
    goal = approach_goal(person=pose(0.0, 0.0), robot=pose(0.01, 0.0))
    assert goal is not None
    assert goal.yaw_deg == pytest.approx(180.0)


# ---- 경계 2: 사람이 안전거리보다 가깝다 (후진 금지) --------------------------


def test_person_closer_than_safety_distance_stays_in_place():
    """수식대로면 goal 이 로봇 뒤 0.4 m 에 생긴다. 그 자리에 머무른다.

    후진은 이 로봇에서 닫힌 축이다 (docs/nav2_backlog.md 9절 `BackUp`) -
    핸들 뒤에 사람이 서기 때문이다. 이미 1.1 m 안쪽이면 말을 걸 수 있는
    거리이므로 더 움직일 이유도 없다.
    """
    robot = pose(0.7, 0.0)
    goal = approach_goal(person=pose(0.0, 0.0), robot=robot)
    assert goal.x == pytest.approx(robot.x)
    assert goal.y == pytest.approx(robot.y)
    assert goal.yaw_deg == pytest.approx(180.0)


def test_close_range_goal_never_moves_backward():
    """어느 방향에서 가까워도 goal 은 로봇 위치 그대로다."""
    for rx, ry in [(0.5, 0.0), (0.0, -0.9), (0.3, 0.4), (-0.2, 0.2)]:
        robot = pose(rx, ry)
        goal = approach_goal(person=pose(0.0, 0.0), robot=robot)
        assert distance(goal, robot) == pytest.approx(0.0)


def test_exactly_at_safety_distance_is_the_same_point():
    """정확히 1.1 m 면 두 갈래(수식·제자리)가 같은 답을 낸다 - 불연속이 없다."""
    robot = pose(1.1, 0.0)
    goal = approach_goal(person=pose(0.0, 0.0), robot=robot)
    assert goal.x == pytest.approx(1.1)
    assert goal.y == pytest.approx(0.0)


def test_close_range_still_turns_to_face_the_person():
    """제자리에 서더라도 사람을 바라보게 돌려야 말을 걸 수 있다."""
    goal = approach_goal(person=pose(1.0, 1.0), robot=pose(1.5, 1.5))
    assert goal.yaw_deg == pytest.approx(-135.0)


# ---- 경계 3: yaw 가 +-180도 경계를 넘는다 ------------------------------------


@pytest.mark.parametrize(
    "raw,expected",
    [
        (0.0, 0.0),
        (180.0, 180.0),
        (-180.0, 180.0),
        (190.0, -170.0),
        (-190.0, 170.0),
        (360.0, 0.0),
        (540.0, 180.0),
        (-540.0, 180.0),
        (359.0, -1.0),
    ],
)
def test_normalize_yaw_deg(raw, expected):
    """yaw 는 (-180, 180] 한 바퀴 안에 들어온다. -180 은 180 으로 모은다."""
    assert normalize_yaw_deg(raw) == pytest.approx(expected)


def test_yaw_is_always_within_one_turn():
    """어느 배치에서든 goal.yaw_deg 는 (-180, 180] 을 벗어나지 않는다."""
    for angle_deg in range(0, 360, 7):
        rad = math.radians(angle_deg)
        robot = pose(3.0 * math.cos(rad), 3.0 * math.sin(rad))
        goal = approach_goal(person=pose(0.0, 0.0), robot=robot)
        assert -180.0 < goal.yaw_deg <= 180.0


def test_yaw_is_continuous_across_the_boundary():
    """서쪽을 보는 두 배치는 실제로 거의 같은 방향이다.

    로봇이 사람 동쪽 3 m 에서 y 로 1 mm 만 어긋나면 yaw 는 +179.98 과 -179.98 로
    갈린다. 숫자는 360도 가까이 벌어지지만 **각도 차이는 0.04도**다. 두 yaw 를
    그냥 빼서 임계와 비교하는 코드(goal 갱신 판정 등)는 여기서 터진다.
    """
    above = approach_goal(person=pose(0.0, 0.0), robot=pose(3.0, -0.001))
    below = approach_goal(person=pose(0.0, 0.0), robot=pose(3.0, 0.001))
    assert above.yaw_deg > 179.9
    assert below.yaw_deg < -179.9
    assert angular_diff_deg(above.yaw_deg, below.yaw_deg) < 0.1


def test_due_west_heading_uses_positive_180():
    """정서쪽은 +180 으로 고정한다 - 같은 방향에 두 표기가 생기지 않게.

    로봇이 사람 정동쪽에 있으면 atan2 는 -0.0 을 받아 -180 을 낸다. 그대로 두면
    바로 옆 배치(+179.98)와 부호가 갈려 비교가 흔들린다.
    """
    goal = approach_goal(person=pose(0.0, 0.0), robot=pose(3.0, 0.0))
    assert goal.yaw_deg == 180.0


# ---- 잘못된 입력은 조용히 굴러가지 않는다 ------------------------------------


@pytest.mark.parametrize("bad", [0.0, -0.5])
def test_non_positive_safety_distance_raises(bad):
    """안전거리는 파라미터다. 0 이하는 기동 시점에 죽는 편이 안전하다."""
    with pytest.raises(ValueError):
        approach_goal(person=pose(0.0, 0.0), robot=pose(3.0, 0.0), safety_distance_m=bad)


def test_frame_mismatch_raises():
    """map 과 odom 을 섞으면 엉뚱한 좌표로 간다. 배선 실수는 크게 터뜨린다."""
    with pytest.raises(ValueError):
        approach_goal(person=pose(0.0, 0.0), robot=pose(3.0, 0.0, frame_id="odom"))
