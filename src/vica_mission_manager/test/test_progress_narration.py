"""주행 중 남은 거리 안내 검증 (순수 로직).

눈으로 확인할 수 없는 사용자는 도착이 임박했는지 알 방법이 없어, 접근 시점을
음성으로 짚어 준다. 자주 말하면 오히려 방해가 되므로 지점마다 한 번만 말한다.
"""
import pytest

from vica_mission_manager.mission_logic import (
    DISTANCE_MILESTONES_M,
    MSG_DISTANCE_REMAINING,
    Destination,
    IntentData,
    MapBounds,
    MissionLogic,
    NavStatus,
    Pose2D,
    Say,
    SetNavSpeedLimit,
    State,
)


def _destination() -> Destination:
    return Destination(
        id="cafeteria",
        name="식당",
        is_approachable=True,
        pose=Pose2D(x=1.0, y=1.0, yaw_deg=0.0),
        confirm_prompt="",
        arrival_message="",
        unavailable_reason="",
    )


def _bounds() -> MapBounds:
    return MapBounds(min_x=-10.0, max_x=10.0, min_y=-10.0, max_y=10.0)


@pytest.fixture
def navigating() -> MissionLogic:
    """주행 중 상태의 MissionLogic."""
    logic = MissionLogic()
    intent = IntentData(
        intent="navigate",
        matched_destination_id="cafeteria",
        need_confirm=False,
        safety_flag="normal",
    )
    logic.on_intent(intent, _destination(), _bounds(), True, now=0.0)
    assert logic.state == State.NAVIGATING
    return logic


def _spoken(actions) -> list:
    return [a.text for a in actions if isinstance(a, Say)]


def _speed_limits(actions) -> list:
    return [
        action.percent
        for action in actions
        if isinstance(action, SetNavSpeedLimit)
    ]


def test_limits_speed_once_when_entering_three_meter_approach_zone(navigating):
    assert _speed_limits(
        navigating.on_tick(1.0, NavStatus.RUNNING, 3.1)
    ) == []
    assert _speed_limits(
        navigating.on_tick(2.0, NavStatus.RUNNING, 3.0)
    ) == [70.0]
    assert _speed_limits(
        navigating.on_tick(3.0, NavStatus.RUNNING, 2.0)
    ) == []
    # 재계획으로 남은 거리가 늘어도 같은 Goal에서는 제한을 유지한다.
    assert _speed_limits(
        navigating.on_tick(4.0, NavStatus.RUNNING, 3.5)
    ) == []


@pytest.mark.parametrize("status", [NavStatus.SUCCEEDED, NavStatus.FAILED])
def test_clears_approach_speed_limit_when_navigation_finishes(
    navigating,
    status,
):
    navigating.on_tick(1.0, NavStatus.RUNNING, 2.5)

    assert _speed_limits(navigating.on_tick(2.0, status)) == [0.0]


def test_clears_approach_speed_limit_when_estop_cancels_goal(navigating):
    navigating.on_tick(1.0, NavStatus.RUNNING, 2.5)

    assert _speed_limits(navigating.on_estop(True, 2.0)) == [0.0]


def test_announces_when_crossing_milestone(navigating):
    assert _spoken(navigating.on_tick(1.0, NavStatus.RUNNING, 12.0)) == []
    assert _spoken(navigating.on_tick(2.0, NavStatus.RUNNING, 9.5)) == [
        MSG_DISTANCE_REMAINING.format(meters=10)
    ]


def test_each_milestone_announced_once(navigating):
    navigating.on_tick(1.0, NavStatus.RUNNING, 9.0)
    assert _spoken(navigating.on_tick(2.0, NavStatus.RUNNING, 8.0)) == []
    assert _spoken(navigating.on_tick(3.0, NavStatus.RUNNING, 7.0)) == []


def test_announces_next_milestone(navigating):
    navigating.on_tick(1.0, NavStatus.RUNNING, 9.0)
    assert _spoken(navigating.on_tick(2.0, NavStatus.RUNNING, 2.5)) == [
        MSG_DISTANCE_REMAINING.format(meters=3)
    ]


def test_short_trip_announces_nearest_milestone(navigating):
    """2m 앞에서 출발했는데 "10미터 남았다"고 하면 안 된다."""
    assert _spoken(navigating.on_tick(1.0, NavStatus.RUNNING, 2.0)) == [
        MSG_DISTANCE_REMAINING.format(meters=3)
    ]
    # 남은 지점도 지난 것으로 처리돼 중복 안내가 없어야 한다.
    assert _spoken(navigating.on_tick(2.0, NavStatus.RUNNING, 1.0)) == []


def test_ignores_missing_or_zero_distance(navigating):
    """Nav2 는 경로 계산 전에 None/0.0 을 주기도 한다."""
    assert _spoken(navigating.on_tick(1.0, NavStatus.RUNNING, None)) == []
    assert _spoken(navigating.on_tick(2.0, NavStatus.RUNNING, 0.0)) == []
    assert _spoken(navigating.on_tick(3.0, NavStatus.RUNNING, -1.0)) == []


def test_new_goal_resets_milestones(navigating):
    navigating.on_tick(1.0, NavStatus.RUNNING, 2.0)  # 두 지점 모두 소진
    navigating.on_tick(2.0, NavStatus.SUCCEEDED)
    navigating.on_tick(10.0, NavStatus.NONE)  # dwell 경과 -> idle

    intent = IntentData(
        intent="navigate",
        matched_destination_id="cafeteria",
        need_confirm=False,
        safety_flag="normal",
    )
    navigating.on_intent(intent, _destination(), _bounds(), True, now=11.0)

    assert _spoken(navigating.on_tick(12.0, NavStatus.RUNNING, 9.0)) == [
        MSG_DISTANCE_REMAINING.format(meters=10)
    ]


def test_distance_is_narration_priority(navigating):
    actions = navigating.on_tick(1.0, NavStatus.RUNNING, 9.0)
    says = [a for a in actions if isinstance(a, Say)]
    assert says and all(a.priority == "narration" for a in says)


def test_no_announcement_when_not_navigating():
    """대기 중에는 거리 안내가 나오면 안 된다."""
    logic = MissionLogic()
    assert _spoken(logic.on_tick(1.0, NavStatus.RUNNING, 5.0)) == []


def test_milestones_are_descending():
    """가까운 지점이 뒤에 오도록 유지한다 (min() 선택 근거)."""
    assert list(DISTANCE_MILESTONES_M) == sorted(DISTANCE_MILESTONES_M, reverse=True)
