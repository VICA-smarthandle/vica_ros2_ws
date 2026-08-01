"""접근 감속 사다리 순수 로직 검증 (ROS 없이 실행된다).

이 사다리는 시각장애인 사용자가 잡고 있는 핸들의 속도 낙차를 줄이는 장치다.
잘못 동작하는 방향은 두 가지이며 둘 다 여기서 막는다.

1. 감속이 안 걸린다 — 도착 순간 Δv 가 그대로 사용자에게 전달된다.
2. 제한이 오르내린다 — 재계획으로 잔여거리가 출렁일 때마다 울컥거림이 된다.
"""
import pytest

from vica_mission_manager.approach_speed import (
    DEFAULT_APPROACH_STAGES,
    NO_SPEED_LIMIT,
    ApproachSpeedLadder,
    normalize_stages,
    stages_from_lists,
)


# ---- 기본 단계 ---------------------------------------------------------------


def test_default_stages_descend_in_distance_and_percent():
    """먼 거리부터 나열되고, 가까울수록 비율이 낮아야 한다."""
    distances = [d for d, _ in DEFAULT_APPROACH_STAGES]
    percents = [p for _, p in DEFAULT_APPROACH_STAGES]
    assert distances == sorted(distances, reverse=True)
    assert percents == sorted(percents, reverse=True)


def test_default_stages_match_agreed_table():
    """2026-08-01 사용자 확정 값. 첫 감속은 3.0 m 가 아니라 1.5 m 다."""
    assert DEFAULT_APPROACH_STAGES == ((1.5, 70.0), (1.0, 55.0), (0.5, 40.0))


# ---- 경계값 ------------------------------------------------------------------


def test_no_limit_before_first_stage():
    """1.5 m 직전까지는 100 % 로 달린다 — 미리 줄이면 손해만 크다."""
    ladder = ApproachSpeedLadder()
    assert ladder.update(9.0) is None
    assert ladder.update(1.6) is None
    assert ladder.update(1.5001) is None
    assert ladder.percent == NO_SPEED_LIMIT


@pytest.mark.parametrize(
    "distance,expected",
    [(1.5, 70.0), (1.0, 55.0), (0.5, 40.0)],
)
def test_boundary_distance_enters_stage(distance, expected):
    """경계값은 포함이다. 정확히 1.5/1.0/0.5 m 면 그 단계에 들어간다."""
    ladder = ApproachSpeedLadder()
    assert ladder.update(distance) == expected
    assert ladder.percent == expected


def test_stages_engage_in_order():
    ladder = ApproachSpeedLadder()
    assert ladder.update(2.0) is None
    assert ladder.update(1.4) == 70.0
    assert ladder.update(1.2) is None  # 같은 단계 안에서는 다시 발행하지 않는다
    assert ladder.update(0.9) == 55.0
    assert ladder.update(0.6) is None
    assert ladder.update(0.4) == 40.0
    assert ladder.update(0.1) is None


def test_skipping_stages_jumps_to_deepest():
    """한 tick 에 여러 단계를 건너뛰면 중간 단계를 거치지 않고 가장 깊은 단계로 간다.

    0.3 m 인데 70 % 를 먼저 거는 것은 이미 늦은 감속이다.
    """
    ladder = ApproachSpeedLadder()
    assert ladder.update(0.3) == 40.0
    assert ladder.index == 2


# ---- latch: 되돌아가지 않는다 -------------------------------------------------


def test_limit_holds_when_distance_grows_again():
    """재계획으로 잔여거리가 늘어도 제한은 유지된다."""
    ladder = ApproachSpeedLadder()
    assert ladder.update(1.4) == 70.0
    assert ladder.update(3.0) is None
    assert ladder.update(9.0) is None
    assert ladder.percent == 70.0


def test_backward_motion_does_not_raise_limit():
    """역주행(뒤로 밀림)으로 거리가 계속 늘어도 마지막 단계를 유지한다."""
    ladder = ApproachSpeedLadder()
    ladder.update(0.4)
    for distance in (0.6, 0.9, 1.2, 1.6, 4.0):
        assert ladder.update(distance) is None
    assert ladder.percent == 40.0


def test_reset_starts_over_for_new_goal():
    ladder = ApproachSpeedLadder()
    ladder.update(0.4)
    ladder.reset()

    assert ladder.index == -1
    assert ladder.percent == NO_SPEED_LIMIT
    assert ladder.update(2.0) is None
    assert ladder.update(1.5) == 70.0


# ---- 잔여거리 미수신 ---------------------------------------------------------


def test_missing_distance_changes_nothing():
    """Nav2 는 경로 계산 전 None/0.0 을 준다. 이를 '도착'으로 읽으면 안 된다."""
    ladder = ApproachSpeedLadder()
    assert ladder.update(None) is None
    assert ladder.update(0.0) is None
    assert ladder.update(-1.0) is None
    assert ladder.percent == NO_SPEED_LIMIT
    assert ladder.index == -1


def test_missing_distance_keeps_current_stage():
    ladder = ApproachSpeedLadder()
    ladder.update(0.9)
    assert ladder.update(None) is None
    assert ladder.update(0.0) is None
    assert ladder.percent == 55.0


# ---- 단계 목록 검증 -----------------------------------------------------------


def test_stages_are_sorted_by_distance():
    ladder = ApproachSpeedLadder([(0.5, 40.0), (1.5, 70.0), (1.0, 55.0)])
    assert ladder.stages == ((1.5, 70.0), (1.0, 55.0), (0.5, 40.0))


def test_empty_stages_disable_slowdown():
    """빈 목록은 접근 감속을 끄는 스위치다."""
    ladder = ApproachSpeedLadder([])
    assert ladder.stages == ()
    assert ladder.update(0.01) is None
    assert ladder.percent == NO_SPEED_LIMIT


def test_single_stage_reproduces_old_single_latch():
    """이 변경 전 동작(3.0 m 에서 70 % 한 번)은 단계 하나로 그대로 표현된다."""
    ladder = ApproachSpeedLadder([(3.0, 70.0)])
    assert ladder.update(3.0) == 70.0
    assert ladder.update(0.1) is None


@pytest.mark.parametrize("distance", [0.0, -1.0])
def test_rejects_non_positive_distance(distance):
    with pytest.raises(ValueError):
        ApproachSpeedLadder([(distance, 70.0)])


@pytest.mark.parametrize("percent", [0.0, -10.0, 100.1])
def test_rejects_percent_out_of_range(percent):
    with pytest.raises(ValueError):
        ApproachSpeedLadder([(1.5, percent)])


def test_rejects_duplicate_distance():
    with pytest.raises(ValueError):
        ApproachSpeedLadder([(1.5, 70.0), (1.5, 55.0)])


def test_rejects_percent_rising_as_robot_approaches():
    """가까울수록 빨라지는 설정은 오타다. 실주행 전에 죽는다."""
    with pytest.raises(ValueError):
        ApproachSpeedLadder([(1.5, 40.0), (0.5, 70.0)])


def test_rejects_malformed_stage():
    with pytest.raises(ValueError):
        normalize_stages([(1.5, 70.0, 3.0)])


# ---- ROS parameter 두 배열 합치기 --------------------------------------------


def test_stages_from_lists_pairs_by_index():
    assert stages_from_lists([1.5, 1.0, 0.5], [70.0, 55.0, 40.0]) == (
        (1.5, 70.0),
        (1.0, 55.0),
        (0.5, 40.0),
    )


def test_stages_from_lists_accepts_empty():
    assert stages_from_lists([], []) == ()


def test_stages_from_lists_rejects_length_mismatch():
    """짝이 어긋나면 어느 거리에 어느 비율인지 알 수 없다."""
    with pytest.raises(ValueError):
        stages_from_lists([1.5, 1.0, 0.5], [70.0, 55.0])
