"""사람 접근 — 같은 사람 잇기(B6)·중간 재측정·고정·위치 억제 (mission_logic 시험).

설계: 루트 docs/superpowers/specs/2026-10-10-approach-midcourse-recheck-design.md
run84 E·F: 도중 track 번호가 바뀌어(12→16, 24→26) 가까이서 잰 위치가 busy_approaching 으로
거절됐고, 6 m 추정(0.25~0.45 m 멂) 그대로 서서 "네" 뒤 180° 가 사용자 다리에 막혔다.
"""
import pytest

from vica_mission_manager.mission_logic import (
    ApproachRequest,
    Destination,
    GateReason,
    MapBounds,
    MissionLogic,
    Navigate,
    NavStatus,
    Pose2D,
    State,
)

BOUNDS = MapBounds(min_x=-15.1, min_y=-8.59, max_x=10.0, max_y=8.0)

# 사람은 (0, −4), 로봇은 (0, 2.2) 에서 남쪽(−y)으로 다가온다 — 처음 거리 6.2 m, 재측정 시점 3.1 m.
PERSON = Pose2D(x=0.0, y=-4.0, yaw_deg=0.0, frame_id="map")
GOAL = Pose2D(x=0.0, y=-2.9, yaw_deg=-90.0, frame_id="map")


def P(x, y):
    return Pose2D(x=x, y=y, yaw_deg=0.0, frame_id="map")


def request(track_id=12, person=PERSON, goal=GOAL):
    return ApproachRequest(goal=goal, track_id=track_id, approachable=True, person=person)


def home():
    return Destination(id="standby", name="대기 위치",
                       pose=Pose2D(x=-2.0, y=-1.0, yaw_deg=0.0, frame_id="map"), calibrated=True)


def started(robot=P(0.0, 2.2), **kw):
    logic = MissionLogic(return_destination=home(), **kw)
    logic.robot_pose = robot
    actions, reason = logic.on_approach_request(request(), BOUNDS, True, 0.0)
    assert reason == GateReason.OK and logic.state == State.APPROACHING
    return logic


def recheck(logic, at, person, t=5.0, n=5, conf=0.9):
    """로봇을 at 에 두고 같은 위치 검출 n 개. 마지막 결과(새 사람 위치 또는 None)."""
    logic.robot_pose = at
    out = None
    for i in range(n):
        out = logic.on_approach_detection(person, conf, t + 0.2 * i)
    return out


def navs(actions):
    return [a for a in actions if isinstance(a, Navigate)]


class TestSamePersonByPlace:
    def test_switched_track_at_the_same_place_is_the_same_person(self):
        logic = started()
        _, reason = logic.on_approach_request(
            request(track_id=16, person=P(0.1, -3.8)), BOUNDS, True, 2.0)
        assert reason == GateReason.OK
        assert logic.approach_track_id == 16      # 번호를 새 것으로 갈아 단다

    def test_switched_track_at_another_place_is_still_busy(self):
        logic = started()
        _, reason = logic.on_approach_request(
            request(track_id=16, person=P(0.0, -5.5)), BOUNDS, True, 2.0)
        assert reason == GateReason.BUSY_APPROACHING
        assert logic.approach_track_id == 12

    def test_request_without_person_falls_back_to_track(self):
        logic = started()
        _, reason = logic.on_approach_request(
            request(track_id=16, person=None), BOUNDS, True, 2.0)
        assert reason == GateReason.BUSY_APPROACHING


class TestMidcourseRecheck:
    def test_recheck_moves_the_goal_once(self):
        logic = started()
        new_person = recheck(logic, P(0.0, -0.9), P(0.0, -3.7))
        assert new_person is not None and new_person.y == pytest.approx(-3.7)
        new_goal = Pose2D(x=0.0, y=-2.6, yaw_deg=-90.0, frame_id="map")
        actions, moved = logic.on_approach_recheck_goal(new_person, new_goal, BOUNDS, 6.0)
        assert moved == pytest.approx(0.3)
        [nav] = navs(actions)
        assert nav.destination.pose == new_goal
        assert logic.approach_goal_pose == new_goal
        assert logic.approach_person == new_person

    def test_small_correction_keeps_the_goal(self):
        logic = started()
        new_person = recheck(logic, P(0.0, -0.9), P(0.0, -3.95))
        near_goal = Pose2D(x=0.0, y=-2.85, yaw_deg=-90.0, frame_id="map")
        actions, moved = logic.on_approach_recheck_goal(new_person, near_goal, BOUNDS, 6.0)
        assert moved == pytest.approx(0.05)
        assert navs(actions) == []                 # 12 cm 미만 — 서는 자리가 같다
        assert logic.approach_goal_pose == GOAL
        assert logic.approach_person == new_person  # 기준 자리는 더 정확한 값으로

    def test_goal_outside_the_map_is_not_sent(self):
        logic = started()
        new_person = recheck(logic, P(0.0, -0.9), P(0.0, -3.7))
        actions, _ = logic.on_approach_recheck_goal(
            new_person, Pose2D(x=100.0, y=0.0, yaw_deg=0.0, frame_id="map"), BOUNDS, 6.0)
        assert navs(actions) == []
        assert logic.approach_goal_pose == GOAL

    def test_nothing_before_the_halfway_point(self):
        logic = started()
        assert recheck(logic, P(0.0, 1.0), P(0.0, -3.7)) is None   # 로봇↔사람 5.0 m

    def test_short_approach_is_not_rechecked(self):
        logic = started(robot=P(0.0, -1.2))                        # 처음 거리 2.8 m
        assert recheck(logic, P(0.0, -1.5), P(0.0, -3.7)) is None

    def test_detection_outside_an_approach_is_ignored(self):
        logic = MissionLogic()
        logic.robot_pose = P(0.0, -0.9)
        assert logic.on_approach_detection(P(0.0, -3.7), 0.9, 1.0) is None

    def test_recheck_goal_outside_an_approach_does_nothing(self):
        logic = MissionLogic()
        actions, _ = logic.on_approach_recheck_goal(P(0.0, -3.7), GOAL, BOUNDS, 1.0)
        assert actions == []


class TestFreezeAfterRecheck:
    MOVED = Pose2D(x=0.0, y=-2.3, yaw_deg=-90.0, frame_id="map")   # 0.6 m 옮긴 요청

    def test_half_meter_rule_still_works_before_the_recheck(self):
        logic = started()
        logic.robot_pose = P(0.0, 1.0)                              # 아직 5.0 m
        actions, reason = logic.on_approach_request(
            request(person=P(0.0, -3.4), goal=self.MOVED), BOUNDS, True, 2.0)
        assert reason == GateReason.OK
        assert len(navs(actions)) == 1
        assert logic.approach_person == P(0.0, -3.4)

    def test_goal_is_frozen_after_the_recheck(self):
        logic = started()
        new_person = recheck(logic, P(0.0, -0.9), P(0.0, -3.7))
        logic.on_approach_recheck_goal(
            new_person, Pose2D(x=0.0, y=-2.6, yaw_deg=-90.0, frame_id="map"), BOUNDS, 6.0)
        actions, reason = logic.on_approach_request(
            request(person=P(0.0, -3.4), goal=self.MOVED), BOUNDS, True, 7.0)
        assert reason == GateReason.OK
        assert navs(actions) == []
        assert logic.approach_goal_pose.y == pytest.approx(-2.6)

    def test_goal_is_frozen_after_giving_up(self):
        # 검출이 하나도 안 와도 2 m 안에 들어오면 포기 — 그 뒤 0.5 m 갱신도 막는다.
        logic = started()
        logic.robot_pose = P(0.0, -0.9)
        logic.on_approach_request(request(), BOUNDS, True, 5.0)     # 시점 도달
        logic.robot_pose = P(0.0, -2.05)                            # 사람까지 1.95 m
        actions, reason = logic.on_approach_request(
            request(person=P(0.0, -3.4), goal=self.MOVED), BOUNDS, True, 8.0)
        assert reason == GateReason.OK
        assert navs(actions) == []
        assert logic.approach_goal_pose == GOAL


class TestSuppressionByPlace:
    def _decline_and_return(self, logic):
        logic.on_tick(5.0, NavStatus.SUCCEEDED)                    # 도착 → 질문
        logic.on_approach_answer(False, 6.0)                        # 아니요 → 복귀
        logic.on_tick(10.0, NavStatus.SUCCEEDED)                   # 복귀 완료 → IDLE
        assert logic.state == State.IDLE

    def test_new_track_at_the_same_place_is_suppressed(self):
        logic = started()
        self._decline_and_return(logic)
        logic.robot_pose = P(0.0, 2.2)
        _, reason = logic.on_approach_request(
            request(track_id=30, person=P(0.4, -3.3)), BOUNDS, True, 20.0)
        assert reason == GateReason.TRACK_SUPPRESSED

    def test_new_track_elsewhere_is_free(self):
        logic = started()
        self._decline_and_return(logic)
        _, reason = logic.on_approach_request(
            request(track_id=30, person=P(1.5, -4.0)), BOUNDS, True, 20.0)
        assert reason == GateReason.OK

    def test_place_suppression_ends_after_60_s(self):
        logic = started(reapproach_suppress_sec=60.0)
        self._decline_and_return(logic)
        _, reason = logic.on_approach_request(
            request(track_id=30, person=PERSON), BOUNDS, True, 69.9)
        assert reason == GateReason.TRACK_SUPPRESSED
        _, reason = logic.on_approach_request(
            request(track_id=30, person=PERSON), BOUNDS, True, 70.0)
        assert reason == GateReason.OK

    def test_accepted_person_is_suppressed_by_place(self):
        logic = started()
        logic.on_tick(5.0, NavStatus.SUCCEEDED)
        logic.on_approach_answer(True, 6.0)                         # 네 → 회전
        logic.on_tick(8.0, NavStatus.SUCCEEDED)                    # 회전 끝 → IDLE
        _, reason = logic.on_approach_request(
            request(track_id=31, person=P(0.2, -4.1)), BOUNDS, True, 9.0)
        assert reason == GateReason.TRACK_SUPPRESSED

    def test_estopped_person_is_suppressed_by_place(self):
        logic = started(estop_release_grace_sec=2.0)
        logic.on_estop(True, 3.0)
        logic.on_estop(False, 4.0)
        logic.on_tick(6.0, NavStatus.NONE)
        _, reason = logic.on_approach_request(
            request(track_id=32, person=P(-0.3, -4.2)), BOUNDS, True, 7.0)
        assert reason == GateReason.TRACK_SUPPRESSED
