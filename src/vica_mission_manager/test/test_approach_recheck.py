"""사람 접근 중간 재측정·같은 사람 잇기 — 순수 로직 시험.

설계: docs/superpowers/specs/2026-10-10-approach-midcourse-recheck-design.md (루트 저장소).
run84(10-10): 6 m 추정이 다리보다 0.25~0.45 m 멀었고 도중 track 번호가 바뀌어 고쳐지지 않았다.
"""
import math

import pytest

from vica_mission_manager.approach_recheck import (
    MidcourseRecheck,
    RecheckPhase,
    same_person,
)
from vica_mission_manager.mission_logic import Pose2D


def P(x, y, frame="map"):
    return Pose2D(x=x, y=y, yaw_deg=0.0, frame_id=frame)


# 사람은 원점, 로봇은 +x 축 위에서 사람 쪽(-x)으로 다가온다.
PERSON = P(0.0, 0.0)


def robot_at(d):
    return P(d, 0.0)


class TestSamePerson:
    def test_inside_radius_is_same(self):
        assert same_person(P(0.0, 0.0), P(0.5, 0.6), 0.8)

    def test_outside_radius_is_other(self):
        # run84 E: 뒤쪽 오인식 상자는 1.2~1.8 m 떨어져 있었다.
        assert not same_person(P(0.0, 0.0), P(1.2, 0.0), 0.8)

    def test_missing_or_nan_is_never_same(self):
        assert not same_person(None, P(0.0, 0.0), 0.8)
        assert not same_person(P(math.nan, 0.0), P(0.0, 0.0), 0.8)

    def test_other_frame_is_never_same(self):
        assert not same_person(P(0.0, 0.0), P(0.0, 0.0, frame="odom"), 0.8)


class TestStart:
    @pytest.mark.parametrize("d0, trigger", [(6.2, 3.1), (8.0, 3.5), (3.0, 2.5), (4.4, 2.5)])
    def test_trigger_is_half_kept_between_2_5_and_3_5(self, d0, trigger):
        # 절반이 포기선(2.0) 근처면 모을 틈이 없다 — 2.5 m 아래로는 내리지 않는다.
        r = MidcourseRecheck()
        r.start(PERSON, robot_at(d0))
        assert r.phase == RecheckPhase.WAITING
        assert r.trigger_m == pytest.approx(trigger)

    def test_short_approach_is_skipped(self):
        # B(2.8 m): 이미 오차 +0.06 — 다시 잴 필요가 없다.
        r = MidcourseRecheck()
        r.start(PERSON, robot_at(2.8))
        assert r.phase == RecheckPhase.SKIPPED
        assert r.observe(robot_at(1.5), P(0.1, 0.0), 0.9) is None

    def test_unknown_robot_pose_is_skipped(self):
        r = MidcourseRecheck()
        r.start(PERSON, None)
        assert r.phase == RecheckPhase.SKIPPED


def sample(r, d, det=P(-0.3, 0.0), conf=0.9):
    return r.observe(robot_at(d), det, conf)


class TestObserve:
    def started(self, d0=6.2):
        r = MidcourseRecheck()
        r.start(PERSON, robot_at(d0))
        return r

    def test_nothing_before_the_trigger(self):
        r = self.started()
        for _ in range(10):
            assert sample(r, 4.0) is None
        assert r.phase == RecheckPhase.WAITING
        assert r.sample_count == 0

    def test_five_samples_give_the_median(self):
        r = self.started()
        xs = [-0.30, -0.25, -0.35, -0.28, -0.32]
        out = None
        for i, x in enumerate(xs):
            out = sample(r, 3.0 - 0.1 * i, P(x, 0.02))
        assert r.phase == RecheckPhase.DONE
        assert out.x == pytest.approx(-0.30)
        assert out.y == pytest.approx(0.02)
        assert out.frame_id == "map"

    def test_one_overlapping_box_does_not_pull_the_median(self):
        # run84 E: 같은 사람에게 겹친 상자 하나(0.2 m 차이)가 섞였다.
        r = self.started()
        for x in (-0.30, -0.31, 0.25, -0.29, -0.30):
            out = sample(r, 3.0, P(x, 0.0))
        assert out.x == pytest.approx(-0.30)

    def test_outside_radius_low_confidence_and_nan_are_dropped(self):
        r = self.started()
        sample(r, 3.0, P(1.5, 0.0))            # 반경 0.8 밖(뒤쪽 오인식)
        sample(r, 3.0, P(-0.3, 0.0), 0.55)     # 신뢰도 0.6 미만
        sample(r, 3.0, P(math.nan, 0.0))        # 깊이 구멍
        assert r.sample_count == 0
        assert r.phase == RecheckPhase.COLLECTING

    def test_gives_up_inside_2_m(self):
        r = self.started()
        sample(r, 3.0)
        sample(r, 2.5)
        assert sample(r, 1.9) is None
        assert r.phase == RecheckPhase.GAVE_UP
        assert sample(r, 1.8) is None            # 포기 뒤로는 아무것도 안 낸다

    def test_result_comes_only_once(self):
        r = self.started()
        outs = [sample(r, 3.0) for _ in range(8)]
        assert sum(o is not None for o in outs) == 1
        assert r.finished

    def test_update_robot_alone_can_give_up(self):
        # 검출이 하나도 안 와도 2 m 안에 들어오면 포기로 끝난다(0.5 m 규칙 고정 판단용).
        r = self.started()
        r.update_robot(robot_at(3.0))
        assert r.phase == RecheckPhase.COLLECTING
        r.update_robot(robot_at(1.9))
        assert r.phase == RecheckPhase.GAVE_UP
        assert r.finished

    def test_moved_person_moves_the_gate(self):
        # 재측정 전 0.5 m 갱신(사람이 실제로 옮겨 섬)이 오면 반경도 따라간다.
        r = self.started()
        r.move_person(P(-1.0, 0.0))
        sample(r, 2.0, P(-1.2, 0.0))              # 로봇 2.0 = 새 자리에서 3.0 m(시점 3.1 안)
        assert r.sample_count == 1
        sample(r, 2.0, P(0.2, 0.0))              # 옛 자리 근처지만 새 자리에서 1.2 m
        assert r.sample_count == 1

    def test_distance_is_measured_to_the_person(self):
        # 시점은 로봇↔사람 거리로 잰다. 사람이 (−1, 0) 으로 옮겨 서면 로봇 3.0 은 4.0 m 다.
        r = self.started()
        r.move_person(P(-1.0, 0.0))
        sample(r, 3.0, P(-1.0, 0.0))
        assert r.phase == RecheckPhase.WAITING
