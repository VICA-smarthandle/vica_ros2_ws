"""obstacle_judge — 주행 중 장애물 안내 판정 시험 (음성 저장소 tests/test_obstacle_judge.py 를 2026-10-09 옮김)."""
import json
import math

import numpy as np
import pytest

from vica_mission_manager import obstacle_judge as oj


def empty_grid(walls=()):
    """-5~5 m, 0.05 m 칸. walls 는 점유로 둘 (x, y) 지도 좌표."""
    res, ox, oy, n = 0.05, -5.0, -5.0, 200
    occ = np.zeros((n, n), dtype=bool)
    for x, y in walls:
        j = int(math.floor((x - ox) / res))
        i = n - 1 - int(math.floor((y - oy) / res))
        occ[i, j] = True
    return oj.MapGrid(occ, res, ox, oy)


def vcc(state="TRACK", target=0.0, offset=0.0, v=0.5, reason="track"):
    return {"state": state, "reason": reason, "target": target, "offset": offset, "v": v, "w": 0.0}


def make(walls=(), dialog="navigating", goal=(9.0, 0.0)):
    j = oj.ObstacleJudge(empty_grid(walls))
    j.on_dialog(0.0, dialog)
    if goal is not None:
        j.on_goal(0.0, "goal_sent", goal[0], goal[1], "dest-1")
    j.on_rail(0.0, [(-1.0, 0.0), (4.9, 0.0)])
    return j


def obstacle(x, y=0.0, n=5, spread=0.04):
    ys = np.linspace(y - spread, y + spread, n)
    return np.full(n, x), ys


class Clock:
    """10 Hz 로 VCC·자세·라이다를 함께 넣는다. points(t) 가 그 순간 라이다 점(로봇 기준)을 돌려준다."""

    def __init__(self, judge):
        self.j = judge
        self.t = 0.0
        self.out = []

    def run(self, sec, st, points=None, depth=None, pose=(0.0, 0.0, 0.0)):
        steps = int(round(sec / 0.1))
        for _ in range(steps):
            self.t = round(self.t + 0.1, 3)
            self.j.on_pose(self.t, *pose)
            px = points(self.t) if callable(points) else points
            self.j.on_points("scan", self.t, *(px if px is not None else (np.zeros(0), np.zeros(0))))
            if depth is not None:
                self.j.on_points("depth", self.t, *depth)
            if st is not None:
                self.j.on_vcc(self.t, dict(st))
            self.out += self.j.tick(self.t)
        return self

    def flush(self, sec=1.0):
        return self.run(sec, None)

    def decisions(self, kind=None):
        return [d for d in self.out if kind is None or d["kind"] == kind]

    def said(self):
        return [d["phrase"] for d in self.out if d["decision"] == "announce"]


class TestParse:
    def test_state_line(self):
        st = oj.parse_state("state=TRACK reason=track offset=0.12 target=0.30 blocked=0 turn=0 align=0 "
                            "fail=0 v=0.420 w=0.010 us_fresh=0 rot=0 ext=0 defer=0")
        assert st["state"] == "TRACK" and st["target"] == pytest.approx(0.3) and st["v"] == pytest.approx(0.42)

    def test_bad_line(self):
        assert oj.parse_state("hello") is None
        assert oj.parse_state("state=TRACK target=abc") is None

    def test_cm_line(self):
        assert oj.cm_what("Robot to stop due to PolygonStop polygon") == "stop"
        assert oj.cm_what("Robot to slowdown for 40.000000 percents due to PolygonSlow polygon") == "slow"
        assert oj.cm_what("Robot to continue normal operation") == "normal"
        assert oj.cm_what("Collision monitor activated") is None

    def test_goal_event(self):
        ev = oj.goal_event(json.dumps({"event": "goal_sent", "x": 1.5, "y": -2.0, "location_id": "abc", "map_id": "m1"}))
        assert ev == {"event": "goal_sent", "x": 1.5, "y": -2.0, "loc": "abc", "map_id": "m1"}
        assert oj.goal_event("not json") is None


class TestGeometry:
    def test_route_frame_straight(self):
        s, d = oj.route_frame(np.array([[0.0, 0.0], [4.0, 0.0]]), np.array([1.0, 2.0]), np.array([0.2, -0.3]))
        assert s == pytest.approx([1.0, 2.0]) and d == pytest.approx([0.2, -0.3])

    def test_route_frame_extends_first_segment_backward(self):
        s, d = oj.route_frame(np.array([[0.5, 0.0], [4.0, 0.0]]), np.array([0.0]), np.array([0.0]))
        assert s[0] == pytest.approx(-0.5) and d[0] == pytest.approx(0.0)

    def test_route_frame_corner(self):
        poly = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0]])
        s, d = oj.route_frame(poly, np.array([2.1]), np.array([1.0]))
        assert s[0] == pytest.approx(3.0) and abs(d[0]) == pytest.approx(0.1)

    def test_wall_distance(self):
        g = empty_grid(walls=[(1.0, 0.0)])
        near, far = g.wall_dist(np.array([1.05, 2.0]), np.array([0.05, 0.0]))
        assert near < oj.MAP_TOL < far

    def test_ros_grid_rows_are_flipped(self):
        w = h = 4
        data = [0] * (w * h)
        data[0] = 100
        g = oj.MapGrid.from_occupancy(data, w, h, 1.0, 0.0, 0.0)
        assert g.wall_dist(np.array([0.5]), np.array([0.5]))[0] == 0.0
        assert g.wall_dist(np.array([0.5]), np.array([3.5]))[0] > 0.0


class TestLaneShift:
    def test_obstacle_on_rail_says_avoid(self):
        c = Clock(make())
        c.run(1.0, vcc(), points=obstacle(1.5))
        c.run(1.0, vcc(target=0.3), points=obstacle(1.5))
        c.flush()
        lat = c.decisions("LAT")
        assert [d["decision"] for d in lat] == ["announce"]
        assert lat[0]["phrase"] == "avoid" and lat[0]["n"] >= 3 and lat[0]["near"] == pytest.approx(1.5, abs=0.01)
        assert c.said() == ["avoid"]

    def test_decides_fast_when_obstacle_already_seen(self):
        c = Clock(make())
        c.run(1.0, vcc(), points=obstacle(1.5))
        c.run(1.0, vcc(target=0.3), points=obstacle(1.5))
        d = c.decisions("LAT")[0]
        assert d["decided_at"] - d["t"] <= oj.WAIT + 0.11

    def test_no_points_no_words(self):
        c = Clock(make())
        c.run(1.0, vcc())
        c.run(1.0, vcc(target=0.3))
        c.flush()
        assert [d["decision"] for d in c.decisions("LAT")] == ["no_cause"]
        assert c.said() == []

    def test_two_points_are_not_enough(self):
        c = Clock(make())
        c.run(1.0, vcc(), points=obstacle(1.5, n=2))
        c.run(1.0, vcc(target=0.3), points=obstacle(1.5, n=2))
        c.flush()
        assert c.said() == []

    def test_obstacle_beside_the_path_is_not_a_cause(self):
        c = Clock(make())
        c.run(1.0, vcc(), points=obstacle(1.5, y=0.5))
        c.run(1.0, vcc(target=0.3), points=obstacle(1.5, y=0.5))
        c.flush()
        assert c.decisions("LAT")[0]["decision"] == "no_cause"

    def test_points_behind_the_robot_are_ignored(self):
        j = make()
        j.on_rail(0.0, [(2.0, 0.0), (0.0, 0.0), (-3.0, 0.0)])
        c = Clock(j)
        c.run(1.0, vcc(), points=obstacle(-0.8))
        c.run(1.0, vcc(target=0.3), points=obstacle(-0.8))
        c.flush()
        assert c.said() == []

    def test_wall_on_the_map_is_not_a_cause(self):
        walls = [(1.5, y) for y in np.arange(-0.1, 0.11, 0.05)]
        c = Clock(make(walls=walls))
        c.run(1.0, vcc(), points=obstacle(1.5))
        c.run(1.0, vcc(target=0.3), points=obstacle(1.5))
        c.flush()
        d = c.decisions("LAT")[0]
        assert d["decision"] == "no_cause" and d["n_wall"] > 0

    def test_too_far_for_lane_shift(self):
        c = Clock(make())
        c.run(1.0, vcc(), points=obstacle(2.6))
        c.run(1.0, vcc(target=0.3), points=obstacle(2.6))
        c.flush()
        assert c.said() == []

    def test_obstacle_appearing_within_half_second_counts(self):
        c = Clock(make())
        c.run(1.0, vcc())
        start = c.t
        c.run(1.0, vcc(target=0.3), points=lambda t: obstacle(1.5) if t >= start + 0.45 else None)
        c.flush()
        assert c.said() == ["avoid"]

    def test_obstacle_appearing_later_is_too_late(self):
        c = Clock(make())
        c.run(1.0, vcc())
        start = c.t
        c.run(1.5, vcc(target=0.3), points=lambda t: obstacle(1.5) if t >= start + 0.95 else None)
        c.flush()
        assert c.said() == []

    def test_depth_band_points_count(self):
        c = Clock(make())
        c.run(1.0, vcc(), depth=obstacle(1.0))
        c.run(1.0, vcc(target=0.3), depth=obstacle(1.0))
        c.flush()
        d = c.decisions("LAT")[0]
        assert d["decision"] == "announce" and d["n_depth"] >= 3 and d["n_scan"] == 0

    def test_resync_after_turn_is_excluded(self):
        c = Clock(make())
        c.run(1.0, vcc(state="TURN", v=0.0))
        c.run(1.0, vcc(target=0.3, offset=0.3), points=obstacle(1.5))
        c.flush()
        d = c.decisions("LAT")[0]
        assert d["decision"] == "excluded" and "resync" in d["why"] and "after_turn" in d["why"]
        assert c.said() == []

    def test_lane_shift_soon_after_turn_is_excluded(self):
        c = Clock(make())
        c.run(1.0, vcc(state="TURN", v=0.0))
        c.run(1.5, vcc())
        c.run(1.0, vcc(target=0.3), points=obstacle(1.5))
        c.flush()
        assert c.decisions("LAT")[0]["why"] == ["after_turn"]

    def test_straight_corridor_without_rail(self):
        j = oj.ObstacleJudge(empty_grid())
        j.on_dialog(0.0, "navigating")
        c = Clock(j)
        c.run(1.0, vcc(), points=obstacle(1.5))
        c.run(1.0, vcc(target=0.3), points=obstacle(1.5))
        c.flush()
        d = c.decisions("LAT")[0]
        assert d["mode"] == "straight" and d["decision"] == "announce"


class TestSlowAndDetour:
    def test_sudden_slowdown_says_slow(self):
        c = Clock(make())
        c.run(1.0, vcc(v=0.5), points=obstacle(1.2))
        c.run(1.0, vcc(v=0.2), points=obstacle(1.2))
        c.flush()
        dec = c.decisions("DEC")
        assert len(dec) == 1 and dec[0]["decision"] == "announce" and dec[0]["phrase"] == "slow"

    def test_gradual_slowdown_is_not_sudden(self):
        c = Clock(make())
        c.run(1.0, vcc(v=0.5))
        for v in np.linspace(0.5, 0.2, 31):
            c.run(0.1, vcc(v=float(v)), points=obstacle(1.2))
        c.flush()
        assert c.decisions("DEC") == []

    def test_slowdown_from_lane_shift_is_excluded(self):
        c = Clock(make())
        c.run(1.0, vcc(v=0.5, target=0.3, offset=0.0))
        c.run(1.0, vcc(v=0.2, target=0.3, offset=0.1), points=obstacle(1.2))
        c.flush()
        assert "dec_from_lane_shift" in c.decisions("DEC")[0]["why"]

    def test_obstacle_hold_says_stop(self):
        c = Clock(make())
        c.run(1.0, vcc(v=0.2), points=obstacle(0.9))
        c.run(1.0, vcc(state="HOLD", reason="lanes_blocked", v=0.0), points=obstacle(0.9))
        c.flush()
        hold = c.decisions("HOLD")
        assert len(hold) == 1 and hold[0]["phrase"] == "stop"
        assert c.said() == ["stop"]

    def test_turn_blocked_hold_is_not_an_obstacle_stop(self):
        c = Clock(make())
        c.run(1.0, vcc(v=0.2))
        c.run(1.0, vcc(state="HOLD", reason="turn_blocked", v=0.0), points=obstacle(0.9))
        c.flush()
        assert c.decisions("HOLD") == []

    def test_collision_monitor_while_driving(self):
        c = Clock(make())
        c.run(1.0, vcc(v=0.3), points=obstacle(0.8))
        c.j.on_cm(c.t + 0.05, "slow")
        c.run(1.0, vcc(v=0.3), points=obstacle(0.8))
        c.flush()
        cm = c.decisions("CM")
        assert len(cm) == 1 and cm[0]["decision"] == "announce" and cm[0]["phrase"] == "slow"

    def test_collision_monitor_stop_says_stop(self):
        c = Clock(make())
        c.run(1.0, vcc(v=0.3), points=obstacle(0.4))
        c.j.on_cm(c.t + 0.05, "stop")
        c.run(1.0, vcc(v=0.3), points=obstacle(0.4))
        c.flush()
        cm = c.decisions("CM")
        assert len(cm) == 1 and cm[0]["decision"] == "announce" and cm[0]["phrase"] == "stop"

    def test_collision_monitor_slow_then_stop_is_one_stop(self):
        c = Clock(make())
        c.run(1.0, vcc(v=0.3), points=obstacle(0.6))
        c.j.on_cm(c.t + 0.05, "slow")
        c.run(0.4, vcc(v=0.3), points=obstacle(0.4))
        c.j.on_cm(c.t + 0.05, "stop")
        c.run(1.0, vcc(v=0.3), points=obstacle(0.4))
        c.flush()
        assert [d["decision"] for d in c.decisions("CM")] == ["announce", "merged"]
        assert c.said() == ["stop"]

    def test_slow_then_late_stop_keeps_slow(self):
        c = Clock(make())
        c.run(1.0, vcc(v=0.3), points=obstacle(0.6))
        c.j.on_cm(c.t + 0.05, "slow")
        c.run(1.0, vcc(v=0.3), points=obstacle(0.4))
        c.j.on_cm(c.t + 0.05, "stop")
        c.run(1.0, vcc(v=0.3), points=obstacle(0.4))
        c.flush()
        assert [d["decision"] for d in c.decisions("CM")] == ["announce", "merged"]
        assert c.said() == ["slow"]

    def test_sudden_slowdown_then_vcc_hold_is_one_stop(self):
        c = Clock(make())
        c.run(1.0, vcc(v=0.5), points=obstacle(0.9))
        c.run(0.3, vcc(v=0.2), points=obstacle(0.9))
        c.run(1.0, vcc(state="HOLD", reason="collision_imminent", v=0.0), points=obstacle(0.9))
        c.flush()
        assert c.said() == ["stop"]

    def test_hold_at_the_same_moment_as_slowdown_says_stop(self):
        c = Clock(make())
        c.run(1.0, vcc(v=0.5), points=obstacle(0.9))
        c.run(1.0, vcc(state="HOLD", reason="collision_imminent", v=0.0), points=obstacle(0.9))
        c.flush()
        assert c.decisions("DEC")[0]["t"] == c.decisions("HOLD")[0]["t"]
        assert c.said() == ["stop"]

    def test_slow_waits_for_the_upgrade_window(self):
        c = Clock(make())
        c.run(1.0, vcc(v=0.3), points=obstacle(0.8))
        c.j.on_cm(c.t + 0.05, "slow")
        c.run(1.0, vcc(v=0.3), points=obstacle(0.8))
        d = c.decisions("CM")[0]
        assert d["phrase"] == "slow" and d["decided_at"] - d["t"] >= oj.STOP_UPGRADE - 1e-9

    def test_excluded_stop_does_not_upgrade(self):
        c = Clock(make())
        c.run(1.0, vcc(v=0.5), points=obstacle(0.9))
        c.run(0.2, vcc(v=0.2), points=obstacle(0.9))
        c.run(0.2, vcc(state="TURN", v=0.05), points=obstacle(0.9))
        c.j.on_cm(c.t + 0.05, "stop")
        c.run(1.0, vcc(state="TURN", v=0.05), points=obstacle(0.9))
        c.flush()
        assert c.said() == ["slow"]
        assert "cm_not_driving" in c.decisions("CM")[0]["why"]

    def test_collision_monitor_while_turning_is_excluded(self):
        c = Clock(make())
        c.run(1.0, vcc(state="TURN", v=0.05), points=obstacle(0.8))
        c.j.on_cm(c.t + 0.05, "slow")
        c.run(1.0, vcc(state="TURN", v=0.05), points=obstacle(0.8))
        c.flush()
        assert "cm_not_driving" in c.decisions("CM")[0]["why"]

    def test_detour_sees_further_and_merges_within_8s(self):
        c = Clock(make())
        c.run(1.0, vcc(), points=obstacle(2.8))
        c.j.on_bt(c.t + 0.05, "IsRailAheadClear", "FAILURE")
        c.run(1.0, vcc(), points=obstacle(2.8))
        c.j.on_bt(c.t, "IsRailAheadClear", "FAILURE")
        c.flush()
        det = c.decisions("DET")
        assert len(det) == 1 and det[0]["decision"] == "announce" and det[0]["phrase"] == "avoid"

    @pytest.mark.parametrize("st", [vcc(state="HOLD", reason="turn_blocked", v=0.0), vcc(v=0.05)])
    def test_detour_while_standing_is_not_said(self, st):
        c = Clock(make())
        c.run(1.0, st, points=obstacle(0.8))
        c.j.on_bt(c.t + 0.05, "IsRailAheadClear", "FAILURE")
        c.run(1.0, st, points=obstacle(0.8))
        c.flush()
        det = c.decisions("DET")
        assert det[0]["why"] == ["det_not_driving"] and c.said() == []

    def test_other_bt_nodes_are_ignored(self):
        c = Clock(make())
        c.run(1.0, vcc(), points=obstacle(1.0))
        c.j.on_bt(c.t, "IsPathValid", "FAILURE")
        c.j.on_bt(c.t, "IsRailAheadClear", "SUCCESS")
        c.flush()
        assert c.decisions("DET") == []


class TestWhenAndOnce:
    def test_not_while_returning_home(self):
        c = Clock(make(dialog="returning"))
        c.run(1.0, vcc(), points=obstacle(1.5))
        c.run(1.0, vcc(target=0.3), points=obstacle(1.5))
        c.flush()
        assert c.decisions("LAT")[0]["why"] == ["not_guided"]

    def test_not_near_the_goal(self):
        c = Clock(make(goal=(0.8, 0.0)))
        c.run(1.0, vcc(), points=obstacle(1.5))
        c.run(1.0, vcc(target=0.3), points=obstacle(1.5))
        c.flush()
        assert "near_goal" in c.decisions("LAT")[0]["why"]

    def test_stop_near_the_goal_is_said(self):
        c = Clock(make(goal=(0.8, 0.0)))
        c.run(1.0, vcc(v=0.2), points=obstacle(0.5))
        c.run(1.0, vcc(state="HOLD", reason="collision_imminent", v=0.0), points=obstacle(0.5))
        c.flush()
        hold = c.decisions("HOLD")
        assert hold[0]["why"] == [] and c.said() == ["stop"]

    def test_collision_monitor_stop_near_the_goal_is_said_but_slow_is_not(self):
        c = Clock(make(goal=(0.8, 0.0)))
        c.run(1.0, vcc(v=0.3), points=obstacle(0.4))
        c.j.on_cm(c.t + 0.05, "slow")
        c.run(1.0, vcc(v=0.3), points=obstacle(0.4))
        c.j.on_cm(c.t + 0.05, "normal")
        c.run(8.0, vcc(v=0.3))
        c.j.on_cm(c.t + 0.05, "stop")
        c.run(1.0, vcc(v=0.3), points=obstacle(0.4))
        c.flush()
        cm = c.decisions("CM")
        assert "near_goal" in cm[0]["why"] and cm[1]["why"] == []
        assert c.said() == ["stop"]

    def test_no_map_no_words(self):
        j = oj.ObstacleJudge(None)
        j.on_dialog(0.0, "navigating")
        c = Clock(j)
        c.run(1.0, vcc(), points=obstacle(1.5))
        c.run(1.0, vcc(target=0.3), points=obstacle(1.5))
        c.flush()
        assert c.decisions("LAT")[0]["decision"] == "no_map"

    def test_same_obstacle_is_said_once(self):
        c = Clock(make())
        c.run(1.0, vcc(), points=obstacle(1.2))
        c.run(1.0, vcc(target=0.3), points=obstacle(1.2))
        c.run(1.0, vcc(state="HOLD", reason="lanes_blocked", target=0.3, v=0.0), points=obstacle(1.0))
        c.run(1.0, vcc(target=-0.4), points=obstacle(1.0))
        c.flush()
        assert c.said() == ["avoid"]
        assert [d["decision"] for d in c.out if d["kind"] in ("LAT", "HOLD")].count("merged") >= 1

    def test_again_only_after_calm_driving_and_6s(self):
        c = Clock(make())
        c.run(1.0, vcc(), points=obstacle(1.2))
        c.run(1.0, vcc(target=0.3), points=obstacle(1.2))
        c.run(7.0, vcc(target=0.3))
        c.run(0.5, vcc(target=0.0, v=0.2))
        c.run(1.0, vcc(target=-0.3, v=0.2), points=obstacle(1.2))
        c.run(2.5, vcc())
        c.run(1.0, vcc(target=0.3), points=obstacle(1.2))
        c.flush()
        assert c.said() == ["avoid", "avoid"]
        assert [d["decision"] for d in c.decisions("LAT")] == ["announce", "merged", "announce"]

    def test_calm_alone_is_not_enough_within_6s(self):
        c = Clock(make())
        c.run(1.0, vcc(), points=obstacle(1.2))
        c.run(1.0, vcc(target=0.3), points=obstacle(1.2))
        c.run(2.5, vcc())
        c.run(1.0, vcc(target=0.3), points=obstacle(1.2))
        c.flush()
        assert c.said() == ["avoid"]

    def _two_obstacles(self, gap_sec, second_at=3.0):
        """원점에서 1.2 m 앞 장애물을 비키고(말함), 평상 주행 없이(0.2 m/s) second_at 으로 옮겨 가"""
        c = Clock(make())
        c.run(1.0, vcc(), points=obstacle(1.2))
        c.run(1.0, vcc(target=0.3), points=obstacle(1.2))
        c.run(gap_sec - 1.0, vcc(target=0.0, v=0.2), pose=(second_at, 0.0, 0.0))
        c.run(1.0, vcc(target=-0.3, v=0.2), points=obstacle(1.2), pose=(second_at, 0.0, 0.0))
        c.flush()
        return c

    def test_another_obstacle_elsewhere_is_said_again(self):
        c = self._two_obstacles(gap_sec=4.0)
        assert c.said() == ["avoid", "avoid"]

    def test_another_obstacle_too_soon_is_merged(self):
        c = self._two_obstacles(gap_sec=2.0)
        assert c.said() == ["avoid"]

    def test_same_place_is_still_one_obstacle(self):
        c = self._two_obstacles(gap_sec=4.0, second_at=0.5)
        assert c.said() == ["avoid"]

    def test_first_action_decides_the_phrase(self):
        c = Clock(make())
        c.run(1.0, vcc(v=0.5), points=obstacle(1.2))
        c.run(0.3, vcc(v=0.2), points=obstacle(1.2))
        c.run(1.0, vcc(v=0.2, target=0.3), points=obstacle(1.2))
        c.flush()
        assert c.said() == ["slow"]

    def test_decisions_come_out_in_onset_order(self):
        c = Clock(make())
        c.run(1.0, vcc(v=0.5), points=obstacle(1.2))
        c.run(0.3, vcc(v=0.2), points=obstacle(1.2))
        c.run(1.0, vcc(v=0.2, target=0.3), points=obstacle(1.2))
        c.flush()
        ts = [d["t"] for d in c.out]
        assert ts == sorted(ts)


class TestClearInputs:
    """미션은 안내 주행 중에만 입력을 받는다. 다시 받을 때 지난 주행의 위치·점·VCC·판정 대기를 지운다."""

    def test_old_pose_is_not_used_after_restart(self):
        j = make()
        c = Clock(j)
        c.run(1.0, vcc())
        j.clear_inputs()
        out = []
        for k in range(1, 15):
            t = round(c.t + 0.1 * k, 3)
            j.on_points("scan", t, *obstacle(1.5))
            j.on_vcc(t, vcc(target=0.3 if k > 3 else 0.0))
            out += j.tick(t)
        assert [d["decision"] for d in out if d["kind"] == "LAT"] == ["no_cause"]

    def test_pending_decision_is_dropped(self):
        j = make()
        c = Clock(j)
        c.run(1.0, vcc(), points=obstacle(1.5))
        j.on_vcc(round(c.t + 0.1, 3), vcc(target=0.3))
        assert j.pending
        j.clear_inputs()
        assert j.pending == [] and j.tick(c.t + 2.0) == []

    def test_inputs_cleared_but_rail_goal_and_said_memory_kept(self):
        j = make()
        c = Clock(j)
        c.run(1.0, vcc(), points=obstacle(1.2))
        c.run(1.0, vcc(target=0.3), points=obstacle(1.2))
        assert c.said() == ["avoid"]
        j.clear_inputs()
        assert not j.poses and not j.vcc and not j.frames["scan"] and not j.frames["depth"]
        assert j.rails and j.goal is not None and j.last_ann is not None
        c.run(1.0, vcc(), points=obstacle(1.2))
        c.run(1.0, vcc(target=-0.3), points=obstacle(1.2))
        c.flush()
        assert c.said() == ["avoid"]


class TestNewTripForgets:
    """10-10 12:12·12:14 주행: 앞 안내(12:10)의 '한 번만' 기억이 남아 새 안내의 장애물이 전부 '묶음'이 됐다."""

    def _said_then(self, j, c, end_event):
        c.run(1.0, vcc(), points=obstacle(1.2))
        c.run(1.0, vcc(target=0.3), points=obstacle(1.2))
        assert c.said() == ["avoid"]
        j.on_goal(c.t, end_event)
        j.clear_inputs()
        j.on_goal(c.t + 0.5, "goal_sent", 9.0, 0.0, "dest-2")
        c.t = round(c.t + 0.5, 3)
        c.run(1.0, vcc(), points=obstacle(1.2))
        c.run(1.0, vcc(target=-0.3), points=obstacle(1.2))
        c.flush()

    @pytest.mark.parametrize("end", ["goal_succeeded", "goal_canceled", "goal_failed", "goal_aborted"])
    def test_new_trip_after_the_end_speaks_again(self, end):
        j = make()
        c = Clock(j)
        self._said_then(j, c, end)
        assert c.said() == ["avoid", "avoid"]

    def test_resume_after_pause_keeps_the_memory(self):
        j = make()
        c = Clock(j)
        self._said_then(j, c, "goal_paused")
        assert c.said() == ["avoid"]

    def test_return_home_in_between_also_ends_the_trip(self):
        j = make()
        c = Clock(j)
        c.run(1.0, vcc(), points=obstacle(1.2))
        c.run(1.0, vcc(target=0.3), points=obstacle(1.2))
        j.on_goal(c.t, "goal_succeeded")
        j.on_goal(c.t, "return_home_sent", 0.0, 0.0, "home")
        j.on_goal(c.t, "return_home_succeeded")
        assert j.last_ann is None and j.calm_done is None
