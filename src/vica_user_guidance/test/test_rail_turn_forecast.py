"""레일 회전 예고 순수 로직 시험 (2026-09-28 합의 규칙)."""

import math

from vica_user_guidance.rail_turn_forecast import (
    RailPose,
    RailTurnArbiter,
    find_corners,
    project_to_path,
)
from vica_user_guidance.turn_detector import (
    DIRECTION_LEFT,
    DIRECTION_NONE,
    DIRECTION_RIGHT,
    PHASE_COMPLETE,
    PHASE_IDLE,
    PHASE_NOW,
    PHASE_PREPARE,
    TurnDecision,
)


def line(p0, p1, step=0.05):
    n = max(1, int(math.dist(p0, p1) / step))
    return [(p0[0] + (p1[0] - p0[0]) * k / n, p0[1] + (p1[1] - p0[1]) * k / n)
            for k in range(n + 1)]


def l_path(first=5.0, second=5.0, left=True):
    """(0,0) 에서 +x 로 first 만큼, 90° 꺾어 second 만큼."""
    y = second if left else -second
    return line((0, 0), (first, 0)) + line((first, 0), (first, y))[1:]


def idle(seq=0):
    return TurnDecision(DIRECTION_NONE, PHASE_IDLE, 0.0, seq, False)


def now(direction, angle=25.0, seq=1):
    return TurnDecision(direction, PHASE_NOW, angle if direction == DIRECTION_LEFT else -angle,
                        seq, False)


def pose_at(s, total=10.0, off=0.0):
    return RailPose(s=s, offtrack_m=off, remaining_m=total - s, heading_rad=0.0)


def test_right_angle_corner_is_found_with_direction():
    cs = find_corners(l_path(left=True))
    assert len(cs) == 1
    assert cs[0].direction == DIRECTION_LEFT
    assert 80.0 <= cs[0].angle_deg <= 100.0
    assert 4.5 <= cs[0].start_s <= 5.2


def test_small_bend_below_45_is_ignored():
    a = (5.0, 0.0)
    b = (5.0 + 5.0 * math.cos(math.radians(30)), 5.0 * math.sin(math.radians(30)))
    assert find_corners(line((0, 0), a) + line(a, b)[1:]) == []


def test_s_bend_does_not_add_up():
    pts = [(0.0, 0.0), (3.0, 0.0)]
    h = math.radians(30)
    pts.append((pts[-1][0] + math.cos(h), pts[-1][1] + math.sin(h)))
    pts.append((pts[-1][0] + 3.0, pts[-1][1]))
    path = []
    for p, q in zip(pts, pts[1:]):
        path += line(p, q)[1:] if path else line(p, q)
    assert find_corners(path) == []


def test_long_gentle_drift_is_not_one_giant_corner():
    pts, h, x, y = [(0.0, 0.0)], 0.0, 0.0, 0.0
    for _ in range(30):
        x, y = x + 0.8 * math.cos(h), y + 0.8 * math.sin(h)
        pts.append((x, y))
        h += math.radians(1.5)
    path = []
    for p, q in zip(pts, pts[1:]):
        path += line(p, q)[1:] if path else line(p, q)
    assert find_corners(path) == []


def test_same_direction_pieces_close_together_merge():
    pts, h = [(0.0, 0.0), (3.0, 0.0)], 0.0
    for d in (-50, -50):
        h += math.radians(d)
        pts.append((pts[-1][0] + 0.8 * math.cos(h), pts[-1][1] + 0.8 * math.sin(h)))
    pts.append((pts[-1][0] + 3.0 * math.cos(h), pts[-1][1] + 3.0 * math.sin(h)))
    path = []
    for p, q in zip(pts, pts[1:]):
        path += line(p, q)[1:] if path else line(p, q)
    cs = find_corners(path)
    assert len(cs) == 1
    assert cs[0].direction == DIRECTION_RIGHT
    assert cs[0].angle_deg <= -90.0


def test_projection_reports_heading_and_remaining():
    path = line((0, 0), (10, 0))
    p = project_to_path(path, (4.0, 0.3))
    assert abs(p.s - 4.0) < 1e-6 and abs(p.offtrack_m - 0.3) < 1e-6
    assert abs(p.remaining_m - 6.0) < 1e-6 and abs(p.heading_rad) < 1e-9


def test_prepare_fires_about_two_seconds_plus_lookahead_before_corner():
    cs = find_corners(l_path())
    arb = RailTurnArbiter()
    c0 = cs[0].start_s
    early = arb.resolve(idle(), cs, pose_at(c0 - 1.9), 0.4, True, 0.0)
    assert early.direction == DIRECTION_NONE
    on = arb.resolve(idle(), cs, pose_at(c0 - 1.75), 0.4, True, 0.0)
    assert on.direction == DIRECTION_LEFT and on.phase == PHASE_PREPARE


def test_no_prepare_while_stopped_misaligned_or_off_rail():
    cs = find_corners(l_path())
    s = cs[0].start_s - 1.5
    assert RailTurnArbiter().resolve(idle(), cs, pose_at(s), 0.0, True, 0.0).direction == 0
    assert RailTurnArbiter().resolve(idle(), cs, pose_at(s), 0.4, True, math.radians(60)).direction == 0
    assert RailTurnArbiter().resolve(idle(), cs, pose_at(s, off=0.9), 0.4, True, 0.0).direction == 0
    assert RailTurnArbiter().resolve(idle(), cs, pose_at(s), 0.4, False, 0.0).direction == 0


def test_corner_inside_goal_handoff_is_not_announced():
    path = l_path(second=1.0)
    cs = find_corners(path)
    total = cs[0].end_s + 1.0
    cue = RailTurnArbiter().resolve(idle(), cs, pose_at(cs[0].start_s - 1.5, total), 0.4, True, 0.0)
    assert cue.direction == DIRECTION_NONE


def vcc_arbiter():
    """VCC: 레일 차선을 따라 코너 시작에서 돈다 — 조준거리 보정 0(2026-10-12)."""
    return RailTurnArbiter(controller_lookahead_time_sec=0.0, controller_lookahead_min_m=0.0,
                           controller_lookahead_max_m=0.0)


def test_prepare_fires_two_seconds_before_corner_without_controller_lookahead():
    cs = find_corners(l_path())
    c0 = cs[0].start_s
    arb = vcc_arbiter()
    assert arb.resolve(idle(), cs, pose_at(c0 - 1.1), 0.5, True, 0.0).direction == DIRECTION_NONE
    on = arb.resolve(idle(), cs, pose_at(c0 - 0.95), 0.5, True, 0.0)
    assert on.direction == DIRECTION_LEFT and on.phase == PHASE_PREPARE
    assert abs(on.distance_m - 0.95) < 1e-6


def test_projection_behind_path_start_extends_the_first_segment():
    path = line((0, 0), (5, 0))
    old = project_to_path(path, (-0.9, 0.1))
    assert old.s == 0.0 and abs(old.offtrack_m - math.hypot(0.9, 0.1)) < 1e-6
    p = project_to_path(path, (-0.9, 0.1), behind_start_m=1.5)
    assert abs(p.s + 0.9) < 1e-6 and abs(p.offtrack_m - 0.1) < 1e-6
    assert abs(p.remaining_m - 5.9) < 1e-6
    far = project_to_path(path, (-2.0, 0.1), behind_start_m=1.5)
    assert far.s == 0.0 and abs(far.offtrack_m - math.hypot(2.0, 0.1)) < 1e-6


def arc_l_path(first=5.0, radius=1.0, second=4.0, start=0.0):
    """(start,0) 에서 +x 로 first 까지, 반지름 radius 호로 왼쪽 90°, 위로 second. 실제 레일처럼"""
    pts = line((start, 0), (first, 0))
    n = 30
    for k in range(1, n + 1):
        a = -math.pi / 2 + (math.pi / 2) * k / n
        pts.append((first + radius * math.cos(a), radius + radius * math.sin(a)))
    return pts + line((first + radius, radius), (first + radius, radius + second))[1:]


def test_prepare_survives_rail_recut_ahead_of_robot():
    full = arc_l_path()
    cs = find_corners(full)
    robot = (4.1, 0.05)

    def recut(behind):
        arb = vcc_arbiter()
        on = arb.resolve(idle(), cs, project_to_path(full, robot, behind_start_m=behind),
                         0.5, True, 0.0)
        assert on.phase == PHASE_PREPARE and on.direction == DIRECTION_LEFT
        cut = arc_l_path(start=5.0)
        pose = project_to_path(cut, robot, behind_start_m=behind)
        return on, arb.resolve(idle(), find_corners(cut), pose, 0.5, True, 0.0)

    on, held = recut(1.5)
    assert held.direction == DIRECTION_LEFT and held.phase == PHASE_PREPARE
    assert held.sequence_id == on.sequence_id
    assert abs(held.distance_m - on.distance_m) < 0.25
    _, dropped = recut(0.0)
    assert dropped.direction == DIRECTION_NONE


def test_corner_too_close_is_left_to_detector_without_lookahead():
    cs = find_corners(l_path())
    c0 = cs[0].start_s
    near = pose_at(c0 - 0.3)
    assert vcc_arbiter().resolve(idle(), cs, near, 0.2, True, 0.0).phase == PHASE_PREPARE
    guarded = RailTurnArbiter(controller_lookahead_time_sec=0.0, controller_lookahead_min_m=0.0,
                              controller_lookahead_max_m=0.0, min_prepare_dist_m=0.5)
    assert guarded.resolve(idle(), cs, near, 0.2, True, 0.0).direction == DIRECTION_NONE
    assert guarded.resolve(idle(), cs, pose_at(c0 - 0.95), 0.5, True, 0.0).phase == PHASE_PREPARE


def u_path(start=0.0, tail=5.0):
    """∩ 의 U: +x 로 5 m → 반지름 1 왼쪽 90° → 위로 0.6 m → 반지름 1 왼쪽 90° → -x 로 tail m."""
    pts = line((start, 0), (5, 0))
    for k in range(1, 31):
        a = -math.pi / 2 + (math.pi / 2) * k / 30
        pts.append((5 + math.cos(a), 1 + math.sin(a)))
    pts += line((6, 1), (6, 1.6))[1:]
    for k in range(1, 31):
        a = (math.pi / 2) * k / 30
        pts.append((5 + math.cos(a), 1.6 + math.sin(a)))
    return pts + line((5, 2.6), (5 - tail, 2.6))[1:]


def test_signal_holds_through_the_u_when_the_rail_is_recut_inside_it():
    full = u_path()
    cs = find_corners(full)
    assert len(cs) == 1 and cs[0].angle_deg > 170

    def run(rehold):
        arb = RailTurnArbiter(controller_lookahead_time_sec=0.0, controller_lookahead_min_m=0.0,
                              controller_lookahead_max_m=0.0, min_prepare_dist_m=0.5,
                              rehold_end_m=rehold)
        p = arb.resolve(idle(), cs, project_to_path(full, (4.1, 0.0)), 0.5, True, 0.0)
        assert p.phase == PHASE_PREPARE
        n = arb.resolve(now(DIRECTION_LEFT), cs, project_to_path(full, (5.7, 0.3)), 0.4, True, 0.0)
        assert n.phase == PHASE_NOW and n.sequence_id == p.sequence_id
        cut = full[next(i for i, q in enumerate(full) if abs(q[0] - 6) < 1e-9 and q[1] >= 1.4):]
        pose = project_to_path(cut, (6.0, 1.3), behind_start_m=1.5)
        return p, arb.resolve(idle(), find_corners(cut), pose, 0.4, True, 0.0)

    p, held = run(0.5)
    assert held.direction == DIRECTION_LEFT and held.phase == PHASE_NOW
    assert held.sequence_id == p.sequence_id
    _, ended = run(0.0)
    assert ended.direction == DIRECTION_NONE


def vcc_guarded(rehold=0.5):
    return RailTurnArbiter(controller_lookahead_time_sec=0.0, controller_lookahead_min_m=0.0,
                           controller_lookahead_max_m=0.0, min_prepare_dist_m=0.5,
                           rehold_end_m=rehold)


def test_rehold_is_not_used_near_the_goal():
    full = u_path(tail=0.1)
    cs = find_corners(full)
    arb = vcc_guarded()
    n = arb.resolve(now(DIRECTION_LEFT), cs, project_to_path(full, (5.7, 0.3)), 0.4, True, 0.0)
    assert n.phase == PHASE_NOW
    cut = full[next(i for i, q in enumerate(full) if abs(q[0] - 6) < 1e-9 and q[1] >= 1.4):]
    pose = project_to_path(cut, (6.0, 1.3), behind_start_m=1.5)
    assert pose.remaining_m <= 2.0
    ended = arb.resolve(idle(), find_corners(cut), pose, 0.0, True, 0.0)
    assert ended.direction == DIRECTION_NONE


def test_rehold_does_not_jump_to_a_separate_next_corner():
    pts = line((0, 0), (5, 0))
    for k in range(1, 31):
        a = -math.pi / 2 + (math.pi / 2) * k / 30
        pts.append((5 + math.cos(a), 1 + math.sin(a)))
    pts += line((6, 1), (6, 2.4))[1:]
    for k in range(1, 31):
        a = (math.pi / 2) * k / 30
        pts.append((5 + math.cos(a), 2.4 + math.sin(a)))
    full = pts + line((5, 3.4), (0, 3.4))[1:]
    cs = find_corners(full)
    assert len(cs) == 2
    arb = vcc_guarded()
    n = arb.resolve(now(DIRECTION_LEFT), cs, project_to_path(full, (5.7, 0.3)), 0.4, True, 0.0)
    assert n.phase == PHASE_NOW
    cut = full[next(i for i, q in enumerate(full) if abs(q[0] - 6) < 1e-9 and q[1] >= 1.3):]
    pose = project_to_path(cut, (6.0, 1.2), behind_start_m=1.5)
    after = arb.resolve(idle(), find_corners(cut), pose, 0.4, True, 0.0)
    assert after.phase != PHASE_NOW


def test_close_corner_blocks_preparing_the_one_beyond_it():
    path = line((0, 0), (5, 0)) + line((5, 0), (5, 0.6))[1:] + line((5, 0.6), (10, 0.6))[1:]
    cs = find_corners(path)
    assert [c.direction for c in cs] == [DIRECTION_LEFT, DIRECTION_RIGHT]
    pose = project_to_path(path, (cs[0].start_s - 0.3, 0.0))
    for v in (0.5, 0.6):
        assert vcc_guarded().resolve(idle(), cs, pose, v, True, 0.0).direction == DIRECTION_NONE


def test_corner_already_inside_lookahead_is_left_to_detector():
    cs = find_corners(l_path())
    cue = RailTurnArbiter().resolve(idle(), cs, pose_at(cs[0].start_s - 0.5), 0.4, True, 0.0)
    assert cue.direction == DIRECTION_NONE


def test_actual_turn_inherits_prepare_without_gap():
    cs = find_corners(l_path())
    arb = RailTurnArbiter()
    c0 = cs[0].start_s
    p = arb.resolve(idle(), cs, pose_at(c0 - 1.7), 0.4, True, 0.0)
    h = arb.resolve(idle(), cs, pose_at(c0 - 0.2), 0.4, True, 0.0)
    n = arb.resolve(now(DIRECTION_LEFT), cs, pose_at(c0 + 0.1), 0.4, True, 0.0)
    assert [p.direction, h.direction, n.direction] == [DIRECTION_LEFT] * 3
    assert p.sequence_id == h.sequence_id == n.sequence_id
    assert n.reason == "now_inherit"


def test_signal_held_until_corner_end_even_if_detector_closes_early():
    cs = find_corners(l_path())
    arb = RailTurnArbiter()
    c = cs[0]
    arb.resolve(idle(), cs, pose_at(c.start_s - 1.7), 0.4, True, 0.0)
    arb.resolve(now(DIRECTION_LEFT), cs, pose_at(c.start_s), 0.4, True, 0.0)
    mid = arb.resolve(TurnDecision(DIRECTION_NONE, PHASE_COMPLETE, 2.0, 1, False),
                      cs, pose_at((c.start_s + c.end_s) / 2), 0.4, True, 0.0)
    again = arb.resolve(now(DIRECTION_LEFT, seq=2), cs, pose_at(c.end_s), 0.4, True, 0.0)
    assert mid.direction == DIRECTION_LEFT and again.direction == DIRECTION_LEFT
    assert mid.sequence_id == again.sequence_id


def test_signal_held_past_corner_end_while_still_turning():
    cs = find_corners(l_path())
    arb = RailTurnArbiter()
    c = cs[0]
    arb.resolve(idle(), cs, pose_at(c.start_s - 1.7), 0.4, True, 0.0)
    late = arb.resolve(TurnDecision(DIRECTION_NONE, PHASE_IDLE, 15.0, 0, False),
                       cs, pose_at(c.end_s + 1.0), 0.4, True, 0.0)
    assert late.direction == DIRECTION_LEFT
    done = arb.resolve(TurnDecision(DIRECTION_NONE, PHASE_IDLE, 2.0, 0, False),
                       cs, pose_at(c.end_s + 1.1), 0.4, True, 0.0)
    assert done.direction == DIRECTION_NONE


def test_announced_corner_is_never_announced_again():
    cs = find_corners(l_path())
    arb = RailTurnArbiter()
    c = cs[0]
    arb.resolve(idle(), cs, pose_at(c.start_s - 1.7), 0.4, True, 0.0)
    arb.resolve(idle(), cs, pose_at(c.start_s - 1.7, off=0.9), 0.4, True, 0.0)
    back = arb.resolve(idle(), cs, pose_at(c.start_s - 1.6), 0.4, True, 0.0)
    assert back.direction == DIRECTION_NONE


def test_early_actual_turn_claims_the_corner_ahead():
    cs = find_corners(l_path())
    arb = RailTurnArbiter()
    c = cs[0]
    arb.resolve(now(DIRECTION_LEFT), cs, pose_at(c.start_s - 1.0), 0.0, True, 0.0)
    arb.resolve(TurnDecision(DIRECTION_NONE, PHASE_COMPLETE, 2.0, 1, False),
                cs, pose_at(c.end_s + 0.5), 0.4, True, 0.0)
    cue = arb.resolve(idle(), cs, pose_at(c.end_s + 0.5), 0.4, True, 0.0)
    assert cue.direction == DIRECTION_NONE


def test_opposite_actual_turn_overrides_prepare():
    cs = find_corners(l_path())
    arb = RailTurnArbiter()
    c = cs[0]
    p = arb.resolve(idle(), cs, pose_at(c.start_s - 1.7), 0.4, True, 0.0)
    small = arb.resolve(now(DIRECTION_RIGHT), cs, pose_at(c.start_s - 1.6), 0.4, True, 0.0)
    assert small.direction == DIRECTION_LEFT and small.sequence_id == p.sequence_id
    r = arb.resolve(now(DIRECTION_RIGHT, angle=40.0), cs, pose_at(c.start_s - 1.5), 0.4, True, 0.0)
    assert r.direction == DIRECTION_RIGHT and r.sequence_id != p.sequence_id


def test_same_direction_next_corner_chains_without_gap():
    path = (line((0, 0), (5, 0)) + line((5, 0), (5, 2))[1:] + line((5, 2), (0, 2))[1:])
    cs = find_corners(path)
    assert len(cs) == 2
    arb = RailTurnArbiter()
    a, b = cs
    total = 15.0
    arb.resolve(idle(), cs, pose_at(a.start_s - 1.7, total), 0.4, True, 0.0)
    arb.resolve(now(DIRECTION_LEFT), cs, pose_at(a.start_s, total), 0.4, True, 0.0)
    s_end = max(a.end_s + 0.31, b.start_s - 1.75)
    cue = arb.resolve(TurnDecision(DIRECTION_NONE, PHASE_COMPLETE, 2.0, 1, False),
                      cs, RailPose(s_end, 0.0, total - s_end, math.pi / 2), 0.4, True, math.pi / 2)
    assert cue.direction == DIRECTION_LEFT


def test_turn_near_the_goal_does_not_hold_after_stopping():
    cs = find_corners(l_path(first=5.0, second=1.0))
    assert len(cs) == 1
    c = cs[0]
    total = 6.0
    arb = RailTurnArbiter()
    on = arb.resolve(now(DIRECTION_LEFT), cs, pose_at(c.start_s, total), 0.0, False, 0.0)
    assert on.direction == DIRECTION_LEFT
    arb.resolve(TurnDecision(DIRECTION_NONE, PHASE_COMPLETE, 20.0, 1, False),
                cs, pose_at(c.start_s, total), 0.0, False, 0.0)
    off = arb.resolve(idle(), cs, pose_at(c.start_s, total), 0.0, False, 0.0)
    assert off.direction == DIRECTION_NONE


def test_lane_change_swerve_on_straight_rail_does_not_light():
    path = line((0, 0), (10, 0))
    cs = find_corners(path)
    arb = RailTurnArbiter()
    for angle in (22.0, 28.0, 33.0):
        cue = arb.resolve(now(DIRECTION_LEFT, angle=angle), cs, pose_at(3.0), 0.4, True, 0.0)
        assert cue.direction == DIRECTION_NONE
    back = arb.resolve(now(DIRECTION_RIGHT, angle=25.0), cs, pose_at(3.3), 0.4, True, 0.0)
    assert back.direction == DIRECTION_NONE


def test_bigger_turn_while_moving_still_lights():
    cs = find_corners(line((0, 0), (10, 0)))
    arb = RailTurnArbiter()
    assert arb.resolve(now(DIRECTION_LEFT, angle=25.0), cs, pose_at(3.0), 0.4, True,
                       0.0).direction == DIRECTION_NONE
    cue = arb.resolve(now(DIRECTION_LEFT, angle=36.0), cs, pose_at(3.1), 0.4, True, 0.0)
    assert cue.direction == DIRECTION_LEFT and cue.phase == PHASE_NOW


def test_slow_turn_lights_at_the_usual_angle():
    cs = find_corners(line((0, 0), (10, 0)))
    for speed in (0.0, 0.15):
        arb = RailTurnArbiter()
        cue = arb.resolve(now(DIRECTION_LEFT, angle=22.0), cs, pose_at(3.0), speed, True, 0.0)
        assert cue.direction == DIRECTION_LEFT


def test_turn_at_a_rail_corner_lights_at_the_usual_angle():
    cs = find_corners(l_path())
    c = cs[0]
    arb = RailTurnArbiter()
    cue = arb.resolve(now(DIRECTION_LEFT, angle=22.0), cs, pose_at(c.start_s), 0.5, True, 0.0)
    assert cue.direction == DIRECTION_LEFT


def test_minor_turn_filter_can_be_switched_off():
    cs = find_corners(line((0, 0), (10, 0)))
    arb = RailTurnArbiter(minor_turn_deg=0.0)
    cue = arb.resolve(now(DIRECTION_LEFT, angle=22.0), cs, pose_at(3.0), 0.5, True, 0.0)
    assert cue.direction == DIRECTION_LEFT
