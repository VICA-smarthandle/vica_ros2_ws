"""레일 회전 예고 순수 로직 시험 (2026-09-28 합의 규칙).

시각·위치는 전부 손으로 만든다. 핸들이 '두 번 알림'을 느끼는 것은 방향이 한 tick 이라도
NONE 으로 떨어졌다 다시 켜질 때뿐이므로, 시험은 방향이 끊기지 않는지를 본다.
"""

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


# ── find_corners ────────────────────────────────────

def test_right_angle_corner_is_found_with_direction():
    cs = find_corners(l_path(left=True))
    assert len(cs) == 1
    assert cs[0].direction == DIRECTION_LEFT
    assert 80.0 <= cs[0].angle_deg <= 100.0
    assert 4.5 <= cs[0].start_s <= 5.2


def test_small_bend_below_45_is_ignored():
    # 30° 꺾임 — 예고 대상이 아니다(사후 20° 판정 몫)
    a = (5.0, 0.0)
    b = (5.0 + 5.0 * math.cos(math.radians(30)), 5.0 * math.sin(math.radians(30)))
    assert find_corners(line((0, 0), a) + line(a, b)[1:]) == []


def test_s_bend_does_not_add_up():
    # 좌 30° 뒤 바로 우 30° — 합쳐서 60° 가 되면 안 된다(0903_d ±18° 지그재그와 같은 모양)
    pts = [(0.0, 0.0), (3.0, 0.0)]
    h = math.radians(30)
    pts.append((pts[-1][0] + math.cos(h), pts[-1][1] + math.sin(h)))
    pts.append((pts[-1][0] + 3.0, pts[-1][1]))
    path = []
    for p, q in zip(pts, pts[1:]):
        path += line(p, q)[1:] if path else line(p, q)
    assert find_corners(path) == []


def test_long_gentle_drift_is_not_one_giant_corner():
    # 1.5° 씩 0.8 m 간격으로 30번 = 45° 지만 긴 복도의 잔굽이다(0903_d 22 m '코너' 사건)
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
    # 오른쪽 50° 뒤 짧은 직진, 다시 오른쪽 50° (꺾임 사이 0.8 m) — 한 코너(0630 −48°·−83°·−47° 사건)
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


# ── 예고 시점 ─────────────────────────────────────

def test_prepare_fires_about_two_seconds_plus_lookahead_before_corner():
    cs = find_corners(l_path())
    arb = RailTurnArbiter()
    c0 = cs[0].start_s
    # 0.4 m/s: 조준거리 1.0 m + 2 s x 0.4 = 1.8 m 앞에서 켜진다
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
    # 레일 끝이 코너 바로 뒤 1 m — 코너가 목적지 2 m 안에 걸린다
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
    # 0.5 m/s: 보정 0 이면 2 s x 0.5 = 1.0 m 앞에서 켜진다(RPP 값이면 2.2 m 앞)
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
    # 허용보다 멀리 뒤면 옛 계산 그대로(첫 점까지 거리)
    far = project_to_path(path, (-2.0, 0.1), behind_start_m=1.5)
    assert far.s == 0.0 and abs(far.offtrack_m - math.hypot(2.0, 0.1)) < 1e-6


def arc_l_path(first=5.0, radius=1.0, second=4.0, start=0.0):
    """(start,0) 에서 +x 로 first 까지, 반지름 radius 호로 왼쪽 90°, 위로 second. 실제 레일처럼
    꼭짓점을 호로 둥글린 모양(호 시작 = x first)."""
    pts = line((start, 0), (first, 0))
    n = 30
    for k in range(1, n + 1):
        a = -math.pi / 2 + (math.pi / 2) * k / n
        pts.append((first + radius * math.cos(a), radius + radius * math.sin(a)))
    return pts + line((first + radius, radius), (first + radius, radius + second))[1:]


def test_prepare_survives_rail_recut_ahead_of_robot():
    # Route Server 는 레일을 로봇 앞 노드부터 다시 잘라 보낸다. 로봇이 새 첫 점(여기선 호가
    # 시작하는 노드) 뒤 0.9 m 에 있어도 '레일 벗어남'으로 예고를 내리면 안 된다
    # (10-12 녹화: 켜진 예고 15번 중 6번이 꺼졌다 몸이 돌 때 다시 켜짐).
    full = arc_l_path()
    cs = find_corners(full)
    robot = (4.1, 0.05)                      # 호 시작(x 5.0) 0.9 m 앞

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
    # 늘이지 않으면(옛 동작) 첫 점까지 0.9 m 가 '레일 밖 거리'가 돼 예고가 꺼진다
    _, dropped = recut(0.0)
    assert dropped.direction == DIRECTION_NONE


def test_corner_too_close_is_left_to_detector_without_lookahead():
    # 보정 0 이면 조준거리가 '늦은 예고 막기'를 못 한다 — min_prepare_dist_m 이 대신 막는다
    cs = find_corners(l_path())
    c0 = cs[0].start_s
    near = pose_at(c0 - 0.3)                 # 0.2 m/s: 2 s x 0.2 = 0.4 m 안 → 시간 조건은 맞음
    assert vcc_arbiter().resolve(idle(), cs, near, 0.2, True, 0.0).phase == PHASE_PREPARE
    guarded = RailTurnArbiter(controller_lookahead_time_sec=0.0, controller_lookahead_min_m=0.0,
                              controller_lookahead_max_m=0.0, min_prepare_dist_m=0.5)
    assert guarded.resolve(idle(), cs, near, 0.2, True, 0.0).direction == DIRECTION_NONE
    # 0.5 m/s 정상 예고(1.0 m 앞)는 막지 않는다
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
    # U 를 도는 중 레일이 U 안(두 호 사이 직선)에서 다시 잘리면 코너 시작점이 바뀐다. 사이 직선에서
    # 몸이 잠깐 덜 돌아도 신호가 끝났다 다시 켜지면 안 된다(10-12 녹화 재생: 같은 U 두 번 알림 5번).
    full = u_path()
    cs = find_corners(full)
    assert len(cs) == 1 and cs[0].angle_deg > 170       # 두 호가 한 코너로 합쳐진다

    def run(rehold):
        arb = RailTurnArbiter(controller_lookahead_time_sec=0.0, controller_lookahead_min_m=0.0,
                              controller_lookahead_max_m=0.0, min_prepare_dist_m=0.5,
                              rehold_end_m=rehold)
        p = arb.resolve(idle(), cs, project_to_path(full, (4.1, 0.0)), 0.5, True, 0.0)
        assert p.phase == PHASE_PREPARE
        n = arb.resolve(now(DIRECTION_LEFT), cs, project_to_path(full, (5.7, 0.3)), 0.4, True, 0.0)
        assert n.phase == PHASE_NOW and n.sequence_id == p.sequence_id
        # 두 호 사이 직선(x 6, y 1.0~1.6)의 y 1.4 에서 다시 잘린 레일, 로봇은 그 0.1 m 뒤
        cut = full[next(i for i, q in enumerate(full) if abs(q[0] - 6) < 1e-9 and q[1] >= 1.4):]
        pose = project_to_path(cut, (6.0, 1.3), behind_start_m=1.5)
        return p, arb.resolve(idle(), find_corners(cut), pose, 0.4, True, 0.0)

    p, held = run(0.5)
    assert held.direction == DIRECTION_LEFT and held.phase == PHASE_NOW
    assert held.sequence_id == p.sequence_id
    _, ended = run(0.0)                       # 옛 동작: 사이 직선에서 끝난다
    assert ended.direction == DIRECTION_NONE


def vcc_guarded(rehold=0.5):
    return RailTurnArbiter(controller_lookahead_time_sec=0.0, controller_lookahead_min_m=0.0,
                           controller_lookahead_max_m=0.0, min_prepare_dist_m=0.5,
                           rehold_end_m=rehold)


def test_rehold_is_not_used_near_the_goal():
    # 레일 끝(목적지 2 m 안)의 코너에는 다시 잡기로 신호를 묶지 않는다 — 멈춘 로봇에서 신호가
    # 끝나지 않던 10-08 run69 사건과 같은 구조(검토 지적). U 바로 뒤가 레일 끝인 경로.
    full = u_path(tail=0.1)
    cs = find_corners(full)
    arb = vcc_guarded()
    n = arb.resolve(now(DIRECTION_LEFT), cs, project_to_path(full, (5.7, 0.3)), 0.4, True, 0.0)
    assert n.phase == PHASE_NOW                      # 아직 목적지 2 m 밖 — 코너에 묶인다
    cut = full[next(i for i, q in enumerate(full) if abs(q[0] - 6) < 1e-9 and q[1] >= 1.4):]
    pose = project_to_path(cut, (6.0, 1.3), behind_start_m=1.5)
    assert pose.remaining_m <= 2.0
    ended = arb.resolve(idle(), find_corners(cut), pose, 0.0, True, 0.0)
    assert ended.direction == DIRECTION_NONE


def test_rehold_does_not_jump_to_a_separate_next_corner():
    # 왼쪽 90° 두 개가 1.4 m 떨어져 따로 있는 길. 첫 코너를 다 돈 뒤 레일이 그 뒤에서 잘리면 첫 코너는
    # 사라진다. 끝점이 다른 둘째 코너에 신호를 이어 붙이면 안 된다(검토 지적).
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
    # S자: 왼쪽 90° 가 0.3 m 앞, 바로 뒤 오른쪽 90°. 가까운 왼쪽을 건너뛰고 오른쪽을 예고하면
    # '예고는 오른쪽, 실제는 왼쪽'이 된다(검토 지적). 둘 다 사후 판정에 맡긴다.
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


# ── 두 번 알림 방지 ─────────────────────────────────

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
    # 긴 코너를 yaw 판정이 중간에 닫았다 다시 열어도(0630 −177° 사건) 핸들은 한 번이다
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
    arb.resolve(idle(), cs, pose_at(c.start_s - 1.7, off=0.9), 0.4, True, 0.0)   # 레일 이탈 → 취소
    back = arb.resolve(idle(), cs, pose_at(c.start_s - 1.6), 0.4, True, 0.0)
    assert back.direction == DIRECTION_NONE


def test_early_actual_turn_claims_the_corner_ahead():
    # 예고 전에 컨트롤러가 먼저 돌기 시작 → 끝난 뒤 같은 코너를 다시 예고하면 안 된다
    cs = find_corners(l_path())
    arb = RailTurnArbiter()
    c = cs[0]
    arb.resolve(now(DIRECTION_LEFT), cs, pose_at(c.start_s - 1.0), 0.0, True, 0.0)
    arb.resolve(TurnDecision(DIRECTION_NONE, PHASE_COMPLETE, 2.0, 1, False),
                cs, pose_at(c.end_s + 0.5), 0.4, True, 0.0)
    cue = arb.resolve(idle(), cs, pose_at(c.end_s + 0.5), 0.4, True, 0.0)
    assert cue.direction == DIRECTION_NONE


def test_opposite_actual_turn_overrides_prepare():
    # 장애물 회피로 예고와 반대로 크게 틀면 그 회전을 새로 알린다(B안 — 사후 판정 그대로).
    # 작은 비킴(minor_turn_deg 미만)은 예고를 그대로 둔다(10-08, 아래 차선 옮김 시험).
    cs = find_corners(l_path())
    arb = RailTurnArbiter()
    c = cs[0]
    p = arb.resolve(idle(), cs, pose_at(c.start_s - 1.7), 0.4, True, 0.0)
    small = arb.resolve(now(DIRECTION_RIGHT), cs, pose_at(c.start_s - 1.6), 0.4, True, 0.0)
    assert small.direction == DIRECTION_LEFT and small.sequence_id == p.sequence_id
    r = arb.resolve(now(DIRECTION_RIGHT, angle=40.0), cs, pose_at(c.start_s - 1.5), 0.4, True, 0.0)
    assert r.direction == DIRECTION_RIGHT and r.sequence_id != p.sequence_id


def test_same_direction_next_corner_chains_without_gap():
    # 왼쪽 90° 두 개가 2 m 간격 — 첫 회전이 끝나는 tick 에 다음 예고가 서면 끊지 않는다
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
    # run69(10-08) 홈 도착: 레일 끝 2 m 안의 가짜 U턴 코너에 도착 정렬 회전이 묶여, 멈춘 뒤에도
    # '코너 끝까지 유지'가 풀리지 않아 왼쪽 신호가 63 s·180 s 남았다. 목적지 2 m 안 코너는
    # 예고처럼 묶지 않는다 — 회전이 끝나면 신호도 끝난다.
    cs = find_corners(l_path(first=5.0, second=1.0))
    assert len(cs) == 1
    c = cs[0]
    total = 6.0
    arb = RailTurnArbiter()
    on = arb.resolve(now(DIRECTION_LEFT), cs, pose_at(c.start_s, total), 0.0, False, 0.0)
    assert on.direction == DIRECTION_LEFT              # 실제 회전은 알린다
    arb.resolve(TurnDecision(DIRECTION_NONE, PHASE_COMPLETE, 20.0, 1, False),
                cs, pose_at(c.start_s, total), 0.0, False, 0.0)
    off = arb.resolve(idle(), cs, pose_at(c.start_s, total), 0.0, False, 0.0)
    assert off.direction == DIRECTION_NONE             # 멈췄으면 꺼진다


# ── 차선 옮김 거르기 (2026-10-08 run69) ────────────────

def test_lane_change_swerve_on_straight_rail_does_not_light():
    # 곧은 레일에서 달리며 장애물을 비키는 S자(21~33°)는 신호를 켜지 않는다. run69 에서
    # 짧은 깜빡임(0.8~1.3 s) 13번 중 약 9번이 이것이었다.
    path = line((0, 0), (10, 0))
    cs = find_corners(path)
    arb = RailTurnArbiter()
    for angle in (22.0, 28.0, 33.0):
        cue = arb.resolve(now(DIRECTION_LEFT, angle=angle), cs, pose_at(3.0), 0.4, True, 0.0)
        assert cue.direction == DIRECTION_NONE
    back = arb.resolve(now(DIRECTION_RIGHT, angle=25.0), cs, pose_at(3.3), 0.4, True, 0.0)
    assert back.direction == DIRECTION_NONE


def test_bigger_turn_while_moving_still_lights():
    # 계속 돌아 35° 를 넘으면(진짜 회전) 그 tick 에 켠다.
    cs = find_corners(line((0, 0), (10, 0)))
    arb = RailTurnArbiter()
    assert arb.resolve(now(DIRECTION_LEFT, angle=25.0), cs, pose_at(3.0), 0.4, True,
                       0.0).direction == DIRECTION_NONE
    cue = arb.resolve(now(DIRECTION_LEFT, angle=36.0), cs, pose_at(3.1), 0.4, True, 0.0)
    assert cue.direction == DIRECTION_LEFT and cue.phase == PHASE_NOW


def test_slow_turn_lights_at_the_usual_angle():
    # 제자리 회전·천천히 호를 그리는 출발 유턴(0.25 m/s 미만)은 지금처럼 20° 에서 켠다.
    cs = find_corners(line((0, 0), (10, 0)))
    for speed in (0.0, 0.15):
        arb = RailTurnArbiter()
        cue = arb.resolve(now(DIRECTION_LEFT, angle=22.0), cs, pose_at(3.0), speed, True, 0.0)
        assert cue.direction == DIRECTION_LEFT


def test_turn_at_a_rail_corner_lights_at_the_usual_angle():
    # 레일 코너에 묶이는 회전은 빠르게 달려도 20° 에서 켠다(코너 신호는 늦추지 않는다).
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
