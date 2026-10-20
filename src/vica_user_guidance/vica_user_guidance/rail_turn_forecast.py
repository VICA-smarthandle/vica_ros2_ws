"""레일(Route Server) 경로에서 큰 코너를 미리 찾아 예고한다 (순수 로직)."""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from .turn_detector import (
    DIRECTION_LEFT,
    DIRECTION_NONE,
    DIRECTION_RIGHT,
    PHASE_CANCELED,
    PHASE_COMPLETE,
    PHASE_IDLE,
    PHASE_NOW,
    PHASE_PREPARE,
    TurnDecision,
    normalize_angle,
)

Point = Tuple[float, float]


@dataclass(frozen=True)
class RailCorner:
    """레일 위 한 코너. 거리 s 는 경로 첫 점부터 잰 길이다."""

    direction: int
    angle_deg: float
    start_s: float
    end_s: float
    start_xy: Point
    end_xy: Point = (float("nan"), float("nan"))


@dataclass(frozen=True)
class RailPose:
    """로봇을 레일에 투영한 결과."""

    s: float
    offtrack_m: float
    remaining_m: float
    heading_rad: float = float("nan")


def cumulative_lengths(path_xy: Sequence[Point]) -> List[float]:
    """각 점까지의 누적 길이."""
    out = [0.0]
    for i in range(1, len(path_xy)):
        out.append(out[-1] + math.dist(path_xy[i - 1], path_xy[i]))
    return out


def project_to_path(
    path_xy: Sequence[Point],
    robot_xy: Point,
    cum: Optional[List[float]] = None,
    s_hint: Optional[float] = None,
    hint_back_m: float = 1.0,
    hint_ahead_m: float = 3.0,
    behind_start_m: float = 0.0,
) -> Optional[RailPose]:
    """로봇을 경로 선분에 수선으로 투영한다."""
    if len(path_xy) < 2:
        return None
    if cum is None:
        cum = cumulative_lengths(path_xy)
    best = None
    for i in range(len(path_xy) - 1):
        if s_hint is not None and (
            cum[i + 1] < s_hint - hint_back_m or cum[i] > s_hint + hint_ahead_m
        ):
            continue
        (x0, y0), (x1, y1) = path_xy[i], path_xy[i + 1]
        dx, dy = x1 - x0, y1 - y0
        seg2 = dx * dx + dy * dy
        raw = 0.0 if seg2 <= 0.0 else (
            (robot_xy[0] - x0) * dx + (robot_xy[1] - y0) * dy) / seg2
        seg = math.sqrt(seg2)
        if cum[i] <= 0.0 and seg > 0.0 and raw < 0.0 and -raw * seg <= behind_start_m:
            d = abs((robot_xy[0] - x0) * dy - (robot_xy[1] - y0) * dx) / seg
            s = raw * seg
        else:
            t = max(0.0, min(1.0, raw))
            px, py = x0 + dx * t, y0 + dy * t
            d = math.hypot(robot_xy[0] - px, robot_xy[1] - py)
            s = cum[i] + t * seg
        if best is None or d < best[0]:
            best = (d, s, math.atan2(dy, dx))
    if best is None:
        return None
    return RailPose(s=best[1], offtrack_m=best[0], remaining_m=cum[-1] - best[1],
                    heading_rad=best[2])


def _resample(path_xy: Sequence[Point], step_m: float) -> List[Tuple[float, Point]]:
    """경로를 step_m 간격으로 다시 찍는다. [(s, (x, y)), ...]"""
    if not path_xy:
        return []
    out = [(0.0, path_xy[0])]
    traveled, next_mark, prev = 0.0, step_m, path_xy[0]
    for cur in path_xy[1:]:
        seg = math.dist(prev, cur)
        if seg <= 0.0:
            continue
        while traveled + seg >= next_mark:
            r = (next_mark - traveled) / seg
            out.append((next_mark, (prev[0] + (cur[0] - prev[0]) * r,
                                    prev[1] + (cur[1] - prev[1]) * r)))
            next_mark += step_m
        traveled += seg
        prev = cur
    if traveled - out[-1][0] > 1e-6:
        out.append((traveled, prev))
    return out


def find_corners(
    path_xy: Sequence[Point],
    threshold_deg: float = 45.0,
    step_m: float = 0.2,
    deadband_deg: float = 2.0,
    max_gap_m: float = 0.6,
    merge_gap_m: float = 1.2,
) -> List[RailCorner]:
    """경로 전체에서 한 방향으로 threshold_deg 이상 꺾이는 코너를 모두 찾는다."""
    pts = _resample(path_xy, step_m)
    heads = []
    for i in range(len(pts) - 1):
        (s0, (x0, y0)), (_, (x1, y1)) = pts[i], pts[i + 1]
        if math.hypot(x1 - x0, y1 - y0) > 1e-9:
            heads.append((s0, (x0, y0), math.atan2(y1 - y0, x1 - x0)))
    deadband = math.radians(deadband_deg)
    threshold = math.radians(threshold_deg)

    runs: List[List] = []
    sign, acc, start_s, start_xy, end_s, end_xy = 0, 0.0, 0.0, (0.0, 0.0), 0.0, (0.0, 0.0)
    last_turn_s = None

    def close():
        if sign != 0:
            runs.append([sign, acc, start_s, start_xy, end_s, end_xy])

    for i in range(len(heads) - 1):
        s_b, xy_b, h_to = heads[i + 1]
        d = normalize_angle(h_to - heads[i][2])
        if abs(d) <= deadband:
            continue
        cur = 1 if d > 0 else -1
        if cur == sign and last_turn_s is not None and s_b - last_turn_s <= max_gap_m:
            acc += d
        else:
            close()
            sign, acc, start_s, start_xy = cur, d, s_b, xy_b
        end_s, end_xy = s_b, xy_b
        last_turn_s = s_b
    close()

    merged: List[List] = []
    for r in runs:
        if merged and merged[-1][0] == r[0] and r[2] - merged[-1][4] <= merge_gap_m:
            merged[-1][1] += r[1]
            merged[-1][4] = r[4]
            merged[-1][5] = r[5]
        else:
            merged.append(list(r))
    return [
        RailCorner(direction=DIRECTION_LEFT if a > 0 else DIRECTION_RIGHT,
                   angle_deg=math.degrees(a), start_s=s0, end_s=s1, start_xy=xy, end_xy=xy1)
        for _, a, s0, xy, s1, xy1 in merged if abs(a) >= threshold
    ]


@dataclass
class RailCue:
    """이 tick의 최종 cue. 노드는 이 값을 TurnGuide.msg로 옮기기만 한다."""

    direction: int
    phase: int
    distance_m: float
    turn_angle_deg: float
    sequence_id: int
    source_stale: bool = False
    reason: str = ""


@dataclass
class _Active:
    phase: int = PHASE_IDLE
    direction: int = DIRECTION_NONE
    corner_xy: Optional[Point] = None
    corner_end_xy: Optional[Point] = None


@dataclass
class RailTurnArbiter:
    """레일 예고(PREPARE)와 yaw 사후 판정(NOW)을 하나의 cue로 합친다."""

    lead_time_sec: float = 2.0
    min_speed_mps: float = 0.2
    max_offtrack_m: float = 0.8
    handoff_m: float = 2.0
    pass_margin_m: float = 0.3
    same_corner_m: float = 0.5
    consume_back_m: float = 1.0
    consume_ahead_m: float = 1.5
    still_turning_deg: float = 10.0

    max_heading_error_deg: float = 45.0

    min_moving_speed_mps: float = 0.05

    controller_lookahead_time_sec: float = 2.5
    controller_lookahead_min_m: float = 0.6
    controller_lookahead_max_m: float = 1.2

    minor_turn_deg: float = 35.0
    minor_turn_min_speed_mps: float = 0.25

    min_prepare_dist_m: float = 0.0
    rehold_end_m: float = 0.0

    _seq: int = 0
    _active: _Active = field(default_factory=_Active)
    _announced: List[Point] = field(default_factory=list)

    def reset_goal(self) -> None:
        """새 goal. 알린 코너 기억을 비운다."""
        self._announced = []
        self._active = _Active()

    def _known(self, xy: Point) -> bool:
        return any(math.dist(xy, a) <= self.same_corner_m for a in self._announced)

    def _remember(self, xy: Point) -> None:
        if not self._known(xy):
            self._announced.append(xy)

    def _find(self, corners, xy) -> Optional[RailCorner]:
        if xy is None:
            return None
        for c in corners:
            if math.dist(c.start_xy, xy) <= self.same_corner_m:
                return c
        return None

    def _refind(self, corners, pose, direction, end_xy) -> Optional[RailCorner]:
        """붙잡은 코너를 시작점으로 못 찾을 때, 끝점이 같은 같은 방향 코너(잘린 레일의 그 코너 나머지)."""
        if self.rehold_end_m <= 0.0 or pose is None or end_xy is None:
            return None
        if pose.remaining_m <= self.handoff_m:
            return None
        for c in corners:
            if (c.direction == direction and math.dist(c.end_xy, end_xy) <= self.rehold_end_m
                    and pose.s <= c.end_s + self.pass_margin_m):
                return c
        return None

    def _lookahead(self, speed: float) -> float:
        return max(self.controller_lookahead_min_m,
                   min(self.controller_lookahead_max_m,
                       speed * self.controller_lookahead_time_sec))

    def _minor_swerve(self, decision: TurnDecision, speed_mps: float) -> bool:
        """달리는 중의 작은 회전(차선 옮김)이라 아직 신호를 켜지 않을 것인가."""
        if self.minor_turn_deg <= 0.0 or speed_mps < self.minor_turn_min_speed_mps:
            return False
        angle = decision.turn_angle_deg
        return not math.isnan(angle) and abs(angle) < self.minor_turn_deg

    def _claim(self, corners, pose, direction) -> Optional[RailCorner]:
        """예고 없이 시작된 실제 회전이 레일 코너였다면 그 코너에 묶고 '알린 것'으로 친다."""
        if pose is None or pose.offtrack_m > self.max_offtrack_m:
            return None
        if pose.remaining_m <= self.handoff_m:
            return None
        for c in corners:
            if c.direction != direction:
                continue
            if (c.start_s <= pose.s <= c.end_s
                    or pose.s - self.consume_back_m <= c.start_s <= pose.s + self.consume_ahead_m):
                self._remember(c.start_xy)
                return c
        return None

    def resolve(
        self,
        decision: TurnDecision,
        corners: Sequence[RailCorner],
        pose: Optional[RailPose],
        speed_mps: float,
        rail_fresh: bool,
        robot_yaw_rad: Optional[float] = None,
    ) -> RailCue:
        nan = float("nan")
        if decision.source_stale:
            self._active = _Active()
            return RailCue(DIRECTION_NONE, PHASE_IDLE, nan, nan, self._seq, True, "stale")

        near_rail = pose is not None and pose.offtrack_m <= self.max_offtrack_m
        on_rail = rail_fresh and near_rail
        a = self._active

        if decision.phase == PHASE_NOW:
            if a.phase != PHASE_IDLE and a.direction == decision.direction:
                reason = "now_inherit" if a.phase == PHASE_PREPARE else "now"
                a.phase = PHASE_NOW
                return RailCue(decision.direction, PHASE_NOW, nan,
                               decision.turn_angle_deg, self._seq, False, reason)
            c = self._claim(corners, pose, decision.direction)
            if c is not None or not self._minor_swerve(decision, speed_mps):
                self._seq += 1
                self._active = _Active(PHASE_NOW, decision.direction,
                                       c.start_xy if c else None, c.end_xy if c else None)
                return RailCue(decision.direction, PHASE_NOW, nan,
                               decision.turn_angle_deg, self._seq, False, "now_new")

        if a.phase != PHASE_IDLE:
            corner = self._find(corners, a.corner_xy) if near_rail else None
            if corner is None and near_rail and a.corner_xy is not None:
                corner = self._refind(corners, pose, a.direction, a.corner_end_xy)
                if corner is not None:
                    a.corner_xy = corner.start_xy
                    self._remember(corner.start_xy)
            if corner is not None:
                a.corner_end_xy = corner.end_xy
            in_corner = corner is not None and pose.s <= corner.end_s + self.pass_margin_m
            sign = 1 if a.direction == DIRECTION_LEFT else -1
            turning = (not math.isnan(decision.turn_angle_deg)
                       and decision.turn_angle_deg * sign >= self.still_turning_deg)
            if in_corner or turning:
                dist = max(0.0, corner.start_s - pose.s) if (
                    corner is not None and a.phase == PHASE_PREPARE) else nan
                return RailCue(a.direction, a.phase, dist,
                               corner.angle_deg if corner else decision.turn_angle_deg,
                               self._seq, False,
                               "prepare_hold" if a.phase == PHASE_PREPARE else "now_hold")
            nxt = self._next_prepare(corners, pose, speed_mps, on_rail, robot_yaw_rad)
            if nxt is not None and nxt.direction == a.direction:
                return self._start_prepare(nxt, pose, "prepare_chain")
            was_prepare = a.phase == PHASE_PREPARE
            self._active = _Active()
            if was_prepare:
                return RailCue(DIRECTION_NONE, PHASE_CANCELED, nan, nan, self._seq, False,
                               "prepare_passed" if corner is not None else "prepare_off_rail")
            return RailCue(DIRECTION_NONE, PHASE_COMPLETE, nan,
                           decision.turn_angle_deg, self._seq, False, "complete")

        nxt = self._next_prepare(corners, pose, speed_mps, on_rail, robot_yaw_rad)
        if nxt is not None:
            return self._start_prepare(nxt, pose, "prepare")

        return RailCue(DIRECTION_NONE, PHASE_IDLE, nan, 0.0, self._seq, False, "idle")

    def _next_prepare(self, corners, pose, speed_mps, on_rail, robot_yaw_rad):
        """지금 예고를 시작할 코너가 있으면 돌려준다. 없으면 None."""
        if not on_rail or pose.remaining_m <= self.handoff_m:
            return None
        if speed_mps < self.min_moving_speed_mps:
            return None
        if (robot_yaw_rad is not None and not math.isnan(pose.heading_rad)
                and abs(normalize_angle(robot_yaw_rad - pose.heading_rad))
                > math.radians(self.max_heading_error_deg)):
            return None
        rail_end_s = pose.s + pose.remaining_m - self.handoff_m
        v = max(speed_mps, self.min_speed_mps)
        body_turn_ahead = self._lookahead(speed_mps)
        near_ahead = max(body_turn_ahead, self.min_prepare_dist_m)
        if any(c.start_s - near_ahead < pose.s < c.end_s + self.pass_margin_m
               for c in corners):
            return None
        for c in corners:
            if c.end_s + self.pass_margin_m < pose.s:
                continue
            if self._known(c.start_xy):
                continue
            if (c.start_s - body_turn_ahead - pose.s) / v > self.lead_time_sec:
                return None
            if c.start_s - pose.s < max(body_turn_ahead, self.min_prepare_dist_m):
                continue
            if c.end_s > rail_end_s:
                return None
            return c
        return None

    def _start_prepare(self, c: RailCorner, pose: RailPose, reason: str) -> RailCue:
        self._seq += 1
        self._active = _Active(PHASE_PREPARE, c.direction, c.start_xy, c.end_xy)
        self._remember(c.start_xy)
        return RailCue(c.direction, PHASE_PREPARE, max(0.0, c.start_s - pose.s),
                       c.angle_deg, self._seq, False, reason)
