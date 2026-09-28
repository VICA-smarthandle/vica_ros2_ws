"""레일(Route Server) 경로에서 큰 코너를 미리 찾아 예고한다 (순수 로직).

rclpy에 의존하지 않는다. 좌표는 호출자가 전부 같은 frame(map)으로 맞춰 넣는다.
이 계층은 어떤 구동 명령도 만들지 않고 경로를 바꾸지도 않는다. 읽기만 한다.

2026-09-28 합의(사용자):

    예고   레일에서 한 방향으로 45° 이상 꺾이는 코너만, 코너까지 2초 남았을 때
    사후   지금의 yaw 판정(turn_detector, 20°)은 그대로 둔다 — 레일 밖(장애물 회피·
           목적지 근처)의 변화는 지금처럼 돈 뒤에 알린다
    중복   예고한 코너를 실제로 돌 때 신호가 끊겼다 다시 켜지면 안 된다

[왜 /plan 이 아니라 레일인가] 2026-09-02 롤백한 예고는 `/plan`을 읽었다. `/plan`은
매초 백지에서 다시 계산되어 1초 간격 두 경로의 예고 방향이 57~73 % 어긋났다.
레일은 고정 그래프(geojson)이고 점수가 거리 하나뿐이라 같은 목적지면 같은 선이다.

[중복을 막는 세 규칙]
    ① 이어받기     예고 중 같은 방향 실제 회전이 오면 같은 번호로 이어 간다
    ② 코너 끝까지   예고는 시간이 아니라 **위치**로 붙잡는다. 코너 끝을 지나거나
                   레일을 벗어날 때만 내린다(옛 구현의 50 ms 틈 대책)
    ③ 알린 코너 기억 한 번 알린(또는 실제로 돈) 코너는 이번 goal 에서 다시 안 알린다.
                   레일이 1 Hz 로 다시 와도 코너 위치(map 좌표)로 같은 코너를 알아본다
"""

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

    direction: int          # DIRECTION_LEFT / DIRECTION_RIGHT
    angle_deg: float        # 코너 전체 꺾임. 부호: CCW(+) = LEFT
    start_s: float          # 꺾이기 시작하는 곳
    end_s: float            # 꺾임이 끝나는 곳
    start_xy: Point         # 코너 신원. 경로가 다시 와도 이 좌표로 같은 코너를 찾는다


@dataclass(frozen=True)
class RailPose:
    """로봇을 레일에 투영한 결과."""

    s: float                # 레일 위 위치(경로 첫 점부터)
    offtrack_m: float       # 레일선까지 거리
    remaining_m: float      # 레일 끝까지 남은 길이
    heading_rad: float = float("nan")   # 그 자리 레일 방향


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
) -> Optional[RailPose]:
    """로봇을 경로 선분에 수선으로 투영한다.

    ``s_hint``를 주면 그 근처 구간만 본다. 레일이 고리이거나 복도가 나란히 붙어 있으면
    전역 최근접점이 반대편 차선으로 튈 수 있어서다.
    """
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
        t = 0.0 if seg2 <= 0.0 else max(
            0.0, min(1.0, ((robot_xy[0] - x0) * dx + (robot_xy[1] - y0) * dy) / seg2))
        px, py = x0 + dx * t, y0 + dy * t
        d = math.hypot(robot_xy[0] - px, robot_xy[1] - py)
        if best is None or d < best[0]:
            best = (d, cum[i] + t * math.sqrt(seg2), math.atan2(dy, dx))
    if best is None:
        return None
    return RailPose(s=best[1], offtrack_m=best[0], remaining_m=cum[-1] - best[1],
                    heading_rad=best[2])


def _resample(path_xy: Sequence[Point], step_m: float) -> List[Tuple[float, Point]]:
    """경로를 step_m 간격으로 다시 찍는다. [(s, (x, y)), ...]

    레일 점은 path_density 0.05 m 간격이다. 이웃 두 점의 방향은 1 cm 오차에도 11°
    흔들리므로 간격을 넓혀 방향을 잰다(path_lookahead.resample_forward 와 같은 이유).
    """
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
    """경로 전체에서 한 방향으로 threshold_deg 이상 꺾이는 코너를 모두 찾는다.

    같은 부호 꺾임을 이어서 더한다. 부호가 바뀌면 코너가 끝난다 — S자에서 좌 20°와
    우 20°가 상쇄돼 '직진'이 되거나 합쳐져 40°가 되는 것을 막는다. 꺾임이 없는 짧은
    직선(데드밴드)은 코너를 끊지 않는다. 0630 레일의 90° 코너는 호가 여러 노드에
    나뉘어 있고 그 사이에 곧은 조각이 낀다.

    다만 곧은 구간이 ``max_gap_m`` 을 넘으면 코너를 닫는다. 09-28 오프라인 계산에서
    0903_d 복도의 1~2° 잔굽이가 22 m 에 걸쳐 한 '코너'로 이어져 신호가 57 초 켜져
    있었다. 0.6 m 는 0630 90° 코너 안의 곧은 조각(최대 약 0.4 m)보다 길다.

    그렇게 나뉜 **같은 방향** 조각이 ``merge_gap_m`` 안에 다시 이어지면 한 코너로
    합친다(문턱 검사는 합친 뒤). 0630 레일은 오른쪽 48°·83°·47° 처럼 같은 방향 꺾임이
    짧은 직선을 사이에 두고 이어져, 따로 알리면 신호가 꺼졌다 3초 안에 다시 켜졌다
    (두 번 알림). 1.2 m 는 0.4 m/s 로 3초 — 핸들이 '다른 코너'로 느끼기 전의 거리다.
    """
    pts = _resample(path_xy, step_m)
    heads = []
    for i in range(len(pts) - 1):
        (s0, (x0, y0)), (_, (x1, y1)) = pts[i], pts[i + 1]
        if math.hypot(x1 - x0, y1 - y0) > 1e-9:
            heads.append((s0, (x0, y0), math.atan2(y1 - y0, x1 - x0)))
    deadband = math.radians(deadband_deg)
    threshold = math.radians(threshold_deg)

    runs: List[List] = []   # [sign, acc, start_s, start_xy, end_s]
    sign, acc, start_s, start_xy, end_s = 0, 0.0, 0.0, (0.0, 0.0), 0.0
    last_turn_s = None

    def close():
        if sign != 0:
            runs.append([sign, acc, start_s, start_xy, end_s])

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
        end_s = s_b
        last_turn_s = s_b
    close()

    merged: List[List] = []
    for r in runs:
        if merged and merged[-1][0] == r[0] and r[2] - merged[-1][4] <= merge_gap_m:
            merged[-1][1] += r[1]
            merged[-1][4] = r[4]
        else:
            merged.append(list(r))
    return [
        RailCorner(direction=DIRECTION_LEFT if a > 0 else DIRECTION_RIGHT,
                   angle_deg=math.degrees(a), start_s=s0, end_s=s1, start_xy=xy)
        for _, a, s0, xy, s1 in merged if abs(a) >= threshold
    ]


@dataclass
class RailCue:
    """이 tick의 최종 cue. 노드는 이 값을 TurnGuide.msg로 옮기기만 한다."""

    direction: int
    phase: int
    distance_m: float       # PREPARE에서만 코너까지 거리. 그 밖엔 NaN
    turn_angle_deg: float
    sequence_id: int
    source_stale: bool = False
    reason: str = ""


@dataclass
class _Active:
    phase: int = PHASE_IDLE             # IDLE / PREPARE(아직 안 돎) / NOW(돌았거나 도는 중)
    direction: int = DIRECTION_NONE
    corner_xy: Optional[Point] = None   # 이 신호가 묶인 레일 코너. 없으면 레일 밖 회전


@dataclass
class RailTurnArbiter:
    """레일 예고(PREPARE)와 yaw 사후 판정(NOW)을 하나의 cue로 합친다.

    사실(yaw)이 예정(레일)을 이긴다. 예고 중 반대 방향으로 실제로 돌면 예고를 버리고
    그 회전을 새로 알린다(장애물 회피 — 사용자가 알아야 하는 변화다).

    신호 하나는 **코너 하나**에 묶인다. 켜진 신호는 다음 둘 중 하나가 참인 동안 꺼지지
    않는다 — 이것이 '두 번 알림'을 막는 규칙 ②다.

        코너 안   로봇이 그 코너 끝(+pass_margin)을 아직 안 지났다
        도는 중   yaw 판정의 창 누적이 같은 방향으로 exit 문턱 이상이다

    2026-09-28 오프라인 계산: 이 규칙 전에는 (a) 긴 코너(0630 −177°)를 yaw 판정이
    중간에 한 번 닫았다 다시 열어 같은 방향이 2.6 s 만에 재점등됐고(지금 방식에도 있는
    현상), (b) 코너 끝을 지나는 순간 예고가 내려간 직후 늦은 실제 회전이 켜져 50 ms 틈이
    생겼다.
    """

    lead_time_sec: float = 2.0
    min_speed_mps: float = 0.2      # 멈춰 있어도 나눗셈이 되게. 0.2 m/s면 2초 = 0.4 m
    max_offtrack_m: float = 0.8     # BT NearRailGate 와 같은 값 — 이 밖이면 레일 모드가 아니다
    handoff_m: float = 2.0          # BT handoff_dist_to_goal — 남은 레일이 이 안이면 새 예고 없음
    pass_margin_m: float = 0.3      # 코너 끝을 이만큼 지나면 신호를 내린다
    same_corner_m: float = 0.5      # 코너 신원 비교 반경
    consume_back_m: float = 1.0     # 예고 없이 돈 회전이 '어느 코너였나' 찾는 창(뒤)
    consume_ahead_m: float = 1.5    # (앞) — 컨트롤러는 조준거리만큼 코너를 일찍 돈다
    still_turning_deg: float = 10.0  # turn_detector exit_threshold_deg 와 같은 값

    # 로봇이 레일 방향과 이만큼 넘게 어긋나 있으면 새 예고를 안 한다. 출발 제자리 회전·
    # 레일 복귀 중이라 로봇이 레일을 '따라가는' 중이 아니다. 09-28 오프라인 계산에서
    # 출발 지점(목적지 방향 ≠ 레일 방향)의 어긋남이 코너로 잡혀, 예고는 왼쪽인데 실제
    # 제자리 회전은 오른쪽인 **반대 방향 예고**가 나왔다. 그 회전은 사후 판정 몫이다.
    max_heading_error_deg: float = 45.0

    # 이보다 느리면(사실상 정지) 새 예고를 안 한다. 출발 직후 로봇은 제자리에서 조준점
    # 쪽으로 먼저 돈다. 코너가 코앞(조준거리 안)에 있으면 그 제자리 회전이 레일 코너와
    # 반대 방향일 수 있다(09-28: 안내소 출발 129° 헤어핀 — 예고 왼쪽, 실제 오른쪽).
    min_moving_speed_mps: float = 0.05

    # 컨트롤러가 코너를 **얼마나 일찍 돌기 시작하나**. RPP 는 조준점(lookahead)이 코너에
    # 닿는 순간 틀기 시작하므로 몸이 도는 곳은 코너 시작보다 조준거리만큼 앞이다.
    # 이것을 빼면 "2초 전"이 실제로는 0.3초 전이 된다(09-28 오프라인 계산 중앙값).
    # 조준거리 = clamp(v x time, min, max) — nav2_params.yaml FollowPath 와 같은 값.
    # 컨트롤러를 바꾸면(VCC) 이 세 값도 그 컨트롤러의 조준 규칙으로 바꾼다.
    controller_lookahead_time_sec: float = 2.5
    controller_lookahead_min_m: float = 0.6
    controller_lookahead_max_m: float = 1.2

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

    def _lookahead(self, speed: float) -> float:
        return max(self.controller_lookahead_min_m,
                   min(self.controller_lookahead_max_m,
                       speed * self.controller_lookahead_time_sec))

    def _claim(self, corners, pose, direction) -> Optional[RailCorner]:
        """예고 없이 시작된 실제 회전이 레일 코너였다면 그 코너에 묶고 '알린 것'으로 친다.

        이게 없으면 컨트롤러가 코너를 일찍 돌아 NOW가 먼저 나온 뒤, 아직 앞에 남은 같은
        코너를 다시 예고해 '두 번 알림'이 된다(규칙 ③).
        """
        if pose is None or pose.offtrack_m > self.max_offtrack_m:
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

        on_rail = (rail_fresh and pose is not None
                   and pose.offtrack_m <= self.max_offtrack_m)
        a = self._active

        # ── 실제 회전(사실) ──
        if decision.phase == PHASE_NOW:
            if a.phase != PHASE_IDLE and a.direction == decision.direction:
                reason = "now_inherit" if a.phase == PHASE_PREPARE else "now"
                a.phase = PHASE_NOW                      # ① 이어받기: 번호 그대로
            else:
                self._seq += 1                           # 예고 없던(또는 반대) 회전
                c = self._claim(corners, pose, decision.direction)
                self._active = _Active(PHASE_NOW, decision.direction,
                                       c.start_xy if c else None)
                reason = "now_new"
            return RailCue(decision.direction, PHASE_NOW, nan,
                           decision.turn_angle_deg, self._seq, False, reason)

        # ── 켜진 신호 붙잡기(② 코너 끝까지 · 도는 중) ──
        if a.phase != PHASE_IDLE:
            corner = self._find(corners, a.corner_xy) if on_rail else None
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
            # 끝나는 바로 그 tick에 같은 방향 다음 코너 예고가 설 차례면 끄지 않고 넘겨준다.
            # 끄면 50 ms 꺼졌다 켜져 '두 번 알림'이 된다(09-28 오프라인 계산 0630: 14건).
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

        # ── 새 예고 ──
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
        # 목적지 2 m 안은 BT 가 레일을 버리고 planner 로 그린다. 그 안에 걸친 코너는 레일
        # 모양대로 돌지 않으므로(09-28: 예고 왼쪽 70° → 실제로는 도착 정렬 오른쪽) 알리지 않는다.
        rail_end_s = pose.s + pose.remaining_m - self.handoff_m
        v = max(speed_mps, self.min_speed_mps)
        body_turn_ahead = self._lookahead(speed_mps)
        # 아직 코너 한가운데(조준거리 앞 ~ 끝)면 다음 코너를 예고하지 않는다. 몸은 지금
        # 코너를 도는 중이라, 다음 코너가 반대 방향이면 '예고는 오른쪽, 실제는 왼쪽'이
        # 된다(09-28 0630 홈→화장실 0.5 m/s: +132° 헤어핀 안에서 −87° 예고).
        if any(c.start_s - body_turn_ahead < pose.s < c.end_s + self.pass_margin_m
               for c in corners):
            return None
        for c in corners:
            if c.end_s + self.pass_margin_m < pose.s:
                continue                                 # 이미 지난 코너
            if self._known(c.start_xy):
                continue                                 # ③ 알린 코너
            if (c.start_s - body_turn_ahead - pose.s) / v > self.lead_time_sec:
                return None                              # 가장 가까운 새 코너가 아직 멀다
            if c.start_s - pose.s < body_turn_ahead:
                # 이미 조준거리 안에 들어온 코너. 컨트롤러는 코너 너머 조준점을 보고 있어서
                # 날카로운 코너(헤어핀)를 레일과 **반대쪽으로** 짧게 돌 수 있다(09-28 0630
                # 안내소 출발 129°, 홈→화장실 132°). 늦은 예고는 예고 노릇도 못 한다.
                # 이 회전은 사후 판정이 알린다.
                continue
            if c.end_s > rail_end_s:
                return None
            return c
        return None

    def _start_prepare(self, c: RailCorner, pose: RailPose, reason: str) -> RailCue:
        self._seq += 1
        self._active = _Active(PHASE_PREPARE, c.direction, c.start_xy)
        self._remember(c.start_xy)
        return RailCue(c.direction, PHASE_PREPARE, max(0.0, c.start_s - pose.s),
                       c.angle_deg, self._seq, False, reason)
