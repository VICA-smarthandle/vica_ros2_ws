#!/usr/bin/env python3
"""주행 중 장애물 안내 판정 — 순수 코드(ROS·스피커 없음).

설계서: 루트 docs/superpowers/specs/2026-10-08-obstacle-narration-design.md 3절(판정)·5절 2단계(미션에 넣기).

로봇의 행동(차선 옮김·경로 우회·급감속·장애물 정지·충돌감시)이 시작되면, 그 순간부터 0.5초 동안
가던 길 위에 '지도에 없는' 점이 3개 이상 있는지 본다. 있으면 한 번 말하고, 같은 장애물에는 다시
말하지 않는다. 숫자는 녹화본 10개(run60~69)로 맞췄고 시운전 run81(10-09)에서 16번 모두 실물이었다.

음성 저장소 scripts/obstacle_judge.py(1단계 점검 도구)를 2026-10-09 그대로 옮겼다 — 이제 이 파일이 정본이다.
미션 노드가 실시간으로 쓰고, 녹화본 재현은 1단계 도구(avoid_cue.py --bag)로 한다.
입력 시각은 모두 '받은 시각'(초)이다.
"""
from __future__ import annotations

import bisect
import json
import math
from collections import deque
from pathlib import Path
from typing import Optional

import numpy as np

# ------------------------------------------------------------------ 설계서 값 (3절)

FRONT = 0.305           # base_footprint 에서 차체 앞단까지(nav2_params footprint x +0.305)
HALF_W = 0.30           # 레일 양옆 — 몸 반폭 0.2275 + 여유. 0.25 면 run67 실제 우회를 놓친다
MAP_TOL = 0.15          # 지도 점유 칸 이 거리 안의 점은 벽·고정 구조물
N_MIN = 3               # 원인으로 칠 최소 점 수(라이다 한 장 + 깊이 띠 한 장 합)
WINDOW = 0.5            # 행동 시작부터 원인을 보는 시간
STEP = 0.1              # 그 안에서 보는 간격
TOL = {"scan": 0.15, "depth": 0.25}   # 그 순간과 한 장의 시각 차 허용
WAIT = 0.25             # 그 순간의 가장 가까운 한 장이 도착하기를 기다리는 시간
RAIL_FRESH = 2.5        # 레일이 이보다 오래되면 레일 주행이 아니다(turn_guide 와 같은 값)
RAIL_OK = 0.7           # 로봇이 레일에서 이보다 멀면 로봇 정면 직사각형으로 본다
D_AHEAD = {"LAT": 2.0, "DET": 3.2, "DEC": 1.6, "HOLD": 1.6, "CM": 1.6}
LAT_M = 0.3             # 차선 옮김: |target| 이 이만큼 넘으면 크게 비킨다
RESYNC_M = 0.15         # 시작 때 |target - offset| 이 이보다 작으면 유턴 뒤 자리 맞추기
TURN_QUIET = 3.0        # TURN·ALIGN 뒤 이 시간 안의 차선 옮김은 뺀다
DEC_WINDOW = 1.5        # 급감속: 이 시간 안 최대 속도에서
DEC_FROM = 0.35         #   이만큼 이상이던 것이
DEC_RATIO = 0.5         #   이 비율 이하로 떨어지면
STATE_GAP = 0.5         # /vcc/state 가 이만큼 끊기면 이어진 것으로 보지 않는다
OBST_REASONS = ("lanes_blocked", "collision_imminent", "motion_collision")
GOAL_QUIET = 1.0        # 목적지 이 거리 안에서는 말하지 않는다(시연장 크기 기준 출발값)
DET_MERGE = 8.0         # 우회 결정(FAILURE)이 이 시간 안에 다시 오면 같은 우회
CM_MIN_V = 0.10         # 충돌감시는 VCC 가 이 속도 이상 달릴 때만(회전·정지 중 제외)
MIN_GAP = 6.0           # 한 번 말하면 최소 이만큼
CALM_SEC = 2.0          # 그리고 평상 주행이 이만큼 이어진 뒤에만 다시 말한다
CALM_V = 0.3            # 평상 주행: TRACK·|target|<0.3·명령 속도 이 이상
HISTORY_SEC = 12.0      # VCC·자세 기록을 이만큼 둔다
FRAME_KEEP = 3.0        # 점 한 장을 이만큼 둔다

KIND_PHRASE = {"LAT": "avoid", "DET": "avoid", "DEC": "slow", "HOLD": "slow", "CM": "slow"}
PHRASES = {"avoid": "앞에 장애물이 있어 피해 갈게요.", "slow": "앞에 장애물이 있어 천천히 갈게요."}
GOAL_START = ("goal_sent", "return_home_sent")
GOAL_END = ("goal_succeeded", "goal_canceled", "goal_failed", "goal_aborted", "goal_paused", "goal_rejected",
            "return_home_succeeded", "return_home_failed", "return_home_canceled", "return_home_aborted")

# ------------------------------------------------------------------ 줄 읽기

_FLOATS = ("offset", "target", "v", "w")


def parse_state(text: str) -> Optional[dict]:
    """/vcc/state 한 줄을 dict 로. 모양이 다르면 None."""
    kv = dict(p.split("=", 1) for p in text.split() if "=" in p)
    if "state" not in kv or "target" not in kv:
        return None
    try:
        for k in _FLOATS:
            if k in kv:
                kv[k] = float(kv[k])
    except ValueError:
        return None
    return kv


def cm_what(msg: str) -> Optional[str]:
    """collision_monitor 로그 한 줄 → 'stop' | 'slow' | 'normal' | None."""
    if msg.startswith("Robot to stop"):
        return "stop"
    if msg.startswith("Robot to slowdown"):
        return "slow"
    if msg.startswith("Robot to continue"):
        return "normal"
    return None


def goal_event(text: str) -> Optional[dict]:
    """/vica_goal_event JSON → {event, x, y, loc, map_id}. 모양이 다르면 None."""
    try:
        j = json.loads(text)
    except (ValueError, TypeError):
        return None
    if not isinstance(j, dict) or "event" not in j:
        return None
    return {"event": j.get("event"), "x": j.get("x"), "y": j.get("y"),
            "loc": j.get("location_id") or j.get("destination_id") or "", "map_id": j.get("map_id")}


# ------------------------------------------------------------------ 레일 좌표·지도

def route_frame(poly, px, py):
    """polyline(N×2) 기준 점들의 (앞 거리 s, 옆 거리 d, 왼쪽 +).

    첫 구간은 뒤로, 끝 구간은 앞으로 늘인다 — 레일은 로봇보다 앞 마디부터 온다(run67 10:11:25).
    구간이 없으면 None.
    """
    poly = np.asarray(poly, dtype=float).reshape(-1, 2)
    a, b = poly[:-1], poly[1:]
    seg = b - a
    L = np.hypot(seg[:, 0], seg[:, 1])
    keep = L > 1e-6
    a, seg, L = a[keep], seg[keep], L[keep]
    if len(L) == 0:
        return None
    cum = np.concatenate([[0.0], np.cumsum(L)[:-1]])
    P = np.stack([np.asarray(px, float), np.asarray(py, float)], axis=1)[:, None, :]
    rel = P - a[None, :, :]
    u = (rel[..., 0] * seg[None, :, 0] + rel[..., 1] * seg[None, :, 1]) / (L[None, :] ** 2)
    lo = np.zeros_like(u)
    lo[:, 0] = -np.inf
    hi = np.ones_like(u)
    hi[:, -1] = np.inf
    uc = np.clip(u, lo, hi)
    qx = a[None, :, 0] + uc * seg[None, :, 0]
    qy = a[None, :, 1] + uc * seg[None, :, 1]
    dist = np.hypot(P[..., 0] - qx, P[..., 1] - qy)
    k = np.argmin(dist, axis=1)
    idx = np.arange(P.shape[0])
    s = cum[k] + uc[idx, k] * L[k]
    cross = seg[k, 0] * rel[idx, k, 1] - seg[k, 1] * rel[idx, k, 0]
    return s, np.sign(cross) * dist[idx, k]


def _read_pgm(path) -> np.ndarray:
    data = Path(path).read_bytes()
    parts, idx = [], 0
    while len(parts) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b"#":
            while data[idx:idx + 1] not in (b"\n", b""):
                idx += 1
            continue
        j = idx
        while not data[j:j + 1].isspace():
            j += 1
        parts.append(data[idx:j])
        idx = j
    if parts[0] != b"P5":
        raise ValueError(f"P5 pgm 만 읽는다: {path}")
    w, h = int(parts[1]), int(parts[2])
    return np.frombuffer(data[idx + 1:idx + 1 + w * h], dtype=np.uint8).reshape(h, w)


class MapGrid:
    """정지 지도의 '벽까지 거리'. occ 는 점유 칸(bool, 0번 줄 = 위쪽 = y 최대, pgm 순서)."""

    def __init__(self, occ, res: float, ox: float, oy: float):
        from scipy import ndimage
        occ = np.asarray(occ, dtype=bool)
        self.res, self.ox, self.oy = float(res), float(ox), float(oy)
        self.H, self.W = occ.shape
        self.dist = ndimage.distance_transform_edt(~occ) * self.res

    @classmethod
    def from_yaml(cls, path) -> "MapGrid":
        import yaml
        y = yaml.safe_load(open(path))
        img = _read_pgm(Path(path).parent / y["image"])
        p = (255.0 - img) / 255.0 if not y.get("negate", 0) else img / 255.0
        return cls(p >= y["occupied_thresh"], y["resolution"], y["origin"][0], y["origin"][1])

    @classmethod
    def from_occupancy(cls, data, width: int, height: int, res: float, ox: float, oy: float) -> "MapGrid":
        """nav_msgs/OccupancyGrid(0번 줄 = 아래). map_server 는 점유를 100 으로 준다."""
        g = np.asarray(data, dtype=np.int16).reshape(height, width)
        return cls(np.flipud(g >= 65), res, ox, oy)

    def wall_dist(self, X, Y) -> np.ndarray:
        X = np.asarray(X, float)
        Y = np.asarray(Y, float)
        j = np.floor((X - self.ox) / self.res).astype(int)
        i = self.H - 1 - np.floor((Y - self.oy) / self.res).astype(int)
        ok = (i >= 0) & (i < self.H) & (j >= 0) & (j < self.W)
        out = np.full(X.shape, 99.0)
        out[ok] = self.dist[i[ok], j[ok]]
        return out


# ------------------------------------------------------------------ 판정

def _nearest(items, t):
    """(시각, …) 목록에서 t 에 가장 가까운 것. 비면 None."""
    if not items:
        return None
    ts = [x[0] for x in items]
    i = bisect.bisect_left(ts, t)
    if i <= 0:
        return items[0]
    if i >= len(items):
        return items[-1]
    return items[i] if ts[i] - t < t - ts[i - 1] else items[i - 1]


class ObstacleJudge:
    """입력을 시간 순서로 받고 tick() 이 '결정된 행동'을 시작 시각 순서로 돌려준다.

    결정 하나 = dict(t, kind, decision, phrase, why, n, n_scan, n_depth, n_wall, near, lat, mode,
    goal_dist, detail, decided_at). decision: announce | merged | excluded | no_cause | no_map.
    """

    def __init__(self, grid: Optional[MapGrid] = None):
        self.grid = grid
        self.dialog = ""
        self.goal: Optional[dict] = None
        self.rails: deque = deque(maxlen=8)
        self.poses: deque = deque()
        self.frames = {"scan": deque(), "depth": deque()}
        self.vcc: deque = deque()
        self.pending: list = []
        self.dec_armed = True
        self.cm_prev = "normal"
        self.det_last = -1e9
        self.last_ann: Optional[float] = None
        self.calm_start: Optional[float] = None
        self.calm_done: Optional[float] = None

    # ---- 입력
    def on_dialog(self, t: float, state: str) -> None:
        self.dialog = state or ""

    def on_goal(self, t: float, event: str, x=None, y=None, loc: str = "") -> None:
        if event in GOAL_START:
            self.goal = {"x": x, "y": y, "loc": loc or ""}
        elif event in GOAL_END:
            self.goal = None

    def on_rail(self, t: float, xy) -> None:
        a = np.asarray(xy, dtype=float).reshape(-1, 2)
        if len(a) >= 2:
            self.rails.append((t, a))

    def on_pose(self, t: float, X: float, Y: float, A: float) -> None:
        self.poses.append((t, float(X), float(Y), float(A)))
        while self.poses and self.poses[0][0] < t - HISTORY_SEC:
            self.poses.popleft()

    def on_points(self, sensor: str, t: float, x, y) -> None:
        q = self.frames[sensor]
        q.append((t, np.asarray(x, dtype=float), np.asarray(y, dtype=float)))
        while q and q[0][0] < t - FRAME_KEEP:
            q.popleft()

    def on_vcc(self, t: float, st: dict) -> None:
        prev = self.vcc[-1] if self.vcc else None
        self.vcc.append((t, st))
        while len(self.vcc) > 2 and self.vcc[1][0] < t - HISTORY_SEC:
            self.vcc.popleft()
        state, reason = st.get("state", ""), st.get("reason", "")
        tgt, off, v = float(st.get("target", 0.0)), float(st.get("offset", 0.0)), float(st.get("v", 0.0))
        joined = prev is not None and t - prev[0] <= STATE_GAP

        # 차선 옮김: TRACK 에서 |target| 이 0.3 m 이상으로 올라선 순간
        big = state == "TRACK" and abs(tgt) >= LAT_M
        was = joined and abs(float(prev[1].get("target", 0.0))) >= LAT_M and prev[1].get("state") in ("TRACK", "HOLD")
        if big and not was:
            self._onset("LAT", t, st, resync=abs(tgt - off) < RESYNC_M, after_turn=self._turned_within(t))

        # 급감속: 1.5초 안 최대가 0.35 이상이었는데 지금 그 절반 이하
        vmax = self._vmax(t)
        if v >= DEC_FROM:
            self.dec_armed = True
        if (self.dec_armed and vmax >= DEC_FROM and v <= DEC_RATIO * vmax and state in ("TRACK", "HOLD")
                and (prev is None or joined)):
            self._onset("DEC", t, st, vmax=round(vmax, 3))
            self.dec_armed = False

        # 장애물 정지
        if state == "HOLD" and reason in OBST_REASONS and (prev is None or prev[1].get("state") != "HOLD" or not joined):
            self._onset("HOLD", t, st)

        self._calm_step(t, st, prev)

    def on_bt(self, t: float, node: str, status: str) -> None:
        if node != "IsRailAheadClear" or status != "FAILURE":
            return
        if t - self.det_last > DET_MERGE:
            self._onset("DET", t, None)
        self.det_last = t

    def on_cm(self, t: float, what: str) -> None:
        if what in ("slow", "stop") and self.cm_prev == "normal":
            self._onset("CM", t, None, what=what)
        if what in ("slow", "stop", "normal"):
            self.cm_prev = what

    # ---- 결정
    def tick(self, now: float) -> list:
        out = []
        while self.pending:
            item = self.pending[0]
            if not self._try_decide(item, now):
                break
            item["decided_at"] = now
            out.append(self.pending.pop(0))
        return out

    def _try_decide(self, item: dict, now: float) -> bool:
        if self.grid is None:
            item.update(decision="no_map")
            return True
        eligible = not item["why"]
        t0, D = item["t"], D_AHEAD[item["kind"]]
        found = None
        while item["k"] <= int(round(WINDOW / STEP)) and now >= t0 + item["k"] * STEP + WAIT - 1e-9:
            f = self._front(t0 + item["k"] * STEP, D)
            item["k"] += 1
            if f is not None and (item["best"] is None or f["n"] > item["best"]["n"]):
                item["best"] = f
            if eligible and f is not None and f["n"] >= N_MIN:
                found = f
                break
        window_over = item["k"] > int(round(WINDOW / STEP))
        if found is not None:
            item.update(found)
            if self.last_ann is not None and (t0 - self.last_ann < MIN_GAP - 1e-9
                                              or self.calm_done is None or self.calm_done > t0):
                item["decision"] = "merged"
            else:
                item["decision"] = "announce"
                item["phrase"] = KIND_PHRASE[item["kind"]]
                self._reset_calm(t0)
            return True
        if not window_over:
            return False
        if item["best"] is not None:
            item.update(item["best"])
        item["decision"] = "no_cause" if eligible else "excluded"
        return True

    # ---- 행동 시작
    def _onset(self, kind: str, t: float, st: Optional[dict], **detail) -> None:
        why = []
        if self.dialog != "navigating":
            why.append("not_guided")
        if kind == "LAT":
            if detail.get("resync"):
                why.append("resync")
            if detail.get("after_turn"):
                why.append("after_turn")
        gd = self._goal_dist(t)
        if gd < GOAL_QUIET:
            why.append("near_goal")
        if kind == "CM":
            vs = self._vcc_at(t)
            detail["vcc_state"] = None if vs is None else vs.get("state")
            detail["vcc_v"] = None if vs is None else float(vs.get("v", 0.0))
            if vs is None or float(vs.get("v", 0.0)) < CM_MIN_V or vs.get("state") != "TRACK":
                why.append("cm_not_driving")
        if kind == "DEC" and st is not None and st.get("state") == "TRACK" and abs(float(st.get("target", 0.0))) >= LAT_M:
            why.append("dec_from_lane_shift")
        if st is not None:
            detail.update(state=st.get("state"), reason=st.get("reason"), target=float(st.get("target", 0.0)),
                          offset=float(st.get("offset", 0.0)), v=float(st.get("v", 0.0)))
        item = {"t": t, "kind": kind, "why": why, "goal_dist": round(gd, 2), "detail": detail,
                "decision": None, "phrase": None, "n": 0, "n_scan": 0, "n_depth": 0, "n_wall": 0,
                "near": None, "lat": None, "mode": None, "k": 0, "best": None}
        ts = [p["t"] for p in self.pending]
        self.pending.insert(bisect.bisect_right(ts, t), item)

    def _turned_within(self, t: float) -> bool:
        """[t-3, t] 의 VCC 표본(그 직전 하나 포함)에 TURN·ALIGN 이 있었나."""
        items = list(self.vcc)
        start = 0
        for i, (ti, _) in enumerate(items):
            if ti <= t - TURN_QUIET:
                start = i
        return any(st.get("state") in ("TURN", "ALIGN") for ti, st in items[start:] if ti <= t)

    def _vmax(self, t: float) -> float:
        items = list(self.vcc)
        start = 0
        for i, (ti, _) in enumerate(items):
            if ti <= t - DEC_WINDOW:
                start = i
        vs = [float(st.get("v", 0.0)) for ti, st in items[start:] if ti <= t]
        return max(vs) if vs else 0.0

    def _vcc_at(self, t: float) -> Optional[dict]:
        best = None
        for ti, st in self.vcc:
            if ti <= t:
                best = (ti, st)
            else:
                break
        if best is None or t - best[0] > STATE_GAP:
            return None
        return best[1]

    def _goal_dist(self, t: float) -> float:
        P = self._pose_at(t)
        if P is None or self.goal is None or self.goal.get("x") is None:
            return 99.0
        return math.hypot(self.goal["x"] - P[1], self.goal["y"] - P[2])

    # ---- 한 번만: 말한 뒤 평상 주행이 2초 이어졌나
    def _calm_step(self, t: float, st: dict, prev) -> None:
        if self.last_ann is None or t <= self.last_ann:
            return
        ok = (st.get("state") == "TRACK" and abs(float(st.get("target", 0.0))) < LAT_M
              and float(st.get("v", 0.0)) >= CALM_V and (prev is None or t - prev[0] <= STATE_GAP))
        if ok:
            if self.calm_start is None:
                self.calm_start = t
            if self.calm_done is None and t - self.calm_start >= CALM_SEC - 1e-9:
                self.calm_done = t
        else:
            self.calm_start = None

    def _reset_calm(self, t0: float) -> None:
        self.last_ann, self.calm_start, self.calm_done = t0, None, None
        prev = None
        for ti, st in self.vcc:
            if ti > t0:
                self._calm_step(ti, st, prev)
            prev = (ti, st)

    # ---- 원인: 가던 길 위 '지도에 없는' 점
    def _pose_at(self, t: float):
        return _nearest(self.poses, t)

    def _rail_at(self, t: float):
        best = None
        for tr, xy in self.rails:
            if tr <= t:
                best = (tr, xy)
        if best is None or t - best[0] > RAIL_FRESH:
            return None
        return best[1]

    def _front(self, te: float, D: float) -> Optional[dict]:
        P = self._pose_at(te)
        if P is None:
            return None
        rail = self._rail_at(te)
        mode, s_robot = "straight", 0.0
        if rail is not None:
            rf = route_frame(rail, [P[1]], [P[2]])
            if rf is not None and abs(rf[1][0]) <= RAIL_OK:
                mode, s_robot = "rail", float(rf[0][0])
        res = {"mode": mode, "n": 0, "n_scan": 0, "n_depth": 0, "n_wall": 0, "near": None, "lat": None}
        best_along = math.inf
        for sensor in ("scan", "depth"):
            fr = _nearest(self.frames[sensor], te)
            if fr is None or abs(fr[0] - te) > TOL[sensor]:
                continue
            ts, x, y = fr
            P2 = self._pose_at(ts)
            if P2 is None or not len(x):
                continue
            c, s = math.cos(P2[3]), math.sin(P2[3])
            X, Y = P2[1] + c * x - s * y, P2[2] + s * x + c * y
            if mode == "rail":
                pre = (x > 0.0) & (x < D + 0.5) & (np.abs(y) < 1.5)
                if not pre.any():
                    continue
                X, Y = X[pre], Y[pre]
                sp, dp = route_frame(rail, X, Y)
                along, lat = sp - s_robot, dp
                m = (along >= FRONT) & (along <= D) & (np.abs(lat) <= HALF_W)
            else:
                along, lat = x, y
                m = (x >= FRONT) & (x <= D) & (np.abs(y) <= HALF_W)
            if not m.any():
                continue
            new = self.grid.wall_dist(X[m], Y[m]) > MAP_TOL
            res["n_wall"] += int((~new).sum())
            n = int(new.sum())
            res["n_" + sensor] += n
            res["n"] += n
            if n:
                a_new, l_new = along[m][new], lat[m][new]
                k = int(np.argmin(a_new))
                if a_new[k] < best_along:
                    best_along = float(a_new[k])
                    res["near"], res["lat"] = round(best_along, 2), round(float(l_new[k]), 2)
        return res
