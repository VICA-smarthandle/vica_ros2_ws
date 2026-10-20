#!/usr/bin/env python3
"""주행 중 장애물 안내 판정 — 순수 코드(ROS·스피커 없음)."""
from __future__ import annotations

import bisect
import json
import math
from collections import deque
from pathlib import Path
from typing import Optional

import numpy as np


FRONT = 0.305
HALF_W = 0.30
MAP_TOL = 0.15
N_MIN = 3
WINDOW = 0.5
STEP = 0.1
TOL = {"scan": 0.15, "depth": 0.25}
WAIT = 0.25
RAIL_FRESH = 2.5
RAIL_OK = 0.7
D_AHEAD = {"LAT": 2.0, "DET": 3.2, "DEC": 1.6, "HOLD": 1.6, "CM": 1.6}
LAT_M = 0.3
RESYNC_M = 0.15
TURN_QUIET = 3.0
DEC_WINDOW = 1.5
DEC_FROM = 0.35
DEC_RATIO = 0.5
STATE_GAP = 0.5
OBST_REASONS = ("lanes_blocked", "collision_imminent", "motion_collision")
GOAL_QUIET = 1.0
DET_MERGE = 8.0
CM_MIN_V = 0.10
MIN_GAP = 6.0
CALM_SEC = 2.0
CALM_V = 0.3
STOP_UPGRADE = 0.7
NEW_OBSTACLE_M = 1.0
NEW_OBSTACLE_GAP = 3.0
HISTORY_SEC = 12.0
FRAME_KEEP = 3.0

KIND_PHRASE = {"LAT": "avoid", "DET": "avoid", "DEC": "slow", "HOLD": "stop", "CM": "slow"}
PHRASES = {"avoid": "앞에 장애물이 있어 비켜갈게요.", "slow": "앞에 장애물이 있어 감속할게요.",
           "stop": "앞에 장애물이 있어 정지합니다."}
GOAL_START = ("goal_sent", "return_home_sent")
GOAL_END = ("goal_succeeded", "goal_canceled", "goal_failed", "goal_aborted", "goal_paused", "goal_rejected",
            "return_home_succeeded", "return_home_failed", "return_home_canceled", "return_home_aborted")


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


def route_frame(poly, px, py):
    """polyline(N×2) 기준 점들의 (앞 거리 s, 옆 거리 d, 왼쪽 +)."""
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
    """입력을 시간 순서로 받고 tick() 이 '결정된 행동'을 시작 시각 순서로 돌려준다."""

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
        self.last_ann_xy: Optional[tuple] = None
        self.calm_start: Optional[float] = None
        self.calm_done: Optional[float] = None
        self.trip_ended = False

    def clear_inputs(self) -> None:
        """입력을 끊었다 다시 받기 전에 지난 주행의 위치·점·VCC 기록·결정 대기를 지운다(2026-10-10 CPU ①)."""
        self.poses.clear()
        self.frames["scan"].clear()
        self.frames["depth"].clear()
        self.vcc.clear()
        self.pending = []
        self.dec_armed = True
        self.cm_prev = "normal"
        self.calm_start = None

    def on_dialog(self, t: float, state: str) -> None:
        self.dialog = state or ""

    def on_goal(self, t: float, event: str, x=None, y=None, loc: str = "") -> None:
        if event in GOAL_START:
            if self.trip_ended:
                self.last_ann = self.last_ann_xy = self.calm_start = self.calm_done = None
            self.trip_ended = False
            self.goal = {"x": x, "y": y, "loc": loc or ""}
        elif event in GOAL_END:
            self.goal = None
            self.trip_ended = event != "goal_paused"

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

        big = state == "TRACK" and abs(tgt) >= LAT_M
        was = joined and abs(float(prev[1].get("target", 0.0))) >= LAT_M and prev[1].get("state") in ("TRACK", "HOLD")
        if big and not was:
            self._onset("LAT", t, st, resync=abs(tgt - off) < RESYNC_M, after_turn=self._turned_within(t))

        vmax = self._vmax(t)
        if v >= DEC_FROM:
            self.dec_armed = True
        if (self.dec_armed and vmax >= DEC_FROM and v <= DEC_RATIO * vmax and state in ("TRACK", "HOLD")
                and (prev is None or joined)):
            self._onset("DEC", t, st, vmax=round(vmax, 3))
            self.dec_armed = False

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
        elif what == "stop" and self.cm_prev == "slow":
            self._onset("CM", t, None, what=what)
        if what in ("slow", "stop", "normal"):
            self.cm_prev = what

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
        found = item["found"]
        while found is None and item["k"] <= int(round(WINDOW / STEP)) and now >= t0 + item["k"] * STEP + WAIT - 1e-9:
            f = self._front(t0 + item["k"] * STEP, D)
            item["k"] += 1
            if f is not None and (item["best"] is None or f["n"] > item["best"]["n"]):
                item["best"] = f
            if eligible and f is not None and f["n"] >= N_MIN:
                found = item["found"] = f
                break
        window_over = item["k"] > int(round(WINDOW / STEP))
        if found is not None:
            merged = self._same_obstacle(t0, found)
            phrase = self._phrase(item)
            if not merged and phrase == "slow":
                if now < t0 + STOP_UPGRADE - 1e-9:
                    return False
                if self._stop_follows(item):
                    phrase = "stop"
            item.update(found)
            if merged:
                item["decision"] = "merged"
            else:
                item["decision"] = "announce"
                item["phrase"] = phrase
                self._reset_calm(t0)
                self.last_ann_xy = found.get("near_xy")
            return True
        if not window_over:
            return False
        if item["best"] is not None:
            item.update(item["best"])
        item["decision"] = "no_cause" if eligible else "excluded"
        return True

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
        if gd < GOAL_QUIET and not self._is_stop(kind, detail):
            why.append("near_goal")
        if kind == "CM":
            vs = self._vcc_at(t)
            detail["vcc_state"] = None if vs is None else vs.get("state")
            detail["vcc_v"] = None if vs is None else float(vs.get("v", 0.0))
            if vs is None or float(vs.get("v", 0.0)) < CM_MIN_V or vs.get("state") != "TRACK":
                why.append("cm_not_driving")
        if kind == "DET":
            vs = self._vcc_at(t)
            detail["vcc_state"] = None if vs is None else vs.get("state")
            detail["vcc_v"] = None if vs is None else float(vs.get("v", 0.0))
            if vs is None or float(vs.get("v", 0.0)) < CM_MIN_V or vs.get("state") != "TRACK":
                why.append("det_not_driving")
        if kind == "DEC" and st is not None and st.get("state") == "TRACK" and abs(float(st.get("target", 0.0))) >= LAT_M:
            why.append("dec_from_lane_shift")
        if st is not None:
            detail.update(state=st.get("state"), reason=st.get("reason"), target=float(st.get("target", 0.0)),
                          offset=float(st.get("offset", 0.0)), v=float(st.get("v", 0.0)))
        item = {"t": t, "kind": kind, "why": why, "goal_dist": round(gd, 2), "detail": detail,
                "decision": None, "phrase": None, "n": 0, "n_scan": 0, "n_depth": 0, "n_wall": 0,
                "near": None, "lat": None, "mode": None, "k": 0, "best": None, "found": None}
        ts = [p["t"] for p in self.pending]
        self.pending.insert(bisect.bisect_right(ts, t), item)

    @staticmethod
    def _is_stop(kind: str, detail: dict) -> bool:
        """실제로 선 행동인가 — VCC 장애물 정지·충돌감시 정지."""
        return kind == "HOLD" or (kind == "CM" and detail.get("what") == "stop")

    @classmethod
    def _phrase(cls, item: dict) -> str:
        if item["kind"] == "CM":
            return "stop" if cls._is_stop("CM", item["detail"]) else "slow"
        return KIND_PHRASE[item["kind"]]

    def _stop_follows(self, item: dict) -> bool:
        """이 감속과 같은 순간부터 STOP_UPGRADE 안에 뺄 이유 없는 정지(VCC 장애물 정지·충돌감시 정지)가 시작됐나."""
        t0 = item["t"]
        return any(p is not item and t0 - 1e-9 <= p["t"] <= t0 + STOP_UPGRADE + 1e-9 and not p["why"]
                   and self._phrase(p) == "stop" for p in self.pending)

    def _same_obstacle(self, t0: float, found: dict) -> bool:
        """'한 번만' — 지난번 말한 것과 같은 장애물이면 True(묶음)."""
        if self.last_ann is None:
            return False
        here, there = found.get("near_xy"), self.last_ann_xy
        if here is not None and there is not None and math.dist(here, there) >= NEW_OBSTACLE_M:
            return t0 - self.last_ann < NEW_OBSTACLE_GAP - 1e-9
        return (t0 - self.last_ann < MIN_GAP - 1e-9
                or self.calm_done is None or self.calm_done > t0)

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
                    res["near_xy"] = (float(X[m][new][k]), float(Y[m][new][k]))
        return res
