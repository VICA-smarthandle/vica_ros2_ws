#!/usr/bin/env python3
"""레일 회전 예고 오프라인 계산 — 로봇 없이 지도·목적지만으로 숫자를 낸다.

2026-09-28. 방향지시등 사전예고를 레일 기준으로 다시 켜기 전에, 실제 코드
(``vica_user_guidance.rail_turn_forecast`` + ``turn_detector``)를 그대로 돌려
"예고가 몇 번, 몇 초 전에 나오고, 두 번 알림이 있는가"를 먼저 본다.

무엇을 흉내 내나
    레일     maps/<지도>_route.geojson 을 Dijkstra(거리 점수 = route_server
             DistanceScorer)로 목적지 쌍마다 잇는다. 같은 목적지면 같은 선이다.
    주행     RPP 근사. 조준거리 = clamp(v x 2.5, 0.6, 1.2), 조준각 > 1.3 rad 면
             제자리 0.35 rad/s, 곡률 반경 < 1.2 m 면 감속(최저 0.12), w 상한 0.5.
             값은 nav2_params.yaml FollowPath(test_Route_Server) 에서 가져왔다.
    도착     레일 끝에서 목적지 yaw 로 제자리 정렬(허용 14°).
    판정     /odom 30 Hz -> TurnDetector(20°, 1.5 s, 0.6 s) 20 Hz, 예고 20 Hz.

흉내 내지 않는 것 (한계)
    장애물·사람, 당근 모드, 목적지 2 m 안 planner 경로 모양, 위치추정 흔들림,
    knob 속도 변화. 그래서 이 숫자는 "레일이 깨끗할 때의 상한"이다.

사용
    python3 scripts/vica_rail_turn_offline.py                 # 0630·0903_d 둘 다
    python3 scripts/vica_rail_turn_offline.py --maps vica_map_0630 --speeds 0.4 -v
"""

import argparse
import heapq
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import yaml

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "src" / "vica_user_guidance"))

from vica_user_guidance.rail_turn_forecast import (  # noqa: E402
    RailTurnArbiter,
    cumulative_lengths,
    find_corners,
    project_to_path,
)
from vica_user_guidance.turn_detector import (  # noqa: E402
    DIRECTION_NONE,
    PHASE_NOW,
    TurnDetector,
)

NS = 1_000_000_000

# RPP (test_Route_Server nav2_params.yaml FollowPath)
LOOKAHEAD_TIME = 2.5
MIN_LOOKAHEAD, MAX_LOOKAHEAD = 0.6, 1.2
ROTATE_MIN_ANGLE = 1.3
ROTATE_W = 0.35
REG_MIN_RADIUS, REG_MIN_SPEED = 1.2, 0.12
MAX_W = 0.5                 # velocity_smoother max_velocity[2]
ACC_V, ACC_W = 1.0, 1.2
YAW_TOL = math.radians(14.0)
HANDOFF_M = 2.0

DUP_WINDOW_SEC = 3.0


# ── 레일 ──────────────────────────────────────────────

def load_graph(path):
    g = json.loads(Path(path).read_text(encoding="utf-8"))
    pos, adj = {}, defaultdict(list)
    for f in g["features"]:
        if f["geometry"]["type"] == "Point":
            pos[f["properties"]["id"]] = tuple(f["geometry"]["coordinates"][:2])
    for f in g["features"]:
        if f["geometry"]["type"] != "Point":
            a, b = f["properties"]["startid"], f["properties"]["endid"]
            adj[a].append(b)
    return pos, adj


def nearest_node(pos, xy):
    return min(pos, key=lambda n: math.dist(pos[n], xy))


def route(pos, adj, a, b):
    dist, prev, pq = {a: 0.0}, {}, [(0.0, a)]
    while pq:
        d, n = heapq.heappop(pq)
        if n == b:
            break
        if d > dist.get(n, 1e18):
            continue
        for m in adj[n]:
            nd = d + math.dist(pos[n], pos[m])
            if nd < dist.get(m, 1e18):
                dist[m], prev[m] = nd, n
                heapq.heappush(pq, (nd, m))
    if b not in dist:
        return None
    out = [b]
    while out[-1] != a:
        out.append(prev[out[-1]])
    return [pos[n] for n in reversed(out)]


def densify(pts, step=0.05):
    out = [pts[0]]
    for p, q in zip(pts, pts[1:]):
        n = max(1, int(math.dist(p, q) / step))
        for k in range(1, n + 1):
            out.append((p[0] + (q[0] - p[0]) * k / n, p[1] + (q[1] - p[1]) * k / n))
    return out


def point_at(path, cum, s):
    s = max(0.0, min(cum[-1], s))
    lo, hi = 0, len(cum) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if cum[mid] <= s:
            lo = mid
        else:
            hi = mid
    seg = cum[hi] - cum[lo]
    r = 0.0 if seg <= 0 else (s - cum[lo]) / seg
    return (path[lo][0] + (path[hi][0] - path[lo][0]) * r,
            path[lo][1] + (path[hi][1] - path[lo][1]) * r)


def load_destinations(name):
    root = Path.home() / "vica_data" / "destinations" / name
    out = []
    data = yaml.safe_load((root / "destinations.yaml").read_text(encoding="utf-8")) or {}
    for d in data.get("destinations", []):
        po = d.get("pose") or {}
        if po.get("x") is not None:
            out.append((d.get("name", "?"), float(po["x"]), float(po["y"]),
                        math.radians(float(po.get("yaw", 0.0)))))
    home = root / "home.yaml"
    if home.exists():
        h = (yaml.safe_load(home.read_text(encoding="utf-8")) or {}).get("pose") or {}
        if h.get("x") is not None:
            out.append(("홈", float(h["x"]), float(h["y"]), math.radians(float(h.get("yaw", 0.0)))))
    return out


# ── 한 구간 주행 ─────────────────────────────────────

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def approach(cur, target, rate, dt):
    step = rate * dt
    return cur + max(-step, min(step, target - cur))


def drive_leg(rail, start, goal_yaw, v_nom, dt=0.01):
    cum = cumulative_lengths(rail)
    corners = find_corners(rail)
    det = TurnDetector(int(1.5 * NS), math.radians(20.0), math.radians(10.0),
                       int(0.6 * NS), int(0.5 * NS))
    det_only = TurnDetector(int(1.5 * NS), math.radians(20.0), math.radians(10.0),
                            int(0.6 * NS), int(0.5 * NS))
    arb = RailTurnArbiter()
    x, y, th = start
    v = w = 0.0
    s_hint = 0.0
    travel = 0.0
    t = 0.0
    k = 0
    log = []          # (t, cue, pose, v, w, zone)
    base = []         # 기존 방식(detector 단독) 방향 기록
    aligning = False
    v_cmd = w_cmd = 0.0
    while t < 400.0:
        now = int(t * NS)
        if k % 3 == 0:
            det.add_odom(th, now)
            det_only.add_odom(th, now)
        if k % 5 == 0:
            pose = project_to_path(rail, (x, y), cum, s_hint)
            s_hint = pose.s
            # 제어 (RPP 근사)
            if not aligning and pose.remaining_m < 0.05:
                aligning = True
            if aligning:
                err = wrap(goal_yaw - th)
                if abs(err) < YAW_TOL and abs(w) < 0.05 and abs(v) < 0.03:
                    break
                v_cmd, w_cmd = 0.0, (math.copysign(ROTATE_W, err) if abs(err) >= YAW_TOL else 0.0)
            else:
                L = max(MIN_LOOKAHEAD, min(MAX_LOOKAHEAD, abs(v) * LOOKAHEAD_TIME))
                cx, cy = point_at(rail, cum, pose.s + L)
                dx, dy = cx - x, cy - y
                lx = math.cos(th) * dx + math.sin(th) * dy
                ly = -math.sin(th) * dx + math.cos(th) * dy
                ang = math.atan2(ly, lx)
                d2 = lx * lx + ly * ly
                if abs(ang) > ROTATE_MIN_ANGLE:
                    v_cmd, w_cmd = 0.0, math.copysign(ROTATE_W, ang)
                elif d2 < 1e-6:
                    v_cmd, w_cmd = 0.0, 0.0
                else:
                    kappa = 2.0 * ly / d2
                    v_cmd = v_nom
                    if abs(kappa) > 1e-6 and 1.0 / abs(kappa) < REG_MIN_RADIUS:
                        v_cmd = max(v_nom * (1.0 / abs(kappa)) / REG_MIN_RADIUS, REG_MIN_SPEED)
                    v_cmd = min(v_cmd, max(pose.remaining_m, 0.05) / 0.5)   # 끝에서 서서히
                    w_cmd = max(-MAX_W, min(MAX_W, v_cmd * kappa))
            dec = det.evaluate(now)
            dec0 = det_only.evaluate(now)
            cue = arb.resolve(dec, corners, pose, abs(v), True, th)
            if aligning or pose.remaining_m <= HANDOFF_M:
                zone = "goal"
            elif travel < 0.3:
                zone = "start"
            elif pose.offtrack_m > 0.8:
                zone = "off"
            else:
                zone = "rail"
            log.append((t, cue, pose, v, w, zone))
            base.append((t, dec0.direction if dec0.phase == PHASE_NOW else DIRECTION_NONE))
        v = approach(v, v_cmd, ACC_V, dt)
        w = approach(w, w_cmd, ACC_W, dt)
        th = wrap(th + w * dt)
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        travel += abs(v) * dt
        t += dt
        k += 1
    return corners, cum[-1], log, base, t


# ── 지표 ─────────────────────────────────────────────

def segments(timeline):
    """[(t, dir, s)] -> [(dir, t_on, t_off, s_on, s_off)]"""
    out, cur, t_on, s_on = [], DIRECTION_NONE, None, None
    for t, d, s in timeline:
        if d != cur:
            if cur != DIRECTION_NONE:
                out.append((cur, t_on, t, s_on, s))
            cur, t_on, s_on = d, t, s
    if cur != DIRECTION_NONE:
        out.append((cur, t_on, timeline[-1][0], s_on, timeline[-1][2]))
    return out


SAME_TURN_M = 1.2   # rail_turn_forecast.find_corners merge_gap_m 과 같은 값


def dup_count(segs):
    """같은 방향 신호가 끊겼다 DUP_WINDOW_SEC 안에 다시 켜진 횟수를 둘로 나눈다.

    진짜 중복   다시 켜진 곳이 앞 신호가 꺼진 곳에서 SAME_TURN_M 안 — 같은 회전을 두 번
    다른 코너   그보다 멀다 — 같은 방향 코너가 연달아 있어 회전이 실제로 두 번이다
    """
    same, other = 0, 0
    for (d0, _, off0, _, s_off0), (d1, on1, _, s_on1, _) in zip(segs, segs[1:]):
        if d0 == d1 and on1 - off0 <= DUP_WINDOW_SEC:
            if s_on1 - s_off0 < SAME_TURN_M:
                same += 1
            else:
                other += 1
    return same, other


def analyze(corners, total, log, base):
    m = defaultdict(float)
    ev = []
    m["big_corners"] = sum(1 for c in corners if c.start_s < total - HANDOFF_M)
    prepare_t = {}
    for i, (t, cue, pose, v, w, zone) in enumerate(log):
        if cue.reason == "prepare":
            m["prepare"] += 1
            prepare_t[cue.sequence_id] = (t, cue.direction, cue.distance_m, cue.turn_angle_deg)
        elif cue.reason == "now_inherit":
            m["inherit"] += 1
            if cue.sequence_id in prepare_t:
                t0, d, dist, ang = prepare_t[cue.sequence_id]
                # 실제로 돌기 시작한 때: 예고 뒤 같은 방향 w 가 0.15 rad/s 를 처음 넘은 순간
                sign = 1 if ang > 0 else -1
                t_turn = next((tt for tt, _, _, _, ww, _ in log
                               if tt >= t0 and ww * sign > 0.15), t)
                ev.append(("lead", t - t0, t_turn - t0, dist, ang))
        elif cue.reason == "now_new":
            m["now_new_" + zone] += 1
            if i > 0 and log[i - 1][1].reason in ("prepare", "prepare_hold") \
                    and log[i - 1][1].direction != cue.direction:
                m["wrong_dir"] += 1
                ev.append(("wrong", t))
        elif cue.reason in ("prepare_passed", "prepare_off_rail"):
            m["cancel"] += 1
            ev.append(("cancel", t, cue.reason))
    # 예고는 코너보다 앞에서 켜지므로 위치를 '예고한 코너 시작점'으로 잡는다.
    new_segs = segments([
        (t, cue.direction,
         pose.s + cue.distance_m if cue.reason in ("prepare", "prepare_chain") else pose.s)
        for t, cue, pose, *_ in log])
    old_segs = segments([(t, d, log[i][2].s) for i, (t, d) in enumerate(base)])
    m["dup_new"], m["seq_new"] = dup_count(new_segs)
    m["dup_old"], m["seq_old"] = dup_count(old_segs)
    m["signals_new"] = len(new_segs)
    m["signals_old"] = len(old_segs)
    m["flip_new"] = sum(1 for a, b in zip(new_segs, new_segs[1:]) if a[0] != b[0] and b[1] - a[2] < 0.1)
    return m, ev


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--maps", nargs="+", default=["vica_map_0630", "vica_map_0903_d"])
    ap.add_argument("--speeds", nargs="+", type=float, default=[0.3, 0.4, 0.5])
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()

    for name in a.maps:
        pos, adj = load_graph(WS / "maps" / f"{name}_route.geojson")
        dests = load_destinations(name)
        print(f"\n══ {name}: 레일 노드 {len(pos)} · 목적지 {len(dests)} ══")
        for v_nom in a.speeds:
            tot, leads, cancels, legs, drive_t = defaultdict(float), [], [], 0, 0.0
            for sn, sx, sy, syaw in dests:
                for gn, gx, gy, gyaw in dests:
                    if sn == gn:
                        continue
                    na, nb = nearest_node(pos, (sx, sy)), nearest_node(pos, (gx, gy))
                    pts = route(pos, adj, na, nb)
                    if pts is None or len(pts) < 2:
                        continue
                    # route_server 경로는 출발 노드에서 시작한다(use_start=false). 로봇 자리는 안 넣는다.
                    rail = densify(pts)
                    corners, total, log, base, t_end = drive_leg(rail, (sx, sy, syaw), gyaw, v_nom)
                    m, ev = analyze(corners, total, log, base)
                    legs += 1
                    drive_t += t_end
                    for key, val in m.items():
                        tot[key] += val
                    leads += [e for e in ev if e[0] == "lead"]
                    cancels += [(sn, gn) + e[1:] for e in ev if e[0] == "cancel"]
                    if a.verbose:
                        print(f"  {sn}->{gn} {total:5.1f} m 코너{int(m['big_corners'])} "
                              f"예고{int(m['prepare'])} 이어받기{int(m['inherit'])} "
                              f"사후(레일){int(m['now_new_rail'])} 사후(출발){int(m['now_new_start'])} "
                              f"사후(도착){int(m['now_new_goal'])} 취소{int(m['cancel'])} "
                              f"중복 새{int(m['dup_new'])}/옛{int(m['dup_old'])} 반대{int(m['wrong_dir'])} "
                              f"코너각 {[round(c.angle_deg) for c in corners]}")
            print(f"\n  속도 {v_nom} m/s · 구간 {legs} · 총 주행 {drive_t / 60:.1f} 분")
            print(f"    레일 큰 코너(≥45°, 목적지 2 m 밖)  {int(tot['big_corners'])}")
            print(f"    예고 {int(tot['prepare'])}  →  실제 회전이 이어받음 {int(tot['inherit'])}  ·  "
                  f"예고만 하고 끝남(취소) {int(tot['cancel'])}")
            print(f"    예고 없는 사후 신호: 레일 {int(tot['now_new_rail'])} · 출발 {int(tot['now_new_start'])} · "
                  f"도착 근처 {int(tot['now_new_goal'])} · 레일 밖 {int(tot['now_new_off'])}")
            print(f"    신호 켜진 횟수  새 방식 {int(tot['signals_new'])}  vs  지금 방식 {int(tot['signals_old'])}")
            print(f"    같은 회전을 두 번 알림(진짜 중복)  새 방식 {int(tot['dup_new'])}  vs  "
                  f"지금 방식 {int(tot['dup_old'])}")
            print(f"    같은 방향 다른 코너가 3초 안에 연달아  새 방식 {int(tot['seq_new'])}  vs  "
                  f"지금 방식 {int(tot['seq_old'])}")
            print(f"    예고와 반대로 실제 회전  {int(tot['wrong_dir'])}")
            print(f"    좌우 즉시 넘어감(반대 코너가 붙어 있음)  {int(tot['flip_new'])}")
            if leads:
                a_now = sorted(e[1] for e in leads)
                a_turn = sorted(e[2] for e in leads)
                med = lambda xs: xs[len(xs) // 2]
                print(f"    예고 → 몸이 돌기 시작   중앙 {med(a_turn):.2f} s (최소 {a_turn[0]:.2f} · 최대 {a_turn[-1]:.2f})")
                print(f"    예고 → 지금 방식 신호   중앙 {med(a_now):.2f} s (최소 {a_now[0]:.2f} · 최대 {a_now[-1]:.2f})"
                      f"  = 지금보다 이만큼 먼저")
            if cancels and a.verbose:
                for c in cancels:
                    print(f"      취소: {c}")


if __name__ == "__main__":
    main()
