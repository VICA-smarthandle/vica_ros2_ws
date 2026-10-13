"""방향지시 대조 시험 시나리오 생성기 — python3 turn_gen.py <seed> <ticks> [params.json] [options.json]

options: {"r0": <ROS 시계 시작 ns>} — sim time 0 이나 2038년 경계(sec int32) 같은 ROS 시계 끝값을 본다.

가상 로봇이 레일(코너·U턴·S자·같은 방향 연속 코너)을 따라 달린다. 출발 제자리 회전, 차선 옮김(비켜가기),
레일 이탈, 중간 정지, 도착 정렬, 목적지 바꾸기, /odom 끊김, map->odom 끊김(TF 실패), 레일 1 Hz 재전송
(로봇 앞 노드부터 잘라서), frame 이 다른·짧은·NaN 섞인 레일을 섞는다. 명령 형식은 turn_law_py.py 머리말.
"""
import json
import math
import random
import sys

T0 = 1_000_000_000_000          # 정상 시계 시작(1000 s)
R0 = 1_791_000_000_000_000_000  # ROS 시계 시작(2026-10)
DT = 0.01                       # 시뮬레이션 칸(10 ms)


def hx(s):
    return s.encode('utf-8').hex()


def fr(x):
    return repr(float(x))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Route:
    """레일 한 줄(0.05 m 간격 점)과 그 위 노드(약 1 m 간격)."""

    def __init__(self, rng, x, y, heading):
        pts = [(x, y)]
        h = heading
        last_ang = 0.0
        for k in range(rng.randint(2, 6)):
            straight = rng.choice([rng.uniform(0.6, 2.0), rng.uniform(2.0, 7.0), rng.uniform(0.2, 1.3)])
            ang = math.radians(rng.choice([
                90, -90, 90, -90, 45, -45, 60, -60, 120, -135, 170, -175, 180, 25, -30, 15, -50,
                rng.uniform(-180, 180)]))
            radius = rng.choice([0.2, 0.4, 0.6, 1.0, rng.uniform(0.15, 1.5)])
            if k > 0 and rng.random() < 0.3:
                # 같은 방향 코너가 합침 거리(1.2 m) 바로 너머에 이어진다 — 예고를 끄지 않고 넘겨주는 자리
                straight = rng.uniform(1.25, 1.7)
                ang = math.copysign(rng.uniform(math.radians(50), math.radians(95)), last_ang)
                radius = rng.choice([0.2, 0.3, 0.4])
            pts += self._line(pts[-1], h, straight)
            last_ang = ang
            arc, h = self._arc(pts[-1], h, ang, radius)
            pts += arc
        pts += self._line(pts[-1], h, rng.uniform(0.5, 4.0))
        if rng.random() < 0.15:   # 잔떨림
            pts = [(px + rng.uniform(-0.004, 0.004), py + rng.uniform(-0.004, 0.004)) for px, py in pts]
        self.pts = pts
        self.cum = [0.0]
        for i in range(1, len(pts)):
            self.cum.append(self.cum[-1] + math.dist(pts[i - 1], pts[i]))
        self.nodes = [0]
        for i, s in enumerate(self.cum):
            if s - self.cum[self.nodes[-1]] >= rng.choice([0.9, 1.0, 1.1]):
                self.nodes.append(i)

    @staticmethod
    def _line(p, h, length):
        n = max(1, int(length / 0.05))
        return [(p[0] + math.cos(h) * 0.05 * i, p[1] + math.sin(h) * 0.05 * i) for i in range(1, n + 1)]

    @staticmethod
    def _arc(p, h, ang, radius):
        n = max(1, int(abs(ang) * radius / 0.05))
        out = []
        x, y = p
        step = ang / n
        chord = 2 * radius * math.sin(abs(step) / 2)
        for _ in range(n):
            h2 = h + step / 2
            x, y = x + math.cos(h2) * chord, y + math.sin(h2) * chord
            h += step
            out.append((x, y))
        return out, h

    def length(self):
        return self.cum[-1]

    def at(self, s):
        s = max(0.0, min(s, self.cum[-1]))
        lo, hi = 0, len(self.cum) - 1
        while lo < hi - 1:
            mid = (lo + hi) // 2
            if self.cum[mid] <= s:
                lo = mid
            else:
                hi = mid
        (x0, y0), (x1, y1) = self.pts[lo], self.pts[hi]
        seg = self.cum[hi] - self.cum[lo]
        t = 0.0 if seg <= 0 else (s - self.cum[lo]) / seg
        return x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, math.atan2(y1 - y0, x1 - x0)

    def project(self, x, y, s_prev):
        best = None
        for i in range(len(self.pts) - 1):
            if self.cum[i] > s_prev + 3.0 or self.cum[i + 1] < s_prev - 1.0:
                continue
            (x0, y0), (x1, y1) = self.pts[i], self.pts[i + 1]
            dx, dy = x1 - x0, y1 - y0
            seg2 = dx * dx + dy * dy
            t = 0.0 if seg2 <= 0 else max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / seg2))
            d = math.hypot(x - x0 - dx * t, y - y0 - dy * t)
            if best is None or d < best[0]:
                best = (d, self.cum[i] + t * math.sqrt(seg2))
        return s_prev if best is None else best[1]

    def cut(self, s_robot, rng):
        """Route Server 처럼 로봇 앞 노드부터 끝까지(가끔 가장 가까운 노드부터)."""
        ahead = [i for i in self.nodes if self.cum[i] > s_robot]
        if not ahead or rng.random() < 0.15:
            near = min(self.nodes, key=lambda i: abs(self.cum[i] - s_robot))
            start = near
        else:
            start = ahead[0]
        return self.pts[start:]


class World:
    def __init__(self, seed, ticks, r0=R0):
        self.r0 = r0
        self.rng = random.Random(seed)
        self.ticks = ticks
        self.lines = []
        self.t = T0
        rng = self.rng
        self.x, self.y = rng.uniform(-5, 5), rng.uniform(-5, 5)
        self.th = rng.uniform(-math.pi, math.pi)
        self.v = 0.0
        self.w = 0.0
        # map -> odom (AMCL 보정)
        self.ox, self.oy, self.oth = rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-0.5, 0.5)
        self.odom_drift = 0.0
        self.route = None
        self.s = 0.0
        self.mode = 'wait'
        self.mode_until = 1.0 + rng.uniform(0, 2)
        self.vmax = 0.5
        self.offset_plan = []     # [(s0, s1, peak)] 차선 옮김·이탈
        self.final_yaw = 0.0
        self.next_odom = 0.0
        self.odom_gap_until = -1.0
        self.next_tf_map = 0.0
        self.tf_map_gap_until = -1.0
        self.tf_start = rng.choice([0.0, 0.0, 2.0, 6.0])   # TF 가 늦게 들어오는 기동
        self.next_rail = 0.0
        self.rail_quiet = False
        self.next_tick = 0.05
        self.last_stamp = {}
        self.no_map_tf = rng.random() < 0.05

    # ── 출력 ──
    def put(self, line):
        self.lines.append(line)

    def clock(self, sim_t):
        ns = T0 + int(round(sim_t * 1e9))
        self.put(f'T {ns}')
        self.put(f'R {self.r0 + (ns - T0)}')
        return self.r0 + (ns - T0)

    def stamp_tf(self, key, r_ns):
        last = self.last_stamp.get(key, -1)
        if r_ns <= last:
            r_ns = last + 1000
        self.last_stamp[key] = r_ns
        return r_ns // 1_000_000_000, r_ns % 1_000_000_000

    # ── 로봇 ──
    def offset_at(self, s):
        off = 0.0
        for s0, s1, peak in self.offset_plan:
            if s0 <= s <= s1:
                mid = (s0 + s1) / 2
                off += peak * (1 - abs(s - mid) / ((s1 - s0) / 2))
        return off

    def target_point(self, s):
        x, y, h = self.route.at(s)
        off = self.offset_at(s)
        return x - math.sin(h) * off, y + math.cos(h) * off, h

    def new_route(self):
        rng = self.rng
        h0 = self.th + rng.choice([0.0, 0.0, rng.uniform(-0.3, 0.3), rng.uniform(-math.pi, math.pi)])
        self.route = Route(rng, self.x, self.y, h0)
        self.s = 0.0
        self.vmax = rng.choice([0.4, 0.5, 0.5, 0.6, 0.6, 0.3, rng.uniform(0.15, 0.6)])
        self.offset_plan = []
        L = self.route.length()
        s = rng.uniform(1.0, 4.0)
        while s < L - 2.0:
            k = rng.random()
            if k < 0.35:      # 비켜가기(차선 옮김)
                span = rng.uniform(1.5, 4.0)
                self.offset_plan.append((s, s + span, rng.choice([-1, 1]) * rng.uniform(0.15, 0.5)))
            elif k < 0.45:    # 레일 이탈
                span = rng.uniform(2.0, 5.0)
                self.offset_plan.append((s, s + span, rng.choice([-1, 1]) * rng.uniform(0.9, 1.6)))
            s += rng.uniform(2.0, 8.0)
        self.final_yaw = rng.uniform(-math.pi, math.pi)
        self.mode = 'align_start'
        self.next_rail = 0.0
        self.rail_quiet = False

    def step(self, sim_t):
        rng = self.rng
        if self.mode == 'wait':
            self.v, self.w = 0.0, 0.0
            if sim_t >= self.mode_until:
                self.new_route()
        elif self.mode == 'pause':
            self.v = max(0.0, self.v - 0.5 * DT * 2)
            self.w = 0.0
            if sim_t >= self.mode_until:
                self.mode = 'drive'
        elif self.mode == 'align_start':
            tx, ty, _ = self.target_point(self.s + 0.6)
            err = wrap(math.atan2(ty - self.y, tx - self.x) - self.th)
            self.v = 0.0
            if abs(err) < 0.12:
                self.mode = 'drive'
                self.w = 0.0
            else:
                self.w = math.copysign(min(0.35, abs(err) * 2), err)
        elif self.mode == 'align_end':
            err = wrap(self.final_yaw - self.th)
            self.v = 0.0
            if abs(err) < 0.05:
                self.mode = 'wait'
                self.mode_until = sim_t + rng.uniform(1.0, 8.0)
                self.w = 0.0
            else:
                self.w = math.copysign(min(0.35, abs(err) * 2), err)
        elif self.mode == 'drive':
            self.s = self.route.project(self.x, self.y, self.s)
            remaining = self.route.length() - self.s
            if remaining < 0.12:
                self.mode = 'align_end'
                self.v = 0.0
            else:
                look = max(0.5, min(1.2, 0.5 + self.v))
                tx, ty, _ = self.target_point(self.s + look)
                alpha = wrap(math.atan2(ty - self.y, tx - self.x) - self.th)
                # 코너 바로 앞(0.6 m 안)의 굽은 정도로만 감속 — VCC 는 0.5 m/s 로 다가가 코너에서 돈다
                _, _, h1 = self.route.at(self.s + 0.1)
                _, _, h2 = self.route.at(self.s + 0.7)
                bend = abs(wrap(h2 - h1))
                vt = min(self.vmax, max(0.12, self.vmax * (1 - bend / 2.5)), max(0.1, remaining * 0.6))
                if abs(alpha) > 1.2:
                    vt = 0.0
                    self.w = math.copysign(0.35, alpha)
                else:
                    self.w = max(-0.7, min(0.7, 2 * max(self.v, 0.1) * math.sin(alpha) / look))
                self.v += max(-0.6 * DT, min(0.5 * DT, vt - self.v))
                if rng.random() < 0.0008:
                    self.mode = 'pause'
                    self.mode_until = sim_t + rng.uniform(0.5, 6.0)
                if rng.random() < 0.0002:
                    self.new_route()       # 목적지 바꾸기
        self.th = wrap(self.th + self.w * DT)
        self.x += self.v * math.cos(self.th) * DT
        self.y += self.v * math.sin(self.th) * DT
        # AMCL 보정: 천천히 걷다가 가끔 튄다
        self.ox += rng.gauss(0, 0.0004)
        self.oy += rng.gauss(0, 0.0004)
        self.oth += rng.gauss(0, 0.0002)
        if rng.random() < 0.002:
            self.ox += rng.uniform(-0.15, 0.15)
            self.oy += rng.uniform(-0.15, 0.15)
            self.oth += rng.uniform(-0.05, 0.05)
        self.odom_drift += rng.gauss(0, 0.00005)

    # ── 사건 ──
    def odom_pose(self):
        c, s = math.cos(self.oth), math.sin(self.oth)
        dx, dy = self.x - self.ox, self.y - self.oy
        return c * dx + s * dy, -s * dx + c * dy, wrap(self.th - self.oth)

    def emit_events(self, sim_t):
        rng = self.rng
        if sim_t >= self.next_odom:
            self.next_odom = sim_t + 1 / 30 + rng.uniform(-0.004, 0.004)
            if sim_t >= self.odom_gap_until:
                if rng.random() < 0.003:
                    self.odom_gap_until = sim_t + rng.choice([0.3, 0.6, 1.0, rng.uniform(0.2, 2.5)])
                r_ns = self.clock(sim_t)
                ox, oy, oyaw = self.odom_pose()
                yaw = oyaw + self.odom_drift
                vx = self.v + rng.gauss(0, 0.005)
                self.put(f'odom 0.0 0.0 {fr(math.sin(yaw / 2))} {fr(math.cos(yaw / 2))} {fr(vx)}')
                if sim_t >= self.tf_start:
                    sec, nsec = self.stamp_tf('ob', r_ns)
                    self.put(f'tf {hx("odom")} {hx("base_footprint")} {sec} {nsec} {fr(ox)} {fr(oy)} 0.0 '
                             f'0.0 0.0 {fr(math.sin(oyaw / 2))} {fr(math.cos(oyaw / 2))} 0')
        if sim_t >= self.next_tf_map and sim_t >= self.tf_start and not self.no_map_tf:
            self.next_tf_map = sim_t + rng.choice([0.1, 0.2, 0.5]) + rng.uniform(-0.01, 0.01)
            if sim_t >= self.tf_map_gap_until:
                if rng.random() < 0.004:
                    self.tf_map_gap_until = sim_t + rng.choice([2.0, 12.0, 15.0])
                r_ns = self.clock(sim_t)
                future = rng.choice([0, 200_000_000, 1_000_000_000])   # AMCL transform_tolerance
                sec, nsec = self.stamp_tf('mo', r_ns + future)
                self.put(f'tf {hx("map")} {hx("odom")} {sec} {nsec} {fr(self.ox)} {fr(self.oy)} 0.0 '
                         f'0.0 0.0 {fr(math.sin(self.oth / 2))} {fr(math.cos(self.oth / 2))} 0')
        if self.route is not None and self.mode in ('align_start', 'drive', 'pause', 'align_end') \
                and sim_t >= self.next_rail:
            self.next_rail = sim_t + 1.0 + rng.uniform(-0.05, 0.05)
            remaining = self.route.length() - self.s
            if remaining < 2.0 and rng.random() < 0.8:
                self.rail_quiet = True        # BT 가 목적지 2 m 안에서 레일을 버린다
            if not self.rail_quiet and rng.random() > 0.03:
                self.clock(sim_t)
                self.put(self.rail_line(self.route.cut(self.s, rng)))
        if sim_t >= self.next_tick:
            stall = rng.choice([0.0] * 30 + [0.03, 0.08, 0.15])
            self.next_tick += 0.05 + rng.uniform(-0.003, 0.003) + stall
            self.clock(sim_t)
            self.put('tick')
            return True
        return False

    def rail_line(self, pts):
        rng = self.rng
        frame = 'map'
        k = rng.random()
        if k < 0.08:
            frame = ''
        elif k < 0.10:
            frame = 'odom'
        if rng.random() < 0.02:
            pts = pts[:1]
        elif rng.random() < 0.01:
            pts = []
        elif rng.random() < 0.01 and len(pts) > 4:
            pts = list(pts)
            pts[rng.randrange(len(pts))] = (float('nan'), pts[0][1])
        elif rng.random() < 0.02 and len(pts) > 4:
            pts = list(pts)
            i = rng.randrange(len(pts) - 1)
            pts.insert(i, pts[i])     # 겹친 점(길이 0 선분)
        body = ' '.join(f'{fr(x)} {fr(y)}' for x, y in pts)
        return f'rail {hx(frame)} {len(pts)} {body}'.rstrip()

    def run(self):
        sim_t = 0.0
        ticks = 0
        while ticks < self.ticks:
            self.step(sim_t)
            if self.emit_events(sim_t):
                ticks += 1
            sim_t += DT
        self.put('idle')
        return self.lines


def main():
    seed, ticks = int(sys.argv[1]), int(sys.argv[2])
    params = json.loads(sys.argv[3]) if len(sys.argv) > 3 else {}
    options = json.loads(sys.argv[4]) if len(sys.argv) > 4 else {}
    r0 = int(options.get('r0', R0))
    w = World(seed, ticks, r0)
    lines = [f'P {json.dumps(params, ensure_ascii=False)}']
    lines.append(f'T {T0}')
    lines.append(f'R {r0}')
    lines.append('start')
    lines += w.run()
    sys.stdout.write('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
