"""정답지: 11-17·18 실기 도구 dock_approach_test.py v2.1(sha1 2824aeec1a45)의 V자 인식 부분을 글자 그대로 옮긴 것."""
import math

import numpy as np

LASER_X, LASER_Y, LASER_YAW = 0.031, 0.0, -0.0346
BUMPER_X, HALF_W = 0.151, 0.225
ROT_CLEAR = 0.72
REF_2008 = 0.3129
ACTIVE = ('ROTATE_TO_STAGE', 'DRIVE_TO_STAGE', 'ALIGN', 'APPROACH')


def fit_line(P):
    c = P.mean(0)
    _, _, vt = np.linalg.svd(P - c)
    d = vt[0]
    n = np.array([-d[1], d[0]])
    return c, d, n, float(np.sqrt(np.mean(((P - c) @ n) ** 2)))


def split_merge(P, lo, hi, thr, out):
    """P[lo:hi] 를 끝점 직선에서 가장 먼 점으로 나눈다(재귀)."""
    if hi - lo < 4:
        out.append((lo, hi)); return
    a, b = P[lo], P[hi - 1]
    ab = b - a; L = np.linalg.norm(ab)
    if L < 1e-6:
        out.append((lo, hi)); return
    n = np.array([-ab[1], ab[0]]) / L
    dist = np.abs((P[lo:hi] - a) @ n)
    k = int(np.argmax(dist))
    if dist[k] > thr and 2 <= k <= hi - lo - 3:
        split_merge(P, lo, lo + k + 1, thr, out)
        split_merge(P, lo + k, hi, thr, out)
    else:
        out.append((lo, hi))


class VDetector:
    """한 스캔에서 V자(내각 ~120°, 팔 ~0.20 m, 벌어진 쪽이 로봇)를 찾는다. 결과는 base_footprint 좌표."""

    def __init__(self, inner=120.0, arm=0.20, max_range=3.0, fov_deg=75.0):
        self.inner, self.arm, self.max_range, self.fov = inner, arm, max_range, math.radians(fov_deg)
        self.last = None

    def __call__(self, ranges, amin, inc, rmin):
        r = np.asarray(ranges, dtype=float)
        a = amin + np.arange(len(r)) * inc
        ok = np.isfinite(r) & (r >= max(rmin, 0.16)) & (r <= self.max_range) & (np.abs(a) <= self.fov)
        idx = np.where(ok)[0]
        if len(idx) < 12:
            return None
        pl = np.c_[r[idx] * np.cos(a[idx]), r[idx] * np.sin(a[idx])]
        c, s = math.cos(LASER_YAW), math.sin(LASER_YAW)
        P = np.c_[LASER_X + c * pl[:, 0] - s * pl[:, 1], LASER_Y + s * pl[:, 0] + c * pl[:, 1]]
        rr = r[idx]
        clusters, start = [], 0
        for k in range(1, len(idx) + 1):
            if k == len(idx) or idx[k] - idx[k - 1] > 12 or \
               np.linalg.norm(P[k] - P[k - 1]) > max(0.05, 3.0 * rr[k] * inc * (idx[k] - idx[k - 1])):
                if k - start >= 8:
                    clusters.append((start, k))
                start = k
        best = None
        for lo, hi in clusters:
            segs = []
            split_merge(P, lo, hi, 0.008 + 0.004 * float(np.median(rr[lo:hi])), segs)
            merged = True
            while merged and len(segs) > 1:
                merged = False
                for j in range(len(segs) - 1):
                    (p0, p1), (q0, q1) = segs[j], segs[j + 1]
                    if p1 - p0 >= 3 and q1 - q0 >= 3:
                        _, d1, _, _ = fit_line(P[p0:p1]); _, d2, _, _ = fit_line(P[q0:q1])
                        if abs(float(d1 @ d2)) > math.cos(math.radians(12)):
                            segs[j:j + 2] = [(p0, q1)]; merged = True; break
            for (a0, a1), (b0, b1) in zip(segs, segs[1:]):
                if a1 - a0 < 5 or b1 - b0 < 5:
                    continue
                A, B = P[a0:a1], P[b0:b1]
                ca, da, na, ra = fit_line(A[1:-1] if len(A) > 6 else A)
                cb, db, nb, rb = fit_line(B[1:-1] if len(B) > 6 else B)
                M = np.array([na, nb])
                if abs(np.linalg.det(M)) < 1e-3:
                    continue
                apex = np.linalg.solve(M, np.array([na @ ca, nb @ cb]))
                da = da if da @ (A[0] - apex) > 0 else -da
                db = db if db @ (B[-1] - apex) > 0 else -db
                inner = math.degrees(math.acos(float(np.clip(da @ db, -1, 1))))
                la, lb = float((A[0] - apex) @ da), float((B[-1] - apex) @ db)
                u = da + db
                if np.linalg.norm(u) < 1e-6:
                    continue
                u = u / np.linalg.norm(u)
                if float(u @ (-apex)) <= 0 or abs(inner - self.inner) > 15 or not (0.10 <= la <= 0.30) or not (0.10 <= lb <= 0.30):
                    continue
                if np.linalg.norm(apex - (A[-1] + B[0]) / 2) > 0.06:
                    continue
                score = abs(inner - self.inner) + 50 * (abs(la - self.arm) + abs(lb - self.arm)) + 200 * max(ra, rb)
                if self.last is not None:
                    score += 20 * float(np.linalg.norm(apex - self.last))
                if best is None or score < best['score']:
                    best = dict(score=score, apex=apex, u=u, inner=inner, la=la, lb=lb, res=max(ra, rb),
                                na=a1 - a0, nb=b1 - b0, da=da, db=db)
        if best is not None:
            self.last = best['apex']
        return best


def pose_in_dock(det):
    """로봇(구동축)을 V자 좌표로: (축거리, 좌우 왼쪽+, 방향 반시계+)."""
    a, u = det['apex'], det['u']
    fwd = -u
    left = np.array([-fwd[1], fwd[0]])
    p = -a
    return float(p @ u), float(p @ left), -math.atan2(fwd[1], fwd[0])
