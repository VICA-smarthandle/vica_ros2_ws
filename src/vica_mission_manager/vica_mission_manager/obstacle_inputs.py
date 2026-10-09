"""장애물 안내의 ROS 입력 바꾸기·지도 읽기·로그·보호막 — ROS 없이 시험되는 부분 (2026-10-09 2단계).

판정은 obstacle_judge.py, 말할지 마지막 판단은 MissionLogic.obstacle_cue, ROS 배선은 mission_manager_node 가
한다(설계서 2026-10-08-obstacle-narration-design.md 5절). 라이다 점 바꾸기·라벨은 음성 저장소
scripts/avoid_cue.py(1단계 점검 도구)의 것을 그대로 옮겼다.
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from .obstacle_judge import PHRASES, MapGrid

LASER_DEFAULT = (0.031, 0.0)   # base_footprint → laser_frame (tf_static 실측)
BASE_FRAME = "base_footprint"

KIND_KO = {"LAT": "차선 옮김", "DET": "경로 우회", "DEC": "급감속", "HOLD": "장애물 정지", "CM": "충돌감시"}
DECISION_KO = {"announce": "말함", "merged": "묶음(같은 장애물)", "excluded": "뺌", "no_cause": "원인 없음",
               "no_map": "지도 없음"}
WHY_KO = {"not_guided": "안내 주행 아님", "resync": "유턴 뒤 자리 맞추기", "after_turn": "유턴 직후",
          "near_goal": "목적지 1 m 안", "cm_not_driving": "회전·정지 중 충돌감시", "dec_from_lane_shift": "차선 옮김 감속"}


def scan_points(msg, offset=(0.0, 0.0)):
    """LaserScan → base_footprint 기준 (x, y). 앞 -0.3~4.0 m, 옆 ±1.5 m 만."""
    r = np.asarray(msg.ranges, dtype=float)
    a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
    ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
    x = offset[0] + r[ok] * np.cos(a[ok])
    y = offset[1] + r[ok] * np.sin(a[ok])
    keep = (x >= -0.3) & (x <= 4.0) & (np.abs(y) <= 1.5)
    return x[keep], y[keep]


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def depth_frame_ok(frame_id: str) -> bool:
    """depth_band_to_scan 은 base_footprint 로 낸다(10-08 확인). 다른 틀이면 쓰지 않는다."""
    return frame_id in ("", BASE_FRAME)


def load_grid(map_yaml: str) -> tuple[Optional[MapGrid], str]:
    """정지 지도를 읽는다 → (지도, "") 또는 (None, 이유). 못 읽으면 노드는 장애물 안내만 끈다."""
    if not map_yaml:
        return None, "map_yaml 이 비어 있습니다"
    path = Path(map_yaml).expanduser()
    try:
        return MapGrid.from_yaml(path), ""
    except Exception as exc:  # noqa: BLE001 — 어떤 이유든 안내만 끄고 미션은 뜬다
        return None, f"{path}: {exc!r}"


def format_decision(d: dict) -> str:
    """판정 한 건을 미션 로그 한 줄로(1단계 기록 CSV 와 같은 칸)."""
    kind = KIND_KO.get(d["kind"], d["kind"])
    dec = DECISION_KO.get(d["decision"], d["decision"])
    why = "·".join(WHY_KO.get(w, w) for w in d.get("why", []))
    say = f" '{PHRASES[d['phrase']]}'" if d.get("phrase") else ""
    tail = f" ({why})" if why else ""
    where = "" if d.get("near") is None else f" 앞 {d['near']:.2f} m 옆 {d['lat']:+.2f} m"
    return (f"{kind} → {dec}{say}{tail} · 점 {d['n']}(라이다 {d['n_scan']}·깊이 {d['n_depth']}·벽 {d['n_wall']})"
            f"{where} · 목적지 {d['goal_dist']} m")


def take_all(q) -> list:
    """deque 에 쌓인 것을 다 꺼낸다. 다른 스레드가 같은 deque 를 비워도 IndexError 로 죽지 않는다
    (2026-10-09 최종 검토 I-1: 대화 줄 _tick 의 예외는 MultiThreadedExecutor 가 다시 던져 미션을 끝낸다)."""
    out = []
    while True:
        try:
            out.append(q.popleft())
        except IndexError:
            return out


class Guard:
    """장애물 안내 콜백 보호막 — 예외가 한 번이라도 나면 안내를 끄고 미션은 계속 돈다
    (2026-10-09 사용자 요구: 오류가 나면 장애물 안내만 꺼지고 안내 주행은 계속)."""

    def __init__(self, on_error: Callable[[BaseException], None]):
        self.enabled = True
        self._on_error = on_error

    def wrap(self, fn: Callable) -> Callable:
        def run(*args, **kwargs):
            if not self.enabled:
                return None
            try:
                return fn(*args, **kwargs)
            except Exception as exc:  # noqa: BLE001
                self.enabled = False
                self._on_error(exc)
                return None
        return run
