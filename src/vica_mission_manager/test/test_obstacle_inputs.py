"""장애물 안내의 입력 바꾸기·지도 읽기·로그·보호막 (2026-10-09 2단계, 설계서 5절).

라이다 점 시험 셋은 음성 저장소 tests/test_avoid_cue.py 의 TestScanPoints 를 옮겼다.
"""
import math
from types import SimpleNamespace

import pytest

from vica_mission_manager.obstacle_inputs import (
    Guard, depth_frame_ok, format_decision, load_grid, scan_points, take_all, yaw_of,
)


def scan(ranges, angle_min=-math.pi / 2, inc=math.pi / 2, rmin=0.15, rmax=12.0):
    return SimpleNamespace(ranges=ranges, angle_min=angle_min, angle_increment=inc, range_min=rmin, range_max=rmax)


class TestScanPoints:
    def test_points_in_robot_frame_with_laser_offset(self):
        # 오른쪽(-90°) 1 m, 앞(0°) 2 m, 왼쪽(+90°) 1 m — 라이다는 차체 기준 x +0.031
        x, y = scan_points(scan([1.0, 2.0, 1.0]), offset=(0.031, 0.0))
        assert x == pytest.approx([0.031, 2.031, 0.031], abs=1e-9)
        assert y == pytest.approx([-1.0, 0.0, 1.0], abs=1e-9)

    def test_drops_invalid_and_far_points(self):
        x, y = scan_points(scan([float("inf"), 0.1, 5.0], angle_min=0.0, inc=0.1))
        assert len(x) == 0   # inf·최소 거리 미만·앞 4 m 밖

    def test_keeps_only_box_around_robot(self):
        x, y = scan_points(scan([2.0, 2.0], angle_min=math.pi / 2, inc=math.pi / 2))
        assert len(x) == 0   # 옆 2 m(상자 ±1.5 m 밖)와 뒤 2 m(상자 -0.3 m 밖)


def test_yaw_of_quarter_turn():
    q = SimpleNamespace(x=0.0, y=0.0, z=math.sin(math.pi / 4), w=math.cos(math.pi / 4))
    assert yaw_of(q) == pytest.approx(math.pi / 2)


def test_depth_frame_ok_only_for_the_robot_frame():
    assert depth_frame_ok("") and depth_frame_ok("base_footprint")
    assert not depth_frame_ok("camera_link")


def _write_map(tmp_path, rows):
    """rows: 위쪽 줄부터, '#' = 벽. 칸 0.1 m, 원점 (0, 0)."""
    h, w = len(rows), len(rows[0])
    pix = bytes(0 if c == "#" else 254 for row in rows for c in row)
    (tmp_path / "m.pgm").write_bytes(f"P5\n{w} {h}\n255\n".encode() + pix)
    (tmp_path / "m.yaml").write_text(
        "image: m.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.196\n", encoding="utf-8")
    return tmp_path / "m.yaml"


def test_load_grid_reads_the_map(tmp_path):
    grid, why = load_grid(str(_write_map(tmp_path, ["#...", "....", "...."])))
    assert why == "" and grid is not None
    assert grid.wall_dist([0.05], [0.25])[0] == pytest.approx(0.0)              # 맨 윗줄 왼쪽 칸 = 벽
    assert grid.wall_dist([0.35], [0.05])[0] == pytest.approx(0.36, abs=0.01)   # 대각선 3·2칸


def test_load_grid_without_a_path_turns_narration_off():
    grid, why = load_grid("")
    assert grid is None and "map_yaml" in why


def test_load_grid_with_a_broken_path_turns_narration_off(tmp_path):
    grid, why = load_grid(str(tmp_path / "없는지도.yaml"))
    assert grid is None and "없는지도.yaml" in why


def _decision(**kw):
    d = {"t": 1.0, "kind": "LAT", "decision": "announce", "phrase": "avoid", "why": [], "n": 5, "n_scan": 2,
         "n_depth": 3, "n_wall": 0, "near": 1.2, "lat": 0.1, "mode": "rail", "goal_dist": 4.5, "detail": {}}
    d.update(kw)
    return d


def test_format_decision_announce():
    assert format_decision(_decision()) == (
        "차선 옮김 → 말함 '앞에 장애물이 있어 피해 갈게요.' · 점 5(라이다 2·깊이 3·벽 0) "
        "앞 1.20 m 옆 +0.10 m · 목적지 4.5 m")


def test_format_decision_excluded_shows_the_reasons():
    d = _decision(kind="CM", decision="excluded", phrase=None, why=["near_goal", "cm_not_driving"],
                  n=0, n_scan=0, n_depth=0, near=None, lat=None, goal_dist=0.8)
    assert format_decision(d) == (
        "충돌감시 → 뺌 (목적지 1 m 안·회전·정지 중 충돌감시) · 점 0(라이다 0·깊이 0·벽 0) · 목적지 0.8 m")


class TestGuard:
    def test_error_turns_it_off_once_and_never_raises(self):
        errors, calls = [], []

        def boom(x):
            calls.append(x)
            raise ValueError("bad scan")

        f = Guard(errors.append)
        wrapped = f.wrap(boom)
        assert wrapped(1) is None
        assert wrapped(2) is None
        assert calls == [1]
        assert f.enabled is False
        assert len(errors) == 1 and isinstance(errors[0], ValueError)

    def test_passes_values_through_while_healthy(self):
        g = Guard(lambda e: None)
        assert g.wrap(lambda a, b: a + b)(2, 3) == 5
        assert g.enabled


class TestTakeAll:
    """최종 검토 I-1(2026-10-09): 판정 줄이 같은 deque 를 비우는 순간에도 대화 줄이 IndexError 로 죽지 않는다."""

    def test_takes_everything_in_order(self):
        from collections import deque
        q = deque([("avoid", 1.0), ("slow", 2.0)])
        assert take_all(q) == [("avoid", 1.0), ("slow", 2.0)]
        assert not q

    def test_survives_another_thread_emptying_the_queue(self):
        class Raced:
            """'비어 있지 않다'고 답한 직후 다른 스레드가 비운 것처럼 동작한다."""
            def __init__(self):
                self.items = [("avoid", 1.0)]

            def __bool__(self):
                return True

            def popleft(self):
                if not self.items:
                    raise IndexError("pop from an empty deque")
                return self.items.pop(0)

        assert take_all(Raced()) == [("avoid", 1.0)]
