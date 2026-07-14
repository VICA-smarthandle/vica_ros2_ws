"""destinations.py 로더 unit test — 실제 destinations.yaml 스키마와 지도 yaml 기준."""
import textwrap

import pytest

from vica_mission_manager.destinations import load_destinations, load_map_bounds


@pytest.fixture()
def dest_yaml(tmp_path):
    p = tmp_path / "destinations.yaml"
    p.write_text(
        textwrap.dedent(
            """
            destinations:
              - id: "room_407"
                name: "윤지영 교수님 사무실"
                aliases: ["407호"]
                is_approachable: true
                unavailable_reason: ""
                pose: {frame_id: "map", x: 3.0, y: 2.0, yaw: 90.0}
                confirm_prompt: "윤지영 교수님 사무실로 안내해드릴까요?"
                arrival_message: "도착했습니다."
              - id: "placeholder"
                name: "미캘리브레이션 목적지"
                pose: {frame_id: "map", x: 0.0, y: 0.0, yaw: 0.0}
              - id: "calibrated_dest"
                name: "캘리브레이션 완료 목적지"
                calibrated: true
                pose: {frame_id: "map", x: 1.5, y: -2.0, yaw: 180.0}
              - id: "blocked"
                name: "접근 불가"
                is_approachable: false
                unavailable_reason: "공사 중"
                pose: {frame_id: "map", x: 1.0, y: 1.0, yaw: 0.0}
            """
        ),
        encoding="utf-8",
    )
    return str(p)


class TestLoadDestinations:
    def test_loads_all(self, dest_yaml):
        dests = load_destinations(dest_yaml)
        assert set(dests) == {"room_407", "placeholder", "calibrated_dest", "blocked"}

    def test_fields(self, dest_yaml):
        d = load_destinations(dest_yaml)["room_407"]
        assert d.name == "윤지영 교수님 사무실"
        assert d.pose.x == 3.0 and d.pose.y == 2.0 and d.pose.yaw_deg == 90.0
        assert d.pose.frame_id == "map"
        assert d.calibrated is None  # 필드 없음 → None
        assert d.arrival_message == "도착했습니다."

    def test_calibrated_flag(self, dest_yaml):
        assert load_destinations(dest_yaml)["calibrated_dest"].calibrated is True

    def test_not_approachable(self, dest_yaml):
        d = load_destinations(dest_yaml)["blocked"]
        assert d.is_approachable is False
        assert d.unavailable_reason == "공사 중"

    def test_missing_key_raises(self, tmp_path):
        p = tmp_path / "bad.yaml"
        p.write_text("something_else: []", encoding="utf-8")
        with pytest.raises(ValueError):
            load_destinations(str(p))

    def test_real_repo_yaml_if_present(self):
        # 실기 통합 워크스페이스에서 실행되면 실제 파일도 검증한다
        real = "/home/ji_w/tony/vica-voice-llm/config/destinations.yaml"
        try:
            dests = load_destinations(real)
        except FileNotFoundError:
            pytest.skip("실제 destinations.yaml 없음 (CI 등)")
        assert len(dests) >= 1
        assert all(d.pose.frame_id == "map" for d in dests.values())


class TestLoadMapBounds:
    def _write_pgm(self, path, width=200, height=100):
        header = f"P5\n# test map\n{width} {height}\n255\n".encode()
        path.write_bytes(header + b"\x00" * (width * height))

    def test_pgm_bounds(self, tmp_path):
        self._write_pgm(tmp_path / "map.pgm", 200, 100)
        (tmp_path / "map.yaml").write_text(
            "image: map.pgm\nresolution: 0.05\norigin: [-15.1, -8.59, 0]\n",
            encoding="utf-8",
        )
        b = load_map_bounds(str(tmp_path / "map.yaml"))
        assert b.min_x == pytest.approx(-15.1)
        assert b.min_y == pytest.approx(-8.59)
        assert b.max_x == pytest.approx(-15.1 + 200 * 0.05)
        assert b.max_y == pytest.approx(-8.59 + 100 * 0.05)

    def test_real_map_if_present(self):
        real = "/home/ji_w/tony/vica_ros2_ws/maps/vica_map_0604.yaml"
        try:
            b = load_map_bounds(real)
        except FileNotFoundError:
            pytest.skip("실기 지도 없음 (CI 등)")
        assert b.max_x > b.min_x and b.max_y > b.min_y
