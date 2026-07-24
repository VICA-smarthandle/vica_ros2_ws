"""destinations.py 로더 unit test — 실제 destinations.yaml 스키마와 지도 yaml 기준."""
import textwrap

import pytest

from vica_mission_manager.destinations import load_destinations, load_map_bounds

ROOM_ID = "11111111-1111-4111-8111-111111111111"
PLACEHOLDER_ID = "22222222-2222-4222-8222-222222222222"
CALIBRATED_ID = "33333333-3333-4333-8333-333333333333"
BLOCKED_ID = "44444444-4444-4444-8444-444444444444"


@pytest.fixture()
def dest_yaml(tmp_path):
    p = tmp_path / "destinations.yaml"
    p.write_text(
        textwrap.dedent(
            """
            destinations:
              - id: "11111111-1111-4111-8111-111111111111"
                name: "윤지영 교수님 사무실"
                aliases: ["407호"]
                is_approachable: true
                unavailable_reason: ""
                pose: {frame_id: "map", x: 3.0, y: 2.0, yaw: 90.0}
                confirm_prompt: "윤지영 교수님 사무실로 안내해드릴까요?"
                arrival_message: "도착했습니다."
              - id: "22222222-2222-4222-8222-222222222222"
                name: "미캘리브레이션 목적지"
                pose: {frame_id: "map", x: 0.0, y: 0.0, yaw: 0.0}
              - id: "33333333-3333-4333-8333-333333333333"
                name: "캘리브레이션 완료 목적지"
                calibrated: true
                pose: {frame_id: "map", x: 1.5, y: -2.0, yaw: 180.0}
              - id: "44444444-4444-4444-8444-444444444444"
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
        assert set(dests) == {
            ROOM_ID,
            PLACEHOLDER_ID,
            CALIBRATED_ID,
            BLOCKED_ID,
        }

    def test_fields(self, dest_yaml):
        d = load_destinations(dest_yaml)[ROOM_ID]
        assert d.name == "윤지영 교수님 사무실"
        assert d.pose.x == 3.0 and d.pose.y == 2.0 and d.pose.yaw_deg == 90.0
        assert d.pose.frame_id == "map"
        assert d.calibrated is None  # 필드 없음 → None
        assert d.arrival_message == "도착했습니다."

    def test_calibrated_flag(self, dest_yaml):
        assert load_destinations(dest_yaml)[CALIBRATED_ID].calibrated is True

    def test_not_approachable(self, dest_yaml):
        d = load_destinations(dest_yaml)[BLOCKED_ID]
        assert d.is_approachable is False
        assert d.unavailable_reason == "공사 중"

    def test_missing_key_raises(self, tmp_path):
        p = tmp_path / "bad.yaml"
        p.write_text("something_else: []", encoding="utf-8")
        with pytest.raises(ValueError):
            load_destinations(str(p))

    def test_missing_catalog_is_empty(self, tmp_path):
        assert load_destinations(str(tmp_path / "destinations.yaml")) == {}


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
