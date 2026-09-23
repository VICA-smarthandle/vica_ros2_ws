"""지도 한 장 = 한 층. 목적지 폴더의 map.yaml 에서 건물·층을 읽는다."""
from vica_mission_manager.map_meta import MapMeta, load_map_meta


def test_reads_building_and_floor(tmp_path):
    (tmp_path / "map.yaml").write_text("building: 로봇관\nfloor: 4\n", encoding="utf-8")
    meta = load_map_meta(str(tmp_path / "destinations.yaml"))
    assert meta == MapMeta(building="로봇관", floor=4)


def test_missing_file_is_unknown(tmp_path):
    assert load_map_meta(str(tmp_path / "destinations.yaml")) == MapMeta(building="", floor=-1)


def test_bad_values_fall_back(tmp_path):
    (tmp_path / "map.yaml").write_text("building: 5\nfloor: 사층\n", encoding="utf-8")
    meta = load_map_meta(str(tmp_path / "destinations.yaml"))
    assert meta.building == "5" and meta.floor == -1


def test_non_mapping_yaml_is_unknown(tmp_path):
    (tmp_path / "map.yaml").write_text("- 1\n- 2\n", encoding="utf-8")
    assert load_map_meta(str(tmp_path / "destinations.yaml")) == MapMeta()


def test_invalid_utf8_is_empty(tmp_path):
    (tmp_path / "map.yaml").write_bytes(b"\xff\xfe")
    assert load_map_meta(str(tmp_path / "destinations.yaml")) == MapMeta()
