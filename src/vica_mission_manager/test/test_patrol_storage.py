"""경비(순찰) 경로 파일·경보 기록 파일 (2026-11-14, SPEC 3.2·3.5)."""
import json

import pytest
import yaml

from vica_mission_manager.mission_logic import Destination, Pose2D
from vica_mission_manager.patrol_storage import (
    ROUTE_DUPLICATE_ADJACENT,
    ROUTE_OK,
    ROUTE_TOO_FEW,
    ROUTE_UNKNOWN_DESTINATION,
    PatrolEventLog,
    PatrolRouteStorage,
    alarm_record,
    check_route_ids,
    clean_watch_off,
    clip_record,
    nearest_place_name,
)


def _dest(dest_id, name, x, y, **kw):
    return Destination(id=dest_id, name=name, pose=Pose2D(x, y, 0.0, "map"), calibrated=True, **kw)


CATALOG = {d.id: d for d in (
    _dest("a", "로비", 1.0, 0.0), _dest("b", "복도 끝", 1.0, 2.0), _dest("c", "창고 앞", -1.0, 2.0))}


# ---- 경로 검사 --------------------------------------------------------------


def test_route_needs_two_known_places():
    assert check_route_ids(["a", "b"], CATALOG) == (ROUTE_OK, "")
    assert check_route_ids(["a", "b", "c", "b"], CATALOG) == (ROUTE_OK, "")   # 같은 곳을 두 번 들러도 된다
    assert check_route_ids(["a"], CATALOG)[0] == ROUTE_TOO_FEW
    assert check_route_ids([], CATALOG)[0] == ROUTE_TOO_FEW
    assert check_route_ids(["a", "zzz"], CATALOG)[0] == ROUTE_UNKNOWN_DESTINATION


def test_same_place_in_a_row_is_refused_including_last_to_first():
    code, message = check_route_ids(["a", "b", "b", "c"], CATALOG)
    assert code == ROUTE_DUPLICATE_ADJACENT and "복도 끝" in message
    # 끝 지점 다음은 첫 지점이다 — 첫·끝이 같으면 연달아 같은 장소다.
    code, message = check_route_ids(["a", "b", "a"], CATALOG)
    assert code == ROUTE_DUPLICATE_ADJACENT and "첫 지점" in message
    assert check_route_ids(["a", "a"], CATALOG)[0] == ROUTE_DUPLICATE_ADJACENT


def test_every_rejection_has_a_korean_line():
    for ids in (["a"], ["a", "zzz"], ["a", "a", "b"]):
        code, message = check_route_ids(ids, CATALOG)
        assert code != ROUTE_OK and message


# ---- 경로 파일 --------------------------------------------------------------


@pytest.fixture
def storage(tmp_path):
    (tmp_path / "map_1").mkdir()
    return PatrolRouteStorage(tmp_path)


def test_route_round_trip_keeps_the_tap_order(storage):
    storage.write("map_1", ["c", "a", "b"], saved_at="2026-11-14T20:00:00")
    assert storage.read("map_1") == (["c", "a", "b"], [])
    doc = yaml.safe_load(storage.path_for("map_1").read_text(encoding="utf-8"))
    assert doc == {"schema_version": 1, "map_id": "map_1", "destination_ids": ["c", "a", "b"],
                   "watch_off": [], "saved_at": "2026-11-14T20:00:00"}
    assert storage.path_for("map_1").name == "patrol.yaml"
    # 임시 파일을 남기지 않는다.
    assert sorted(p.name for p in storage.path_for("map_1").parent.iterdir()) == ["patrol.yaml"]


def test_missing_or_broken_route_reads_as_none(storage):
    assert storage.read("map_1") == (None, [])
    path = storage.path_for("map_1")
    for text in ("", "- just\n- a list\n", "destination_ids: nope\n", "destination_ids: []\n",
                 "{broken: [yaml"):
        path.write_text(text, encoding="utf-8")
        assert storage.read("map_1") == (None, [])


def test_watch_off_round_trip_keeps_only_known_kinds_in_a_fixed_order(storage):
    storage.write("map_1", ["a", "b"], ["sound", "LIGHT ", "camera_lost", "sound", 7])
    assert storage.read("map_1") == (["a", "b"], ["light", "sound"])
    doc = yaml.safe_load(storage.path_for("map_1").read_text(encoding="utf-8"))
    assert doc["watch_off"] == ["light", "sound"]


def test_a_file_without_watch_off_means_everything_is_watched(storage):
    storage.path_for("map_1").write_text("destination_ids: [a, b]\n", encoding="utf-8")
    assert storage.read("map_1") == (["a", "b"], [])
    storage.path_for("map_1").write_text("destination_ids: [a, b]\nwatch_off: light\n", encoding="utf-8")
    assert storage.read("map_1") == (["a", "b"], [])        # 목록이 아니면 버린다


def test_a_hand_edited_file_with_all_three_off_reads_as_all_on(storage):
    storage.path_for("map_1").write_text(
        "destination_ids: [a, b]\nwatch_off: [intruder, light, sound]\n", encoding="utf-8")
    assert storage.read("map_1") == (["a", "b"], [])


def test_watch_off_survives_without_a_route(storage):
    storage.write("map_1", [], ["intruder"])
    assert storage.read("map_1") == (None, ["intruder"])


def test_clean_watch_off_takes_only_lists():
    assert clean_watch_off(["sound", "intruder"]) == ["intruder", "sound"]
    assert clean_watch_off(("light",)) == ["light"]
    for bad in (None, "light", 3, {"light": True}):
        assert clean_watch_off(bad) == []


def test_delete_reports_whether_there_was_a_route(storage):
    assert storage.delete("map_1") is False
    storage.write("map_1", ["a", "b"])
    assert storage.delete("map_1") is True
    assert storage.read("map_1") == (None, [])


def test_write_needs_the_map_folder_and_a_safe_map_id(storage):
    with pytest.raises(OSError):
        storage.write("no_such_map", ["a", "b"])
    for bad in ("../etc", "", "a/b"):
        with pytest.raises(ValueError):
            storage.path_for(bad)


# ---- 가장 가까운 장소 -------------------------------------------------------


def test_nearest_place_is_the_closest_saved_place():
    assert nearest_place_name(Pose2D(0.9, 1.8, 0.0, "map"), CATALOG) == "복도 끝"
    assert nearest_place_name(Pose2D(30.0, -30.0, 0.0, "map"), CATALOG) == "로비"   # 멀어도 가장 가까운 곳
    assert nearest_place_name(None, CATALOG) == ""
    assert nearest_place_name(Pose2D(0.0, 0.0, 0.0, "map"), {}) == ""


def test_nearest_place_skips_home_and_unregistered_poses():
    places = dict(CATALOG)
    places["__home__"] = _dest("__home__", "홈", 0.1, 0.1)
    places["z"] = Destination(id="z", name="좌표 없음", pose=Pose2D(0.0, 0.0, 0.0, "map"))
    assert nearest_place_name(Pose2D(0.1, 0.1, 0.0, "map"), places) == "로비"


# ---- 기록 파일 --------------------------------------------------------------


def test_alarm_record_has_exactly_the_spec_keys():
    record = alarm_record("20261114_203455_sound", "sound", "2026-11-14T20:34:55", "로비",
                          1.2, -0.3, "map_1", level_db=71.5)
    assert record == {
        "type": "alarm", "alarm_id": "20261114_203455_sound", "kind": "sound",
        "time": "2026-11-14T20:34:55", "place": "로비", "x": 1.2, "y": -0.3, "map_id": "map_1",
        "clip_id": None, "photo": None, "level_db": 71.5,
    }
    assert clip_record("20261114_203455_sound", "c1") == {
        "type": "clip", "alarm_id": "20261114_203455_sound", "clip_id": "c1"}


def test_log_appends_one_json_object_per_line(tmp_path):
    log = PatrolEventLog(tmp_path / "vica_patrol")          # 폴더가 없어도 만든다
    assert log.append(alarm_record("id1", "intruder", "2026-11-14T20:34:55", "로비", None, None,
                                   "map_1", clip_id="c1"))
    assert log.append(clip_record("id1", "c2"))
    lines = log.path.read_text(encoding="utf-8").splitlines()
    assert log.path.name == "events.jsonl" and len(lines) == 2
    first, second = (json.loads(line) for line in lines)
    assert first["clip_id"] == "c1" and first["x"] is None and first["place"] == "로비"
    assert second == {"type": "clip", "alarm_id": "id1", "clip_id": "c2"}
    assert "로비" in lines[0]                                # 한글을 그대로 쓴다


def test_log_failure_is_reported_not_raised(tmp_path):
    blocker = tmp_path / "file"
    blocker.write_text("x", encoding="utf-8")
    log = PatrolEventLog(blocker / "sub")                    # 파일 밑에는 폴더를 못 만든다
    assert log.append(clip_record("id1", "c1")) is False


def test_log_expands_the_home_folder(monkeypatch, tmp_path):
    monkeypatch.setenv("HOME", str(tmp_path))
    log = PatrolEventLog("~/workspaces/isaac_ros-dev/vica_patrol")
    assert log.path == tmp_path / "workspaces" / "isaac_ros-dev" / "vica_patrol" / "events.jsonl"
