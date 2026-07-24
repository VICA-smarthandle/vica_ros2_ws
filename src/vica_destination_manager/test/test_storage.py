from pathlib import Path
from uuid import uuid4

import pytest
import yaml

from vica_destination_manager.storage import (
    DestinationStorage,
    normalize_destination,
    validate_map_id,
)


def make_destination(destination_id: str | None = None) -> dict:
    return {
        "id": destination_id or str(uuid4()),
        "name": "테스트 목적지",
        "aliases": ["테스트 목적지", "테스트"],
        "category1": "education",
        "category2": "lab",
        "building": "starlight",
        "floor": 1,
        "authorization": "public",
        "is_approachable": True,
        "pose": {"frame_id": "map", "x": 1.0, "y": 2.0, "yaw": 450.0},
    }


def test_empty_map_has_no_destinations(tmp_path: Path) -> None:
    assert DestinationStorage(tmp_path).read("vica_map_0630") == []


def test_upsert_writes_map_scoped_yaml_atomically(tmp_path: Path) -> None:
    storage = DestinationStorage(tmp_path)
    destination = make_destination()
    result = storage.upsert("vica_map_0630", destination)

    assert result[0]["id"] == destination["id"]
    assert result[0]["pose"]["yaw"] == 90.0
    path = tmp_path / "vica_map_0630" / "destinations.yaml"
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert document["schema_version"] == 1
    assert document["map_id"] == "vica_map_0630"
    assert len(document["destinations"]) == 1
    assert not list(path.parent.glob("*.tmp"))


def test_private_destination_is_stored_for_admin_listing(tmp_path: Path) -> None:
    destination = make_destination()
    destination["authorization"] = "private"
    result = DestinationStorage(tmp_path).upsert("map_1", destination)
    assert result[0]["authorization"] == "private"


def test_rejects_non_uuid_destination_id() -> None:
    with pytest.raises(ValueError, match="UUID"):
        normalize_destination(make_destination("human_readable_id"))


@pytest.mark.parametrize("map_id", ["../maps", "/tmp/map", "", "map.id"])
def test_rejects_unsafe_map_id(map_id: str) -> None:
    with pytest.raises(ValueError):
        validate_map_id(map_id)
