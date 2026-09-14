from pathlib import Path
from uuid import uuid4

import pytest
import yaml

from vica_destination_manager.storage import (
    DestinationStorage,
    normalize_contact_phone,
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


# -- 연락처 (물류 배송 도착 문자) -------------------------------------------------
#
# **이 시험이 지키는 결함**: normalize_destination 은 적힌 키만 새 dict 로 만든다.
# 앱이 contact_phone 을 보내도 여기 없으면 저장은 성공한 것처럼 보이고 다시
# 불러오면 비어 있다. 왕복(저장 → 파일 → 읽기)으로 확인한다.


def test_contact_phone_survives_round_trip(tmp_path: Path) -> None:
    destination = make_destination()
    destination["contact_phone"] = "010-1234-5678"
    storage = DestinationStorage(tmp_path)
    storage.upsert("map_1", destination)
    assert storage.read("map_1")[0]["contact_phone"] == "01012345678"


@pytest.mark.parametrize(
    "raw, expected",
    [
        ("01012345678", "01012345678"),
        ("010-1234-5678", "01012345678"),
        ("010 1234 5678", "01012345678"),
        ("011-123-4567", "0111234567"),
        ("", ""),
        (None, ""),
    ],
)
def test_contact_phone_is_normalized_to_digits(raw, expected) -> None:
    assert normalize_contact_phone(raw) == expected


@pytest.mark.parametrize("raw", ["02-123-4567", "1234", "0101234567890", "abc"])
def test_rejects_non_mobile_contact_phone(raw: str) -> None:
    with pytest.raises(ValueError, match="contact_phone"):
        normalize_contact_phone(raw)


def test_missing_contact_phone_defaults_to_empty() -> None:
    # 연락처 없는 장소는 정상이다. 옛 파일에도 이 키가 없다.
    assert normalize_destination(make_destination())["contact_phone"] == ""
