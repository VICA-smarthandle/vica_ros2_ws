"""지도별 destinations.yaml 저장 로직.

ROS와 분리해 파일 검증·원자적 저장을 단위 테스트할 수 있게 한다.
기존 locations.json은 읽거나 이관하지 않는다.
"""
from __future__ import annotations

import math
import os
import re
import tempfile
from pathlib import Path
from typing import Any
from uuid import UUID

import yaml

_MAP_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,127}$")
_AUTHORIZATIONS = {"public", "private"}


def validate_map_id(map_id: str) -> str:
    """파일 경로로 안전하게 사용할 수 있는 map_id만 허용한다."""
    value = str(map_id).strip()
    if not _MAP_ID_PATTERN.fullmatch(value):
        raise ValueError("map_id는 영문·숫자로 시작하고 영문·숫자·_·-만 사용할 수 있습니다")
    return value


def validate_destination_id(destination_id: str) -> str:
    """앱에서 생성한 UUID v4 목적지 ID를 canonical 문자열로 반환한다."""
    value = str(destination_id).strip().lower()
    try:
        parsed = UUID(value)
    except (ValueError, AttributeError) as exc:
        raise ValueError("목적지 id는 UUID여야 합니다") from exc
    if parsed.version != 4 or str(parsed) != value:
        raise ValueError("목적지 id는 canonical UUID v4여야 합니다")
    return value


def normalize_destination(raw: dict[str, Any]) -> dict[str, Any]:
    """전송 JSON을 영구 YAML에 저장할 목적지 항목으로 검증·정규화한다."""
    destination_id = validate_destination_id(raw.get("id", ""))
    name = str(raw.get("name", "")).strip()
    if not name:
        raise ValueError("목적지 name이 비어 있습니다")

    aliases: list[str] = []
    for value in raw.get("aliases", []):
        alias = str(value).strip()
        if alias and alias not in aliases:
            aliases.append(alias)
    if name not in aliases:
        aliases.insert(0, name)

    authorization = str(raw.get("authorization", "public")).strip().lower()
    if authorization not in _AUTHORIZATIONS:
        raise ValueError("authorization은 public 또는 private여야 합니다")

    approachable = raw.get("is_approachable", True)
    if not isinstance(approachable, bool):
        raise ValueError("is_approachable은 bool이어야 합니다")
    unavailable_reason = str(raw.get("unavailable_reason", "") or "").strip()
    if not approachable and not unavailable_reason:
        raise ValueError("접근 불가 목적지는 unavailable_reason이 필요합니다")

    pose_raw = raw.get("pose")
    if not isinstance(pose_raw, dict):
        raise ValueError("pose 객체가 필요합니다")
    frame_id = str(pose_raw.get("frame_id", "map")).strip()
    if frame_id != "map":
        raise ValueError("목적지 pose.frame_id는 map이어야 합니다")
    try:
        x = float(pose_raw["x"])
        y = float(pose_raw["y"])
        yaw = float(pose_raw.get("yaw", 0.0))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("pose.x/y/yaw는 숫자여야 합니다") from exc
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        raise ValueError("pose.x/y/yaw는 유한한 숫자여야 합니다")

    return {
        "id": destination_id,
        "name": name,
        "aliases": aliases,
        "category1": str(raw.get("category1", "")).strip(),
        "category2": str(raw.get("category2", "")).strip(),
        "building": str(raw.get("building", "")).strip(),
        "floor": int(raw.get("floor", 0)),
        "owner": str(raw.get("owner", "") or "").strip(),
        "authorization": authorization,
        "is_approachable": approachable,
        "unavailable_reason": unavailable_reason,
        "pose": {
            "frame_id": frame_id,
            "x": x,
            "y": y,
            "yaw": yaw % 360.0,
        },
        "confirm_prompt": str(raw.get("confirm_prompt", "") or "").strip(),
        "arrival_message": str(raw.get("arrival_message", "") or "").strip(),
    }


class DestinationStorage:
    """`<root>/<map_id>/destinations.yaml` 카탈로그를 관리한다."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root).expanduser().resolve()

    def path_for(self, map_id: str) -> Path:
        safe_map_id = validate_map_id(map_id)
        return self.root / safe_map_id / "destinations.yaml"

    def read(self, map_id: str) -> list[dict[str, Any]]:
        path = self.path_for(map_id)
        if not path.exists():
            return []
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if not isinstance(data, dict) or not isinstance(data.get("destinations"), list):
            raise ValueError(f"잘못된 목적지 YAML 구조: {path}")
        return [
            normalize_destination(item)
            for item in data["destinations"]
            if isinstance(item, dict)
        ]

    def upsert(self, map_id: str, raw: dict[str, Any]) -> list[dict[str, Any]]:
        destination = normalize_destination(raw)
        destinations = self.read(map_id)
        for index, current in enumerate(destinations):
            if current["id"] == destination["id"]:
                destinations[index] = destination
                break
        else:
            destinations.append(destination)
        self._write(map_id, destinations)
        return destinations

    def delete(self, map_id: str, destination_id: str) -> list[dict[str, Any]]:
        safe_id = validate_destination_id(destination_id)
        destinations = [
            item for item in self.read(map_id) if item["id"] != safe_id
        ]
        self._write(map_id, destinations)
        return destinations

    def _write(self, map_id: str, destinations: list[dict[str, Any]]) -> None:
        safe_map_id = validate_map_id(map_id)
        path = self.path_for(safe_map_id)
        path.parent.mkdir(parents=True, exist_ok=True)
        document = {
            "schema_version": 1,
            "map_id": safe_map_id,
            "destinations": destinations,
        }
        fd, temp_name = tempfile.mkstemp(
            prefix=".destinations-",
            suffix=".yaml.tmp",
            dir=path.parent,
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                yaml.safe_dump(
                    document,
                    stream,
                    allow_unicode=True,
                    sort_keys=False,
                )
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp_name, path)
        except Exception:
            try:
                os.unlink(temp_name)
            except FileNotFoundError:
                pass
            raise
