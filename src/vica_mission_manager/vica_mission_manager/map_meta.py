"""지도 한 장 = 한 층 (스펙 3.1). 목적지 폴더의 `map.yaml` 에서 건물·층을 읽는다."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class MapMeta:
    building: str = ""
    floor: int = -1


def load_map_meta(destinations_path: str) -> MapMeta:
    path = Path(destinations_path).expanduser().parent / "map.yaml"
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, yaml.YAMLError):
        return MapMeta()
    if not isinstance(data, dict):
        return MapMeta()
    building = str(data.get("building") or "").strip()
    try:
        floor = int(data.get("floor"))
    except (TypeError, ValueError):
        floor = -1
    return MapMeta(building=building, floor=floor)
