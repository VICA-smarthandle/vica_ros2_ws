"""지도 한 장 = 한 층 (스펙 3.1). 목적지 폴더의 `map.yaml` 에서 건물·층을 읽는다.

    ~/vica_data/destinations/<map_id>/destinations.yaml   목적지 카탈로그 (기존)
    ~/vica_data/destinations/<map_id>/home.yaml           홈 위치 (기존)
    ~/vica_data/destinations/<map_id>/map.yaml            건물·층 (이 모듈)   예) building: 로봇관 / floor: 4

없거나 깨지면 "모름"(빈 문자열·-1)이다 — 층을 몰라도 미션은 돌아야 한다.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class MapMeta:
    building: str = ""
    floor: int = -1  # -1 = 모름


def load_map_meta(destinations_path: str) -> MapMeta:
    path = Path(destinations_path).expanduser().parent / "map.yaml"
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, yaml.YAMLError):
        # ValueError 는 UnicodeDecodeError(파일이 utf-8 이 아님)를 포함한다.
        # 이 로더는 노드 __init__ 에서 불리므로 못 잡으면 노드 기동 자체가
        # 죽는다(최종 리뷰 2절 M2).
        return MapMeta()
    if not isinstance(data, dict):
        return MapMeta()
    building = str(data.get("building") or "").strip()
    try:
        floor = int(data.get("floor"))
    except (TypeError, ValueError):
        floor = -1
    return MapMeta(building=building, floor=floor)
