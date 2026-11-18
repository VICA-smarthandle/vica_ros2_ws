"""경비(순찰) 경로 파일·경보 기록 파일 (순수 로직, rclpy 비의존). 2026-11-14.

    ~/vica_data/destinations/<map_id>/patrol.yaml     순찰 경로(장소 id 순서). 지도당 1개
    <patrol_media_dir>/events.jsonl                   경보 기록. 한 줄에 JSON 하나, 덧붙이기만

경로는 장소 id 만 적는다 — 이름·좌표는 읽을 때 목적지 catalog 에서 찾는다.
같은 파일에 꺼 둔 감시 항목(watch_off)도 적는다. 칸이 없으면 모두 켬이다(2026-11-14 사용자 요청).
기록 파일은 경보가 났을 때와 영상 id 를 뒤늦게 알았을 때만 쓴다(주기 쓰기 없음).
"""
from __future__ import annotations

import json
import math
import os
import shutil
import tempfile
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import yaml

from .home_storage import validate_map_id
from .mission_logic import PATROL_WATCH_KINDS, Destination, Pose2D, pose_valid

SCHEMA_VERSION = 1
HOME_ID = "__home__"

# SavePatrolRoute.reason 코드(SPEC 3.2).
ROUTE_OK = "ok"
ROUTE_UNKNOWN_MAP = "unknown_map"
ROUTE_UNKNOWN_DESTINATION = "unknown_destination"
ROUTE_TOO_FEW = "too_few"
ROUTE_DUPLICATE_ADJACENT = "duplicate_adjacent"
ROUTE_NOTHING_WATCHED = "nothing_watched"
ROUTE_IO_ERROR = "io_error"

ROUTE_MIN_PLACES = 2


def check_route_ids(ids: Sequence[str], catalog: Dict[str, Destination]) -> Tuple[str, str]:
    """경로 검사 → (reason 코드, 한국어 한 줄). 통과하면 ("ok", "").

    끝 지점 다음은 첫 지점이므로 첫·끝이 같은 것도 '연달아 같은 장소'다.
    """
    if len(ids) < ROUTE_MIN_PLACES:
        return ROUTE_TOO_FEW, f"순찰 경로는 장소 {ROUTE_MIN_PLACES}곳 이상이어야 합니다."
    for dest_id in ids:
        if dest_id not in catalog:
            return ROUTE_UNKNOWN_DESTINATION, "저장되지 않은 장소가 경로에 있습니다."
    for i, dest_id in enumerate(ids):
        if dest_id == ids[(i + 1) % len(ids)]:
            name = catalog[dest_id].name
            if i == len(ids) - 1:
                return (ROUTE_DUPLICATE_ADJACENT,
                        f"첫 지점과 끝 지점이 같습니다({name}). 끝 지점 다음은 첫 지점으로 갑니다.")
            return ROUTE_DUPLICATE_ADJACENT, f"같은 장소가 연달아 있습니다({name})."
    return ROUTE_OK, ""


def clean_watch_off(raw) -> List[str]:
    """끌 감시 항목 목록을 정리한다 — 아는 이름만, PATROL_WATCH_KINDS 순서, 겹침 없이."""
    if not isinstance(raw, (list, tuple)):
        return []
    named = {str(item).strip().lower() for item in raw}
    return [kind for kind in PATROL_WATCH_KINDS if kind in named]


class PatrolRouteStorage:
    """`<root>/<map_id>/patrol.yaml`. root 는 목적지 저장 폴더와 같다(HomeStorage 와 같은 규약)."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root).expanduser().resolve()

    def path_for(self, map_id: str) -> Path:
        return self.root / validate_map_id(map_id) / "patrol.yaml"

    def read(self, map_id: str) -> Tuple[Optional[List[str]], List[str]]:
        """(장소 id 목록, 꺼 둔 감시 항목). 파일이 없거나 깨졌으면 (None, []), 경로가 비었으면 id 쪽만 None.

        셋 다 꺼진 파일은 저장 서비스가 만들지 않는다 — 손으로 고친 것이므로 모두 켬으로 읽는다.
        """
        path = self.path_for(map_id)
        if not path.exists():
            return None, []
        try:
            data = yaml.safe_load(path.read_text(encoding="utf-8"))
        except (OSError, ValueError, yaml.YAMLError):
            return None, []
        if not isinstance(data, dict):
            return None, []
        off = clean_watch_off(data.get("watch_off"))
        if len(off) == len(PATROL_WATCH_KINDS):
            off = []
        raw = data.get("destination_ids")
        if not isinstance(raw, list):
            return None, off
        ids = [str(item).strip().lower() for item in raw if str(item).strip()]
        return ids or None, off

    def write(self, map_id: str, ids: Sequence[str], watch_off: Sequence[str] = (),
              saved_at: Optional[str] = None) -> None:
        """임시 파일에 쓰고 교체한다(home.yaml 과 같은 방식). 지도 폴더가 없으면 OSError."""
        path = self.path_for(map_id)
        if not path.parent.is_dir():
            raise FileNotFoundError(f"지도 폴더가 없습니다: {path.parent}")
        document = {
            "schema_version": SCHEMA_VERSION,
            "map_id": validate_map_id(map_id),
            "destination_ids": [str(i) for i in ids],
            "watch_off": clean_watch_off(list(watch_off)),
            "saved_at": saved_at or datetime.now().isoformat(timespec="seconds"),
        }
        fd, temp_name = tempfile.mkstemp(prefix=".patrol-", suffix=".yaml.tmp", dir=path.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                yaml.safe_dump(document, stream, allow_unicode=True, sort_keys=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp_name, path)
        except Exception:
            try:
                os.unlink(temp_name)
            except FileNotFoundError:
                pass
            raise

    def delete(self, map_id: str) -> bool:
        """경로를 지운다. 지울 것이 없었으면 False."""
        try:
            self.path_for(map_id).unlink()
        except FileNotFoundError:
            return False
        return True


def nearest_place_name(pose: Optional[Pose2D], destinations: Dict[str, Destination]) -> str:
    """가장 가까운 저장 장소 이름. 위치나 장소가 없으면 ""(홈·미등록 좌표는 뺀다)."""
    if pose is None:
        return ""
    best_name, best_dist = "", math.inf
    for dest in destinations.values():
        if dest.id == HOME_ID or not pose_valid(dest, None):
            continue
        dist = math.hypot(dest.pose.x - pose.x, dest.pose.y - pose.y)
        if dist < best_dist:
            best_name, best_dist = dest.name, dist
    return best_name


def alarm_record(alarm_id: str, kind: str, time_iso: str, place: str,
                 x: Optional[float], y: Optional[float], map_id: str,
                 clip_id: str = "", photo: str = "", level_db: Optional[float] = None) -> dict:
    """events.jsonl 의 경보 한 줄(SPEC 3.5). 없는 값은 null."""
    return {
        "type": "alarm",
        "alarm_id": alarm_id,
        "kind": kind,
        "time": time_iso,
        "place": place,
        "x": x,
        "y": y,
        "map_id": map_id,
        "clip_id": clip_id or None,
        "photo": photo or None,
        "level_db": level_db,
    }


def clip_record(alarm_id: str, clip_id: str) -> dict:
    """events.jsonl 의 '영상 id 를 나중에 알았다' 한 줄(SPEC 3.5)."""
    return {"type": "clip", "alarm_id": alarm_id, "clip_id": clip_id}


class PatrolEventLog:
    """`<patrol_media_dir>/events.jsonl` — 덧붙이기만 한다. 지우는 것은 clear() 한 곳뿐이다."""

    # 같은 폴더에 감지 노드·지도 서버가 두는 영상·사진 폴더. clear() 가 그 안의 것만 지운다.
    MEDIA_FOLDERS = ("clips", "photos")

    def __init__(self, media_dir: str | Path) -> None:
        self.path = Path(str(media_dir)).expanduser() / "events.jsonl"

    def clear(self) -> int:
        """경보 기록과 영상·사진을 모두 지운다(2026-11-16 사용자 요청 — 앱 '경비 기록' 삭제). 지운 항목 수.

        clips/·photos/ 안의 것과 events.jsonl 만 지운다 — 폴더 자체와 그 밖의 파일은 남긴다.
        못 지우면 OSError 를 그대로 올린다. 경비 중에는 부르지 않는다(감지 노드가 장면을 쓰고 있을 수 있다).
        """
        removed = 0
        for name in self.MEDIA_FOLDERS:
            folder = self.path.parent / name
            if folder.is_symlink() or not folder.is_dir():
                continue
            for entry in folder.iterdir():
                if entry.is_dir() and not entry.is_symlink():
                    shutil.rmtree(entry)
                else:
                    entry.unlink()
                removed += 1
        if self.path.is_file():
            self.path.unlink()
            removed += 1
        return removed

    def append(self, record: dict) -> bool:
        """한 줄 덧붙인다. 실패하면 False — 기록 실패로 순찰을 세우지 않는다."""
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.path, "a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, ensure_ascii=False) + "\n")
            return True
        except (OSError, TypeError, ValueError):
            return False
