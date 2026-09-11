"""지도별 홈 위치(`home.yaml`) 저장 로직."""
from __future__ import annotations

import math
import os
import re
import tempfile
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

import yaml

SCHEMA_VERSION = 1

_MAP_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,127}$")


def validate_map_id(map_id: str) -> str:
    """파일 경로로 안전하게 쓸 수 있는 map_id 만 통과시킨다."""
    value = str(map_id).strip()
    if not _MAP_ID_PATTERN.fullmatch(value):
        raise ValueError(
            "map_id는 영문·숫자로 시작하고 영문·숫자·_·-만 사용할 수 있습니다"
        )
    return value

SOURCE_ROBOT_STANDING = "robot_standing"
SOURCE_MAP_PICK = "map_pick"

_SOURCES = {SOURCE_ROBOT_STANDING, SOURCE_MAP_PICK}


@dataclass(frozen=True)
class HomePosition:
    """저장된 홈 위치 한 건."""

    map_id: str
    x: float
    y: float
    yaw: float
    source: str
    score: float
    label: str
    visited_ok: bool
    saved_at: str

    def to_document(self) -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "map_id": self.map_id,
            "pose": {
                "frame_id": "map",
                "x": self.x,
                "y": self.y,
                "yaw": self.yaw,
            },
            "source": self.source,
            "score": self.score,
            "label": self.label,
            "visited_ok": self.visited_ok,
            "saved_at": self.saved_at,
        }


def normalize_yaw_deg(yaw: float) -> float:
    """yaw 를 0~360 도로 맞춘다. `destinations.yaml` 의 pose 와 같은 규약이다."""
    value = float(yaw) % 360.0
    return value + 360.0 if value < 0 else value


def build_home(
    map_id: str,
    x: float,
    y: float,
    yaw: float,
    source: str,
    score: float = 0.0,
    label: str = "",
    saved_at: Optional[str] = None,
) -> HomePosition:
    """전송받은 값을 검증해 저장 가능한 홈으로 만든다."""
    safe_map_id = validate_map_id(map_id)

    source_value = str(source).strip()
    if source_value not in _SOURCES:
        raise ValueError(
            f"source 는 {SOURCE_ROBOT_STANDING} 또는 {SOURCE_MAP_PICK} 여야 합니다"
        )

    try:
        x_value = float(x)
        y_value = float(y)
        yaw_value = float(yaw)
    except (TypeError, ValueError) as exc:
        raise ValueError("x/y/yaw 는 숫자여야 합니다") from exc
    if not all(math.isfinite(v) for v in (x_value, y_value, yaw_value)):
        raise ValueError("x/y/yaw 는 유한한 숫자여야 합니다")

    try:
        score_value = float(score)
    except (TypeError, ValueError) as exc:
        raise ValueError("score 는 숫자여야 합니다") from exc
    if not math.isfinite(score_value) or not 0.0 <= score_value <= 100.0:
        raise ValueError("score 는 0~100 이어야 합니다")

    return HomePosition(
        map_id=safe_map_id,
        x=x_value,
        y=y_value,
        yaw=normalize_yaw_deg(yaw_value),
        source=source_value,
        score=score_value,
        label=str(label or "").strip(),
        visited_ok=False,
        saved_at=saved_at or datetime.now().isoformat(timespec="seconds"),
    )


class HomeStorage:
    """`<root>/<map_id>/home.yaml` 을 관리한다."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root).expanduser().resolve()

    def path_for(self, map_id: str) -> Path:
        return self.root / validate_map_id(map_id) / "home.yaml"

    def read(self, map_id: str) -> Optional[HomePosition]:
        """홈을 읽는다. 없으면 None 이며 이것은 오류가 아니다."""
        path = self.path_for(map_id)
        if not path.exists():
            return None
        try:
            data = yaml.safe_load(path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError):
            return None
        if not isinstance(data, dict):
            return None

        pose = data.get("pose")
        if not isinstance(pose, dict):
            return None
        try:
            x = float(pose["x"])
            y = float(pose["y"])
            yaw = float(pose.get("yaw", 0.0))
        except (KeyError, TypeError, ValueError):
            return None
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return None

        source = str(data.get("source", SOURCE_MAP_PICK))
        if source not in _SOURCES:
            source = SOURCE_MAP_PICK

        try:
            score = float(data.get("score", 0.0))
        except (TypeError, ValueError):
            score = 0.0
        if not math.isfinite(score):
            score = 0.0

        return HomePosition(
            map_id=validate_map_id(str(data.get("map_id", map_id))),
            x=x,
            y=y,
            yaw=normalize_yaw_deg(yaw),
            source=source,
            score=score,
            label=str(data.get("label", "") or ""),
            visited_ok=bool(data.get("visited_ok", False)),
            saved_at=str(data.get("saved_at", "") or ""),
        )

    def write(self, home: HomePosition) -> HomePosition:
        """홈을 저장한다. 같은 지도의 이전 홈은 덮어쓴다."""
        self._write_document(home.map_id, home.to_document())
        return home

    def mark_visited(self, map_id: str, visited_ok: bool) -> Optional[HomePosition]:
        """'가보기' 결과를 반영한다. 홈이 없으면 None 을 돌려준다."""
        home = self.read(map_id)
        if home is None:
            return None
        updated = HomePosition(
            map_id=home.map_id,
            x=home.x,
            y=home.y,
            yaw=home.yaw,
            source=home.source,
            score=home.score,
            label=home.label,
            visited_ok=bool(visited_ok),
            saved_at=home.saved_at,
        )
        self._write_document(updated.map_id, updated.to_document())
        return updated

    def invalidate_for_new_map(self, map_id: str) -> Optional[HomePosition]:
        """지도를 다시 그렸을 때 확인 상태만 되돌린다."""
        return self.mark_visited(map_id, False)

    def invalidate_if_map_is_newer(
        self, map_id: str, map_path: str | Path
    ) -> Optional[HomePosition]:
        """지도 파일이 홈보다 나중에 만들어졌으면 확인 상태를 되돌린다."""
        home = self.read(map_id)
        if home is None or not home.visited_ok or not home.saved_at:
            return home
        try:
            map_mtime = Path(map_path).expanduser().stat().st_mtime
        except OSError:
            return home
        try:
            saved_at = datetime.fromisoformat(home.saved_at).timestamp()
        except ValueError:
            return home
        if map_mtime <= saved_at:
            return home
        return self.mark_visited(map_id, False)

    def delete(self, map_id: str) -> bool:
        """홈을 지운다. 지울 것이 없었으면 False 를 돌려준다."""
        path = self.path_for(map_id)
        try:
            path.unlink()
        except FileNotFoundError:
            return False
        return True

    def _write_document(self, map_id: str, document: dict[str, Any]) -> None:
        """임시 파일에 쓰고 교체한다."""
        path = self.path_for(map_id)
        path.parent.mkdir(parents=True, exist_ok=True)
        fd, temp_name = tempfile.mkstemp(
            prefix=".home-",
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
