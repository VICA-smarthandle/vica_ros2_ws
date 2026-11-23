"""도크 위치(dock.yaml) 읽기 — 미션이 '도킹 시작' 때 준비 위치로 보낼 좌표를 얻는다(2026-11-18)."""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from .mission_logic import Destination, Pose2D

BUMPER_X = 0.151
DOCK_DESTINATION_ID = "__dock_staging__"


@dataclass(frozen=True)
class DockSpot:
    map_id: str
    apex_x: float
    apex_y: float
    fwd_yaw: float
    stage_cm: float = 100.0
    target_cm: float = 14.0
    visited_ok: bool = False


def dock_yaml_path(destinations_path: str) -> Path:
    return Path(destinations_path).expanduser().parent / "dock.yaml"


def read_dock_spot(path: Path) -> Optional[DockSpot]:
    """평평한 key: value 파일. 없거나 깨졌으면 None."""
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return None
    kv: dict[str, str] = {}
    for line in text.splitlines():
        line = line.split("#", 1)[0]
        if ":" not in line:
            continue
        k, v = line.split(":", 1)
        kv[k.strip()] = v.strip().strip("\"'")
    try:
        return DockSpot(
            map_id=kv["map_id"],
            apex_x=float(kv["apex_x"]),
            apex_y=float(kv["apex_y"]),
            fwd_yaw=float(kv["fwd_yaw"]),
            stage_cm=float(kv.get("stage_cm", 100.0)),
            target_cm=float(kv.get("target_cm", 14.0)),
            visited_ok=kv.get("visited_ok", "false") == "true",
        )
    except (KeyError, ValueError):
        return None


def staging_destination(spot: DockSpot) -> Destination:
    """준비 위치(도크 앞 1 m, 도크를 봄)를 Nav2 목적지로."""
    d = BUMPER_X + spot.stage_cm / 100.0
    x = spot.apex_x - d * math.cos(spot.fwd_yaw)
    y = spot.apex_y - d * math.sin(spot.fwd_yaw)
    return Destination(
        id=DOCK_DESTINATION_ID,
        name="도크 앞",
        pose=Pose2D(x=x, y=y, yaw_deg=math.degrees(spot.fwd_yaw)),
        is_approachable=False,
        calibrated=True,
    )
