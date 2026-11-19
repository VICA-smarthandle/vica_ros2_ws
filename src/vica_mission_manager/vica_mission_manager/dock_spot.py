"""도크 위치(dock.yaml) 읽기 — 미션이 '도킹 시작' 때 준비 위치로 보낼 좌표를 얻는다(2026-11-18).

파일은 vica_docking 노드가 쓴다(앱 '도크 위치로 저장' → /vica/dock/register). 미션은 읽기만 한다.
자리: 목적지 폴더의 dock.yaml — home.yaml 옆, 지도당 1개(11-18 사용자: 홈과 도크는 따로).

좌표 정의는 vica_docking/src/dock_spot.cpp 와 같다.
  apex_x·apex_y = V자 꼭짓점(지도), fwd_yaw = 로봇이 도크를 정면으로 볼 때의 방향(rad)
  준비 위치 = 꼭짓점에서 도크 바깥쪽으로 (범퍼 0.151 + stage_cm/100) m, 방향 fwd_yaw
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from .mission_logic import Destination, Pose2D

BUMPER_X = 0.151              # VICA.xacro 구동축 → 범퍼 앞면
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
    # u(도크에서 로봇 쪽) = −fwd
    x = spot.apex_x - d * math.cos(spot.fwd_yaw)
    y = spot.apex_y - d * math.sin(spot.fwd_yaw)
    return Destination(
        id=DOCK_DESTINATION_ID,
        name="도크 앞",
        pose=Pose2D(x=x, y=y, yaw_deg=math.degrees(spot.fwd_yaw)),
        is_approachable=False,
        calibrated=True,
    )
