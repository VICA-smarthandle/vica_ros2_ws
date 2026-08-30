"""destinations.yaml / Nav2 지도 yaml 로더 (rclpy 비의존, unit test 대상).

정본은 vica_destination_manager가 관리하는 지도별 destinations.yaml이다.
기존 locations.json 또는 음성 저장소의 정적 catalog는 이 로더가 읽지 않는다.
"""
from __future__ import annotations

import os
import struct
from pathlib import Path
from typing import Dict, Optional
from uuid import UUID

import yaml

from .mission_logic import Destination, MapBounds, Pose2D


def load_destinations(path: str) -> Dict[str, Destination]:
    if not Path(path).exists():
        return {}
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not data or "destinations" not in data:
        raise ValueError(f"destinations 키가 없습니다: {path}")

    result: Dict[str, Destination] = {}
    for entry in data["destinations"]:
        dest_id = str(entry["id"]).strip().lower()
        try:
            parsed_id = UUID(dest_id)
        except ValueError as exc:
            raise ValueError(f"목적지 id가 UUID가 아닙니다: {dest_id}") from exc
        if parsed_id.version != 4 or str(parsed_id) != dest_id:
            raise ValueError(f"목적지 id가 canonical UUID v4가 아닙니다: {dest_id}")
        if dest_id in result:
            raise ValueError(f"중복 목적지 id: {dest_id}")
        pose_raw = entry.get("pose") or {}
        pose = Pose2D(
            x=float(pose_raw.get("x", 0.0)),
            y=float(pose_raw.get("y", 0.0)),
            yaw_deg=float(pose_raw.get("yaw", 0.0)),
            frame_id=str(pose_raw.get("frame_id", "map")),
        )
        calibrated = entry.get("calibrated")  # 없으면 None (mission_logic 이 추정)
        result[dest_id] = Destination(
            id=dest_id,
            name=str(entry.get("name", dest_id)),
            pose=pose,
            authorization=str(entry.get("authorization", "public")).strip().lower(),
            is_approachable=bool(entry.get("is_approachable", True)),
            unavailable_reason=str(entry.get("unavailable_reason", "") or ""),
            calibrated=None if calibrated is None else bool(calibrated),
            confirm_prompt=str(entry.get("confirm_prompt", "") or ""),
            arrival_message=str(entry.get("arrival_message", "") or ""),
            category=str(entry.get("category2", "") or "").strip().lower(),
        )
    return result


def _read_pgm_size(path: str) -> tuple:
    """binary/ascii PGM(P5/P2) 헤더에서 (width, height)를 읽는다."""
    with open(path, "rb") as f:
        content = f.read(4096)

    tokens = []
    i = 0
    while i < len(content) and len(tokens) < 3:
        c = content[i : i + 1]
        if c == b"#":  # 주석은 줄 끝까지 건너뜀
            while i < len(content) and content[i : i + 1] != b"\n":
                i += 1
        elif c.isspace():
            i += 1
        else:
            start = i
            while i < len(content) and not content[i : i + 1].isspace():
                i += 1
            tokens.append(content[start:i])
    if len(tokens) < 3 or tokens[0] not in (b"P5", b"P2"):
        raise ValueError(f"PGM 헤더를 읽을 수 없습니다: {path}")
    return int(tokens[1]), int(tokens[2])


def _read_png_size(path: str) -> tuple:
    with open(path, "rb") as f:
        header = f.read(24)
    if len(header) < 24 or header[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError(f"PNG 헤더를 읽을 수 없습니다: {path}")
    width, height = struct.unpack(">II", header[16:24])
    return width, height


def load_map_bounds(map_yaml_path: str) -> Optional[MapBounds]:
    """Nav2 지도 yaml(origin/resolution)과 이미지 크기로 지도 경계를 계산한다.

    게이트 ⑤의 '지도 경계 내' 검증에 쓰인다. 반환 None 은 없음 —
    파일이 이상하면 예외를 던져 노드 시작 시점에 바로 드러나게 한다.
    """
    with open(map_yaml_path, "r", encoding="utf-8") as f:
        meta = yaml.safe_load(f)

    resolution = float(meta["resolution"])
    origin = meta["origin"]
    origin_x, origin_y = float(origin[0]), float(origin[1])

    image_path = str(meta["image"])
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(map_yaml_path), image_path)

    ext = os.path.splitext(image_path)[1].lower()
    if ext == ".pgm":
        width, height = _read_pgm_size(image_path)
    elif ext == ".png":
        width, height = _read_png_size(image_path)
    else:
        raise ValueError(f"지원하지 않는 지도 이미지 형식: {image_path}")

    return MapBounds(
        min_x=origin_x,
        min_y=origin_y,
        max_x=origin_x + width * resolution,
        max_y=origin_y + height * resolution,
    )


def load_home(destinations_path: str) -> Optional[Destination]:
    """목적지 폴더의 home.yaml 을 복귀 목적지(__home__)로 읽는다.

    도착 후 대화·홈 복귀용. 없으면 None — 그러면 도착 후 대화가 제자리
    대기로 폴백한다. 형식은 앱팀의 홈 저장 기능과 같아 나중에 그대로
    대체된다 (pose{frame_id,x,y,yaw}). id 는 UUID 가 아닌 __home__ 이라
    음성 경로로는 지목되지 않는다(관리자·복귀 전용).
    """
    home_path = Path(destinations_path).parent / "home.yaml"
    if not home_path.exists():
        return None
    try:
        data = yaml.safe_load(home_path.read_text()) or {}
        pose = data.get("pose", {})
        return Destination(
            id="__home__",
            name=str(data.get("label", "홈")),
            pose=Pose2D(
                x=float(pose["x"]),
                y=float(pose["y"]),
                yaw_deg=float(pose.get("yaw", 0.0)),
                frame_id=str(pose.get("frame_id", "map")),
            ),
        )
    except (KeyError, TypeError, ValueError, yaml.YAMLError):
        return None
