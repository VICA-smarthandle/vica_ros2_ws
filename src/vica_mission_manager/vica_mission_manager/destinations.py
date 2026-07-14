"""destinations.yaml / Nav2 지도 yaml 로더 (rclpy 비의존, unit test 대상).

destinations.yaml 의 단일 소스는 음성 저장소(vica-voice-llm/config/)다.
목적지 데이터 이원화(함정 3번) 때문에 데모 전에는 앱이 찍은 좌표를
음성 파트 yaml 에 수동 반영하는 절차를 따른다.
"""
from __future__ import annotations

import os
import struct
from typing import Dict, Optional

import yaml

from .mission_logic import Destination, MapBounds, Pose2D


def load_destinations(path: str) -> Dict[str, Destination]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not data or "destinations" not in data:
        raise ValueError(f"destinations 키가 없습니다: {path}")

    result: Dict[str, Destination] = {}
    for entry in data["destinations"]:
        dest_id = str(entry["id"])
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
            is_approachable=bool(entry.get("is_approachable", True)),
            unavailable_reason=str(entry.get("unavailable_reason", "") or ""),
            calibrated=None if calibrated is None else bool(calibrated),
            confirm_prompt=str(entry.get("confirm_prompt", "") or ""),
            arrival_message=str(entry.get("arrival_message", "") or ""),
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
