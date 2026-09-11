"""person_detector_node 의 순수 기하 계산."""
from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np

_BODY_X_KEEP = (0.30, 0.70)
_BODY_Y_KEEP = (0.20, 0.60)


def body_depth_median_m(
    depth_m: np.ndarray,
    bbox_xyxy: Tuple[float, float, float, float],
) -> float:
    """bbox 의 몸통 영역에서 depth 중앙값[m]을 낸다. 표본이 없으면 NaN."""
    h, w = depth_m.shape[:2]
    x1, y1, x2, y2 = bbox_xyxy
    bw, bh = x2 - x1, y2 - y1
    if bw <= 0 or bh <= 0:
        return math.nan
    cx1 = int(max(0, min(w, x1 + bw * _BODY_X_KEEP[0])))
    cx2 = int(max(0, min(w, x1 + bw * _BODY_X_KEEP[1])))
    cy1 = int(max(0, min(h, y1 + bh * _BODY_Y_KEEP[0])))
    cy2 = int(max(0, min(h, y1 + bh * _BODY_Y_KEEP[1])))
    if cx2 <= cx1 or cy2 <= cy1:
        return math.nan
    patch = depth_m[cy1:cy2, cx1:cx2]
    valid = patch[np.isfinite(patch) & (patch > 0.0)]
    if valid.size == 0:
        return math.nan
    return float(np.median(valid))


def pixel_to_camera(
    u: float,
    v: float,
    depth_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> Optional[Tuple[float, float, float]]:
    """픽셀 (u, v) + depth 를 카메라 optical 좌표 (x, y, z)[m] 로 편다."""
    if not math.isfinite(depth_m) or depth_m <= 0.0 or fx == 0.0 or fy == 0.0:
        return None
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    return (x, y, depth_m)


def bbox_center(bbox_xyxy: Tuple[float, float, float, float]) -> Tuple[float, float]:
    """bbox 중심 픽셀. 몸통 depth 와 짝지어 사람 위치의 대표점으로 쓴다."""
    x1, y1, x2, y2 = bbox_xyxy
    return ((x1 + x2) * 0.5, (y1 + y2) * 0.5)
