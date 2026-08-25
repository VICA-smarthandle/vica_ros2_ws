"""person_detector_node 의 순수 기하 계산.

노드에서 떼어 놓은 이유는 detection_gate 와 같다 — 여기 있는 함수는 ROS 도
모델도 모르는 순수 함수라 pytest 만으로 끝난다. 노드는 배선만 갖는다.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np

# 몸통 표본 영역. bbox 안에서 depth 중앙값을 낼 창이다.
#
# 폭은 가운데 40 % 만 쓴다 — 지팡이·팔이 bbox 좌우 가장자리에 걸리는데, 지팡이
# 픽셀이 섞이면 거리가 왜곡된다(PersonDetection.msg 주석, 설계 §12).
# 높이는 위에서 20~60 % 구간이다 — 머리 위 배경(0~20 %)과 다리·바닥(60~100 %)을
# 피해 몸통만 남긴다.
_BODY_X_KEEP = (0.30, 0.70)
_BODY_Y_KEEP = (0.20, 0.60)


def body_depth_median_m(
    depth_m: np.ndarray,
    bbox_xyxy: Tuple[float, float, float, float],
) -> float:
    """bbox 의 몸통 영역에서 depth 중앙값[m]을 낸다. 표본이 없으면 NaN.

    `depth_m` 은 미터 단위 2차원 배열이다. 0 이하와 NaN 은 무효 표본으로 버린다
    (RealSense 는 측정 실패를 0 으로 준다). 반환이 NaN 이면 호출자는
    `distance_m = NaN` 으로 발행한다 — 0.0 은 "거리 0 m"로 오독되므로 금지다.
    """
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
    """픽셀 (u, v) + depth 를 카메라 optical 좌표 (x, y, z)[m] 로 편다.

    optical 규약(REP-103): x=오른쪽, y=아래, z=전방. depth 가 무효(NaN·0 이하)
    이거나 초점거리가 0 이면 None — 좌표를 지어내지 않는다.
    """
    if not math.isfinite(depth_m) or depth_m <= 0.0 or fx == 0.0 or fy == 0.0:
        return None
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    return (x, y, depth_m)


def bbox_center(bbox_xyxy: Tuple[float, float, float, float]) -> Tuple[float, float]:
    """bbox 중심 픽셀. 몸통 depth 와 짝지어 사람 위치의 대표점으로 쓴다."""
    x1, y1, x2, y2 = bbox_xyxy
    return ((x1 + x2) * 0.5, (y1 + y2) * 0.5)
