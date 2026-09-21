"""로봇 대장(스펙 3절) — 미션이 적고 /vica/robot_state 로 방송하는 **사실**. 판단은 없다.

- Ledger: 직전에 간 곳·도착 시각·하려다 만 곳·가는 중인 곳. 파일(ledger.json)에 남겨
  재부팅 뒤 "직전에 간 곳·하려다 만 곳"만 복원한다(가는 중은 복원하지 않는다).
- place_here: AMCL 좌표를 등록 목적지와 견줘 "407호 앞"(NEAR_M 안) / "407호와 화장실 사이"
  (그 밖, 가장 가까운 두 곳) / "407호 근처"(등록 장소가 하나) / ""(좌표 없음·공분산 큼).
- state_fields: 위 사실을 RobotState 메시지 칸 이름 그대로 dict 로 편다. 노드는 복사만 한다.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

from .mission_logic import Destination, Pose2D

NEAR_M = 3.0            # 이 안이면 "OO 앞"
COV_UNKNOWN = 2.0       # x·y 분산 합이 이보다 크면 위치 미확인(초기 위치 전)
HOME_ID = "__home__"    # 홈은 장소 라벨에서 뺀다 — "홈 앞"은 사용자에게 뜻이 없다


@dataclass
class Ledger:
    active_destination: str = ""
    last_destination: str = ""
    last_arrived_at: Optional[float] = None   # epoch 초
    aborted_destination: str = ""

    def to_json(self) -> str:
        return json.dumps(asdict(self), ensure_ascii=False, indent=2)

    @classmethod
    def from_json(cls, text: str) -> "Ledger":
        try:
            data = json.loads(text)
        except (TypeError, ValueError):
            return cls()
        if not isinstance(data, dict):
            return cls()
        arrived = data.get("last_arrived_at")
        return cls(
            active_destination="",
            last_destination=str(data.get("last_destination") or ""),
            last_arrived_at=float(arrived) if isinstance(arrived, (int, float)) else None,
            aborted_destination=str(data.get("aborted_destination") or ""),
        )


def _josa_wa(name: str) -> str:
    """받침 있으면 '과', 없으면 '와'. 한글이 아니면 '와'."""
    if not name:
        return "와"
    ch = name[-1]
    if "가" <= ch <= "힣":
        return "과" if (ord(ch) - 0xAC00) % 28 else "와"
    return "와"


def place_here(pose: Optional[Pose2D], destinations: Dict[str, Destination],
               cov_xy: float = 0.0, near_m: float = NEAR_M) -> Tuple[str, float]:
    """(라벨, 가장 가까운 등록 장소까지 m). 모르면 ("", -1.0)."""
    if pose is None or cov_xy > COV_UNKNOWN:
        return "", -1.0
    ranked = sorted(
        ((math.hypot(d.pose.x - pose.x, d.pose.y - pose.y), d)
         for d in destinations.values() if d.id != HOME_ID),
        key=lambda t: t[0],
    )
    if not ranked:
        return "", -1.0
    dist, nearest = ranked[0]
    if dist <= near_m:
        return f"{nearest.name} 앞", dist
    if len(ranked) >= 2:
        return f"{nearest.name}{_josa_wa(nearest.name)} {ranked[1][1].name} 사이", dist
    return f"{nearest.name} 근처", dist


class LedgerStore:
    """`<목적지 폴더>/ledger.json`. home.yaml 과 같은 자리(스펙과 다른 점: state/ 폴더 대신)."""

    def __init__(self, destinations_path: str) -> None:
        self.path = Path(destinations_path).expanduser().parent / "ledger.json"

    def read(self) -> Ledger:
        try:
            return Ledger.from_json(self.path.read_text(encoding="utf-8"))
        except OSError:
            return Ledger()

    def write(self, ledger: Ledger) -> bool:
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.with_suffix(".json.tmp")
            tmp.write_text(ledger.to_json(), encoding="utf-8")
            tmp.replace(self.path)
            return True
        except OSError:
            return False


def state_fields(ledger: Ledger, dialog_state: str, pose: Optional[Pose2D], cov_xy: float,
                 destinations: Dict[str, Destination], now_epoch: float,
                 wait_minutes: int, wait_left_sec: int, battery_pct: int = -1) -> dict:
    label, dist = place_here(pose, destinations, cov_xy)
    age = int(now_epoch - ledger.last_arrived_at) if ledger.last_arrived_at is not None else -1
    return {
        "dialog_state": dialog_state,
        "place_here": label,
        "place_here_dist_m": float(dist),
        "active_destination": ledger.active_destination,
        "last_destination": ledger.last_destination,
        "last_arrived_age_sec": age,
        "aborted_destination": ledger.aborted_destination,
        "wait_minutes": int(wait_minutes),
        "wait_left_sec": int(wait_left_sec),
        "battery_pct": int(battery_pct),
    }
