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

from .mission_logic import APPROACH_DESTINATION_PREFIX, Destination, Pose2D, State, pose_valid

NEAR_M = 3.0            # 이 안이면 "OO 앞"
COV_UNKNOWN = 2.0       # x·y 분산 합이 이보다 크면 위치 미확인(초기 위치 전)
HOME_ID = "__home__"    # 홈은 장소 라벨·대장 이름에서 뺀다 — "홈 앞"은 사용자에게 뜻이 없다
_INT32_MIN = -(2 ** 31)
_INT32_MAX = 2 ** 31 - 1


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
        # json.loads 는 NaN·Infinity 리터럴도 float 로 돌려준다(파이썬 확장) —
        # math.isfinite 로 걸러야 아래 age 계산에서 죽지 않는다(최종 리뷰 2절).
        arrived_ok = isinstance(arrived, (int, float)) and math.isfinite(arrived)
        return cls(
            active_destination="",
            last_destination=str(data.get("last_destination") or ""),
            last_arrived_at=float(arrived) if arrived_ok else None,
            aborted_destination=str(data.get("aborted_destination") or ""),
        )


def apply_goal_event(ledger: Ledger, event: str, dest_id: str, name: str, now: float) -> bool:
    """Nav2 goal 사건 → 대장 사실. ledger 를 제자리에서 바꾸고, 파일 저장이
    필요하면 True 를 돌려준다 — 노드는 이 값만 보고 `_save_ledger()`를 부른다.

    홈(dest_id == HOME_ID)과 사람 접근 합성 목적지(dest_id 가
    APPROACH_DESTINATION_PREFIX 로 시작)는 실목적지가 아니므로 이름을 ""로
    취급해 대장에 안 적는다(최종 리뷰 4절 M1) — 접근 한 번으로 "아까 어디
    갔었지?"가 "접근 대상"으로 오염되던 결함.

    `now` 는 벽시계 epoch 초다 — last_arrived_at 은 파일 보존·재기동 복원에
    벽시계를 쓴다(최종 리뷰 1절, mission_manager_node 는 time.time() 을 넘긴다).
    """
    if dest_id == HOME_ID or dest_id.startswith(APPROACH_DESTINATION_PREFIX):
        name = ""
    if event == "goal_sent" and name:
        ledger.active_destination = name
        ledger.aborted_destination = ""
        return True  # 최종 리뷰 4절 minor — 출발당 1회 쓰기(옛 aborted 를 남기지 않는다)
    if event == "goal_accepted" and name:
        ledger.active_destination = name
        ledger.aborted_destination = ""
        return False  # goal_sent 가 이미 저장했다 — 이중 쓰기 방지
    if event == "goal_succeeded" and name:
        ledger.last_destination = name
        ledger.last_arrived_at = now
        ledger.active_destination = ""
        return True
    if event in ("goal_failed", "goal_rejected", "goal_canceled"):
        if name:
            ledger.aborted_destination = name
        ledger.active_destination = ""
        return True
    if event in ("return_home_sent", "return_home_succeeded", "return_home_failed",
                 "return_home_canceled", "state_idle"):
        ledger.active_destination = ""
        return False
    return False


def confirm_abort_name(prev_id: Optional[str], cur_id: Optional[str],
                       state_before: State, state_after: State,
                       active_dest_id: Optional[str],
                       destinations: Dict[str, Destination]) -> Optional[str]:
    """CONFIRMING 에서 출발 없이 접힌 목적지 이름(없으면 None). 판단은 없다 —
    확인 대기 id 가 사라졌는데(prev_id 있었다가 cur_id 없음) 주행도 확인도
    아니고 다른 goal 도 안 잡혀 있으면 그 목적지를 "하려다 만 곳"으로 본다.

    CONFIRMING A→B 로 갈아탄 경우 A 는 여기 안 남는다 — 대장은 "마지막
    하나"만 담는 자료형이라 B 가 곧 새로 적힌다(최종 리뷰 7절 보류 1, 그대로
    유지). state_before 는 지금 조건식에서 쓰지 않는다 — 호출부가 이미 갖고
    있는 tick 전후 두 스냅샷을 그대로 받는 시그니처다.
    """
    if not prev_id or cur_id:
        return None
    if state_after in (State.NAVIGATING, State.CONFIRMING):
        return None
    if active_dest_id is not None:
        return None
    dest = destinations.get(prev_id)
    return dest.name if dest is not None else None


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
    # pose_valid(d, None) 은 주행 게이트 ⑤와 같은 기준(calibrated + (0,0) 아님)
    # 으로 후보를 거른다 — 미캘리브레이션·원점 플레이스홀더가 "OO 앞"으로
    # 나오는 것을 막는다(최종 리뷰 4절). bounds 는 여기서 안 보므로 None.
    ranked = sorted(
        ((math.hypot(d.pose.x - pose.x, d.pose.y - pose.y), d)
         for d in destinations.values() if d.id != HOME_ID and pose_valid(d, None)),
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
        except (OSError, UnicodeDecodeError):
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
    if ledger.last_arrived_at is not None:
        # from_json 이 NaN·Infinity 는 걸렀지만 유한하게 거대한 값(예: 1e300)은
        # 통과시킨다 — int() 는 되지만 rosidl 의 int32 assert 가 터지므로 여기서
        # 클램프한다(최종 리뷰 2절 마지막 구멍).
        age = max(_INT32_MIN, min(_INT32_MAX, int(now_epoch - ledger.last_arrived_at)))
    else:
        age = -1
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
