"""지도별 홈 위치(`home.yaml`) 저장 로직.

`rclpy` 에 기대지 않는 순수 로직이다. 로봇 없이 개발 PC 에서 시험할 수 있어야
하기 때문이며, `pose_score.py`·`mapping_session.py` 와 같은 이유다.

    ~/vica_data/destinations/<map_id>/destinations.yaml   목적지 카탈로그 (기존)
    ~/vica_data/destinations/<map_id>/home.yaml           홈 위치     (이 모듈)

**홈은 목적지가 아니라 로봇의 설정값이다.** 사용자가 고르는 것이 아니라 로봇이
안내를 끝낸 뒤 스스로 쓰는 값이라, 목적지 카탈로그에 섞지 않고 파일을 나눈다.
파일이 하나이므로 "지도당 홈 1개"가 저절로 지켜진다.

경로 규약은 `docs/vica_robot_bringup_manual.md` 의 `home_yaml` 인자가 이미
정해 둔 것을 그대로 따른다(`<storage_root>/<map_id>/home.yaml`).
"""
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

# 경로 문자를 막아 `<root>` 밖을 건드리지 못하게 한다. `vica_destination_manager`
# 의 같은 검사와 규칙이 같아야 두 파일이 같은 폴더에 놓인다. 두 패키지가 서로를
# import 하지 않도록 규칙만 맞추고 코드는 각자 가진다.
_MAP_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,127}$")


def validate_map_id(map_id: str) -> str:
    """파일 경로로 안전하게 쓸 수 있는 map_id 만 통과시킨다."""
    value = str(map_id).strip()
    if not _MAP_ID_PATTERN.fullmatch(value):
        raise ValueError(
            "map_id는 영문·숫자로 시작하고 영문·숫자·_·-만 사용할 수 있습니다"
        )
    return value

#: 로봇을 실제 자리에 세우고 라이다로 채점해 잡은 홈.
SOURCE_ROBOT_STANDING = "robot_standing"
#: 지도를 눌러 좌표만 찍은 홈. 로봇이 거기 없었으므로 채점값이 없다.
SOURCE_MAP_PICK = "map_pick"

_SOURCES = {SOURCE_ROBOT_STANDING, SOURCE_MAP_PICK}


@dataclass(frozen=True)
class HomePosition:
    """저장된 홈 위치 한 건.

    ``visited_ok`` 가 이 자료구조의 핵심이다. 나머지 필드는 "어떻게 정했나"를
    적은 것이고, 이 하나만이 **"실제로 갈 수 있는 자리임을 확인했나"** 를 답한다.
    좌표를 정하는 일과 그 자리에 도달할 수 있다는 사실은 별개이기 때문이다.
    """

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
    """전송받은 값을 검증해 저장 가능한 홈으로 만든다.

    새로 만든 홈은 ``visited_ok`` 가 **항상 False** 다. 좌표를 정한 것과 그
    자리에 실제로 갈 수 있는 것은 다른 사실이고, 후자는 한 번 가 봐야만 안다.
    로봇을 세워놓고 잡은 경우에도 마찬가지다 — 그 자리에 서 있다는 것이
    "Nav2 가 거기까지 경로를 그릴 수 있다"를 뜻하지는 않기 때문이다.
    """
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
    """`<root>/<map_id>/home.yaml` 을 관리한다.

    목적지 카탈로그와 같은 폴더를 쓰므로 ``root`` 도 같은 값을 넘긴다
    (`destination_storage_root`, 기본 `~/vica_data/destinations`).
    """

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root).expanduser().resolve()

    def path_for(self, map_id: str) -> Path:
        return self.root / validate_map_id(map_id) / "home.yaml"

    def read(self, map_id: str) -> Optional[HomePosition]:
        """홈을 읽는다. 없으면 None 이며 이것은 오류가 아니다.

        파일이 깨졌을 때도 None 을 돌려준다. 홈이 없으면 자동 복귀만 꺼지고
        안내는 정상 동작하므로, 읽기 실패로 노드를 세우는 편이 더 나쁘다.
        """
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
        """'가보기' 결과를 반영한다. 홈이 없으면 None 을 돌려준다.

        도착에 성공하면 True, 실패하면 False 다. 실패를 기록하는 이유는
        **한 번 확인된 홈이 나중에 못 가는 자리가 될 수 있기** 때문이다 —
        가구가 놓이거나 문이 잠기면 좌표는 그대로인데 도달할 수 없게 된다.
        """
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
        """지도를 다시 그렸을 때 확인 상태만 되돌린다.

        좌표는 지우지 않는다. 새 지도가 같은 공간을 다시 그린 것이면 홈 자리도
        대체로 그대로이고, 지워 버리면 관리자가 처음부터 다시 잡아야 한다.
        **다만 그 자리가 여전히 갈 수 있는 곳인지는 아무도 모른다** — 지도가
        달라지면 벽 위치가 조금씩 옮겨져 홈이 벽 속에 들어갈 수 있다.
        그래서 좌표는 남기고 ``visited_ok`` 만 False 로 내려 다시 확인하게 한다.
        """
        return self.mark_visited(map_id, False)

    def invalidate_if_map_is_newer(
        self, map_id: str, map_path: str | Path
    ) -> Optional[HomePosition]:
        """지도 파일이 홈보다 나중에 만들어졌으면 확인 상태를 되돌린다.

        지도를 새로 저장하면 대개 **새 map_id** 가 생기므로(이름에 날짜가 붙는다)
        홈도 새 폴더에 없어 자연스럽게 해결된다. 지도를 지우면 그 폴더가 통째로
        지워지므로 홈도 함께 사라진다.

        문제가 되는 것은 **같은 map_id 로 지도 파일만 바뀐 경우**다. 같은 날
        같은 이름으로 다시 저장하거나 젯슨에서 파일을 직접 바꾸면 그렇게 된다.
        그때 좌표는 그대로인데 지도가 달라져 **홈이 벽 속에 들어갈 수 있다.**

        파일 시각을 비교해 판정하는 이유는 홈에 새 필드를 더하지 않고도 이
        경우를 잡을 수 있기 때문이다. 판정은 노드가 뜰 때 한 번만 한다.
        시각을 읽을 수 없으면 아무것도 하지 않는다 — 확신 없이 확인 상태를
        내리면 관리자가 이유 없이 다시 확인하게 된다.
        """
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
        """임시 파일에 쓰고 교체한다.

        저장 도중 전원이 끊겨도 이전 홈이 남아 있게 하려는 것이다. 반쯤 쓰인
        `home.yaml` 은 좌표를 못 읽는 것과 같고, 그러면 로봇이 갈 곳을 잃는다.
        `destinations.yaml` 과 같은 방식이다.
        """
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
