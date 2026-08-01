"""E-stop > 도착 > 회전 > 기본 우선순위 병합 (순수 로직).

이 계층은 어떤 구동 명령도 만들지 않는다. 출력은 Smart Handle 상태코드 1개뿐이다.
``/estop_state``는 중앙 래치(emergency_stop_node)의 결과이며, 여기서 reset하거나
해석을 바꾸지 않는다. 소비만 한다.
"""

import json
from dataclasses import dataclass
from typing import Optional

from . import protocol
from .timebase import is_fresh_ns
from .turn_detector import DIRECTION_LEFT, DIRECTION_RIGHT

ARRIVAL_EVENT = "goal_succeeded"


@dataclass(frozen=True)
class GuidanceInputs:
    """한 tick의 입력 스냅샷. 전부 호출자가 채운다."""

    estop_active: bool                  # /estop_state 최신값
    estop_last_ns: Optional[int]        # 마지막 수신 시각 (STEADY_TIME)
    turn_direction: int                 # TurnGuide.direction
    turn_last_ns: Optional[int]
    arrival_started_ns: Optional[int]   # goal_succeeded 수신 시각


@dataclass(frozen=True)
class GuidanceOutcome:
    """병합 결과. state_code는 반드시 전송 가능한 코드다."""

    state_code: int
    reason: str


def parse_goal_event(payload: str) -> Optional[str]:
    """/vica_goal_event JSON에서 event 문자열만 꺼낸다.

    파싱 실패나 event 키 부재는 None이다. 예외를 던지지 않는다 — 잘못된 payload
    하나가 안내 노드를 죽이면 안 된다.
    """
    try:
        data = json.loads(payload)
    except (ValueError, TypeError):
        return None
    if not isinstance(data, dict):
        return None
    event = data.get("event")
    return event if isinstance(event, str) else None


def is_arrival_event(event: Optional[str]) -> bool:
    """goal_succeeded만 도착이다.

    goal_canceled(비상정지·재요청)와 goal_failed는 도착이 아니다. mission_manager는
    goal_sent와 goal_accepted를 같은 tick에 연속 발행하므로 이들도 제외한다.
    """
    return event == ARRIVAL_EVENT


def resolve_state_code(
    inputs: GuidanceInputs,
    now_ns: int,
    estop_timeout_ns: int,
    cue_timeout_ns: int,
    arrival_hold_ns: int,
    estop_required: bool = True,
) -> GuidanceOutcome:
    """우선순위를 배타적으로 적용해 상태코드 1개를 고른다.

    무상태 순수 함수다. 도착 hold는 별도 상태 머신 없이 "시작 시각 + 경과"로 계산해
    E-stop이 끼어들어도 자동으로 우선순위가 지켜지게 한다.
    """
    # ① E-stop 최우선.
    if inputs.estop_active:
        return GuidanceOutcome(protocol.STATE_ESTOP, "estop_active")

    # 중앙 래치 상태를 모르는데 "정상 안내"를 계속 표시하면 거짓 안심을 준다.
    # estop_required=False는 emergency_stop_node 없이 개발할 때만 쓴다.
    if estop_required and not is_fresh_ns(
        inputs.estop_last_ns, now_ns, estop_timeout_ns
    ):
        return GuidanceOutcome(protocol.STATE_ESTOP, "estop_stale")

    # ② 도착 hold.
    if inputs.arrival_started_ns is not None:
        age_ns = now_ns - inputs.arrival_started_ns
        if 0 <= age_ns <= arrival_hold_ns:
            return GuidanceOutcome(protocol.STATE_ARRIVED, "arrival")
        # 시간 역전(age < 0)이나 만료는 아래로 흘려보낸다

    # ③ 회전 cue (fresh할 때만).
    #
    # [실주행 정정 2026-08-01] 좌·우를 **일부러 뒤집어 보낸다.**
    #
    # 사용자 보고: "서보모터는 좌우 회전 피드백을 제대로 주는데 황색 점멸등이
    # 좌우가 바뀌어서 온다." 즉 서보는 맞고 LED만 반대다.
    #
    # 올바른 자리는 펌웨어다. 황색 점멸등은 .ino의 drawLine()이 그리는 ORANGE
    # 흐름선이고 어느 스트립에 그릴지는 WAVE_A/WAVE_B가 정한다. 그런데 젯슨에는
    # arduino-cli도 Arduino IDE도 없어 펌웨어를 올릴 수 없다. 그래서 오늘은
    # 여기서 뒤집는다. **임시 조치이며 정본은 펌웨어다.**
    #
    # 펌웨어를 고칠 수 있게 되면 반드시 이 교환을 되돌리고 .ino를 고친다.
    # 양쪽을 다 뒤집으면 원위치가 되어 다시 반대로 보인다.
    #
    # 기록이 서로 어긋나 있다는 점도 남긴다. .ino 주석은 "bench에서 좌/우 모두
    # LED 방향과 서보 방향이 일치함을 확인했다"고 하지만,
    # devlog/2026-07-28-smart-handle-guidance-plan.md는 같은 항목을
    # "LED 좌우 매핑 [미검증] — A/B 스트립의 물리적 좌·우 위치를 실측으로
    # 확인만 하면 된다"로 남겼다. 오늘 실주행은 devlog 쪽을 지지한다.
    # A/B 스트립의 물리적 좌우를 실측으로 확정하는 것이 남은 숙제다.
    if is_fresh_ns(inputs.turn_last_ns, now_ns, cue_timeout_ns):
        if inputs.turn_direction == DIRECTION_LEFT:
            return GuidanceOutcome(protocol.STATE_RIGHT, "turn_left")
        if inputs.turn_direction == DIRECTION_RIGHT:
            return GuidanceOutcome(protocol.STATE_LEFT, "turn_right")

    # ④ 기본.
    return GuidanceOutcome(protocol.STATE_NORMAL, "normal")
