"""Pure motor final-stage watchdog decision.

cmd(`/cmd_vel_safe`)와 knob(F1 monitor) 두 입력의 신선도를 단일 STEADY_TIME
clock으로 판정한다. 둘 중 하나라도 미수신·시간 역전·timeout이면 0.0을 돌려
motor를 정지시킨다(fail-safe). 정책·토픽·timeout 값은 노드가 그대로 소유한다.
"""

from typing import Optional

from .freshness import is_fresh_ns


def normalize_knob_pct(raw_pct: int, min_pct: int, max_pct: int) -> int:
    """줄이 실제로 움직이는 구간(min~max)을 0~100 으로 편다.

    2026-09-09 실기: 이 로봇의 속도 조절 줄은 knob 55~98 사이만 움직이는데
    코드는 0~100 으로 해석해, 눈금의 가운데 43 %만 쓰고 있었다. 그래서 줄을
    완전히 당겨도 55 % 라 정지 기준(5 %)에 못 미쳤고, 조금 놓으면 60~98 % 가
    전부 주행 상한 0.5 m/s 위여서 속도가 변하지 않았다 — **줄이 아무 역할도
    못 했다.**

    펴고 나면 "당긴 만큼 느려지고 끝까지 당기면 선다" 가 성립한다.

    기본값(0, 100)에서는 원값을 그대로 돌려주므로 종전 동작이 그대로다.
    상한 <= 하한 은 설정 실수인데, 여기서 0 을 내면 로봇이 안 움직이는 이유를
    찾기 어렵다. 그래서 환산을 포기하고 원값을 흘려 증상이 눈에 띄게 둔다.
    """
    if max_pct <= min_pct:
        return raw_pct
    span = max_pct - min_pct
    scaled = round((raw_pct - min_pct) * 100.0 / span)
    return max(0, min(100, int(scaled)))


def motor_speed_ratio(
    cmd_last_ns: Optional[int],
    knob_last_ns: Optional[int],
    knob_pct: int,
    now_ns: int,
    cmd_timeout_ns: int,
    knob_timeout_ns: int,
    deadzone_pct: int,
) -> float:
    """Return the knob-derived speed ratio, or 0.0 when any input is stale.

    cmd 또는 knob 중 하나만 stale이어도 0.0(정지)을 반환한다.
    """
    cmd_fresh = is_fresh_ns(
        cmd_last_ns, now_ns=now_ns, timeout_ns=cmd_timeout_ns
    )
    knob_fresh = is_fresh_ns(
        knob_last_ns, now_ns=now_ns, timeout_ns=knob_timeout_ns
    )
    if not cmd_fresh or not knob_fresh:
        return 0.0
    if knob_pct <= deadzone_pct:
        return 0.0
    return knob_pct / 100.0
