"""주행 중 YOLO 추론 차단 판정 (순수 로직, rclpy 비의존).

왜 주행 중에는 추론하지 않는가:

    접근 요청의 수락 권한은 Mission Manager 가 독점하고, 그 게이트는
    `mission_logic.py` 에서 **`State.IDLE` 일 때만** 통과시킨다("안내를 받고
    있는 사용자가 우선이다"). 주행 중에 사람을 찾아 요청을 보내면 반드시
    거절되므로, 그 추론은 결과가 버려질 것이 확정된 계산이다.

    `detection_gate` 의 접근 대상도 "3초간 0.3 m 미만으로 움직인 사람"이라
    주행 중에 스쳐 지나가는 사람은 어차피 전부 걸러진다. 즉 이 게이트는 기능을
    줄이지 않는다 — 원래 나올 수 없던 결과를 위한 연산만 줄인다.

이 게이트가 **하지 않는** 것:

    **추론 결과를 거르지 않는다.** 그것은 `detection_gate` 의 몫이다. 여기서
    정하는 것은 "이번 프레임에 모델을 부를 것인가" 하나뿐이다.

    **구독을 끊지 않는다.** 이미지 구독을 해제했다가 다시 걸면 DDS 가 짝을
    다시 찾는 동안 재개가 늦는다. 도착 직후가 사람접근이 가장 필요한 순간이라
    그 지연을 받지 않기로 했다(2026-09-01 사용자 결정). 구독은 살려 두고
    콜백 앞에서 되돌아 나온다.

모르면 켜는 쪽으로 넘어지는 이유:

    안전 계층의 fail-safe 는 "모르면 정지"지만 여기는 반대다. 이 노드는 안전
    회로가 아니다 — `/vica/person_detection` 은 소비자가 없고, 장애물은 라이다와
    `depth_band_to_scan` 이 담당한다. 추론이 켜져 있어서 생기는 위험은 없고,
    꺼져 있으면 사람접근 기능이 통째로 죽는다. 게다가 상태를 발행하는 Mission
    Manager 가 죽었다면 주행 자체가 불가능하다. 그래서 상태를 모르는 동안은
    **켠다.**

시간 계약:

    이 모듈은 `time.monotonic_ns()` 를 부르지 않는다. 모든 시각은 호출자(ROS
    노드)가 소유한 단일 STEADY_TIME clock 의 **정수 나노초**로 들어온다. 계약
    정본은 `vica_safety/freshness.py` 이며, 미수신은 `None`(0.0 sentinel 금지),
    시간 역전은 stale 로 넘어진다. 안전 계층에 대한 역방향 의존을 만들지 않으려고
    `sec_to_ns` 만 여기 복제한다 — `detection_gate.py` 와 같은 방식이다.
"""
from __future__ import annotations

from enum import Enum
from typing import Optional

# 이 시간 동안 `/vica/robot_state` 가 오지 않으면 상태를 모르는 것으로 본다.
# 발행 주기는 1 Hz(mission_manager_node 의 create_timer)이므로 3초는 3회
# 결손에 해당한다. 짧게 잡으면 순간 부하에 게이트가 깜빡이고, 길게 잡으면
# 미션이 죽은 뒤에도 한참 꺼져 있는다.
DEFAULT_STATE_TIMEOUT_S = 3.0


def sec_to_ns(seconds: float) -> int:
    """초 단위 시간을 정수 나노초로 바꾼다.

    계약 정본은 `vica_safety/freshness.py`. 안전 계층에 대한 역방향 의존을
    만들지 않기 위한 의도적 복제다.
    """
    return int(seconds * 1_000_000_000)


class InferenceReason(str, Enum):
    """추론을 했는가 / 안 했는가, 그리고 **왜** 인가.

    로그와 진단에 그대로 쓴다. "안 돈다"만 남으면 실기에서 원인을 못 찾는다 —
    미션이 주행 중이라 안 도는 것과 상태를 못 받아 도는 것은 완전히 다른 상황이다.
    """

    # 추론한다.
    OK = "ok"                        # 대기 중(IDLE 로 추정) — 정상 동작
    NO_STATE = "no_state"            # 상태를 한 번도 못 받았다
    STATE_STALE = "state_stale"      # 상태가 끊겼다(또는 시간 역전)
    DISABLED = "disabled"            # 게이트를 파라미터로 껐다
    # 추론하지 않는다.
    MOVING = "moving"                # 목적지로 주행 중
    PAUSED = "paused"                # 일시정지(목적지를 기억한 채 멈춤)


# 이 사유일 때만 추론을 건너뛴다. 나머지는 전부 "켜는" 쪽이다.
_BLOCKING = (InferenceReason.MOVING, InferenceReason.PAUSED)


class InferenceGate:
    """`/vica/robot_state` 를 보고 이번 프레임에 모델을 부를지 정한다."""

    def __init__(
        self,
        state_timeout_s: float = DEFAULT_STATE_TIMEOUT_S,
        enabled: bool = True,
    ) -> None:
        if not state_timeout_s > 0.0:
            # 0 이면 받자마자 stale 이 되어 게이트가 영영 안 걸린다. 조용히
            # 굴러가는 것보다 기동 시점에 죽는 편이 안전하다.
            raise ValueError(
                f"상태 timeout 은 0보다 커야 한다: {state_timeout_s}"
            )
        self._timeout_ns = sec_to_ns(state_timeout_s)
        self._enabled = enabled
        self._last_ns: Optional[int] = None
        self._is_moving = False
        self._is_paused = False

    @property
    def enabled(self) -> bool:
        return self._enabled

    # ---- 입력 ---------------------------------------------------------------

    def observe_state(
        self,
        now_ns: int,
        is_moving: bool,
        is_paused: bool,
    ) -> None:
        """`RobotState` 한 건을 넣는다. 수신 시각은 호출자의 STEADY clock 이다.

        메시지에 시각 필드가 없으므로(`RobotState.msg` 는 header 가 없다)
        **수신 시각**으로 신선도를 잰다.
        """
        self._last_ns = now_ns
        self._is_moving = bool(is_moving)
        self._is_paused = bool(is_paused)

    # ---- 판정 ---------------------------------------------------------------

    def reason(self, now_ns: int) -> InferenceReason:
        """지금 추론을 하는가 / 안 하는가의 사유."""
        if not self._enabled:
            return InferenceReason.DISABLED
        if self._last_ns is None:
            return InferenceReason.NO_STATE

        age_ns = now_ns - self._last_ns
        if age_ns > self._timeout_ns or age_ns < 0:
            # 시간 역전(age < 0)도 stale 로 본다 — freshness 계약.
            return InferenceReason.STATE_STALE

        if self._is_moving:
            return InferenceReason.MOVING
        if self._is_paused:
            return InferenceReason.PAUSED
        return InferenceReason.OK

    def should_infer(self, now_ns: int) -> bool:
        """이번 프레임에 모델을 부를 것인가."""
        return self.reason(now_ns) not in _BLOCKING
