"""초음파 Range 발행 게이트 (순수 로직, rclpy·tf2 비의존).

왜 발행하는 쪽에서 막는가:

    Nav2 의 `RangeSensorLayer` 는 Range 를 받을 때마다 그 메시지 시각의
    `odom -> usonic_*` 변환을 조회하는데, **그 조회가 던지는 예외를 잡는 곳이
    없다.** 2026-09-07 18:44 에 TF 가 34 초 끊긴 사이 `controller_server` 가
    이 경로에서 SIGABRT 로 죽었다(크래시 덤프 스택으로 확인).

    Nav2 는 설치본 바이너리라 우리가 못 고친다. 대신 `updateCostmap()` 은
    수신 버퍼를 순회할 뿐이므로 **버퍼가 비면 그 층은 아예 돌지 않는다.**
    그래서 TF 가 없는 동안 Range 를 보내지 않는 것으로 예외를 원천 차단한다.

이 게이트가 **하지 않는** 것:

    **TF 를 조회하지 않는다.** 조회는 노드가 tf2 로 하고, 여기에는 그 결과
    (bool)만 들어온다. `vica_perception/inference_gate.py` 와 같은 분리다 —
    판정 로직을 rclpy 없이 시험할 수 있게 하려는 것이다.

    **끊김을 판정하지 않는다.** 몇 초를 끊김으로 볼지는 tf2 의 `can_transform`
    이 이미 정한다. 여기서 다시 시간을 재면 판정이 두 곳으로 갈린다.

막는 동안 잃는 것:

    그 시간 동안 초음파는 costmap 에 들어가지 않는다. 다만 TF 가 끊기는
    상황은 곧 `/wheel/odom` 이 끊긴 상황이고, 그때 로봇은 `motor_can_stale`
    로 이미 E-stop 상태다 — 서 있는 로봇의 costmap 에 초음파가 한동안 빠지는
    것과, 컨트롤러가 죽어 주행 자체가 불가능해지는 것 중 앞을 택했다.
"""
from __future__ import annotations

from typing import List, Optional


class RangeTfGate:
    """채널별로 "지금 발행해도 되는가"를 정한다."""

    def __init__(self, channels: int) -> None:
        if channels <= 0:
            # 채널 0 개는 설정 실수다. 조용히 굴러가면 "초음파가 안 나가는데
            # 이유를 모르는" 상태가 되므로 기동 시점에 죽는 편이 낫다.
            raise ValueError(f"채널 수는 1 이상이어야 한다: {channels}")
        # 시작은 허용으로 본다. 기동 직후 TF 가 아직 없다고 경고부터 내면
        # 정상 기동이 사고처럼 보인다.
        self._allowed: List[bool] = [True] * channels
        self._pending: List[Optional[bool]] = [None] * channels
        self._blocked: List[int] = [0] * channels

    # ---- 판정 -------------------------------------------------------------

    def allow(self, channel: int, tf_ok: bool) -> bool:
        """이 채널을 지금 발행할 것인가. `tf_ok` 는 노드의 tf2 조회 결과다."""
        self._check(channel)
        ok = bool(tf_ok)
        if ok != self._allowed[channel]:
            self._allowed[channel] = ok
            self._pending[channel] = ok
        if not ok:
            self._blocked[channel] += 1
        return ok

    # ---- 보고 -------------------------------------------------------------

    def take_transition(self, channel: int) -> Optional[bool]:
        """상태가 바뀌었으면 새 상태를 1회만 돌려준다(없으면 None).

        초음파는 채널당 10 Hz 다. 막힐 때마다 로그를 내면 정작 중요한 줄이
        묻힌다 — 9/7 에 `controller_server` 의 마지막 로그가 그렇게 묻혔다.
        """
        self._check(channel)
        pending = self._pending[channel]
        self._pending[channel] = None
        return pending

    def blocked_count(self, channel: int) -> int:
        """막은 횟수. 진단에서 '조용히 안 나간 것'을 셀 수 있어야 한다."""
        self._check(channel)
        return self._blocked[channel]

    # ---- 내부 -------------------------------------------------------------

    def _check(self, channel: int) -> None:
        if not 0 <= channel < len(self._allowed):
            raise IndexError(
                f"채널 범위를 벗어났다: {channel} (0..{len(self._allowed) - 1})"
            )
