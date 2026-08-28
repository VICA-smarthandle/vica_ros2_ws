"""Pure policy deciding when a comm-caused latch may clear itself.

이 모델은 걸쇠가 아니다. 걸쇠는 `EmergencyLatch`가 소유한다. 여기서 정하는
것은 **"관리자가 누르던 reset 절차를 대신 불러도 되는가"** 하나뿐이며,
그 절차 자체(`ResetSequence`)는 그대로 fail-closed로 돈다.

원인을 두 갈래로 나눈다.

- **사람이 만든 원인**(`physical_f1`·`app`·`voice`): 누군가 판단해서 눌렀다.
  사람이 푼다. 한 번이라도 섞이면 그 사건 동안 자동 복구는 막힌다.
- **통신에서 비롯된 원인**(`motor_can`·`*_stale`·`*_waiting`): 지금 판단할 수
  없다는 상태다. 통신이 돌아오면 스스로 풀려도 안전하다 -- 그동안 로봇은
  이미 멈춰 있었다.

여기에 사용자가 정한 조건이 하나 더 붙는다: **정지 중일 때만**. 주행 중
(`RUNNING`)에 끊긴 것은 사람이 확인한다.

시각은 모두 정수 나노초(STEADY_TIME)이며 호출자가 주입한다.
"""

from typing import Iterable, Optional


# 사람이 판단해서 만든 원인. 섞이면 자동 복구가 막힌다.
HUMAN_SOURCES = frozenset({"physical_f1", "app", "voice"})

# 통신 상태에서 비롯된 원인. 이것만으로 걸린 래치가 자동 복구 대상이다.
COMM_SOURCES = frozenset({
    "motor_can",
    "physical_stale",
    "motor_can_stale",
    "physical_waiting",
    "motor_can_waiting",
})

# 이 상태로 떨어지면 '주행 중 끊김'이다. RUNNING -> READY_TO_GO 는 정상
# 감속이므로 포함하지 않는다 -- 넣으면 한 번 주행한 뒤 자동 복구가 영영 막힌다.
BLOCKING_STATES = frozenset({"FAULT", "ESTOP_ACTIVE"})


class AutoRecoveryPolicy:
    """Decide whether the comm-caused latch may be cleared without an operator."""

    def __init__(self, settle_ns: int):
        self.settle_ns = settle_ns
        # 사람 개입 또는 주행 중 끊김을 본 적이 있다 -> 이 사건은 관리자 몫이다.
        self.blocked = False
        # 통신 원인을 실제로 본 적이 있다 -> 풀어 줄 것이 있다.
        self.armed = False
        # 원인 목록이 빈 채로 유지되기 시작한 시각.
        self.clear_since_ns: Optional[int] = None

    def observe_sources(self, sources: Iterable[str], now: int) -> None:
        """Record one central-latch source snapshot."""
        names = set(sources)
        if names & HUMAN_SOURCES:
            self.blocked = True
        if names & COMM_SOURCES:
            self.armed = True
        if names:
            # 원인이 하나라도 남아 있으면 안정 시간을 처음부터 다시 센다.
            # 떨리는 접점에서 재출발하지 않게 하는 장치다.
            self.clear_since_ns = None
        elif self.clear_since_ns is None:
            self.clear_since_ns = now

    def observe_safety_state(self, previous: str, current: str) -> None:
        """Record a supervisor state transition to spot a stop while driving."""
        if previous == "RUNNING" and current in BLOCKING_STATES:
            self.blocked = True

    def should_recover(self, now: int) -> bool:
        """Return True only for a settled, comm-only, stopped-at-the-time event."""
        if self.blocked or not self.armed:
            return False
        if self.clear_since_ns is None:
            return False
        return now - self.clear_since_ns >= self.settle_ns

    def mark_attempted(self, success: bool) -> None:
        """Record one automatic attempt; a failure hands the event to the operator."""
        self.armed = False
        self.clear_since_ns = None
        if not success:
            self.blocked = True

    def notify_manual_reset(self) -> None:
        """Return to the initial state after an operator cleared the latch."""
        self.blocked = False
        self.armed = False
        self.clear_since_ns = None
