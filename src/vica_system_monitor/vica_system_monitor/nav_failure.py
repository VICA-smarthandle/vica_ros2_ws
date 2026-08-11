"""Turn one-shot navigation failures into a fault the manager can see.

`docs/proposal_nav_failure_to_app.md` 구현분이다. ROS 의존이 없다 — payload 는 문자열로,
시각은 정수 나노초로 받는다. 판정은 전부 여기 있고 노드 파일은 값을 넘기기만 한다
(`robot_health_monitor_node` 상단 주석의 "판정 로직은 전부 순수 모듈에" 규칙).

## 왜 이 계층이 필요한가

주행이 실패해도 관리자 앱에는 아무 표시가 없었다. 실패가 나가는 문이 둘인데 **둘 다 앱으로
가지 않는다.**

    State.FAILED ─┬─ /vica/tts_request ──▶ 스피커
                  └─ /vica_goal_event ───▶ 손잡이 LED·햅틱   ✗ 앱은 이 토픽을 안 본다

그런데 미션 매니저가 1 Hz 로 `robot_state` 를 계속 보내므로 **박동은 살아 있다.** 감시
계층은 결함 없음으로 판정하고 관리자 화면은 계속 "정상"이다. 로봇이 갇혀 몇 분째 못
움직이고 시각장애인이 손잡이를 잡고 서 있어도 그렇다.

`/vica_goal_event` 의 payload 에는 이미 목적지·사유·시각이 다 들어 있다. 새로 만들 데이터가
없고, 공용 메시지 계약도 바꾸지 않는다. 이 모듈은 그것을 결함 하나로 옮기기만 한다.

## 일회성 사건을 지속 상태로 — 보류 창

감시 계층은 **지속 상태**를 다룬다. 매 tick "지금 참인 결함"의 목록을 다시 만들고, 이번
tick 에 없는 결함은 해소된 것으로 처리한다(`event_deduplicator` 규칙 4).

그런데 `goal_failed` 는 한 번 오고 끝나는 사건이다. 받은 tick 에만 넣으면 바로 다음 tick 에
CLEARED 가 나가 RAISED/CLEARED 가 붙어서 지나간다 — 관리자가 볼 틈이 없다.

그래서 **보류 창(hold)** 을 둔다. 이 창은 두 가지 일을 한다.

    1. 창이 닫힐 때까지 결함을 붙들어 관리자가 볼 시간을 번다
    2. **"반복"의 정의가 된다** — 창이 열려 있는 동안 또 실패하면 같은 곤경이다

반복 판정에 별도 기준("몇 초 안에 몇 번")을 두지 않는 이유는, 값이 둘이 되면 서로 어긋날
수 있고 어느 쪽이 이기는지 모호해지기 때문이다.

## 등급과 승격

기본은 DEGRADED 다. 한 번의 실패로 로봇이 주행 불가가 되는 것은 아니고, 실제로 새 goal 을
받을 수 있다. 그런데 앱은 **STOP(3) 이상만 알림 목록에 남긴다**
(`fault_severity.dart:47` `blocksDriving`, `supervisor_provider.dart:689` 필터).
그 필터에는 근거가 달려 있다 — "WARN·DEGRADED까지 넣으면 알림이 진단 화면과 중복되면서
정작 중요한 항목이 묻힌다."

그래서 **창 안에서 2회째부터 STOP 으로 올린다.** 한 번의 실패는 흔한 일이고, **같은 자리에서
반복되는 실패가 곧 갇힘**이다. 등급만 올리고 fault_code 는 그대로 두므로
`EventDeduplicator` 가 이것을 TRANSITION_ESCALATED 로 낸다 — 다른 문제가 새로 생긴 것이
아니라 같은 문제가 나빠진 것이기 때문이다.

## 이 결함이 readiness 를 건드리지 않는 이유

`NAV2_NOT_ACTIVE` 는 "스택이 없다"이고 이쪽은 "스택은 멀쩡한데 못 갔다"다. goal 하나가
실패해도 Nav2 lifecycle 은 active 이고 다음 goal 을 받는다. readiness 까지 끌어내리면
관리자가 "주행 스택이 죽었다"로 오해한다.
"""

import json
from typing import NamedTuple, Optional

from .fault_catalog import describe, SEVERITY_DEGRADED, SEVERITY_STOP
from .health_logic import Fault


# 카탈로그의 fault code. 두 실패 사건이 이 하나를 공유한다 — 코드를 나누면 키가 달라져
# 승격이 "하나 해소 + 하나 발생"으로 보인다.
NAV_GOAL_FAILED = 'NAV_GOAL_FAILED'

# 결함으로 볼 이벤트. 나머지(goal_sent·goal_accepted·goal_succeeded·goal_canceled)는
# 정상 흐름이므로 여기 없다. goal_canceled 는 사용자가 취소한 것이라 실패가 아니다.
FAILURE_EVENTS = frozenset(('goal_failed', 'goal_rejected'))

# 창 안에서 이 횟수째부터 STOP 으로 올린다.
ESCALATE_AT = 2

# payload 에 값이 없을 때 쓰는 대체 문구. 자리표시자나 빈칸을 관리자에게 보이지 않는다.
UNKNOWN_DESTINATION = '목적지'
UNKNOWN_REASON = '사유 없음'


class GoalFailure(NamedTuple):
    """A parsed navigation failure event."""

    event: str
    name: str
    reason: str


def parse_goal_failure(payload) -> Optional[GoalFailure]:
    """Extract a navigation failure from a ``/vica_goal_event`` payload.

    실패가 아니거나 읽을 수 없으면 None 이다. **예외를 던지지 않는다** — 잘못된 payload
    하나가 감시 노드를 죽이면 상태 표시 자체가 사라진다.
    `vica_user_guidance.guidance_priority.parse_goal_event` 와 같은 방어 계약이다.
    """
    try:
        data = json.loads(payload)
    except (ValueError, TypeError):
        return None
    if not isinstance(data, dict):
        return None

    event = data.get('event')
    if not isinstance(event, str) or event not in FAILURE_EVENTS:
        return None

    return GoalFailure(
        event=event,
        name=_text(data.get('name'), UNKNOWN_DESTINATION),
        reason=_text(data.get('reason'), UNKNOWN_REASON),
    )


def _text(value, fallback: str) -> str:
    """Return a non-empty display string, falling back when the field is unusable."""
    if isinstance(value, str) and value.strip():
        return value.strip()
    return fallback


class NavFailureTracker:
    """Hold the latest navigation failure and decide how loud it should be.

    상태가 셋뿐이다 — 창이 닫히는 시각, 창 안에서 센 실패 횟수, 마지막 실패 내용.
    """

    def __init__(self, hold_ns: int) -> None:
        """Store the hold window in integer nanoseconds.

        ``hold_ns`` 가 0 이하이면 결함이 즉시 만료되어 기능이 꺼진다. 설정으로 끌 수
        있어야 하기 때문이며, 끄면 주행 실패가 앱에 다시 보이지 않게 된다.
        """
        self._hold_ns = hold_ns
        self._until_ns: Optional[int] = None
        self._count = 0
        self._failure: Optional[GoalFailure] = None

    def record(self, failure: Optional[GoalFailure], now_ns: int) -> None:
        """Take one failure into the window. ``None`` 은 무시한다(정상 이벤트)."""
        if failure is None:
            return

        if self._is_held(now_ns):
            # 창이 아직 열려 있다. 같은 곤경이 이어지는 것으로 센다.
            self._count += 1
        else:
            self._count = 1

        self._failure = failure
        # 실패가 이어지는 동안 창을 계속 연장한다. 마지막 실패로부터 hold 만큼이
        # 관리자가 볼 수 있는 시간이다.
        self._until_ns = now_ns + self._hold_ns

    def fault(self, now_ns: int) -> Optional[Fault]:
        """Return the fault to report this tick, or None when the window is closed."""
        if not self._is_held(now_ns) or self._failure is None:
            return None

        severity = SEVERITY_STOP if self._count >= ESCALATE_AT else SEVERITY_DEGRADED
        description = describe(
            NAV_GOAL_FAILED,
            severity=severity,
            name=self._failure.name,
            reason=self._failure.reason,
            count=self._count,
        )
        return Fault(
            component=description.component,
            fault_code=NAV_GOAL_FAILED,
            severity=description.severity,
            detail=description.detail,
            suggested_action=description.suggested_action,
            # 래치가 아니다. 관리자 reset 없이 창이 닫히면 스스로 해소된다.
            # latched=True 로 두면 EventDeduplicator 가 매 tick 재알림한다
            # (같은 파일 _reminder_due 규칙 6).
            latched=False,
        )

    def _is_held(self, now_ns: int) -> bool:
        """Return True while the hold window is still open.

        시계가 뒤로 가면 붙들고 있는 쪽을 고른다. `event_deduplicator._reminder_due`
        와 같은 판단이다 — 조용해지는 방향보다 시끄러워지는 방향이 안전하다.
        """
        if self._until_ns is None:
            return False
        return now_ns < self._until_ns
