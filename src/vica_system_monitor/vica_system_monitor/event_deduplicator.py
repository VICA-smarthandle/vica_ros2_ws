"""Fault state tracking and notification rate limiting.

vica_system_health_monitoring_draft.md 10.3절의 여섯 규칙을 그대로 구현한다.

    1. 상태가 정상에서 fault로 바뀔 때 한 번 알린다.
    2. 같은 fault는 occurrence count만 증가시킨다.
    3. 장시간 유지되면 설정된 간격으로만 재알림한다.
    4. 복구되면 recovery event를 한 번 발행한다.
    5. 더 높은 등급이 발생하면 즉시 기존 알림을 덮어쓴다.
    6. E-stop 알림은 rate limit보다 높은 우선순위를 갖는다.

ROS 의존이 없는 순수 모듈이다. 시각은 정수 나노초로 주입받고 스스로 조회하지
않는다. 표시용 wall clock 시각(first_seen/last_seen)은 별도 인자로 받는다 —
STEADY_TIME과 SYSTEM_TIME을 섞어 계산하지 않기 위해서다.
"""

from typing import Dict, List, NamedTuple, Optional, Tuple


TRANSITION_RAISED = 0
TRANSITION_ESCALATED = 1
TRANSITION_REMINDER = 2
TRANSITION_CLEARED = 3


class Observation(NamedTuple):
    """One fault observed in the current tick."""

    component: str
    fault_code: str
    severity: int
    detail: str
    suggested_action: str
    latched: bool = False


class ActiveFault(NamedTuple):
    """Current state of one tracked fault.

    first_seen_sec/last_seen_sec는 표시용 wall clock(SYSTEM_TIME) 초 값이다.
    신선도 판정에는 쓰지 않는다.
    """

    component: str
    fault_code: str
    severity: int
    detail: str
    suggested_action: str
    latched: bool
    active: bool
    occurrence_count: int
    first_seen_sec: float
    last_seen_sec: float


class Event(NamedTuple):
    """One event to publish on /robot/events this tick."""

    fault: ActiveFault
    transition: int


class _Record:
    """Mutable bookkeeping for one (component, fault_code) key."""

    def __init__(
        self,
        observation: Observation,
        now_ns: int,
        wall_sec: float,
    ) -> None:
        self.component = observation.component
        self.fault_code = observation.fault_code
        self.severity = observation.severity
        self.detail = observation.detail
        self.suggested_action = observation.suggested_action
        self.latched = observation.latched
        self.occurrence_count = 1
        self.first_seen_sec = wall_sec
        self.last_seen_sec = wall_sec
        self.last_notified_ns = now_ns
        # 연속으로 관측되지 않은 tick 수. clear_confirm_ticks 에 도달해야 해소로 본다.
        self.missed_ticks = 0
        # 연속으로 관측된 tick 수와 "발생"을 이미 알렸는가(raise_confirm_ticks).
        self.seen_ticks = 1
        self.raised = False

    def snapshot(self, active: bool) -> ActiveFault:
        """Build an immutable view for publishing."""
        return ActiveFault(
            component=self.component,
            fault_code=self.fault_code,
            severity=self.severity,
            detail=self.detail,
            suggested_action=self.suggested_action,
            latched=self.latched,
            active=active,
            occurrence_count=self.occurrence_count,
            first_seen_sec=self.first_seen_sec,
            last_seen_sec=self.last_seen_sec,
        )


class EventDeduplicator:
    """Turn per-tick fault observations into a minimal event stream."""

    def __init__(
        self,
        reminder_interval_ns: int,
        latched_reminder_interval_ns: Optional[int] = None,
        clear_confirm_ticks: int = 1,
        raise_confirm_ticks: int = 1,
    ) -> None:
        """Store the re-notification interval in integer nanoseconds.

        latched_reminder_interval_ns 는 래치된 결함 전용 간격이다. None 이면 종전
        동작(매 tick 재알림)을 유지한다. 기본값을 바꾸지 않은 이유는 기존 시험이
        그 동작을 계약으로 잡고 있기 때문이다.

        clear_confirm_ticks 는 "해소"를 확정하기까지 결함이 연속으로 관측되지
        않아야 하는 tick 수다. 1 이면 종전대로 한 tick만에 해소로 본다.
        """
        self._reminder_interval_ns = reminder_interval_ns
        self._latched_reminder_interval_ns = latched_reminder_interval_ns
        self._clear_confirm_ticks = max(1, int(clear_confirm_ticks))
        # "발생"을 확정하기까지 연속으로 관측되어야 하는 tick 수(2026-09-03).
        # 임계값 언저리에서 1~2초 튀는 주기 프로브가 발생/해소를 번갈아 내던 것을
        # 막는다. 1 이면 종전대로 첫 tick 에 알린다. **래치 결함(E-stop 계열)은
        # 기다리지 않는다** — 관리자 reset 이 필요한 상태를 늦게 알리는 것이 더
        # 위험하다(규칙 6).
        self._raise_confirm_ticks = max(1, int(raise_confirm_ticks))
        self._records: Dict[Tuple[str, str], _Record] = {}

    def update(
        self,
        observations: List[Observation],
        now_ns: int,
        wall_sec: float,
    ) -> Tuple[List[Event], List[ActiveFault]]:
        """Advance state by one tick.

        Returns (events to publish now, current active fault list). 활성 목록은
        severity 내림차순으로 정렬해 앱이 그대로 표시할 수 있게 한다.
        """
        events: List[Event] = []
        seen_keys = set()

        for observation in observations:
            key = (observation.component, observation.fault_code)
            seen_keys.add(key)
            record = self._records.get(key)

            if record is None:
                record = _Record(observation, now_ns, wall_sec)
                self._records[key] = record
                if self._confirms_raise(record):
                    record.raised = True
                    events.append(
                        Event(record.snapshot(active=True), TRANSITION_RAISED)
                    )
                continue

            record.seen_ticks += 1
            if not record.raised:
                # 아직 후보다. 이번 값으로 갱신하고 확인 tick 을 채웠는지만 본다.
                record.severity = observation.severity
                record.detail = observation.detail
                record.suggested_action = observation.suggested_action
                record.latched = observation.latched
                record.occurrence_count += 1
                record.last_seen_sec = wall_sec
                record.missed_ticks = 0
                if self._confirms_raise(record):
                    record.raised = True
                    record.last_notified_ns = now_ns
                    events.append(
                        Event(record.snapshot(active=True), TRANSITION_RAISED)
                    )
                continue

            previous_severity = record.severity
            record.severity = observation.severity
            record.detail = observation.detail
            record.suggested_action = observation.suggested_action
            record.latched = observation.latched
            record.occurrence_count += 1
            record.last_seen_sec = wall_sec
            record.missed_ticks = 0

            if observation.severity > previous_severity:
                # 규칙 5: 등급 상승은 rate limit과 무관하게 즉시 알린다.
                record.last_notified_ns = now_ns
                events.append(
                    Event(record.snapshot(active=True), TRANSITION_ESCALATED)
                )
                continue

            if self._reminder_due(record, now_ns):
                record.last_notified_ns = now_ns
                events.append(
                    Event(record.snapshot(active=True), TRANSITION_REMINDER)
                )

        # 규칙 4: 이번 tick에 관측되지 않은 fault는 해소된 것으로 보고 한 번만 알린다.
        #
        # 다만 곧바로 해소로 보면 임계값 근처에서 오르내리는 관측이 매 초 "발생 ->
        # 해소"를 반복해 이력을 같은 항목으로 채운다. 실제로 /odom 프로브는
        # min_hz 20 인데 CPU 가 모자랄 때 15.5 Hz 까지 떨어진 기록이 있다
        # (devlog/2026-08-01-drive-tuning-and-duplicate-stack.md).
        #
        # 그래서 clear_confirm_ticks 만큼 연속으로 안 보여야 해소로 확정한다.
        # 지연시키는 쪽은 '해소'뿐이고 '발생'은 종전대로 즉시 알린다 — 결함을
        # 늦게 지우는 것은 안전한 방향, 늦게 알리는 것은 위험한 방향이다.
        for key in list(self._records):
            if key in seen_keys:
                continue
            record = self._records[key]
            if not record.raised:
                # 확인 tick 을 못 채우고 사라진 후보. 알린 적이 없으니 해소도 없다.
                self._records.pop(key)
                continue
            record.missed_ticks += 1
            if record.missed_ticks < self._clear_confirm_ticks:
                continue
            self._records.pop(key)
            record.last_seen_sec = wall_sec
            events.append(Event(record.snapshot(active=False), TRANSITION_CLEARED))

        return events, self._active_faults()

    def _confirms_raise(self, record: _Record) -> bool:
        """Return True once a candidate has been seen long enough to announce."""
        if record.latched:
            return True
        return record.seen_ticks >= self._raise_confirm_ticks

    def _reminder_due(self, record: _Record, now_ns: int) -> bool:
        """Return True when the re-notification interval has elapsed.

        규칙 6: **래치된 결함**은 일반 결함보다 짧은 전용 간격을 쓴다
        (latched_reminder_interval_ns). 관리자 reset이 있어야 풀리는 상태를
        놓치지 않는 것이 폭주 억제보다 우선이기 때문이다.

        [2026-08-21] 원래는 간격을 무시하고 **매 tick**(1 Hz) 재알림했다. 앱은
        reminder 를 화면에 쌓지 않으므로 화면이 넘치지는 않았지만, 그 메시지가
        전부 rosbridge 를 지나간다. rosbridge 는 supervisor_bringup 에서 서비스
        호출과 같은 흐름을 쓰므로 초당 한 건의 군더더기가 그대로 비용이 된다.
        None 을 주면 종전 동작(매 tick)으로 되돌아간다.

        기준이 등급이 아니라 latched인 이유: 등급으로 판정하면 "가장 심각한 등급"이
        곧 "가장 시끄러워야 할 상태"라는 뜻이 되는데 둘은 다르다. 실제로 모터 진단이
        수신되지 않을 뿐인 결함이 초당 한 건씩 알림을 냈다(2026-07-31 실기동에서
        occurrence_count 223회 관측).

        시간이 역전되면(now_ns < last_notified_ns) 알린다. 조용해지는 방향보다
        시끄러워지는 방향이 안전하다.
        """
        if record.latched:
            if self._latched_reminder_interval_ns is None:
                return True
            elapsed_ns = now_ns - record.last_notified_ns
            if elapsed_ns < 0:
                return True
            return elapsed_ns >= self._latched_reminder_interval_ns
        elapsed_ns = now_ns - record.last_notified_ns
        if elapsed_ns < 0:
            return True
        return elapsed_ns >= self._reminder_interval_ns

    def _active_faults(self) -> List[ActiveFault]:
        """Return current faults, most severe first."""
        # 확인 tick 을 못 채운 후보는 활성 목록에도 넣지 않는다.
        faults = [
            record.snapshot(active=True)
            for record in self._records.values()
            if record.raised
        ]
        faults.sort(key=lambda fault: (-fault.severity, fault.component, fault.fault_code))
        return faults

    def highest(self) -> Optional[ActiveFault]:
        """Return the most severe active fault, or None when healthy."""
        faults = self._active_faults()
        return faults[0] if faults else None
