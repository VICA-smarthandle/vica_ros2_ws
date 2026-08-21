"""Publish /robot/health and /robot/events from diagnostics and safety inputs.

이 노드는 **정지 경로에 들어가지 않는다.** 전체 상태를 요약하고 앱에 표시할 이벤트를 내지만,
모터의 긴급정지 경로에 필수 구성요소로 들어가지 않는다
(vica_system_health_monitoring_draft.md 3.1·3.2절). 이 노드가 죽어도 다음은 계속 동작한다.

    /cmd_vel_safe timeout에 의한 모터 정지
    safety_supervisor_node의 정지 명령
    물리 E-stop과 emergency_stop_node의 중앙 래치
    motor driver의 로컬 watchdog

입력 경로가 두 갈래다.

    진단성 정보 : /diagnostics_agg (aggregator 집계)
    안전 상태   : /emergency_stop, /safety_state, TF를 직접 구독

안전 상태를 직접 받는 이유는 aggregator가 1 Hz로 집계해 ESTOP 표시가 최대 1초 늦어지기
때문이다. 진단이 1초 늦는 것은 유지관리에 문제가 없지만 안전 상태는 다르다.

판정 로직은 전부 순수 모듈에 있다(health_logic, event_deduplicator, agg_parser).
이 파일은 ROS I/O와 파라미터 읽기만 한다.
"""

from builtin_interfaces.msg import Time as TimeMsg
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, Updater
from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from vica_interfaces.msg import RobotEvent, RobotFault, RobotHealth, RobotState

from .agg_parser import from_status, is_ignored
from .event_deduplicator import EventDeduplicator, Observation
from .fault_catalog import describe
from .freshness import is_fresh_ns, sec_to_ns
from .health_logic import ComponentProbe, evaluate, SafetyInput, UNKNOWN
from .nav2_liveness import (
    decide_poll_action,
    is_nav2_active,
    POLL_FALLBACK,
    POLL_WAIT,
)


# required_components.yaml에서 컴포넌트별로 읽을 필드.
_COMPONENT_FIELDS = (
    ('required', True),
    ('observable', True),
    ('severity', 3),
    ('grace_sec', 15.0),
)

# RobotHealth의 readiness 필드 이름. 컴포넌트 이름과 1:1로 맞춘다.
_READINESS_FIELDS = {
    'motor': 'motor_readiness',
    'safety': 'safety_readiness',
    'localization': 'localization_readiness',
    'navigation': 'navigation_readiness',
    'lidar': 'lidar_readiness',
    'perception': 'perception_readiness',
    'guidance': 'guidance_readiness',
    'voice': 'voice_readiness',
    'app': 'app_readiness',
}


class RobotHealthMonitorNode(Node):
    """Aggregate diagnostics and safety state into /robot/health and /robot/events."""

    def __init__(self) -> None:
        """Declare parameters, build subscriptions and start the publish timer."""
        super().__init__('robot_health_monitor_node')

        self.declare_parameter('diagnostics_topic', '/diagnostics_agg')
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('reminder_interval_sec', 300.0)
        self.declare_parameter('latched_reminder_interval_sec', 10.0)
        self.declare_parameter('clear_confirm_ticks', 2)
        self.declare_parameter('nav2_state_poll_period_sec', 2.0)
        self.declare_parameter('nav2_lifecycle_node', '/bt_navigator')
        # get_state 호출을 포기하는 시한. 2026-08-01에 이 호출이 10분간 반환되지 않았다.
        # 폴링 주기보다 넉넉히 길게 두되 유한해야 한다 — 시한이 없으면 폴링이 영구 정지한다.
        self.declare_parameter('nav2_call_timeout_sec', 5.0)
        self.declare_parameter('emergency_stop_timeout_sec', 0.5)
        self.declare_parameter('safety_state_timeout_sec', 1.0)
        self.declare_parameter('tf_timeout_sec', 3.0)
        self.declare_parameter('robot_state_timeout_sec', 3.0)
        self.declare_parameter('diagnostics_timeout_sec', 5.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('component_names', [''])

        # 시각 판정은 Safety·motor와 같은 단일 STEADY_TIME clock을 쓴다.
        # 표시용 시각(first_seen/last_seen)만 SYSTEM_TIME을 쓰며 둘을 섞어 계산하지 않는다.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.started_ns = self.steady_clock.now().nanoseconds

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.emergency_timeout_ns = self._timeout_ns('emergency_stop_timeout_sec')
        self.safety_timeout_ns = self._timeout_ns('safety_state_timeout_sec')
        self.tf_timeout_ns = self._timeout_ns('tf_timeout_sec')
        self.robot_state_timeout_ns = self._timeout_ns('robot_state_timeout_sec')
        self.diagnostics_timeout_ns = self._timeout_ns('diagnostics_timeout_sec')
        self.nav2_call_timeout_ns = self._timeout_ns('nav2_call_timeout_sec')

        self.policies = self._read_component_policies()
        self.dedup = EventDeduplicator(
            reminder_interval_ns=self._timeout_ns('reminder_interval_sec'),
            latched_reminder_interval_ns=self._timeout_ns(
                'latched_reminder_interval_sec'
            ),
            clear_confirm_ticks=int(
                self.get_parameter('clear_confirm_ticks').value
            )
        )

        # ---- 입력 상태 -------------------------------------------------
        # 진단은 항목(name)별로 누적한다. 마지막 메시지 하나만 보관하면 ERROR 발행자와
        # 정상 발행자가 번갈아 도착할 때 오류 표시가 깜빡인다
        # (vica_architecture.md 10.3절이 기록한 결함).
        self.diag_items: dict = {}
        self.last_diag_ns = None

        # 기동 이후 한 번이라도 정상이었던 컴포넌트.
        #
        # 기동 유예를 신선도로 판정할 수 없어서 필요하다. aggregator는 아직 뜨지 않은
        # 부품에도 "Missing"을 1 Hz로 계속 발행하므로 그 입력은 언제나 신선하다.
        # 이 집합에 없으면 "아직 안 뜬 것", 있으면 "떴다가 죽은 것"이다.
        self.ever_ok: set = set()

        self.estop_latched = False
        self.last_estop_ns = None
        self.safety_state = 'IDLE'
        self.last_safety_ns = None
        self.last_robot_state_ns = None
        self.last_tf_ns = None
        self.nav2_state = 'unknown'

        # ---- 출력 -----------------------------------------------------
        # transient_local은 ROS 내부 late-joiner용이다. 앱은 1 Hz 상시 발행에 의존한다.
        health_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub_health = self.create_publisher(RobotHealth, '/robot/health', health_qos)
        self.pub_events = self.create_publisher(RobotEvent, '/robot/events', 20)

        # ---- 구독 -----------------------------------------------------
        diag_topic = str(self.get_parameter('diagnostics_topic').value)
        self.create_subscription(
            DiagnosticArray, diag_topic, self.handle_diagnostics, 10
        )
        self.create_subscription(
            Bool, '/emergency_stop', self.handle_emergency_stop, 10
        )
        self.create_subscription(String, '/safety_state', self.handle_safety_state, 10)
        self.create_subscription(
            RobotState, '/vica/robot_state', self.handle_robot_state, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2 lifecycle 폴링. vica_status_app_node의 _poll_map_yaml과 같은 패턴이다.
        lifecycle_node = str(self.get_parameter('nav2_lifecycle_node').value).rstrip('/')
        self.nav2_client = self.create_client(
            GetState, f'{lifecycle_node}/get_state'
        )
        self._nav2_in_flight = False
        self._nav2_call_started_ns = None
        # 시한을 넘겨 포기한 호출의 늦은 응답이 새 판정을 덮어쓰지 않게 한다.
        self._nav2_future = None
        self.create_timer(
            float(self.get_parameter('nav2_state_poll_period_sec').value),
            self.poll_nav2_state,
        )

        # 자기 진단. 모니터가 죽으면 aggregator가 expected 미충족으로 잡는다.
        self.updater = Updater(self)
        self.updater.setHardwareID('vica_monitor')
        self.updater.add('monitor: health monitor', self.diagnose_self)

        self.last_snapshot = None
        self.create_timer(
            float(self.get_parameter('publish_period_sec').value), self.publish_health
        )

        self.get_logger().info(
            f'robot_health_monitor_node ready: diagnostics={diag_topic}, '
            f'{len(self.policies)} components. '
            'safety inputs are subscribed directly (not via aggregator)'
        )

    # ------------------------------------------------------------------
    # 파라미터
    # ------------------------------------------------------------------
    def _timeout_ns(self, name: str) -> int:
        """Read a seconds parameter as integer nanoseconds."""
        return sec_to_ns(float(self.get_parameter(name).value))

    def _read_component_policies(self) -> dict:
        """Read per-component policy from dotted parameters.

        ROS 2 파라미터는 중첩 구조를 담을 수 없으므로 이름 목록 + dotted namespace를 쓴다.
        """
        policies = {}
        names = [n for n in self.get_parameter('component_names').value if n]

        for name in names:
            values = {}
            for field, default in _COMPONENT_FIELDS:
                key = f'{name}.{field}'
                if not self.has_parameter(key):
                    self.declare_parameter(key, default)
                values[field] = self.get_parameter(key).value
            policies[name] = values

        return policies

    # ------------------------------------------------------------------
    # 구독 콜백
    # ------------------------------------------------------------------
    def handle_diagnostics(self, msg: DiagnosticArray) -> None:
        """Accumulate diagnostic items by name.

        항목별로 누적하는 것이 핵심이다. 마지막 메시지 하나만 들고 있으면 발행자가 번갈아
        도착할 때 표시가 깜빡인다.
        """
        now_ns = self.steady_clock.now().nanoseconds
        self.last_diag_ns = now_ns

        for status in msg.status:
            # 한 번도 참인 적 없는 외부 진단은 여기서 버린다. 근거는
            # agg_parser.IGNORED_NAME_FRAGMENTS 주석에 있다.
            if is_ignored(status.name):
                continue
            item = from_status(status.name, status.level, status.message)
            self.diag_items[status.name] = (item, now_ns)

    def handle_emergency_stop(self, msg: Bool) -> None:
        """Record the central latch state. 직접 구독한다(집계 지연 금지)."""
        self.estop_latched = bool(msg.data)
        self.last_estop_ns = self.steady_clock.now().nanoseconds

    def handle_safety_state(self, msg: String) -> None:
        """Record the safety state enum. 직접 구독한다."""
        self.safety_state = str(msg.data).strip()
        self.last_safety_ns = self.steady_clock.now().nanoseconds

    def handle_robot_state(self, _msg: RobotState) -> None:
        """Track the mission heartbeat."""
        self.last_robot_state_ns = self.steady_clock.now().nanoseconds

    def _nav2_state_by_diagnostic(self, now_ns: int) -> str:
        """Read Nav2 liveness from lifecycle_manager's own diagnostic.

        `<node>/get_state` 서비스가 답을 주지 못할 때 쓰는 두 번째 근거다.

        2026-08-01 실기에서 `/bt_navigator/get_state` 호출이 응답 없이 멈췄다
        (CLI로 부른 것도 10분간 반환되지 않았다). 그런데 같은 시각
        `lifecycle_manager_navigation`은 "Nav2 is active"를 정상 발행하고 있었다.
        서비스 하나가 막혔다고 Nav2 전체를 죽은 것으로 보고하면, 관리자 화면에
        "주행 불가"가 뜨는데 실제로는 주행이 되는 어긋남이 생긴다.

        lifecycle_manager는 자기가 관리하는 노드를 bond로 감시하므로, 그것이
        active라고 말하면 그 관리 그룹은 살아 있다. 서비스 응답보다 약한 근거라
        **서비스가 답하지 못할 때만** 쓴다.

        판정 자체는 nav2_liveness에 있다. 문구·등급·신선도를 함께 봐야 하는 이유는
        그 모듈 주석에 적었다.
        """
        active = is_nav2_active(
            self.diag_items, now_ns, self.diagnostics_timeout_ns
        )
        return 'active' if active else 'unavailable'

    def poll_nav2_state(self) -> None:
        """Poll the Nav2 lifecycle state.

        서비스가 없거나(Nav2 미실행) 응답이 오지 않으면 진단으로 판정한다.
        어느 쪽이든 폴링이 멈추지 않는다.
        """
        now_ns = self.steady_clock.now().nanoseconds
        action = decide_poll_action(
            in_flight=self._nav2_in_flight,
            call_started_ns=self._nav2_call_started_ns,
            service_ready=self.nav2_client.service_is_ready(),
            now_ns=now_ns,
            timeout_ns=self.nav2_call_timeout_ns,
        )

        if action == POLL_WAIT:
            return

        if action == POLL_FALLBACK:
            if self._nav2_in_flight:
                # 시한을 넘겼다. 이 호출은 버리고 다음 tick에 다시 묻는다.
                self.get_logger().warn(
                    'Nav2 lifecycle 조회가 시한 내에 응답하지 않아 진단으로 판정한다.'
                )
                self._nav2_in_flight = False
                self._nav2_call_started_ns = None
                self._nav2_future = None
            self.nav2_state = self._nav2_state_by_diagnostic(now_ns)
            return

        self._nav2_in_flight = True
        self._nav2_call_started_ns = now_ns
        future = self.nav2_client.call_async(GetState.Request())
        self._nav2_future = future
        future.add_done_callback(self._on_nav2_state)

    def _on_nav2_state(self, future) -> None:
        """Store the lifecycle label from the service response.

        시한을 넘겨 버린 호출의 늦은 응답은 무시한다. 그것을 반영하면 이미 진단으로 내린
        판정을 낡은 값이 덮어쓴다.
        """
        if future is not self._nav2_future:
            return
        self._nav2_in_flight = False
        self._nav2_call_started_ns = None
        self._nav2_future = None
        now_ns = self.steady_clock.now().nanoseconds
        try:
            response = future.result()
        except Exception as exc:  # ROS future 예외는 배포판마다 다르다
            self.get_logger().warn(f'Nav2 lifecycle 조회 실패: {exc}')
            self.nav2_state = self._nav2_state_by_diagnostic(now_ns)
            return
        if response is None:
            self.nav2_state = self._nav2_state_by_diagnostic(now_ns)
            return
        self.nav2_state = str(response.current_state.label)

    # ------------------------------------------------------------------
    # TF
    # ------------------------------------------------------------------
    def _update_tf(self) -> None:
        """Refresh the last successful map->base lookup time."""
        try:
            self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            return
        self.last_tf_ns = self.steady_clock.now().nanoseconds

    # ------------------------------------------------------------------
    # 판정과 발행
    # ------------------------------------------------------------------
    def publish_health(self) -> None:
        """Evaluate all inputs and publish health plus any transition events."""
        self._update_tf()
        now_ns = self.steady_clock.now().nanoseconds
        wall_sec = self.get_clock().now().nanoseconds / 1e9

        probes = self._build_probes(now_ns)
        safety = SafetyInput(
            state=self.safety_state,
            estop_latched=self.estop_latched,
            fresh=is_fresh_ns(
                self.last_safety_ns, now_ns=now_ns, timeout_ns=self.safety_timeout_ns
            ),
            age_sec=self._age_sec(self.last_safety_ns, now_ns),
            # 한 번이라도 받았으면 이후 끊김은 유예 대상이 아니다.
            ever_fresh=self.last_safety_ns is not None,
        )

        snapshot = evaluate(probes, safety, now_ns=now_ns, started_ns=self.started_ns)
        self.last_snapshot = snapshot

        observations = [
            Observation(
                component=fault.component,
                fault_code=fault.fault_code,
                severity=fault.severity,
                detail=fault.detail,
                suggested_action=fault.suggested_action,
                latched=fault.latched,
            )
            for fault in snapshot.faults
        ]
        events, active = self.dedup.update(observations, now_ns, wall_sec)

        for event in events:
            self.pub_events.publish(self._to_event_msg(event))

        self.pub_health.publish(self._to_health_msg(snapshot, active))

    def _build_probes(self, now_ns: int) -> list:
        """Turn diagnostics and direct inputs into ComponentProbe records."""
        worst = self._worst_diag_by_component(now_ns)
        probes = []

        for name, policy in self.policies.items():
            observable = bool(policy['observable'])
            severity = int(policy['severity'])
            grace_ns = sec_to_ns(float(policy['grace_sec']))

            last_seen_ns, ok, fault_code, detail = self._probe_inputs(
                name, worst, now_ns
            )

            # 정상을 한 번이라도 관측하면 기록한다. 되돌리지 않는다 — 이후의 고장은
            # 유예 대상이 아니라 즉시 보고할 결함이다.
            if ok and observable and last_seen_ns is not None:
                self.ever_ok.add(name)

            probes.append(
                ComponentProbe(
                    name=name,
                    required=bool(policy['required']),
                    observable=observable,
                    last_seen_ns=last_seen_ns,
                    ok=ok,
                    timeout_ns=self._probe_timeout_ns(name),
                    grace_ns=grace_ns,
                    severity=severity,
                    fault_code=fault_code,
                    detail=detail,
                    ever_ok=name in self.ever_ok,
                )
            )

        return probes

    def _probe_inputs(self, name: str, worst: dict, now_ns: int) -> tuple:
        """Return (last_seen_ns, ok, fault_code, detail) for one component.

        navigation과 localization은 진단 외에 직접 입력도 본다. TF와 Nav2 lifecycle은
        진단 항목으로 표현할 수 없기 때문이다.
        """
        item = worst.get(name)
        diag_ok = item is None or not item[0].is_fault
        fault_code = '' if item is None else item[0].fault_code
        # message가 아니라 detail을 쓴다. aggregator의 영문 요약어("Missing", "Stale")가
        # 한국어 화면에 그대로 뜨는 것을 막는 지점이다.
        detail = '' if item is None else item[0].detail
        last_seen_ns = self.last_diag_ns if item is None else item[1]

        if name == 'localization':
            tf_fresh = is_fresh_ns(
                self.last_tf_ns, now_ns=now_ns, timeout_ns=self.tf_timeout_ns
            )
            if not tf_fresh:
                age = self._age_sec(self.last_tf_ns, now_ns)
                if age is None:
                    tf_detail = (
                        'map에서 base_footprint까지의 위치 변환을 '
                        '한 번도 확보하지 못했습니다.'
                    )
                else:
                    tf_detail = describe(
                        'LOCALIZATION_TF_STALE', age_sec=f'{age:.1f}'
                    ).detail
                return self.last_tf_ns, False, 'LOCALIZATION_TF_STALE', tf_detail
        elif name == 'navigation':
            if self.nav2_state != 'active':
                description = describe('NAV2_NOT_ACTIVE', state=self.nav2_state)
                return now_ns, False, 'NAV2_NOT_ACTIVE', description.detail
        elif name == 'safety':
            # safety는 health_logic이 SafetyInput으로 따로 판정한다. 여기서는 진단만 본다.
            pass

        return last_seen_ns, diag_ok, fault_code, detail

    def _age_sec(self, last_ns, now_ns: int):
        """Return the age of a timestamp in seconds, or None when never received.

        None을 그대로 돌려주는 것이 중요하다. 0.0으로 바꾸면 "방금 받았다"와 "한 번도 못
        받았다"가 구별되지 않는다. 문구도 달라야 한다.
        """
        if last_ns is None:
            return None
        age_ns = now_ns - last_ns
        if age_ns < 0:
            # 시간 역전. 값을 만들지 않는다.
            return None
        return age_ns / 1e9

    def _probe_timeout_ns(self, name: str) -> int:
        """Return the freshness timeout used for a component probe.

        진단 항목의 timeout은 aggregator가 소유한다. 여기서는 진단 자체가 끊긴 것만
        본다 — aggregator가 죽으면 모든 컴포넌트가 stale이 된다.
        """
        if name == 'localization':
            return max(self.diagnostics_timeout_ns, self.tf_timeout_ns)
        return self.diagnostics_timeout_ns

    def _worst_diag_by_component(self, now_ns: int) -> dict:
        """Pick the most severe fresh diagnostic item per component."""
        worst: dict = {}

        for _name, (item, seen_ns) in self.diag_items.items():
            if not is_fresh_ns(
                seen_ns, now_ns=now_ns, timeout_ns=self.diagnostics_timeout_ns
            ):
                continue
            component = item.component
            current = worst.get(component)
            if current is None or item.level > current[0].level:
                worst[component] = (item, seen_ns)

        return worst

    # ------------------------------------------------------------------
    # 메시지 변환
    # ------------------------------------------------------------------
    def _to_fault_msg(self, fault) -> RobotFault:
        """Convert an ActiveFault into a RobotFault message."""
        msg = RobotFault()
        msg.component = fault.component
        msg.fault_code = fault.fault_code
        msg.severity = int(fault.severity)
        msg.active = bool(fault.active)
        msg.latched = bool(fault.latched)
        msg.occurrence_count = int(fault.occurrence_count)
        msg.first_seen = self._to_time_msg(fault.first_seen_sec)
        msg.last_seen = self._to_time_msg(fault.last_seen_sec)
        msg.detail = fault.detail
        msg.suggested_action = fault.suggested_action
        return msg

    def _to_time_msg(self, seconds: float) -> TimeMsg:
        """Convert wall-clock seconds into builtin_interfaces/Time.

        표시·로그 전용 SYSTEM_TIME이다. 신선도 판정에는 쓰지 않는다.
        """
        msg = TimeMsg()
        msg.sec = int(seconds)
        msg.nanosec = int((seconds - int(seconds)) * 1e9)
        return msg

    def _to_event_msg(self, event) -> RobotEvent:
        """Convert a deduplicator Event into a RobotEvent message."""
        msg = RobotEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.fault = self._to_fault_msg(event.fault)
        msg.transition = int(event.transition)
        return msg

    def _to_health_msg(self, snapshot, active) -> RobotHealth:
        """Convert a HealthSnapshot plus active fault list into RobotHealth."""
        msg = RobotHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = int(snapshot.state)

        for component, field in _READINESS_FIELDS.items():
            level = snapshot.readiness.get(component, UNKNOWN)
            setattr(msg, field, int(level))

        msg.active_fault_count = len(active)
        msg.highest_severity = int(snapshot.highest_severity)
        msg.primary_fault_code = snapshot.primary_fault_code
        msg.active_faults = [self._to_fault_msg(fault) for fault in active]
        return msg

    # ------------------------------------------------------------------
    # 자기 진단
    # ------------------------------------------------------------------
    def diagnose_self(
        self,
        stat: DiagnosticStatusWrapper,
    ) -> DiagnosticStatusWrapper:
        """Report whether the monitor itself is receiving what it needs.

        진단 입력이 끊긴 것을 여기서 알린다. aggregator가 죽으면 모니터는 아무것도 볼 수
        없는데, 그것을 "모두 정상"으로 표시하면 안 된다.
        """
        now_ns = self.steady_clock.now().nanoseconds
        diag_fresh = is_fresh_ns(
            self.last_diag_ns, now_ns=now_ns, timeout_ns=self.diagnostics_timeout_ns
        )

        if not diag_fresh:
            stat.summary(
                DiagnosticStatus.WARN,
                '진단 입력이 끊겼습니다. 상태 감시가 제한됩니다',
            )
        else:
            stat.summary(DiagnosticStatus.OK, '모니터 정상')

        stat.add('diagnostics_topic', str(self.get_parameter('diagnostics_topic').value))
        stat.add('diagnostics_fresh', 'true' if diag_fresh else 'false')
        stat.add('diag_items', str(len(self.diag_items)))
        stat.add('safety_state', self.safety_state)
        stat.add('estop_latched', 'true' if self.estop_latched else 'false')
        stat.add('nav2_state', self.nav2_state)

        if self.last_snapshot is not None:
            stat.add('health_state', str(self.last_snapshot.state))
            stat.add('active_faults', str(self.last_snapshot.active_fault_count))
            unobservable = [
                name
                for name, level in self.last_snapshot.readiness.items()
                if level == UNKNOWN
            ]
            if unobservable:
                # 관측 불가 목록을 남긴다. "정상"과 혼동되지 않게 한다.
                stat.add('unobservable', ', '.join(unobservable))

        return stat


def main() -> None:
    """Spin the monitor node."""
    rclpy.init()
    node = RobotHealthMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
