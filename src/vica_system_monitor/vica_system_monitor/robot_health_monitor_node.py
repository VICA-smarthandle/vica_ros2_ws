"""Publish /robot/health and /robot/events from diagnostics and safety inputs."""

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


_COMPONENT_FIELDS = (
    ('required', True),
    ('observable', True),
    ('severity', 3),
    ('grace_sec', 15.0),
)

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
        self.declare_parameter('nav2_call_timeout_sec', 5.0)
        self.declare_parameter('emergency_stop_timeout_sec', 0.5)
        self.declare_parameter('safety_state_timeout_sec', 1.0)
        self.declare_parameter('tf_timeout_sec', 3.0)
        self.declare_parameter('robot_state_timeout_sec', 3.0)
        self.declare_parameter('diagnostics_timeout_sec', 5.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('component_names', [''])

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

        self.diag_items: dict = {}
        self.last_diag_ns = None

        self.ever_ok: set = set()

        self.estop_latched = False
        self.last_estop_ns = None
        self.safety_state = 'IDLE'
        self.last_safety_ns = None
        self.last_robot_state_ns = None
        self.last_tf_ns = None
        self.nav2_state = 'unknown'

        health_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub_health = self.create_publisher(RobotHealth, '/robot/health', health_qos)
        self.pub_events = self.create_publisher(RobotEvent, '/robot/events', 20)

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

        lifecycle_node = str(self.get_parameter('nav2_lifecycle_node').value).rstrip('/')
        self.nav2_client = self.create_client(
            GetState, f'{lifecycle_node}/get_state'
        )
        self._nav2_in_flight = False
        self._nav2_call_started_ns = None
        self._nav2_future = None
        self.create_timer(
            float(self.get_parameter('nav2_state_poll_period_sec').value),
            self.poll_nav2_state,
        )

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

    def _timeout_ns(self, name: str) -> int:
        """Read a seconds parameter as integer nanoseconds."""
        return sec_to_ns(float(self.get_parameter(name).value))

    def _read_component_policies(self) -> dict:
        """Read per-component policy from dotted parameters."""
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

    def handle_diagnostics(self, msg: DiagnosticArray) -> None:
        """Accumulate diagnostic items by name."""
        now_ns = self.steady_clock.now().nanoseconds
        self.last_diag_ns = now_ns

        for status in msg.status:
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
        """Read Nav2 liveness from lifecycle_manager's own diagnostic."""
        active = is_nav2_active(
            self.diag_items, now_ns, self.diagnostics_timeout_ns
        )
        return 'active' if active else 'unavailable'

    def poll_nav2_state(self) -> None:
        """Poll the Nav2 lifecycle state."""
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
        """Store the lifecycle label from the service response."""
        if future is not self._nav2_future:
            return
        self._nav2_in_flight = False
        self._nav2_call_started_ns = None
        self._nav2_future = None
        now_ns = self.steady_clock.now().nanoseconds
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Nav2 lifecycle 조회 실패: {exc}')
            self.nav2_state = self._nav2_state_by_diagnostic(now_ns)
            return
        if response is None:
            self.nav2_state = self._nav2_state_by_diagnostic(now_ns)
            return
        self.nav2_state = str(response.current_state.label)

    def _update_tf(self) -> None:
        """Refresh the last successful map->base lookup time."""
        try:
            self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            return
        self.last_tf_ns = self.steady_clock.now().nanoseconds

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
        """Return (last_seen_ns, ok, fault_code, detail) for one component."""
        item = worst.get(name)
        diag_ok = item is None or not item[0].is_fault
        fault_code = '' if item is None else item[0].fault_code
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
            pass

        return last_seen_ns, diag_ok, fault_code, detail

    def _age_sec(self, last_ns, now_ns: int):
        """Return the age of a timestamp in seconds, or None when never received."""
        if last_ns is None:
            return None
        age_ns = now_ns - last_ns
        if age_ns < 0:
            return None
        return age_ns / 1e9

    def _probe_timeout_ns(self, name: str) -> int:
        """Return the freshness timeout used for a component probe."""
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
        """Convert wall-clock seconds into builtin_interfaces/Time."""
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

    def diagnose_self(
        self,
        stat: DiagnosticStatusWrapper,
    ) -> DiagnosticStatusWrapper:
        """Report whether the monitor itself is receiving what it needs."""
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
