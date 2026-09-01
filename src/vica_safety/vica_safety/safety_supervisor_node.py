"""ROS wiring for the VICA drive-command safety gate."""

from diagnostic_updater import Updater
from geometry_msgs.msg import Twist
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .diagnostics import LABEL_GATE, gate_summary
from .freshness import is_fresh_ns, sec_to_ns
from .logging_utils import log_with_severity
from .safety_gate import SafetyGate, SafetyState


def is_zero_twist(msg: Twist, eps: float = 1e-4) -> bool:
    """Check every Twist axis before accepting a reset."""
    return (
        abs(msg.linear.x) < eps
        and abs(msg.linear.y) < eps
        and abs(msg.linear.z) < eps
        and abs(msg.angular.x) < eps
        and abs(msg.angular.y) < eps
        and abs(msg.angular.z) < eps
    )


def limited_twist(
    msg: Twist,
    max_linear_mps: float,
    max_angular_radps: float,
) -> Twist:
    """Clamp and forward only differential-drive axes."""
    out = Twist()
    out.linear.x = max(-max_linear_mps, min(max_linear_mps, msg.linear.x))
    out.angular.z = max(
        -max_angular_radps,
        min(max_angular_radps, msg.angular.z),
    )
    return out


def describe_safety_transition(state: SafetyState) -> tuple[str, str]:
    """Map each safety state to a severity-colored log marker."""
    if state is SafetyState.ESTOP_ACTIVE:
        return "error", "[ESTOP ACTIVE]"
    if state is SafetyState.FAULT:
        return "error", "[FAULT]"
    if state in (
        SafetyState.IDLE,
        SafetyState.ESTOP_RELEASED_WAIT_RESET,
    ):
        return "warn", "[WAIT RESET]"
    if state is SafetyState.READY_TO_GO:
        return "info", "[SAFETY READY]"
    return "info", "[RUNNING]"


class SafetySupervisorNode(Node):
    """Approve `/cmd_vel_req` only after all software safety gates pass."""

    def __init__(self) -> None:
        super().__init__("safety_supervisor_node")

        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("cmd_timeout_sec", 0.5)
        # /emergency_stop 하트비트(30Hz)가 이만큼 안 오면 FAULT(fail-closed).
        # 0.5 -> 2.0 (2026-09-01): CPU·DDS 순단에 하트비트가 0.5초 지각하는
        # 것만으로 감독이 FAULT->WAIT_RESET 로 떨어져, 래치는 안 걸렸는데
        # 관리자 초기화만 요구하는 유령 정지가 실기에서 반복됐다(17:34 실증,
        # 자동복구 사각지대). motor_can 을 3.0초로 늘린 것과 같은 철학 —
        # 순단은 참고, 진짜 두절(2초+)에는 여전히 즉시 잠근다.
        self.declare_parameter("estop_timeout_sec", 2.0)
        self.declare_parameter("max_linear_mps", 1.0)
        self.declare_parameter("max_angular_radps", 2.0)

        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.cmd_timeout_sec = float(
            self.get_parameter("cmd_timeout_sec").value
        )
        self.estop_timeout_sec = float(
            self.get_parameter("estop_timeout_sec").value
        )
        self.max_linear_mps = float(
            self.get_parameter("max_linear_mps").value
        )
        self.max_angular_radps = float(
            self.get_parameter("max_angular_radps").value
        )

        # 모든 watchdog은 단일 STEADY_TIME clock과 정수 나노초를 쓴다.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.cmd_timeout_ns = sec_to_ns(self.cmd_timeout_sec)
        self.estop_timeout_ns = sec_to_ns(self.estop_timeout_sec)

        self.gate = SafetyGate()
        self.last_cmd = Twist()
        self.last_cmd_ns = None
        self.estop_active = True
        self.last_estop_ns = None
        self.last_logged_state = SafetyState.IDLE

        self.pub_cmd_safe = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        self.pub_state = self.create_publisher(String, "/safety_state", 10)
        self.create_subscription(
            Twist,
            "/cmd_vel_req",
            self.cmd_requested_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/emergency_stop",
            self.estop_callback,
            10,
        )
        self.create_service(
            Trigger,
            "/vica_safety/internal/supervisor_reset",
            self.reset_callback,
        )
        self.create_timer(
            1.0 / self.publish_hz,
            self.control_loop,
            clock=self.steady_clock,
        )

        # 게이트가 조용히 죽으면 로봇은 멈추지만 관리자는 이유를 모른다.
        self.diag_updater = Updater(self)
        self.diag_updater.setHardwareID('cmd_vel_gate')
        self.diag_updater.add(LABEL_GATE, self.diagnose_gate)

        self.get_logger().warn(
            "Safety supervisor is a software guard; hardware E-stop remains final."
        )
        self.get_logger().info("Subscribed: /cmd_vel_req, /emergency_stop")
        self.get_logger().info("Publishing: /cmd_vel_safe, /safety_state")
        self.get_logger().info(
            "Internal service: /vica_safety/internal/supervisor_reset"
        )

    def now_ns(self) -> int:
        """Return the current STEADY_TIME instant as integer nanoseconds."""
        return self.steady_clock.now().nanoseconds

    def cmd_requested_callback(self, msg: Twist) -> None:
        """Store the requested command for the periodic safety decision."""
        self.last_cmd = msg
        self.last_cmd_ns = self.now_ns()

    def estop_callback(self, msg: Bool) -> None:
        """Refresh the authoritative central E-stop latch input."""
        self.estop_active = bool(msg.data)
        self.last_estop_ns = self.now_ns()

    def reset_callback(self, request, response):
        """Re-arm drive output only after fresh E-stop and zero command checks."""
        del request
        now = self.now_ns()
        decision = self.gate.request_reset(
            estop_active=self.estop_active,
            estop_fresh=self.estop_is_fresh(now),
            cmd_zero=self.current_requested_cmd_is_zero(now),
        )
        response.success = decision.accepted
        response.message = decision.reason
        if decision.accepted:
            self.get_logger().info(
                "[SUPERVISOR RESET] state=READY_TO_GO /cmd_vel_req=zero"
            )
        else:
            severity = "error" if decision.state is SafetyState.FAULT else "warn"
            log_with_severity(
                self.get_logger(),
                severity,
                (
                    "[RESET REJECTED] step=supervisor_reset "
                    f"reason={decision.reason}"
                ),
            )
        return response

    def control_loop(self) -> None:
        """Publish zero by default and forward only in the RUNNING state."""
        now = self.now_ns()
        cmd_alive = self.cmd_is_alive(now)
        state = self.gate.state_for_command(
            estop_active=self.estop_active,
            estop_fresh=self.estop_is_fresh(now),
            cmd_alive=cmd_alive,
            cmd_zero=not cmd_alive or is_zero_twist(self.last_cmd),
        )

        safe_cmd = Twist()
        if self.gate.can_forward_command:
            safe_cmd = limited_twist(
                self.last_cmd,
                self.max_linear_mps,
                self.max_angular_radps,
            )
        self.pub_cmd_safe.publish(safe_cmd)

        state_msg = String()
        state_msg.data = state.value
        self.pub_state.publish(state_msg)
        self.log_transition_if_needed(state)

    def diagnose_gate(self, stat):
        """Report gate health. A blocked gate is not a fault; a blind one is.

        `control_loop`가 30 Hz로 이미 판정하고 있으므로 여기서는 같은 입력을
        다시 읽기만 한다. 게이트 상태를 여기서 재계산하면 두 개의 진실이 생긴다.
        """
        now = self.now_ns()
        estop_fresh = self.estop_is_fresh(now)
        level, message = gate_summary(
            estop_fresh=estop_fresh,
            gate_state=self.gate.state.value,
        )
        stat.summary(level, message)
        stat.add('gate_state', self.gate.state.value)
        stat.add('reset_armed', str(self.gate.reset_armed))
        stat.add('can_forward_command', str(self.gate.can_forward_command))
        stat.add('estop_active', str(self.estop_active))
        stat.add('estop_fresh', str(estop_fresh))
        stat.add('cmd_alive', str(self.cmd_is_alive(now)))
        return stat

    def estop_is_fresh(self, now: int) -> bool:
        """Reject missing, stale, or time-reversed central E-stop input."""
        return is_fresh_ns(
            self.last_estop_ns,
            now_ns=now,
            timeout_ns=self.estop_timeout_ns,
        )

    def cmd_is_alive(self, now: int) -> bool:
        """Reject stale or time-reversed drive commands from forwarding."""
        return is_fresh_ns(
            self.last_cmd_ns,
            now_ns=now,
            timeout_ns=self.cmd_timeout_ns,
        )

    def current_requested_cmd_is_zero(self, now: int) -> bool:
        """Treat a timed-out command as zero for reset, never for forwarding."""
        return not self.cmd_is_alive(now) or is_zero_twist(self.last_cmd)

    def log_transition_if_needed(self, state: SafetyState) -> None:
        """Log state changes with a severity that drives terminal color."""
        if state is self.last_logged_state:
            return
        old = self.last_logged_state
        severity, marker = describe_safety_transition(state)
        log_with_severity(
            self.get_logger(),
            severity,
            (
                f"{marker} {old.value} -> {state.value} "
                f"estop={self.estop_active} "
                f"reset_armed={self.gate.reset_armed}"
            ),
        )
        self.last_logged_state = state


def main(args=None) -> None:
    """Run the VICA software Safety Supervisor."""
    rclpy.init(args=args)
    node = SafetySupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
