"""ROS wiring for the VICA drive-command safety gate."""

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

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
        self.declare_parameter("estop_timeout_sec", 0.5)
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

        self.gate = SafetyGate()
        self.last_cmd = Twist()
        self.last_cmd_time = 0.0
        self.estop_active = True
        self.last_estop_time = 0.0
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
        self.create_timer(1.0 / self.publish_hz, self.control_loop)

        self.get_logger().warn(
            "Safety supervisor is a software guard; hardware E-stop remains final."
        )
        self.get_logger().info("Subscribed: /cmd_vel_req, /emergency_stop")
        self.get_logger().info("Publishing: /cmd_vel_safe, /safety_state")
        self.get_logger().info(
            "Internal service: /vica_safety/internal/supervisor_reset"
        )

    def cmd_requested_callback(self, msg: Twist) -> None:
        """Store the requested command for the periodic safety decision."""
        self.last_cmd = msg
        self.last_cmd_time = time.time()

    def estop_callback(self, msg: Bool) -> None:
        """Refresh the authoritative central E-stop latch input."""
        self.estop_active = bool(msg.data)
        self.last_estop_time = time.time()

    def reset_callback(self, request, response):
        """Re-arm drive output only after fresh E-stop and zero command checks."""
        del request
        now = time.time()
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
        now = time.time()
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

    def estop_is_fresh(self, now: float) -> bool:
        """Reject missing or stale central E-stop input."""
        return (
            self.last_estop_time > 0.0
            and now - self.last_estop_time <= self.estop_timeout_sec
        )

    def cmd_is_alive(self, now: float) -> bool:
        """Reject stale drive commands from forwarding."""
        return (
            self.last_cmd_time > 0.0
            and now - self.last_cmd_time <= self.cmd_timeout_sec
        )

    def current_requested_cmd_is_zero(self, now: float) -> bool:
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
