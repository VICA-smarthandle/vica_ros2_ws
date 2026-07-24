"""Public E-stop and reset orchestration ROS node."""

from datetime import datetime, timezone
import json
import threading
import time
from typing import Any, Optional

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .logging_utils import log_with_severity
from .reset_sequence import ResetSequence


def has_active_navigation_goals(statuses) -> bool:
    """Treat accepted, executing, and canceling Nav2 goals as active."""
    active = {
        GoalStatus.STATUS_ACCEPTED,
        GoalStatus.STATUS_EXECUTING,
        GoalStatus.STATUS_CANCELING,
    }
    return any(status in active for status in statuses)


def build_app_state(
    app_active: bool,
    emergency_active: bool,
    safety_state: str,
    message: str,
    timestamp: str,
) -> dict:
    """Build the backward-compatible authoritative app state payload."""
    return {
        "node": "app_emergency_node",
        "active": emergency_active,
        "app_active": app_active,
        "emergency_active": emergency_active,
        "safety_state": safety_state,
        "reset_allowed": (
            not emergency_active
            and safety_state == "ESTOP_RELEASED_WAIT_RESET"
        ),
        "message": message,
        "timestamp": timestamp,
    }


class AppEmergencyNode(Node):
    """Own public E-stop services and the complete reset orchestration."""

    def __init__(self) -> None:
        super().__init__("app_emergency_node")

        self.declare_parameter("activate_service", "/app_estop_activate")
        self.declare_parameter("app_reset_service", "/app_estop_reset")
        self.declare_parameter("maintenance_reset_service", "/safety_reset")
        self.declare_parameter("app_emergency_stop_topic", "/app_emergency_stop")
        self.declare_parameter("state_topic", "/app_estop_state")
        self.declare_parameter("estop_reset_service", "/vica_safety/internal/estop_reset")
        self.declare_parameter(
            "supervisor_reset_service",
            "/vica_safety/internal/supervisor_reset",
        )
        self.declare_parameter("navigate_action_name", "/navigate_to_pose")
        self.declare_parameter("app_estop_publish_hz", 10.0)
        self.declare_parameter("state_publish_hz", 1.0)
        self.declare_parameter("call_timeout_sec", 2.0)
        self.declare_parameter("state_timeout_sec", 2.0)
        self.declare_parameter("source_settle_sec", 0.2)

        self.activate_service = str(
            self.get_parameter("activate_service").value
        )
        self.app_reset_service = str(
            self.get_parameter("app_reset_service").value
        )
        self.maintenance_reset_service = str(
            self.get_parameter("maintenance_reset_service").value
        )
        app_topic = str(self.get_parameter("app_emergency_stop_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        estop_reset_service = str(
            self.get_parameter("estop_reset_service").value
        )
        supervisor_reset_service = str(
            self.get_parameter("supervisor_reset_service").value
        )
        action_name = str(
            self.get_parameter("navigate_action_name").value
        ).rstrip("/")
        app_publish_hz = float(
            self.get_parameter("app_estop_publish_hz").value
        )
        state_publish_hz = float(
            self.get_parameter("state_publish_hz").value
        )
        self.call_timeout_sec = float(
            self.get_parameter("call_timeout_sec").value
        )
        self.state_timeout_sec = float(
            self.get_parameter("state_timeout_sec").value
        )
        self.source_settle_sec = float(
            self.get_parameter("source_settle_sec").value
        )

        self.cb_group = ReentrantCallbackGroup()
        self.reset_lock = threading.Lock()
        self.state_condition = threading.Condition()
        self.sequence = ResetSequence()

        self.app_active = False
        self.emergency_active = True
        self.last_emergency_time = 0.0
        self.safety_state = "IDLE"
        self.last_safety_state_time = 0.0
        self.nav_statuses = []
        self.last_nav_status_time = 0.0
        self.last_message = "Safety 상태 확인 대기 중"

        self.app_estop_publisher = self.create_publisher(Bool, app_topic, 10)
        self.state_publisher = self.create_publisher(String, state_topic, 10)
        self.create_subscription(
            Bool,
            "/emergency_stop",
            self.emergency_callback,
            10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            String,
            "/safety_state",
            self.safety_state_callback,
            10,
            callback_group=self.cb_group,
        )
        status_topic = f"{action_name}/_action/status"
        self.create_subscription(
            GoalStatusArray,
            status_topic,
            self.nav_status_callback,
            qos_profile_action_status_default,
            callback_group=self.cb_group,
        )

        cancel_service = f"{action_name}/_action/cancel_goal"
        self.cancel_client = self.create_client(
            CancelGoal,
            cancel_service,
            callback_group=self.cb_group,
        )
        self.estop_reset_client = self.create_client(
            Trigger,
            estop_reset_service,
            callback_group=self.cb_group,
        )
        self.supervisor_reset_client = self.create_client(
            Trigger,
            supervisor_reset_service,
            callback_group=self.cb_group,
        )

        self.create_service(
            Trigger,
            self.activate_service,
            self.handle_activate,
            callback_group=self.cb_group,
        )
        self.create_service(
            Trigger,
            self.app_reset_service,
            self.handle_reset,
            callback_group=self.cb_group,
        )
        self.create_service(
            Trigger,
            self.maintenance_reset_service,
            self.handle_reset,
            callback_group=self.cb_group,
        )
        self.create_timer(
            1.0 / app_publish_hz,
            self.publish_app_estop,
            callback_group=self.cb_group,
        )
        self.create_timer(
            1.0 / state_publish_hz,
            self.publish_state,
            callback_group=self.cb_group,
        )

        self.get_logger().warn(
            "Public /safety_reset is an unauthenticated maintenance [GAP]."
        )
        self.get_logger().info(
            "Reset services share one orchestrator: "
            f"{self.app_reset_service}, {self.maintenance_reset_service}"
        )
        self.get_logger().info(
            "Reset clients: "
            f"{cancel_service}, {estop_reset_service}, "
            f"{supervisor_reset_service}"
        )

    def emergency_callback(self, msg: Bool) -> None:
        """Track the authoritative central latch for app state and reset waits."""
        with self.state_condition:
            self.emergency_active = bool(msg.data)
            self.last_emergency_time = time.time()
            self.state_condition.notify_all()

    def safety_state_callback(self, msg: String) -> None:
        """Track Safety Supervisor state for the final reset confirmation."""
        with self.state_condition:
            self.safety_state = str(msg.data)
            self.last_safety_state_time = time.time()
            self.state_condition.notify_all()

    def nav_status_callback(self, msg: GoalStatusArray) -> None:
        """Track active Nav2 action states, including canceling goals."""
        with self.state_condition:
            self.nav_statuses = [item.status for item in msg.status_list]
            self.last_nav_status_time = time.time()
            self.state_condition.notify_all()

    def handle_activate(self, request, response):
        """Latch the app source immediately and cancel Nav2 goals best-effort."""
        del request
        self.app_active = True
        self.publish_app_estop()
        nav_goal_clear, note = self.ensure_no_active_nav_goal()
        self.last_message = "앱 E-stop이 활성화되었습니다."
        if nav_goal_clear:
            self.last_message += " Nav2에 활성 Goal이 없음을 확인했습니다."
        else:
            self.last_message += f" Nav2 Goal 정리 확인 실패: {note}"
        self.get_logger().error(
            "[ESTOP ACTIVE] estop=True source=app "
            f"nav_goal_check={nav_goal_clear} reason={note}"
        )
        response.success = True
        response.message = self.last_message
        return response

    def handle_reset(self, request, response):
        """Run the same fail-closed orchestration for app and terminal reset."""
        del request
        if not self.reset_lock.acquire(blocking=False):
            return self.reject_reset(
                response,
                "nav_goal_check",
                "another reset is already in progress",
                sequence_started=False,
            )
        try:
            return self.run_reset_sequence(response)
        finally:
            self.reset_lock.release()

    def run_reset_sequence(self, response):
        """Cancel goals, clear the latch, then re-arm the supervisor."""
        self.sequence.begin()
        self.app_active = False
        self.publish_app_estop()

        nav_goal_clear, message = self.ensure_no_active_nav_goal()
        if not nav_goal_clear:
            return self.reject_reset(response, "nav_goal_check", message)
        self.sequence.record("nav_goal_check", True, message)

        time.sleep(max(0.0, self.source_settle_sec))
        accepted, message = self.call_trigger(self.estop_reset_client, "estop_reset")
        if not accepted:
            return self.reject_reset(response, "estop_reset", message)
        self.sequence.record("estop_reset", True, message)

        if not self.wait_for_emergency_clear():
            return self.reject_reset(
                response,
                "emergency_clear",
                "fresh /emergency_stop=false was not observed",
            )
        self.sequence.record("emergency_clear", True, "emergency stop cleared")

        accepted, message = self.call_trigger(
            self.supervisor_reset_client,
            "supervisor_reset",
        )
        if not accepted:
            return self.reject_reset(response, "supervisor_reset", message)
        self.sequence.record("supervisor_reset", True, message)

        if not self.wait_for_safety_ready():
            return self.reject_reset(
                response,
                "ready",
                "fresh /safety_state=READY_TO_GO was not observed",
            )
        result = self.sequence.record("ready", True, "READY_TO_GO")
        self.last_message = (
            "Safety reset 완료: Nav2 Goal 없음, 중앙 래치 해제, "
            "Supervisor READY_TO_GO 확인"
        )
        self.get_logger().info(
            "[RESET ACCEPTED] ESTOP_RELEASED_WAIT_RESET -> READY_TO_GO "
            "estop=False"
        )
        response.success = result.success
        response.message = self.last_message
        return response

    def reject_reset(
        self,
        response,
        step: str,
        reason: str,
        sequence_started: bool = True,
    ):
        """Stop the sequence at the first failed step and report its cause."""
        if sequence_started and self.sequence.next_step == step:
            self.sequence.record(step, False, reason)
        self.last_message = f"Safety reset 거부: step={step}, reason={reason}"
        severity = "error" if "stale" in reason or "not observed" in reason else "warn"
        log_with_severity(
            self.get_logger(),
            severity,
            f"[RESET REJECTED] step={step} reason={reason}",
        )
        response.success = False
        response.message = self.last_message
        return response

    def ensure_no_active_nav_goal(self) -> tuple[bool, str]:
        """Use the latest Nav2 state, requiring a new terminal state after cancel."""
        with self.state_condition:
            nav_status_seen = self.last_nav_status_time > 0.0
            nav_statuses = tuple(self.nav_statuses)

        if not nav_status_seen:
            nav2_running = self.cancel_client.wait_for_service(timeout_sec=0.0)
            if not nav2_running:
                return True, "Nav2 is not running; goal check skipped"
            return True, "Nav2 has no goal status history; goal check skipped"
        if not has_active_navigation_goals(nav_statuses):
            return True, "no active Nav2 goal"

        if not self.cancel_client.wait_for_service(timeout_sec=1.0):
            return False, "Nav2 cancel service is unavailable"
        request_started = time.time()
        result = self.call_sync(self.cancel_client, CancelGoal.Request())
        if result is None:
            return False, "Nav2 cancel response timed out"
        if result.return_code != CancelGoal.Response.ERROR_NONE:
            return False, f"Nav2 cancel return_code={result.return_code}"
        if not result.goals_canceling:
            return True, "no active Nav2 goal at cancel time"

        confirmed = self.wait_for_condition(
            lambda: (
                self.last_nav_status_time >= request_started
                and not has_active_navigation_goals(self.nav_statuses)
            )
        )
        if not confirmed:
            return False, "active or canceling Nav2 goal remains"
        return True, "all Nav2 goals canceled"

    def call_trigger(self, client, step: str) -> tuple[bool, str]:
        """Call one internal Trigger service with a finite deadline."""
        if not client.wait_for_service(timeout_sec=1.0):
            return False, f"{step} service is unavailable"
        result = self.call_sync(client, Trigger.Request())
        if result is None:
            return False, f"{step} response timed out"
        return bool(result.success), str(result.message)

    def call_sync(self, client, request) -> Optional[Any]:
        """Wait for a ROS service future while other executor threads run."""
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda unused: done.set())
        if not done.wait(timeout=self.call_timeout_sec):
            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().error(f"service call failed: {exc}")
            return None

    def wait_for_condition(self, predicate) -> bool:
        """Wait for subscription state with a finite monotonic deadline."""
        deadline = time.monotonic() + self.state_timeout_sec
        with self.state_condition:
            while not predicate():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self.state_condition.wait(timeout=min(remaining, 0.1))
        return True

    def wait_for_emergency_clear(self) -> bool:
        """Require a fresh false central latch after its internal reset."""
        started = time.time()
        return self.wait_for_condition(
            lambda: (
                self.last_emergency_time >= started
                and not self.emergency_active
            )
        )

    def wait_for_safety_ready(self) -> bool:
        """Require a fresh READY_TO_GO state after Supervisor reset."""
        started = time.time()
        return self.wait_for_condition(
            lambda: (
                self.last_safety_state_time >= started
                and self.safety_state == "READY_TO_GO"
            )
        )

    def publish_app_estop(self) -> None:
        """Repeat the app source level without ever clearing the central latch."""
        msg = Bool()
        msg.data = self.app_active
        self.app_estop_publisher.publish(msg)

    def publish_state(self) -> None:
        """Publish authoritative integrated E-stop JSON for Flutter."""
        payload = build_app_state(
            app_active=self.app_active,
            emergency_active=self.emergency_active,
            safety_state=self.safety_state,
            message=self.last_message,
            timestamp=datetime.now(timezone.utc).isoformat(timespec="seconds"),
        )
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.state_publisher.publish(msg)


def main(args=None) -> None:
    """Run the public reset orchestrator with reentrant service callbacks."""
    rclpy.init(args=args)
    node = AppEmergencyNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
