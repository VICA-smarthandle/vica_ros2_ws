"""ROS wiring for the central VICA emergency-stop latch."""

import can
import rclpy
from diagnostic_updater import Updater
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .diagnostics import LABEL_LATCH, STALE, latch_summary, sources_text
from .emergency_latch import EmergencyLatch, LatchSnapshot
from .freshness import sec_to_ns
from .logging_utils import log_with_severity


PID_PNT_IO_MONITOR = 0xF1


def f1_frame_means_estop_active(
    data: bytes,
    byte_1: int,
    byte_2: int,
    mask: int,
    active_value: int,
) -> bool:
    """Decode the configured active-low bits from an MDROBOT F1 frame."""
    if not 0 <= byte_1 < len(data) or not 0 <= byte_2 < len(data):
        raise ValueError("F1 byte index is outside the received frame")
    return (
        data[byte_1] & mask == active_value
        and data[byte_2] & mask == active_value
    )


def classify_latch_state(snapshot: LatchSnapshot) -> str:
    """Name the operator-facing latch state for a snapshot.

    입력이 끊긴 원인(`*_stale`)은 FAULT다. 모터 노드가 죽어 `/motor/can_ok`가
    끊긴 경우를 ESTOP_ACTIVE로 찍으면 아무도 누르지 않은 물리 버튼을 찾게 된다.
    """
    stale_sources = ("physical_stale", "motor_can_stale")
    if any(source in snapshot.active_sources for source in stale_sources):
        return "FAULT"
    if snapshot.active_sources:
        return "ESTOP_ACTIVE"
    if snapshot.latched:
        return "ESTOP_RELEASED_WAIT_RESET"
    return "CLEARED"


def describe_latch_transition(old: str, new: str) -> tuple[str, str]:
    """Map a latch state transition to a ROS log severity and marker."""
    del old
    if new == "ESTOP_ACTIVE":
        return "error", "[ESTOP ACTIVE]"
    if new == "FAULT":
        return "error", "[FAULT]"
    if new == "ESTOP_RELEASED_WAIT_RESET":
        return "warn", "[WAIT RESET]"
    return "info", "[ESTOP CLEARED]"


class EmergencyStopNode(Node):
    """Latch physical, app, and voice E-stop sources into one ROS state."""

    def __init__(self) -> None:
        super().__init__("emergency_stop_node")

        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("input_mode", "can_f1")
        self.declare_parameter("can_iface", "can1")
        self.declare_parameter("driver_response_id", 0x701)
        self.declare_parameter("f1_required_packet_index", 0)
        self.declare_parameter("f1_check_byte_1", 2)
        self.declare_parameter("f1_check_byte_2", 3)
        self.declare_parameter("f1_check_mask", 0x10)
        self.declare_parameter("f1_active_value", 0x00)
        self.declare_parameter("f1_timeout_sec", 0.5)
        self.declare_parameter("log_f1_frames", True)
        self.declare_parameter("motor_can_timeout_sec", 0.5)

        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.input_mode = str(self.get_parameter("input_mode").value)
        self.can_iface = str(self.get_parameter("can_iface").value)
        self.driver_response_id = int(
            self.get_parameter("driver_response_id").value
        )
        self.f1_required_packet_index = int(
            self.get_parameter("f1_required_packet_index").value
        )
        self.f1_check_byte_1 = int(
            self.get_parameter("f1_check_byte_1").value
        )
        self.f1_check_byte_2 = int(
            self.get_parameter("f1_check_byte_2").value
        )
        self.f1_check_mask = int(self.get_parameter("f1_check_mask").value)
        self.f1_active_value = int(
            self.get_parameter("f1_active_value").value
        )
        self.f1_timeout_sec = float(
            self.get_parameter("f1_timeout_sec").value
        )
        self.log_f1_frames = bool(self.get_parameter("log_f1_frames").value)
        self.motor_can_timeout_sec = float(
            self.get_parameter("motor_can_timeout_sec").value
        )

        # 모든 watchdog·throttle은 단일 STEADY_TIME clock과 정수 나노초를 쓴다.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.f1_timeout_ns = sec_to_ns(self.f1_timeout_sec)
        self.motor_can_timeout_ns = sec_to_ns(self.motor_can_timeout_sec)

        self.latch = EmergencyLatch(
            f1_timeout_ns=self.f1_timeout_ns,
            motor_can_timeout_ns=self.motor_can_timeout_ns,
            initially_latched=True,
        )
        self.bus = None
        self.last_f1_log_ns = None
        self.last_can_error_log_ns = None
        self.last_latch_state = "IDLE"
        # 진단은 publish_loop가 이미 만든 스냅샷을 읽는다. 진단 콜백에서
        # latch.evaluate()를 다시 부르면 CAN을 두 번 비우고 래치를 다시 세우는
        # 부작용이 생긴다. 관측은 관측만 해야 한다.
        self.last_snapshot: LatchSnapshot | None = None

        self.pub_estop = self.create_publisher(Bool, "/emergency_stop", 10)
        self.pub_estop_state = self.create_publisher(Bool, "/estop_state", 10)
        self.create_subscription(
            Bool,
            "/app_emergency_stop",
            self.app_estop_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/voice_emergency_stop",
            self.voice_estop_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/emergency_stop_input",
            self.test_input_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/motor/can_ok",
            self.motor_can_callback,
            10,
        )
        self.create_service(
            Trigger,
            "/vica_safety/internal/estop_reset",
            self.reset_callback,
        )
        self.create_timer(
            1.0 / self.publish_hz,
            self.publish_loop,
            clock=self.steady_clock,
        )

        # /diagnostics가 없으면 aggregator의 Safety 경로가 항목 0개로 남아 경로
        # 자체가 Stale이 된다. 정상인데도 관리자 화면에 고장으로 뜬다.
        self.diag_updater = Updater(self)
        self.diag_updater.setHardwareID(self.can_iface)
        self.diag_updater.add(LABEL_LATCH, self.diagnose_latch)

        if self.input_mode == "can_f1":
            self.open_can_bus()
        elif self.input_mode != "test_topic":
            self.get_logger().error(
                f"[FAULT] unsupported input_mode={self.input_mode}; "
                "physical input remains stale"
            )

        self.get_logger().warn(
            "This software latch does not replace hardware torque removal."
        )
        self.get_logger().info(
            "Subscribed: /app_emergency_stop, /voice_emergency_stop, "
            "/motor/can_ok"
        )
        self.get_logger().info(
            "Publishing central latch: /emergency_stop, /estop_state"
        )
        self.get_logger().info(
            "Internal service: /vica_safety/internal/estop_reset"
        )

    def open_can_bus(self) -> None:
        """Open a read-only SocketCAN handle; stale input remains fail-safe."""
        try:
            self.bus = can.interface.Bus(
                channel=self.can_iface,
                interface="socketcan",
            )
        except (can.CanError, OSError) as exc:
            self.bus = None
            self.get_logger().error(
                f"[FAULT] cannot open CAN F1 input iface={self.can_iface}: {exc}"
            )
            return
        self.get_logger().info(
            "CAN F1 input enabled: "
            f"iface={self.can_iface} "
            f"response_id=0x{self.driver_response_id:03X} "
            f"packet_index={self.f1_required_packet_index}"
        )

    def now_ns(self) -> int:
        """Return the current STEADY_TIME instant as integer nanoseconds."""
        return self.steady_clock.now().nanoseconds

    def app_estop_callback(self, msg: Bool) -> None:
        """Update the app source without granting reset authority to false."""
        self.latch.update_source("app", bool(msg.data), self.now_ns())

    def voice_estop_callback(self, msg: Bool) -> None:
        """Update the voice source without granting reset authority to false."""
        self.latch.update_source("voice", bool(msg.data), self.now_ns())

    def test_input_callback(self, msg: Bool) -> None:
        """Use the test topic as the physical source only in explicit test mode."""
        if self.input_mode == "test_topic":
            self.latch.mark_physical_seen(bool(msg.data), self.now_ns())

    def motor_can_callback(self, msg: Bool) -> None:
        """Feed the motor CAN link report into the central latch."""
        self.latch.mark_motor_can_seen(bool(msg.data), self.now_ns())

    def reset_callback(self, request, response):
        """Clear only the central latch after every source is safe and fresh."""
        del request
        accepted, message = self.latch.try_reset(self.now_ns())
        response.success = accepted
        response.message = message
        if accepted:
            self.get_logger().info(
                "[ESTOP LATCH CLEARED] estop=False; supervisor remains locked"
            )
        else:
            self.get_logger().warn(
                f"[RESET REJECTED] step=estop_reset reason={message}"
            )
        return response

    def publish_loop(self) -> None:
        """Refresh physical input, evaluate the latch, and publish it."""
        if self.input_mode == "can_f1":
            self.drain_can_f1_frames()

        snapshot = self.latch.evaluate(self.now_ns())
        self.last_snapshot = snapshot
        msg = Bool()
        msg.data = snapshot.latched
        self.pub_estop.publish(msg)
        self.pub_estop_state.publish(msg)
        self.log_transition_if_needed(snapshot)

    def can_input_is_ready(self) -> bool:
        """Report whether the configured physical input path is actually open."""
        if self.input_mode == "can_f1":
            return self.bus is not None
        # test_topic 모드는 CAN을 쓰지 않는다. 경로 없음을 고장으로 보고하지 않는다.
        return True

    def diagnose_latch(self, stat):
        """Report whether this node can still judge safety, not whether it stopped.

        래치가 걸려 있는 것은 결함이 아니라 상태다. 등급은 판정 능력만 반영하고
        래치는 값으로 싣는다. 근거는 diagnostics 모듈 docstring에 있다.
        """
        snapshot = self.last_snapshot
        if snapshot is None:
            stat.summary(STALE, '아직 첫 판정을 수행하지 않았습니다')
            return stat

        level, message = latch_summary(
            can_ready=self.can_input_is_ready(),
            physical_fresh=snapshot.physical_fresh,
            # LatchSnapshot은 motor_can 신선도를 따로 담지 않는다. evaluate()가
            # 그것을 active_sources에 넣으므로 여기서 되읽는다.
            motor_can_fresh='motor_can_stale' not in snapshot.active_sources,
            latch_state=classify_latch_state(snapshot),
        )
        stat.summary(level, message)
        stat.add('latched', str(snapshot.latched))
        stat.add('latch_state', classify_latch_state(snapshot))
        stat.add('active_sources', sources_text(snapshot.active_sources))
        stat.add('physical_fresh', str(snapshot.physical_fresh))
        stat.add('reset_allowed', str(snapshot.reset_allowed))
        stat.add('input_mode', self.input_mode)
        return stat

    def drain_can_f1_frames(self) -> None:
        """Drain matching F1 frames without transmitting CAN data."""
        if self.bus is None:
            return
        try:
            for _ in range(50):
                msg = self.bus.recv(timeout=0.0)
                if msg is None:
                    break
                if msg.arbitration_id != self.driver_response_id:
                    continue
                data = bytes(msg.data)
                if len(data) != 8 or data[0] != PID_PNT_IO_MONITOR:
                    continue
                if data[1] != self.f1_required_packet_index:
                    continue
                active = f1_frame_means_estop_active(
                    data,
                    self.f1_check_byte_1,
                    self.f1_check_byte_2,
                    self.f1_check_mask,
                    self.f1_active_value,
                )
                now = self.now_ns()
                self.latch.mark_physical_seen(active, now)
                self.log_f1_frame_if_needed(data, active, now)
        except (can.CanError, OSError) as exc:
            self.log_can_error_if_needed(exc)

    def log_f1_frame_if_needed(
        self,
        data: bytes,
        active: bool,
        now: int,
    ) -> None:
        """Throttle raw F1 diagnostics while preserving state transition logs."""
        if not self.log_f1_frames:
            return
        if (
            self.last_f1_log_ns is not None
            and 0 <= now - self.last_f1_log_ns < sec_to_ns(0.2)
        ):
            return
        hex_data = " ".join(f"{value:02X}" for value in data)
        self.get_logger().info(
            f"F1 data={hex_data} physical_estop={active}"
        )
        self.last_f1_log_ns = now

    def log_can_error_if_needed(self, exc: Exception) -> None:
        """Throttle repeated CAN receive failures."""
        now = self.now_ns()
        if (
            self.last_can_error_log_ns is not None
            and 0 <= now - self.last_can_error_log_ns < sec_to_ns(1.0)
        ):
            return
        self.get_logger().error(f"[FAULT] CAN F1 receive failed: {exc}")
        self.last_can_error_log_ns = now

    def log_transition_if_needed(self, snapshot: LatchSnapshot) -> None:
        """Emit a severity-colored log only when the latch state changes."""
        state = classify_latch_state(snapshot)
        if state == self.last_latch_state:
            return

        severity, marker = describe_latch_transition(
            self.last_latch_state,
            state,
        )
        sources = ",".join(snapshot.active_sources) or "none"
        text = (
            f"{marker} {self.last_latch_state} -> {state} "
            f"estop={snapshot.latched} source={sources}"
        )
        log_with_severity(self.get_logger(), severity, text)
        self.last_latch_state = state

    def destroy_node(self) -> None:
        """Close the read-only CAN handle before shutting down ROS."""
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except (can.CanError, OSError):
                pass
        super().destroy_node()


def main(args=None) -> None:
    """Run the central E-stop latch node."""
    rclpy.init(args=args)
    node = EmergencyStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
