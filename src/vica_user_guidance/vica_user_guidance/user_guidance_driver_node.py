"""TurnGuide cue와 Safety 상태를 병합해 Smart Handle로 상태코드를 전송한다."""

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from vica_interfaces.msg import SmartHandleState, TurnGuide

from . import protocol
from .guidance_priority import (
    GuidanceInputs,
    is_arrival_event,
    parse_goal_event,
    resolve_state_code,
)
from .range_tf_gate import RangeTfGate
from .serial_link import SerialLink
from .timebase import is_fresh_ns, sec_to_ns
from .touch_frame import TouchFrameAccumulator, resolve_contact
from .ultrasonic_frame import FrameAccumulator


class UserGuidanceDriverNode(Node):
    """cue를 병합해 아두이노로 1바이트 상태코드를 보낸다."""

    def __init__(self) -> None:
        super().__init__("user_guidance_driver_node")

        self.declare_parameter("serial_port", "/dev/vica_smart_handle")
        self.declare_parameter("baudrate", protocol.FIRMWARE_BAUDRATE)
        self.declare_parameter("send_rate_hz", 10.0)
        self.declare_parameter("diag_rate_hz", 2.0)
        self.declare_parameter("write_timeout_sec", 0.05)
        self.declare_parameter("cue_timeout_sec", 1.0)
        self.declare_parameter("estop_timeout_sec", 1.0)
        self.declare_parameter("estop_required", True)
        self.declare_parameter("reconnect_interval_sec", 2.0)
        self.declare_parameter("arrival_hold_sec", 4.0)
        self.declare_parameter("enable_serial", True)

        self.declare_parameter("ultrasonic_enabled", True)
        self.declare_parameter("ultrasonic_rate_hz", 20.0)
        self.declare_parameter("uplink_rate_hz", 20.0)
        self.declare_parameter("touch_enabled", True)
        self.declare_parameter("touch_stale_sec", 0.5)
        self.declare_parameter("touch_publish_min_interval_sec", 0.1)
        self.declare_parameter(
            "ultrasonic_topics", ["/ultrasonic/front_left", "/ultrasonic/front_right"]
        )
        self.declare_parameter(
            "ultrasonic_frame_ids", ["usonic_front_left", "usonic_front_right"]
        )
        self.declare_parameter("ultrasonic_fov_rad", 0.524)
        self.declare_parameter("ultrasonic_min_range_m", 0.02)
        self.declare_parameter("ultrasonic_max_range_m", 0.30)
        self.declare_parameter("ultrasonic_measurement_delay_ms", [210, 105])
        self.declare_parameter("ultrasonic_stale_warn_sec", 2.0)
        self.declare_parameter("ultrasonic_tf_gate", True)
        self.declare_parameter("ultrasonic_tf_target_frame", "odom")

        self.cue_timeout_ns = sec_to_ns(
            float(self.get_parameter("cue_timeout_sec").value)
        )
        self.estop_timeout_ns = sec_to_ns(
            float(self.get_parameter("estop_timeout_sec").value)
        )
        self.arrival_hold_ns = sec_to_ns(
            float(self.get_parameter("arrival_hold_sec").value)
        )
        self.estop_required = bool(self.get_parameter("estop_required").value)

        self._warn_if_arrival_hold_too_short()

        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        enable_serial = bool(self.get_parameter("enable_serial").value)
        self.link = SerialLink(
            port=str(self.get_parameter("serial_port").value),
            baudrate=int(self.get_parameter("baudrate").value),
            enabled=enable_serial,
            write_timeout_sec=float(self.get_parameter("write_timeout_sec").value),
            reconnect_interval_ns=sec_to_ns(
                float(self.get_parameter("reconnect_interval_sec").value)
            ),
        )

        self.estop_active = False
        self.estop_last_ns = None
        self.turn_direction = TurnGuide.DIRECTION_NONE
        self.turn_last_ns = None
        self.arrival_started_ns = None
        self.last_reason = None

        self.create_subscription(TurnGuide, "/vica/turn_guide", self.cb_turn, 10)
        self.create_subscription(Bool, "/estop_state", self.cb_estop, 10)
        self.create_subscription(String, "/vica_goal_event", self.cb_goal, 10)
        self.create_subscription(
            String, "/vica/haptic_request", self.cb_haptic_request, 10
        )

        self.pub_state = self.create_publisher(
            SmartHandleState, "/vica/smart_handle_state", 10
        )

        self.create_timer(
            1.0 / float(self.get_parameter("send_rate_hz").value),
            self.send_loop,
            clock=self.steady_clock,
        )
        self.create_timer(
            1.0 / float(self.get_parameter("diag_rate_hz").value),
            self.diag_loop,
            clock=self.steady_clock,
        )

        self.touch_enabled = bool(self.get_parameter("touch_enabled").value)
        self.touch_contact = False
        self.touch_last_frame_ns = None
        self.touch_stale_ns = sec_to_ns(
            float(self.get_parameter("touch_stale_sec").value)
        )
        self.touch_pub_min_ns = sec_to_ns(
            float(self.get_parameter("touch_publish_min_interval_sec").value)
        )
        self._touch_pub_last_ns = None

        self.us_enabled = bool(self.get_parameter("ultrasonic_enabled").value)
        if self.us_enabled:
            self._setup_ultrasonic()
        if self.touch_enabled:
            self._setup_touch()
        if self.us_enabled or self.touch_enabled:
            self.create_timer(
                1.0 / float(self.get_parameter("uplink_rate_hz").value),
                self.uplink_loop,
                clock=self.steady_clock,
            )

        self.get_logger().info(
            "Subscribed: /vica/turn_guide, /estop_state, /vica_goal_event"
        )
        self.get_logger().info("Publishing: /vica/smart_handle_state + serial")
        self.get_logger().info(
            "This node never publishes /cmd_vel* nor resets the E-stop latch."
        )
        if not enable_serial:
            self.get_logger().warn("enable_serial=false — mock mode, no serial write")
        if not self.estop_required:
            self.get_logger().warn(
                "estop_required=false — stale /estop_state will NOT force ESTOP. "
                "Development only."
            )

    def _warn_if_arrival_hold_too_short(self) -> None:
        """도착 표시가 잘리는 설정을 기동 시 경고한다."""
        required = protocol.firmware_arrival_duration_sec()
        configured = float(self.get_parameter("arrival_hold_sec").value)
        if configured < required:
            self.get_logger().error(
                f"arrival_hold_sec={configured}s < firmware animation {required}s. "
                "도착 표시가 잘립니다. 값을 늘리세요."
            )

    def now_ns(self) -> int:
        """Return the current STEADY_TIME instant as integer nanoseconds."""
        return self.steady_clock.now().nanoseconds

    def cb_turn(self, msg: TurnGuide) -> None:
        """cue를 받는다."""
        self.turn_direction = msg.direction
        self.turn_last_ns = self.now_ns()

    def cb_estop(self, msg: Bool) -> None:
        """중앙 래치 결과를 구독만 한다. 여기서 reset하지 않는다."""
        self.estop_active = bool(msg.data)
        self.estop_last_ns = self.now_ns()

    def cb_goal(self, msg: String) -> None:
        """goal_succeeded만 도착으로 본다."""
        event = parse_goal_event(msg.data)
        if event is None:
            self.get_logger().warn(
                "/vica_goal_event 파싱 실패 — 도착 표시가 동작하지 않습니다. "
                f"JSON에 event 키가 필요합니다. payload={msg.data[:120]!r}",
                throttle_duration_sec=5.0,
            )
            return
        if is_arrival_event(event):
            self.arrival_started_ns = self.now_ns()
            self.get_logger().info("도착 이벤트 수신 — Smart Handle 도착 표시 시작")

    def send_loop(self) -> None:
        now = self.now_ns()
        self.link.maybe_reconnect(now)

        outcome = resolve_state_code(
            GuidanceInputs(
                estop_active=self.estop_active,
                estop_last_ns=self.estop_last_ns,
                turn_direction=self.turn_direction,
                turn_last_ns=self.turn_last_ns,
                arrival_started_ns=self.arrival_started_ns,
            ),
            now_ns=now,
            estop_timeout_ns=self.estop_timeout_ns,
            cue_timeout_ns=self.cue_timeout_ns,
            arrival_hold_ns=self.arrival_hold_ns,
            estop_required=self.estop_required,
        )

        self.link.send(outcome.state_code, now)
        self._log_on_change(outcome)

    def _log_on_change(self, outcome) -> None:
        """상태가 바뀔 때만 로그를 남긴다. 10Hz 로그는 로그를 무용지물로 만든다."""
        if outcome.reason == self.last_reason:
            return
        self.last_reason = outcome.reason
        name = protocol.STATE_NAMES.get(outcome.state_code, "?")
        message = f"[HANDLE] {name}({outcome.state_code}) reason={outcome.reason}"
        if outcome.state_code == protocol.STATE_ESTOP:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def diag_loop(self) -> None:
        """진단은 전송보다 느린 주기로 발행한다."""
        msg = SmartHandleState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "smart_handle"

        connected = self.link.connected
        msg.connected = connected

        msg.uplink_fresh = self._uplink_fresh(self.now_ns())
        msg.user_contact = resolve_contact(self.touch_contact, msg.uplink_fresh)

        msg.servo_ok = connected
        msg.left_led_ok = connected
        msg.right_led_ok = connected
        msg.haptic_ok = False

        msg.fault_code = self.link.fault_code
        msg.last_state_code = self.link.last_state_code
        msg.write_error_count = self.link.write_error_count
        self.pub_state.publish(msg)

    HAPTIC_PATTERNS = {
        "short": protocol.HAPTIC_CMD_SHORT,
        "long": protocol.HAPTIC_CMD_LONG,
    }

    def cb_haptic_request(self, msg: String) -> None:
        """진동모터 수동 명령. 패턴 이름 하나를 받아 바이트 하나를 흘려보낸다."""
        name = msg.data.strip().lower()
        code = self.HAPTIC_PATTERNS.get(name)
        if code is None:
            self.get_logger().warn(
                f"[HAPTIC] 모르는 패턴 '{msg.data}' — "
                f"{sorted(self.HAPTIC_PATTERNS)} 중 하나여야 합니다. 무시합니다."
            )
            return
        ok = self.link.send_raw(code, self.now_ns())
        if ok:
            self.get_logger().info(f"[HAPTIC] {name} (0x{code:02X}) 전송")
        else:
            self.get_logger().warn(
                f"[HAPTIC] {name} 전송 실패 — 포트 상태 fault={self.link.fault_code}"
            )

    def _setup_ultrasonic(self) -> None:
        topics = [str(t) for t in self.get_parameter("ultrasonic_topics").value]
        self.us_frame_ids = [
            str(f) for f in self.get_parameter("ultrasonic_frame_ids").value
        ]
        delays = [
            int(d) for d in self.get_parameter("ultrasonic_measurement_delay_ms").value
        ]
        if not (
            len(topics) == len(self.us_frame_ids) == len(delays) == protocol.US_CHANNELS
        ):
            raise ValueError(
                "ultrasonic_topics/frame_ids/measurement_delay_ms 는 모두 "
                f"채널 수 {protocol.US_CHANNELS}개여야 합니다: "
                f"{len(topics)}/{len(self.us_frame_ids)}/{len(delays)}"
            )

        self.us_fov = float(self.get_parameter("ultrasonic_fov_rad").value)
        self.us_min_range = float(self.get_parameter("ultrasonic_min_range_m").value)
        self.us_max_range = float(self.get_parameter("ultrasonic_max_range_m").value)
        self.us_delay_ns = [ms * 1_000_000 for ms in delays]
        self.us_stale_warn_ns = sec_to_ns(
            float(self.get_parameter("ultrasonic_stale_warn_sec").value)
        )

        self.us_acc = FrameAccumulator()
        self.us_pubs = [self.create_publisher(Range, t, 10) for t in topics]
        self.us_last_frame_ns = None
        self.us_stale_warned = False

        self.us_tf_gate_enabled = bool(
            self.get_parameter("ultrasonic_tf_gate").value
        )
        self.us_tf_target = str(
            self.get_parameter("ultrasonic_tf_target_frame").value
        )
        self.us_gate = RangeTfGate(channels=protocol.US_CHANNELS)
        if self.us_tf_gate_enabled:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f"Ultrasonic Range publishing: {topics} "
            f"(TF 게이트 {'켬' if self.us_tf_gate_enabled else '끔'}"
            f", 기준 프레임 {self.us_tf_target})"
        )

    def _setup_touch(self) -> None:
        self.touch_acc = TouchFrameAccumulator()
        self.get_logger().info(
            "Touch sensor uplink enabled (D11, active-low, firmware 200ms bridge)"
        )

    def _uplink_fresh(self, now_ns: int) -> bool:
        """상향 터치 프레임이 최근에 왔는가."""
        if not self.touch_enabled:
            return False
        return is_fresh_ns(self.touch_last_frame_ns, now_ns, self.touch_stale_ns)

    def uplink_loop(self) -> None:
        """상향 시리얼을 한 번 읽어 초음파·터치 누적기에 함께 먹인다."""
        now = self.now_ns()
        data = self.link.read_available(now)

        if self.us_enabled:
            for frame in self.us_acc.feed(data):
                self.us_last_frame_ns = now
                self.us_stale_warned = False
                self._publish_ranges(frame)
            self._warn_if_ultrasonic_stale(now)

        if self.touch_enabled:
            for frame in self.touch_acc.feed(data):
                self.touch_last_frame_ns = now
                if frame.touched != self.touch_contact:
                    self.touch_contact = frame.touched
                    if not is_fresh_ns(
                        self._touch_pub_last_ns, now, self.touch_pub_min_ns
                    ):
                        self._touch_pub_last_ns = now
                        self.diag_loop()

    def _range_tf_ok(self, ch: int, stamp) -> bool:
        """이 stamp 로 `odom -> usonic_*` 를 조회할 수 있는가."""
        if not self.us_tf_gate_enabled:
            return True
        try:
            return bool(
                self.tf_buffer.can_transform(
                    self.us_tf_target,
                    self.us_frame_ids[ch],
                    Time.from_msg(stamp),
                )
            )
        except TransformException:
            return False

    def _log_range_gate(self, ch: int) -> None:
        """게이트 상태가 바뀐 순간에만 1회 남긴다(10 Hz 로그 폭주 방지)."""
        changed = self.us_gate.take_transition(ch)
        if changed is None:
            return
        frame_id = self.us_frame_ids[ch]
        if changed:
            self.get_logger().info(
                f"[초음파] {frame_id} TF 복귀 — Range 발행 재개 "
                f"(보류한 프레임 {self.us_gate.blocked_count(ch)}개)"
            )
        else:
            self.get_logger().warn(
                f"[초음파] {frame_id} TF 없음 — Range 발행 보류 "
                "(odom 끊김 동안 Nav2 컨트롤러를 지킨다)"
            )

    def _publish_ranges(self, frame) -> None:
        stamp_base = self.get_clock().now()
        for ch, mm in enumerate(frame.distances_mm):
            if mm == protocol.US_DIST_INVALID:
                continue
            stamp = (
                stamp_base - Duration(nanoseconds=self.us_delay_ns[ch])
            ).to_msg()
            allowed = self.us_gate.allow(ch, self._range_tf_ok(ch, stamp))
            self._log_range_gate(ch)
            if not allowed:
                continue
            msg = Range()
            msg.header.stamp = stamp
            msg.header.frame_id = self.us_frame_ids[ch]
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = self.us_fov
            msg.min_range = self.us_min_range
            msg.max_range = self.us_max_range
            if mm == protocol.US_CLEAR_MM or mm / 1000.0 > self.us_max_range:
                msg.range = self.us_max_range
            else:
                msg.range = mm / 1000.0
            self.us_pubs[ch].publish(msg)

    def _warn_if_ultrasonic_stale(self, now_ns: int) -> None:
        """프레임이 끊기면 1회 경고한다. 발행 중단 자체는 설계된 동작이다."""
        if self.us_last_frame_ns is None or self.us_stale_warned:
            return
        if now_ns - self.us_last_frame_ns > self.us_stale_warn_ns:
            self.us_stale_warned = True
            self.get_logger().warn(
                "초음파 프레임 수신 끊김 — Range 발행 중단 (펌웨어·배선·포트 확인)"
            )

    def shutdown_to_neutral(self) -> None:
        """종료 시 기본 상태를 1회 보내고 포트를 닫는다."""
        try:
            self.link.send(protocol.STATE_NORMAL, self.now_ns())
        except Exception:
            pass
        self.link.close()


def main(args=None) -> None:
    """Run the VICA Smart Handle guidance driver."""
    rclpy.init(args=args)
    node = UserGuidanceDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown_to_neutral()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
