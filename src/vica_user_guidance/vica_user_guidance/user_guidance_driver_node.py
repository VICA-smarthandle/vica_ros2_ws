"""TurnGuide cue와 Safety 상태를 병합해 Smart Handle로 상태코드를 전송한다.

[안전 경계] 이 노드가 하지 않는 것:
  - /cmd_vel_req, /cmd_vel_safe를 발행하지 않는다
  - Nav2 goal을 보내거나 취소하지 않는다
  - E-stop 래치를 소유하거나 reset하지 않는다 (/estop_state를 구독만 한다)
  - 상태코드 4(LINK_LOST)를 전송하지 않는다 (펌웨어 워치독 전용)

서보는 조향 장치가 아니며, 시리얼 단절은 표시·진단만 하고 주행 정지 조건으로 삼지
않는다.
"""

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String

from vica_interfaces.msg import SmartHandleState, TurnGuide

from . import protocol
from .guidance_priority import (
    GuidanceInputs,
    is_arrival_event,
    parse_goal_event,
    resolve_state_code,
)
from .serial_link import SerialLink
from .timebase import is_fresh_ns, sec_to_ns
from .touch_frame import TouchFrameAccumulator, resolve_contact
from .ultrasonic_frame import FrameAccumulator


class UserGuidanceDriverNode(Node):
    """cue를 병합해 아두이노로 1바이트 상태코드를 보낸다."""

    def __init__(self) -> None:
        super().__init__("user_guidance_driver_node")

        # config/user_guidance.yaml과 같은 값이어야 한다. launch로 띄우면 YAML이
        # 덮어쓰지만 ros2 run으로 직접 띄우면 이 값이 쓰인다. 두 값이 어긋나면
        # 실행 방식에 따라 다른 포트를 열게 된다 — test_launch_contract.py가 고정한다.
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

        # ── 초음파 (2026-08-31, docs/handoff_jetson_ultrasonic_i2c.md §6.4) ──
        # 같은 포트의 상향 프레임을 읽어 Range 로 발행한다. 노드를 따로 만들지
        # 않는 이유: 시리얼 포트는 하나뿐이고 두 프로세스가 같은 포트를 동시에
        # 열 수 없다.
        self.declare_parameter("ultrasonic_enabled", True)
        self.declare_parameter("ultrasonic_rate_hz", 20.0)  # 프레임 4.8Hz 의 4배
        # 상향 읽기 주기. 초음파·터치가 이 한 루프를 공유한다(스트림이 하나다).
        self.declare_parameter("uplink_rate_hz", 20.0)
        self.declare_parameter("touch_enabled", True)
        self.declare_parameter("touch_stale_sec", 0.5)
        self.declare_parameter(
            "ultrasonic_topics", ["/ultrasonic/front_left", "/ultrasonic/front_right"]
        )
        self.declare_parameter(
            "ultrasonic_frame_ids", ["usonic_front_left", "usonic_front_right"]
        )
        self.declare_parameter("ultrasonic_fov_rad", 0.524)   # 지향각 레벨 1(30도)
        self.declare_parameter("ultrasonic_min_range_m", 0.02)
        self.declare_parameter("ultrasonic_max_range_m", 0.30)
        # 순차 발사라 채널마다 측정 시점이 다르다. 프레임 수신 시각에서 이만큼
        # 과거를 stamp 로 쓴다 — MCU 에 시계가 없어 수신 시각을 그대로 쓰면
        # costmap 이 낡은 값을 새것으로 착각한다.
        self.declare_parameter("ultrasonic_measurement_delay_ms", [210, 105])
        self.declare_parameter("ultrasonic_stale_warn_sec", 2.0)

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

        # 입력 스냅샷. 미수신은 None이며 0으로 초기화하지 않는다.
        self.estop_active = False
        self.estop_last_ns = None
        self.turn_direction = TurnGuide.DIRECTION_NONE
        self.turn_last_ns = None
        self.arrival_started_ns = None
        self.last_reason = None

        self.create_subscription(TurnGuide, "/vica/turn_guide", self.cb_turn, 10)
        self.create_subscription(Bool, "/estop_state", self.cb_estop, 10)
        self.create_subscription(String, "/vica_goal_event", self.cb_goal, 10)
        # 햅틱 수동 명령 (2026-09-04). 주행 중에는 이 노드가 포트를 잡고 있어
        # bench_test.py 가 못 붙는다 — 그래서 여기를 거친다. **수동 전용**이며
        # 이 노드는 스스로 보내지 않는다. 자동 트리거(ESTOP·ARRIVED)는 별도
        # 결정 사항이다.
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

        # 상향 시리얼은 한 스트림이다. 초음파와 터치가 각자 read 하면 서로 바이트를
        # 뺏어 양쪽 다 프레임이 깨진다. 읽기는 uplink_loop 한 곳에서만 하고, 읽은
        # 바이트를 두 누적기에 먹인다.
        self.us_enabled = bool(self.get_parameter("ultrasonic_enabled").value)
        self.touch_enabled = bool(self.get_parameter("touch_enabled").value)
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
        """도착 표시가 잘리는 설정을 기동 시 경고한다.

        펌웨어 재생 시간(3.5초)보다 짧으면 ROS가 먼저 코드 0을 보내 마지막 프레임이
        날아간다. 사용자에게는 "2.5회 점멸"로 보인다.
        """
        required = protocol.firmware_arrival_duration_sec()
        configured = float(self.get_parameter("arrival_hold_sec").value)
        if configured < required:
            self.get_logger().error(
                f"arrival_hold_sec={configured}s < firmware animation {required}s. "
                "도착 표시가 잘립니다. 값을 늘리세요."
            )

    # ── 콜백 ───────────────────────────────────────────

    def now_ns(self) -> int:
        """Return the current STEADY_TIME instant as integer nanoseconds."""
        return self.steady_clock.now().nanoseconds

    def cb_turn(self, msg: TurnGuide) -> None:
        """cue를 받는다.

        phase는 보지 않고 direction만 쓴다. PHASE_COMPLETE는 20Hz에서 50ms만 관측되어
        놓칠 수 있고, phase에 의존하면 COMPLETE 유실 시 서보가 기울어진 채 남는다.
        direction이 NONE이면 자동으로 기본 상태로 떨어진다.
        """
        self.turn_direction = msg.direction
        self.turn_last_ns = self.now_ns()

    def cb_estop(self, msg: Bool) -> None:
        """중앙 래치 결과를 구독만 한다. 여기서 reset하지 않는다."""
        self.estop_active = bool(msg.data)
        self.estop_last_ns = self.now_ns()

    def cb_goal(self, msg: String) -> None:
        """goal_succeeded만 도착으로 본다.

        파싱 실패는 경고로 남긴다. mission_manager가 payload 형식을 바꾸면 도착
        표시가 조용히 사라지는데, 로그가 없으면 단서가 전혀 남지 않는다
        (2026-07-29 실기 검증에서 평문 payload로 실제로 겪었다).

        정상 운영에서는 발생하지 않으므로 로그가 시끄러워지지 않는다. goal_sent
        같은 다른 이벤트는 정상적으로 파싱되므로 여기에 걸리지 않는다.
        """
        event = parse_goal_event(msg.data)
        if event is None:
            # payload를 자른다. 정상 payload에도 name·reason이 들어가 길고,
            # 잘못된 payload는 얼마든지 길 수 있다. 진단에는 앞부분이면 족하다.
            # throttle이 없으면 고빈도 오류 payload가 로그를 덮는다.
            self.get_logger().warn(
                "/vica_goal_event 파싱 실패 — 도착 표시가 동작하지 않습니다. "
                f"JSON에 event 키가 필요합니다. payload={msg.data[:120]!r}",
                throttle_duration_sec=5.0,
            )
            return
        if is_arrival_event(event):
            self.arrival_started_ns = self.now_ns()
            self.get_logger().info("도착 이벤트 수신 — Smart Handle 도착 표시 시작")

    # ── 주기 처리 ──────────────────────────────────────

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

        # 터치는 상향이 살아 있을 때만 사실이다. stale 인데 마지막 값을 그대로 내면
        # 죽은 센서가 "잡고 있다"고 말하게 된다 — 모드가 그 값으로 갈리므로
        # 반드시 false 로 떨어뜨린다.
        msg.uplink_fresh = self._uplink_fresh(self.now_ns())
        msg.user_contact = resolve_contact(self.touch_contact, msg.uplink_fresh)

        # 서보·LED 는 상향으로 관측하지 않는다. connected와 같은 값이며 [미검증]이다.
        msg.servo_ok = connected
        msg.left_led_ok = connected
        msg.right_led_ok = connected
        msg.haptic_ok = False       # 진동 모터 미장착

        msg.fault_code = self.link.fault_code
        msg.last_state_code = self.link.last_state_code
        msg.write_error_count = self.link.write_error_count
        self.pub_state.publish(msg)

    # ── 햅틱 ───────────────────────────────────────────

    HAPTIC_PATTERNS = {
        "short": protocol.HAPTIC_CMD_SHORT,
        "long": protocol.HAPTIC_CMD_LONG,
    }

    def cb_haptic_request(self, msg: String) -> None:
        """진동모터 수동 명령. 패턴 이름 하나를 받아 바이트 하나를 흘려보낸다.

        10 Hz 상태코드 사이에 끼워 넣는 것이라 LED·서보에는 영향이 없다 — 펌웨어가
        이 바이트를 applyState() 로 보내지 않는다. send() 가 아니라 send_raw() 를
        쓰는 이유는 last_state_code 를 더럽히지 않기 위해서다.
        """
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

    # ── 초음파 ─────────────────────────────────────────

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
            # 채널 수가 어긋난 채 돌면 엉뚱한 frame_id 로 발행된다. 기동을 막는
            # 편이 조용히 틀리는 것보다 낫다.
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
        self.get_logger().info(f"Ultrasonic Range publishing: {topics}")

    # ── 터치 ───────────────────────────────────────────

    def _setup_touch(self) -> None:
        self.touch_acc = TouchFrameAccumulator()
        self.touch_stale_ns = sec_to_ns(
            float(self.get_parameter("touch_stale_sec").value)
        )
        self.touch_last_frame_ns = None
        self.touch_contact = False
        self.get_logger().info("Touch sensor uplink enabled (D10)")

    def _uplink_fresh(self, now_ns: int) -> bool:
        """상향 터치 프레임이 최근에 왔는가.

        초음파의 stale 시한(2.0초)과 따로 둔다. 터치는 20Hz 라 훨씬 짧아도 되고,
        길게 잡으면 죽은 센서를 오래 살아 있다고 말하게 된다.
        """
        if not self.touch_enabled:
            return False
        return is_fresh_ns(self.touch_last_frame_ns, now_ns, self.touch_stale_ns)

    # ── 상향 공통 ──────────────────────────────────────

    def uplink_loop(self) -> None:
        """상향 시리얼을 한 번 읽어 초음파·터치 누적기에 함께 먹인다.

        [중요] 읽기는 여기 한 곳뿐이다. 두 곳에서 read 하면 한쪽이 상대의 바이트를
        가져가 양쪽 프레임이 모두 깨진다. 두 누적기는 각자 자기 헤더만 찾고 나머지
        바이트는 1개씩 버리므로, 같은 스트림을 두 번 훑어도 서로를 삼키지 않는다.
        """
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
                    # 주기 발행(2Hz)만으로는 0.5초 판정에 샘플이 1개뿐이다.
                    # 바뀐 순간 한 번 더 내면 상위가 겪는 지연이 프레임 주기
                    # (50ms)로 줄고, 평소 대역폭은 그대로다.
                    self.diag_loop()

    def _publish_ranges(self, frame) -> None:
        stamp_base = self.get_clock().now()
        for ch, mm in enumerate(frame.distances_mm):
            if mm == protocol.US_DIST_INVALID:
                # 채널 무효(3회 연속 실패) — 그 채널만 건너뛴다. 한 센서 고장이
                # 다른 채널을 죽이지 않는다.
                continue
            msg = Range()
            msg.header.stamp = (
                stamp_base - Duration(nanoseconds=self.us_delay_ns[ch])
            ).to_msg()
            msg.header.frame_id = self.us_frame_ids[ch]
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = self.us_fov
            msg.min_range = self.us_min_range
            msg.max_range = self.us_max_range
            if mm == protocol.US_CLEAR_MM or mm / 1000.0 > self.us_max_range:
                # 에코 없음(또는 관심 범위 밖 실거리) = 그 부채꼴은 뚫려 있다.
                # max_range 로 발행해야 RangeSensorLayer 가 부채꼴을 지운다.
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
        """종료 시 기본 상태를 1회 보내고 포트를 닫는다.

        보내지 않으면 핸들이 마지막 표시(예: 주황 물결)를 유지하다 1.5초 뒤 워치독으로
        빨간불이 된다. 중립 표시로 끝내는 편이 낫다.
        """
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
