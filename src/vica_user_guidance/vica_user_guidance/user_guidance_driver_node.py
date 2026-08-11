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
from rclpy.node import Node
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
from .timebase import sec_to_ns


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

        # 터치센서 대체 입력. **enable_serial=false(mock 모드)에서만** 연다.
        #
        # 터치센서가 미장착이라 user_contact 가 False 하드코딩이었고, 그 탓에 모드
        # 진입(터치 3초)·놓침(유예 0.5초)·재개를 하나도 시험할 수 없었다.
        #
        # 값을 고정하지 않고 토픽으로 받는 이유: true 고정은 vica_scenario.md 2-1.3 의
        # fail-safe 를 뒤집는다. 그 절은 Normally Closed 배선으로 "단선 = 놓음 = 정지"를
        # 정했는데, true 고정은 "모르면 잡고 있다"라 정반대이고 실기 경로에 남으면
        # 손 놓음 정지가 영구히 발동하지 않는다. false 고정은 안전하지만 진입 자체가
        # 안 돼 시험할 것이 없다. 무엇보다 판정 유예 0.5초와 재개 흐름은 **잡았다
        # 놓는 시점**이 있어야 재현되므로 고정값으로는 만들 수 없다.
        #
        # 기본값은 False 다 — 모르면 놓은 것으로 본다(fail-safe 방향).
        self._mock_user_contact = False
        self._mock_contact_enabled = not enable_serial
        if self._mock_contact_enabled:
            self.create_subscription(
                Bool, "/vica/mock_user_contact", self.handle_mock_user_contact, 10
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

        self.get_logger().info(
            "Subscribed: /vica/turn_guide, /estop_state, /vica_goal_event"
        )
        self.get_logger().info("Publishing: /vica/smart_handle_state + serial")
        self.get_logger().info(
            "This node never publishes /cmd_vel* nor resets the E-stop latch."
        )
        if not enable_serial:
            self.get_logger().warn("enable_serial=false — mock mode, no serial write")
            self.get_logger().warn(
                "터치센서 mock 활성: /vica/mock_user_contact 로 user_contact 를 조작한다. "
                "계측 전용이며 실기 운용에서는 enable_serial=true 라 열리지 않는다."
            )
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

    def handle_mock_user_contact(self, msg: Bool) -> None:
        """터치센서 대체 입력. mock 모드에서만 구독이 열려 있다.

        상태가 바뀔 때만 로그를 남긴다 — 키보드 도구가 눌린 동안 주기 발행하므로
        매번 찍으면 다른 로그가 묻힌다.
        """
        value = bool(msg.data)
        if value != self._mock_user_contact:
            self.get_logger().info(
                f"[MOCK] user_contact {self._mock_user_contact} -> {value}"
            )
        self._mock_user_contact = value

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
        # 터치센서 미장착이라 실기에서는 항상 False 다. mock 모드에서만
        # /vica/mock_user_contact 가 채운다(위 구독 참고).
        msg.user_contact = self._mock_user_contact
        # 상향 통신이 없어 실제 관측이 아니다. connected와 같은 값이며 [미검증]이다.
        msg.servo_ok = connected
        msg.left_led_ok = connected
        msg.right_led_ok = connected
        msg.haptic_ok = False       # 진동 모터 미장착

        msg.fault_code = self.link.fault_code
        msg.last_state_code = self.link.last_state_code
        msg.write_error_count = self.link.write_error_count
        self.pub_state.publish(msg)

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
