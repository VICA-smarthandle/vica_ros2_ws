#!/usr/bin/env python3
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class SafetyState(Enum):
    # IDLE:
    #   reset이 완료되지 않았거나 아직 주행 명령을 허용하지 않는 기본 대기 상태.
    IDLE = "IDLE"

    # RUNNING:
    #   E-stop이 해제되어 있고 reset이 완료되었으며, 살아 있는 주행 명령을
    #   /cmd_vel_safe로 전달하는 상태.
    RUNNING = "RUNNING"

    # ESTOP_ACTIVE:
    #   /emergency_stop == true 이거나 E-stop 활성 상태로 판단된 상태.
    #   이 상태에서는 어떤 /cmd_vel_req가 들어와도 /cmd_vel_safe는 0이다.
    ESTOP_ACTIVE = "ESTOP_ACTIVE"

    # ESTOP_RELEASED_WAIT_RESET:
    #   E-stop 버튼은 해제되었지만 운전자가 /safety_reset을 호출하기 전 상태.
    #   이전 명령으로 자동 재출발하는 것을 막기 위해 반드시 둔다.
    ESTOP_RELEASED_WAIT_RESET = "ESTOP_RELEASED_WAIT_RESET"

    # SAFE_IDLE:
    #   reset까지 완료되었고 새 주행 명령을 기다리는 안전 대기 상태.
    SAFE_IDLE = "READY_TO_GO"

    # FAULT:
    #   E-stop 토픽 timeout 등 소프트웨어 안전 입력을 신뢰할 수 없는 상태.
    FAULT = "FAULT"


def is_zero_twist(msg: Twist, eps: float = 1e-4) -> bool:
    # reset 시점에 사용자가 아직 조이스틱/키보드 명령을 넣고 있으면 재출발 위험이 있다.
    # 그래서 모든 Twist 축이 0에 가까운지 확인한다.
    return (
        abs(msg.linear.x) < eps
        and abs(msg.linear.y) < eps
        and abs(msg.linear.z) < eps
        and abs(msg.angular.x) < eps
        and abs(msg.angular.y) < eps
        and abs(msg.angular.z) < eps
    )


def zero_twist() -> Twist:
    # geometry_msgs/Twist의 기본값은 모든 선속도/각속도가 0이다.
    return Twist()


class SafetySupervisorNode(Node):
    """
    VICA 주행 명령 보조 안전 계층.

    /cmd_vel_req를 직접 모터로 보내지 않고, E-stop 상태를 확인한 뒤
    /cmd_vel_safe로만 전달한다. 이 노드는 하드웨어 E-stop을 대체하지 않는다.
    """

    def __init__(self):
        super().__init__("safety_supervisor_node")

        # publish_hz:
        #   /cmd_vel_safe를 주기적으로 발행하는 속도.
        #   motor node는 이 토픽이 끊기면 자체 timeout으로 0rpm을 송신한다.
        #
        # cmd_timeout_sec:
        #   /cmd_vel_req가 이 시간 이상 안 들어오면 명령이 끊긴 것으로 보고 0을 발행한다.
        #
        # estop_timeout_sec:
        #   /emergency_stop 상태 토픽이 이 시간 이상 안 들어오면 안전 입력 상실로 판단한다.
        #
        # max_linear_mps, max_angular_radps:
        #   safety layer에서 한 번 더 제한하는 상한값.
        #   실제 최고속도 제한은 motor node의 knob 제한과 함께 적용된다.
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("estop_timeout_sec", 0.5)
        self.declare_parameter("max_linear_mps", 1.0)
        self.declare_parameter("max_angular_radps", 2.0)

        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.cmd_timeout_sec = float(self.get_parameter("cmd_timeout_sec").value)
        self.estop_timeout_sec = float(self.get_parameter("estop_timeout_sec").value)
        self.max_linear_mps = float(self.get_parameter("max_linear_mps").value)
        self.max_angular_radps = float(self.get_parameter("max_angular_radps").value)

        # 내부 상태는 안전한 값에서 시작한다.
        # 특히 estop_active를 true로 시작해서 /emergency_stop이 실제로 들어오기 전까지
        # /cmd_vel_safe가 0으로 고정되도록 한다.
        self.state = SafetyState.IDLE
        self.last_cmd = zero_twist()
        self.last_cmd_time = 0.0
        self.estop_active = True
        self.last_estop_time = 0.0
        self.reset_armed = False
        self.last_state_log_time = 0.0

        # /cmd_vel_safe:
        #   motor node가 유일하게 구독해야 하는 최종 승인 명령.
        #   E-stop, timeout, reset 대기 상태에서는 항상 0 Twist를 발행한다.
        self.pub_cmd_safe = self.create_publisher(Twist, "/cmd_vel_safe", 10)

        # /safety_state:
        #   디버깅과 운영 UI 표시용 상태 문자열.
        self.pub_state = self.create_publisher(String, "/safety_state", 10)

        # /cmd_vel_req:
        #   teleop, Nav2, app 등이 보내는 원본 주행 명령.
        #   이 토픽은 모터 노드에 직접 연결하지 않는다.
        self.sub_cmd_requested = self.create_subscription(
            Twist,
            "/cmd_vel_req",
            self.cmd_requested_callback,
            10,
        )
        # /emergency_stop:
        #   emergency_stop_node가 발행하는 E-stop 상태.
        #   토픽이 끊기면 안전하게 정지 상태로 전환한다.
        self.sub_estop = self.create_subscription(
            Bool,
            "/emergency_stop",
            self.estop_callback,
            10,
        )

        # /safety_reset:
        #   E-stop 해제 후 자동 재출발을 막기 위한 수동 reset 서비스.
        #   reset 시점에 /cmd_vel_req가 0이 아니면 거부한다.
        self.srv_reset = self.create_service(
            Trigger,
            "/safety_reset",
            self.reset_callback,
        )
        self.timer = self.create_timer(1.0 / self.publish_hz, self.control_loop)

        self.get_logger().warn(
            "Safety supervisor is a software guard only. "
            "Use a hardware E-stop for final motor torque removal."
        )
        self.get_logger().info("Subscribed: /cmd_vel_req, /emergency_stop")
        self.get_logger().info("Publishing: /cmd_vel_safe, /safety_state")
        self.get_logger().info("Service: /safety_reset")

    def cmd_requested_callback(self, msg: Twist):
        # 원본 명령은 저장만 하고, control_loop에서 현재 safety state를 본 뒤 전달 여부를 결정한다.
        self.last_cmd = msg
        self.last_cmd_time = time.time()

    def estop_callback(self, msg: Bool):
        # E-stop true가 들어오면 즉시 reset 허가를 취소하고 ESTOP_ACTIVE로 전환한다.
        # E-stop이 false로 돌아와도 바로 RUNNING으로 가지 않고 reset 대기 상태로 둔다.
        was_active = self.estop_active
        self.estop_active = bool(msg.data)
        self.last_estop_time = time.time()

        if self.estop_active:
            self.reset_armed = False
            self.set_state(SafetyState.ESTOP_ACTIVE)
        elif was_active and self.state == SafetyState.ESTOP_ACTIVE:
            self.set_state(SafetyState.ESTOP_RELEASED_WAIT_RESET)

    def reset_callback(self, request, response):
        del request

        # E-stop 상태가 active이거나 토픽이 stale이면 reset을 허용하지 않는다.
        if self.estop_is_active_or_stale():
            response.success = False
            response.message = "reset rejected: emergency stop is active or stale"
            return response

        # reset 버튼을 누르는 순간 원본 명령이 0이 아니면 해제 직후 튀어나갈 수 있다.
        # 그래서 reset은 반드시 원본 명령이 0인 상태에서만 허용한다.
        if not self.current_requested_cmd_is_zero():
            response.success = False
            response.message = "reset rejected: /cmd_vel_req is not zero"
            return response

        self.reset_armed = True
        self.set_state(SafetyState.SAFE_IDLE)
        response.success = True
        response.message = "safety reset accepted"
        return response

    def control_loop(self):
        # 기본 출력은 항상 0으로 시작한다.
        # 아래 조건을 모두 통과해서 RUNNING 상태가 되었을 때만 원본 명령을 제한 후 전달한다.
        safe_cmd = zero_twist()

        if self.estop_is_active_or_stale():
            self.reset_armed = False
            if self.estop_active:
                self.set_state(SafetyState.ESTOP_ACTIVE)
            else:
                self.set_state(SafetyState.FAULT)
        elif self.state == SafetyState.ESTOP_ACTIVE:
            self.set_state(SafetyState.ESTOP_RELEASED_WAIT_RESET)
        elif self.state == SafetyState.ESTOP_RELEASED_WAIT_RESET:
            # E-stop 해제 직후 상태. /safety_reset 전까지 계속 0 명령을 유지한다.
            pass
        elif not self.reset_armed:
            self.set_state(SafetyState.IDLE)
        elif not self.cmd_is_alive():
            self.set_state(SafetyState.SAFE_IDLE)
        elif is_zero_twist(self.last_cmd):
            self.set_state(SafetyState.SAFE_IDLE)
        else:
            self.set_state(SafetyState.RUNNING)
            safe_cmd = self.limited_cmd(self.last_cmd)

        if self.state != SafetyState.RUNNING:
            safe_cmd = zero_twist()

        # motor node는 /cmd_vel_safe만 구독해야 한다.
        # 이 노드가 0을 계속 발행하면 motor node도 0rpm 명령을 송신하게 된다.
        self.pub_cmd_safe.publish(safe_cmd)
        self.publish_state()
        self.log_state_periodically()

    def estop_is_active_or_stale(self) -> bool:
        # E-stop 상태 토픽이 없거나 끊긴 경우에는 안전하게 정지 조건으로 간주한다.
        if self.estop_active:
            return True
        if self.last_estop_time <= 0.0:
            return True
        return (time.time() - self.last_estop_time) > self.estop_timeout_sec

    def cmd_is_alive(self) -> bool:
        # 오래된 주행 명령을 계속 사용하는 것을 막기 위한 timeout 확인.
        if self.last_cmd_time <= 0.0:
            return False
        return (time.time() - self.last_cmd_time) <= self.cmd_timeout_sec

    def current_requested_cmd_is_zero(self) -> bool:
        # 명령이 끊긴 상태는 reset 관점에서는 0 명령으로 취급한다.
        if not self.cmd_is_alive():
            return True
        return is_zero_twist(self.last_cmd)

    def limited_cmd(self, msg: Twist) -> Twist:
        # VICA 안전 계층에서는 현재 differential drive에 필요한 linear.x와 angular.z만 전달한다.
        # 다른 축 값은 의도치 않은 소비자를 위해 전달하지 않고 0으로 둔다.
        out = Twist()
        out.linear.x = max(-self.max_linear_mps, min(self.max_linear_mps, msg.linear.x))
        out.angular.z = max(
            -self.max_angular_radps,
            min(self.max_angular_radps, msg.angular.z),
        )
        return out

    def set_state(self, new_state: SafetyState):
        if self.state != new_state:
            self.get_logger().info(f"safety state: {self.state.value} -> {new_state.value}")
            self.state = new_state

    def publish_state(self):
        msg = String()
        msg.data = self.state.value
        self.pub_state.publish(msg)

    def log_state_periodically(self):
        now = time.time()
        if now - self.last_state_log_time < 1.0:
            return
        self.get_logger().info(
            f"state={self.state.value} "
            f"estop={self.estop_active} "
            f"reset_armed={self.reset_armed}"
        )
        self.last_state_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
