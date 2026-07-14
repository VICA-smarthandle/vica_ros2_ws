#!/usr/bin/env python3
import time

import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


PID_PNT_IO_MONITOR = 0xF1


class EmergencyStopNode(Node):
    """
    VICA E-stop 상태 전달 노드.

    이 노드는 하드웨어 E-stop을 대체하지 않는다. 실제 E-stop은 전원/토크 계통을
    물리적으로 차단해야 하며, 이 노드는 테스트 입력 또는 향후 GPIO/CAN 입력을
    ROS2 /emergency_stop 토픽으로 전달하는 보조 계층이다.
    """

    def __init__(self):
        super().__init__("emergency_stop_node")

        # publish_hz:
        #   safety_supervisor_node가 /emergency_stop 수신 끊김을 감지할 수 있도록
        #   E-stop 상태를 주기적으로 반복 발행한다.
        #
        # default_estop_active:
        #   노드 시작 직후 기본 E-stop 상태.
        #   실제 로봇에서는 입력선이 확인되기 전까지 true로 두는 구성이 더 보수적일 수 있다.
        #   현재 테스트 노드에서는 토픽 테스트 편의를 위해 기본 false를 사용한다.
        #
        # input_mode:
        #   test_topic = 앱 /app_emergency_stop 입력과 /emergency_stop_input만
        #                /emergency_stop으로 전달한다. 물리 버튼은 keyboard_knob가
        #                CAN F1을 직접 읽어 처리한다.
        #   can_f1     = MDROBOT F1 I/O monitor CAN 프레임도 함께 읽는다.
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("default_estop_active", False)
        self.declare_parameter("input_mode", "test_topic")
        self.declare_parameter("can_iface", "can1")
        self.declare_parameter("driver_response_id", 0x701)

        # can_f1 모드에서 사용할 F1 프레임 판정 파라미터.
        #
        # 지금까지 관찰한 VICA candump 패턴:
        #   버튼 해제 예: F1 00 58 58 ...
        #   버튼 누름 예: F1 00 48 48 ...
        #
        # 위 패턴에서는 data[2], data[3]의 0x10 비트가 버튼 눌림 때 0이 된다.
        # 장비 설정이나 배선이 달라지면 byte/mask/value를 launch 또는 CLI에서 조정한다.
        self.declare_parameter("f1_required_packet_index", 0)
        self.declare_parameter("f1_check_byte_1", 2)
        self.declare_parameter("f1_check_byte_2", 3)
        self.declare_parameter("f1_check_mask", 0x10)
        self.declare_parameter("f1_active_value", 0x00)
        self.declare_parameter("f1_timeout_sec", 0.5)
        self.declare_parameter("log_f1_frames", True)

        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.estop_active = bool(self.get_parameter("default_estop_active").value)
        self.app_estop_active = False

        self.input_mode = str(self.get_parameter("input_mode").value)
        self.can_iface = str(self.get_parameter("can_iface").value)
        self.driver_response_id = int(self.get_parameter("driver_response_id").value)
        self.f1_required_packet_index = int(self.get_parameter("f1_required_packet_index").value)
        self.f1_check_byte_1 = int(self.get_parameter("f1_check_byte_1").value)
        self.f1_check_byte_2 = int(self.get_parameter("f1_check_byte_2").value)
        self.f1_check_mask = int(self.get_parameter("f1_check_mask").value)
        self.f1_active_value = int(self.get_parameter("f1_active_value").value)
        self.f1_timeout_sec = float(self.get_parameter("f1_timeout_sec").value)
        self.log_f1_frames = bool(self.get_parameter("log_f1_frames").value)

        self.last_input_time = 0.0
        self.bus = None
        self.last_f1_time = 0.0
        self.last_f1_log_time = 0.0

        # /emergency_stop:
        #   safety_supervisor_node가 보는 표준 E-stop 상태 토픽.
        #   true  = E-stop active, 주행 명령 차단 필요
        #   false = E-stop released, 단 reset 전 자동 재출발 금지
        self.pub_estop = self.create_publisher(Bool, "/emergency_stop", 10)

        # /app_emergency_stop:
        #   VICA Supervisor 앱에서 들어오는 소프트웨어 E-stop 입력.
        #   test_topic 기본 모드에서는 이 입력만 /emergency_stop으로 넘겨
        #   keyboard_knob의 래치를 작동시킨다.
        self.sub_app_estop = self.create_subscription(
            Bool,
            "/app_emergency_stop",
            self.app_estop_callback,
            10,
        )

        # /voice_emergency_stop:
        #   음성 긴급어 경로 입력 (통합 진행순서 ③).
        #   vica_mission_manager 의 emergency_estop_bridge 가 하드 긴급어
        #   ("멈춰" 등) 감지 시 펄스(수 초 true 후 false)로 발행한다.
        #   펄스인 이유: keyboard_knob 의 /estop_reset 은 /emergency_stop 이
        #   아직 true 면 해제를 거부하므로, 입력이 눌러붙으면 해제가 막힌다.
        #   래치 유지는 keyboard_knob 몫이고 이 입력은 방아쇠다.
        self.voice_estop_active = False
        self.sub_voice_estop = self.create_subscription(
            Bool,
            "/voice_emergency_stop",
            self.voice_estop_callback,
            10,
        )

        # /emergency_stop_input:
        #   현재 단계의 테스트용 입력 토픽.
        #   나중에 Jetson GPIO, Arduino serial, 또는 MDROBOT F1 I/O monitor 입력을
        #   읽는 코드로 교체해도 /emergency_stop 출력 인터페이스는 유지한다.
        self.sub_test_input = self.create_subscription(
            Bool,
            "/emergency_stop_input",
            self.estop_input_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / self.publish_hz, self.publish_loop)

        if self.input_mode == "can_f1":
            self.open_can_bus()
        elif self.input_mode != "test_topic":
            self.get_logger().warn(
                f"Unknown input_mode '{self.input_mode}', falling back to test_topic"
            )
            self.input_mode = "test_topic"

        self.get_logger().warn(
            "This node is a ROS2 E-stop state bridge only. "
            "It does not replace a hardware E-stop."
        )
        self.get_logger().info(f"input_mode={self.input_mode}")
        self.get_logger().info("Subscribed: /app_emergency_stop")
        self.get_logger().info("Subscribed: /voice_emergency_stop")
        self.get_logger().info("Subscribed: /emergency_stop_input for test mode")
        self.get_logger().info("Publishing: /emergency_stop")

    def open_can_bus(self):
        # python-can SocketCAN raw socket을 연다.
        # 이 노드는 CAN 송신을 하지 않고 F1 상태 프레임만 수신한다.
        self.bus = can.interface.Bus(
            channel=self.can_iface,
            interface="socketcan",
        )
        self.get_logger().info(
            "CAN F1 input enabled: "
            f"iface={self.can_iface}, response_id=0x{self.driver_response_id:03X}, "
            f"packet_index={self.f1_required_packet_index}, "
            f"bytes=({self.f1_check_byte_1},{self.f1_check_byte_2}), "
            f"mask=0x{self.f1_check_mask:02X}, active_value=0x{self.f1_active_value:02X}"
        )

    def estop_input_callback(self, msg: Bool):
        # test_topic 모드에서만 수동 입력을 반영한다.
        # can_f1 모드에서는 실제 MDROBOT F1 프레임만 E-stop 상태로 사용한다.
        if self.input_mode != "test_topic":
            return

        self.estop_active = bool(msg.data)
        self.last_input_time = time.time()

    def app_estop_callback(self, msg: Bool):
        self.app_estop_active = bool(msg.data)

    def voice_estop_callback(self, msg: Bool):
        if bool(msg.data) and not self.voice_estop_active:
            self.get_logger().warn("음성 긴급어 E-stop 입력 수신")
        self.voice_estop_active = bool(msg.data)

    def publish_loop(self):
        if self.input_mode == "can_f1":
            self.drain_can_f1_frames()
            self.apply_f1_timeout()

        # 상태가 바뀔 때만 발행하지 않고 주기 발행한다.
        # safety_supervisor_node는 이 주기 발행이 끊기면 안전하게 fault/stop으로 전환한다.
        msg = Bool()
        msg.data = self.estop_active or self.app_estop_active or self.voice_estop_active
        self.pub_estop.publish(msg)

    def drain_can_f1_frames(self):
        if self.bus is None:
            return

        # 한 timer 주기에서 CAN RX 큐를 적당히 비운다.
        # motor node도 can1 raw socket으로 F1을 읽을 수 있으므로, 여기서는 송신 없이 상태만 본다.
        for _ in range(50):
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                break

            if msg.arbitration_id != self.driver_response_id:
                continue

            data = bytes(msg.data)
            if len(data) != 8 or data[0] != PID_PNT_IO_MONITOR:
                continue

            # F1은 여러 packet index로 나뉘어 들어온다.
            # 현재 START/STOP 상태는 관찰 로그 기준 d[1] == 0 프레임에서 확인한다.
            if data[1] != self.f1_required_packet_index:
                continue

            self.last_f1_time = time.time()
            self.estop_active = self.f1_frame_means_estop_active(data)
            self.log_f1_frame_if_needed(data)

    def f1_frame_means_estop_active(self, data: bytes) -> bool:
        # 기본 판정:
        #   keyboard_knob의 F1 버튼 판정과 같은 기준을 사용한다.
        #   data[f1_check_byte_1] 또는 data[f1_check_byte_2]의 f1_check_mask 비트 중
        #   하나라도 f1_active_value와 같으면 E-stop active로 판단한다.
        #
        # data[2] & 0x10, data[3] & 0x10이 모두 active_value와 같을 때
        # E-stop active로 판단한다.
        byte_1_active = (
            data[self.f1_check_byte_1] & self.f1_check_mask
        ) == self.f1_active_value
        byte_2_active = (
            data[self.f1_check_byte_2] & self.f1_check_mask
        ) == self.f1_active_value
        return byte_1_active and byte_2_active

    def apply_f1_timeout(self):
        # F1 프레임이 아직 안 들어왔거나 일정 시간 이상 끊기면 버튼 상태를 신뢰할 수 없다.
        # 이 경우 safety_supervisor_node가 정지하도록 /emergency_stop=true를 발행한다.
        if self.last_f1_time <= 0.0:
            self.estop_active = True
            return

        if (time.time() - self.last_f1_time) > self.f1_timeout_sec:
            self.estop_active = True

    def log_f1_frame_if_needed(self, data: bytes):
        if not self.log_f1_frames:
            return

        now = time.time()
        if now - self.last_f1_log_time < 0.2:
            return

        hex_data = " ".join(f"{value:02X}" for value in data)
        self.get_logger().info(
            f"F1 data={hex_data} -> emergency_stop={self.estop_active}"
        )
        self.last_f1_log_time = now

    def destroy_node(self):
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
