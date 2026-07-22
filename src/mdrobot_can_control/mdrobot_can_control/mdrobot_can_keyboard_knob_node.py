#!/usr/bin/env python3
import math
import time
import can

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ============================================================
# ROS2 Humble + teleop_twist_keyboard + MDROBOT CAN 통합
#
# 목적:
#   safety_supervisor_node가 승인한 /cmd_vel_safe를 받아서 CAN으로 모터드라이버 구동
#   최고속도는 드라이버에서 올라오는 knob1 값으로 제한
#
# 모터 매핑:
#   MOT1 = 오른쪽 바퀴
#   MOT2 = 왼쪽 바퀴
#
# 주의:
#   왼쪽 바퀴가 MOT2라는 기존 조건 반영
# ============================================================


# ====== MDROBOT CAN 프로토콜 ======
PID_PNT_IO_MONITOR = 0xF1  # 241
PID_PNT_VEL_CMD    = 0xCF  # 207
PID_COMMAND        = 0x0A  # 10
CMD_PNT_IO_MON_ON  = 0x55  # 85

RET_TYPE_NONE = 0
RET_TYPE_ODOM = 5


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def le_i16_signed(value: int):
    value = int(value)
    value = clamp(value, -32768, 32767)
    u = value & 0xFFFF
    return u & 0xFF, (u >> 8) & 0xFF


class MdrobotCanKeyboardKnobNode(Node):
    def __init__(self):
        super().__init__("mdrobot_can_keyboard_knob_node")

        # =========================
        # 사용자 파라미터
        # =========================
        self.declare_parameter("can_iface", "can1")
        self.declare_parameter("driver_id", 0x001)

        # MDH100 Ø130mm → radius 0.065m
        self.declare_parameter("wheel_radius_m", 0.065)

        # 좌우 바퀴 중심거리. 실제 로봇에서 반드시 줄자로 재서 수정.
        self.declare_parameter("wheel_base_m", 0.37)

        # knob 100%일 때 허용할 최고 속도
        self.declare_parameter("max_linear_mps", 1.0)
        self.declare_parameter("max_angular_radps", 2.0)

        # 모터 자체 rpm 제한
        self.declare_parameter("max_rpm", 400)

        # CAN 송신 주기
        # 30Hz는 모터 버스에 너무 잦을 수 있습니다. 낮은 주기로 CAN 부하를 줄입니다.
        self.declare_parameter("send_hz", 30.0)

        # 동일한 명령을 반복 전송하지 않도록 하는 최소 재전송 간격(초)
        # 너무 길게 설정되어 있으면 제어가 뚝뚝 끊깁니다. 기본은 50ms.
        self.declare_parameter("resend_interval_sec", 0.05)

        # knob 값 0~100 중 이 값 이하는 정지로 처리
        self.declare_parameter("deadzone_pct", 5)

        # knob 패킷이 이 시간 이상 안 들어오면 안전 정지
        self.declare_parameter("knob_timeout_sec", 0.8)

        # /cmd_vel_safe가 이 시간 이상 안 들어오면 안전 정지
        self.declare_parameter("cmd_timeout_sec", 0.5)

        # 방향 보정. 전진 명령에서 바퀴가 반대로 돌면 True로 바꾸세요.
        self.declare_parameter("invert_mot1", False)  # 오른쪽 바퀴
        self.declare_parameter("invert_mot2", False)  # 왼쪽 바퀴

        # 낮은 rpm에서 모터가 꿈틀거리기만 하면 30~50 정도로 사용
        # 처음에는 0 추천
        self.declare_parameter("min_rpm_when_moving", 0)

        # =========================
        # 파라미터 불러오기
        # =========================
        self.can_iface = self.get_parameter("can_iface").value
        self.driver_id = int(self.get_parameter("driver_id").value)

        self.wheel_radius_m = float(self.get_parameter("wheel_radius_m").value)
        self.wheel_base_m = float(self.get_parameter("wheel_base_m").value)

        self.max_linear_mps = float(self.get_parameter("max_linear_mps").value)
        self.max_angular_radps = float(self.get_parameter("max_angular_radps").value)
        self.max_rpm = int(self.get_parameter("max_rpm").value)

        self.send_hz = float(self.get_parameter("send_hz").value)
        self.resend_interval_sec = float(self.get_parameter("resend_interval_sec").value)
        self.deadzone_pct = int(self.get_parameter("deadzone_pct").value)
        self.knob_timeout_sec = float(self.get_parameter("knob_timeout_sec").value)
        self.cmd_timeout_sec = float(self.get_parameter("cmd_timeout_sec").value)

        self.invert_mot1 = bool(self.get_parameter("invert_mot1").value)
        self.invert_mot2 = bool(self.get_parameter("invert_mot2").value)

        self.min_rpm_when_moving = int(
            self.get_parameter("min_rpm_when_moving").value
        )

        # =========================
        # 실행 상태
        # =========================
        self.knob1 = 0
        self.knob2 = 0
        self.last_knob_time = 0.0

        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        self.last_cmd_time = 0.0

        self.last_print_time = 0.0
        self.prev_rpm_mot1 = None
        self.prev_rpm_mot2 = None
        self.last_send_time = 0.0

        # =========================
        # CAN 초기화
        # =========================
        self.bus = can.interface.Bus(
            channel=self.can_iface,
            interface="socketcan"
        )

        self.get_logger().info(f"CAN opened: {self.can_iface}")
        self.get_logger().info(f"Driver ID: 0x{self.driver_id:03X}")

        # 드라이버 PNT I/O monitor broadcasting ON
        self.send_pnt_io_monitor_on()
        time.sleep(0.05)
        self.send_pnt_io_monitor_on()

        # =========================
        # ROS 인터페이스
        # =========================
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            "/cmd_vel_safe",
            self.cmd_vel_callback,
            10
        )

        self.timer = self.create_timer(
            1.0 / self.send_hz,
            self.control_loop
        )

        self.get_logger().info("Subscribed: /cmd_vel_safe")
        self.get_logger().info("knob1 = 최고속도 제한기")
        self.get_logger().info("Ready.")

    # ============================================================
    # CAN 함수
    # ============================================================
    def send_pnt_io_monitor_on(self):
        msg = can.Message(
            arbitration_id=self.driver_id,
            is_extended_id=False,
            data=bytes([
                PID_COMMAND,
                CMD_PNT_IO_MON_ON,
                0, 0, 0, 0, 0, 0
            ])
        )
        self.bus.send(msg)

    def send_vel_cmd(self, rpm1: int, rpm2: int, ret_type: int = RET_TYPE_NONE):
        """
        rpm1 = MOT1 = 오른쪽 바퀴
        rpm2 = MOT2 = 왼쪽 바퀴
        """
        rpm1 = int(clamp(rpm1, -self.max_rpm, self.max_rpm))
        rpm2 = int(clamp(rpm2, -self.max_rpm, self.max_rpm))

        lo1, hi1 = le_i16_signed(rpm1)
        lo2, hi2 = le_i16_signed(rpm2)

        data = bytes([
            PID_PNT_VEL_CMD,
            1,
            lo1,
            hi1,
            1,
            lo2,
            hi2,
            ret_type & 0xFF
        ])

        msg = can.Message(
            arbitration_id=self.driver_id,
            is_extended_id=False,
            data=data
        )

        self.bus.send(msg)

    def drain_can_rx(self):
        """
        F1 monitor packet에서 knob1, knob2 값 읽기.

        기존 사용 코드 기준:
          d[0] == 0xF1
          d[1] == 0
          d[6] = knob1, 0~100
          d[7] = knob2, 0~100
        """
        # 버스 혼잡을 줄이기 위해 사이클당 읽는 CAN 프레임 수를 제한합니다.
        for _ in range(50):
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                break

            d = msg.data
            if len(d) == 8 and d[0] == PID_PNT_IO_MONITOR:
                if d[1] == 0:
                    self.knob1 = clamp(int(d[6]), 0, 100)
                    self.knob2 = clamp(int(d[7]), 0, 100)
                    self.last_knob_time = time.time()

    # ============================================================
    # ROS 콜백
    # ============================================================
    def cmd_vel_callback(self, msg: Twist):
        self.cmd_linear_x = float(msg.linear.x)
        self.cmd_angular_z = float(msg.angular.z)
        self.last_cmd_time = time.time()

    # ============================================================
    # 메인 루프
    # ============================================================
    def control_loop(self):
        now = time.time()

        self.drain_can_rx()

        # -------------------------
        # knob 상태 확인
        # -------------------------
        knob_alive = (
            self.last_knob_time > 0.0 and
            (now - self.last_knob_time) <= self.knob_timeout_sec
        )

        if not knob_alive:
            knob_pct = 0
        else:
            knob_pct = int(self.knob1)

        if knob_pct <= self.deadzone_pct:
            knob_pct = 0

        speed_ratio = knob_pct / 100.0

        # -------------------------
        # /cmd_vel_safe 상태 확인
        # -------------------------
        cmd_alive = (
            self.last_cmd_time > 0.0 and
            (now - self.last_cmd_time) <= self.cmd_timeout_sec
        )

        if not cmd_alive:
            raw_linear_x = 0.0
            raw_angular_z = 0.0
        else:
            raw_linear_x = self.cmd_linear_x
            raw_angular_z = self.cmd_angular_z

        # -------------------------
        # knob 기준 최고속도 계산
        # -------------------------
        allowed_linear = self.max_linear_mps * speed_ratio
        allowed_angular = self.max_angular_radps * speed_ratio

        # 키보드 입력값을 knob가 허용한 최고속도 안으로 제한
        limited_linear_x = clamp(
            raw_linear_x,
            -allowed_linear,
            allowed_linear
        )

        limited_angular_z = clamp(
            raw_angular_z,
            -allowed_angular,
            allowed_angular
        )

        # -------------------------
        # differential drive 계산
        #
        # ROS 기준:
        #   linear.x  + : 전진
        #   angular.z + : 좌회전
        #
        # v_left  = v - w * L/2
        # v_right = v + w * L/2
        # -------------------------
        v_left = limited_linear_x - limited_angular_z * self.wheel_base_m / 2.0
        v_right = limited_linear_x + limited_angular_z * self.wheel_base_m / 2.0

        rpm_left = self.mps_to_rpm(v_left)
        rpm_right = self.mps_to_rpm(v_right)

        # MOT1 = 오른쪽, MOT2 = 왼쪽
        rpm_mot1 = rpm_right
        rpm_mot2 = rpm_left

        # 방향 보정
        if self.invert_mot1:
            rpm_mot1 *= -1
        if self.invert_mot2:
            rpm_mot2 *= -1

        rpm_mot1 = int(round(clamp(rpm_mot1, -self.max_rpm, self.max_rpm)))
        rpm_mot2 = int(round(clamp(rpm_mot2, -self.max_rpm, self.max_rpm)))

        rpm_mot1 = self.apply_min_rpm(rpm_mot1)
        rpm_mot2 = self.apply_min_rpm(rpm_mot2)

        # 타이머마다 동일한 CAN 속도 명령을 반복 전송하지 않습니다.
        # 모터 명령이 바뀌었을 때만 전송하고, 최소 `resend_interval_sec`만큼 간격을 둡니다.
        if (
            self.prev_rpm_mot1 != rpm_mot1 or
            self.prev_rpm_mot2 != rpm_mot2 or
            (now - self.last_send_time) >= self.resend_interval_sec
        ):
            self.send_vel_cmd(rpm_mot1, rpm_mot2, ret_type=RET_TYPE_ODOM)
            self.prev_rpm_mot1 = rpm_mot1
            self.prev_rpm_mot2 = rpm_mot2
            self.last_send_time = now

        # -------------------------
        # 디버그 출력
        # -------------------------
        if now - self.last_print_time > 0.2:
            self.get_logger().info(
                f"knob1={self.knob1:3d}% "
                f"limit=({allowed_linear:.2f}m/s,{allowed_angular:.2f}rad/s) "
                f"cmd=({raw_linear_x:+.2f},{raw_angular_z:+.2f}) "
                f"out=({limited_linear_x:+.2f},{limited_angular_z:+.2f}) "
                f"rpm MOT1/R={rpm_mot1:+4d}, MOT2/L={rpm_mot2:+4d}"
            )
            self.last_print_time = now

    def mps_to_rpm(self, v_mps: float) -> float:
        circumference = 2.0 * math.pi * self.wheel_radius_m
        return (v_mps / circumference) * 60.0

    def apply_min_rpm(self, rpm: int) -> int:
        if self.min_rpm_when_moving <= 0:
            return rpm

        if rpm == 0:
            return 0

        if abs(rpm) < self.min_rpm_when_moving:
            return self.min_rpm_when_moving if rpm > 0 else -self.min_rpm_when_moving

        return rpm

    def stop_motors(self):
        try:
            self.send_vel_cmd(0, 0, ret_type=RET_TYPE_NONE)
            time.sleep(0.02)
            self.send_vel_cmd(0, 0, ret_type=RET_TYPE_NONE)
        except Exception as e:
            self.get_logger().warn(f"stop_motors failed: {e}")

    def destroy_node(self):
        self.get_logger().info("Stopping motors...")
        self.stop_motors()

        try:
            self.bus.shutdown()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = MdrobotCanKeyboardKnobNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
