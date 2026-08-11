#!/usr/bin/env python3

import math
import struct
import time

import can
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


PID_PNT_VEL_CMD = 0xCF
PID_POSI_DATA = 0xC5


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return 0.0, 0.0, qz, qw


def int32_le(byte_list):
    """Decode little-endian signed int32.

    Examples
    --------
      05 00 00 00 -> 5
      F0 FF FF FF -> -16

    """
    return struct.unpack('<i', bytes(byte_list))[0]


def int32_delta(current: int, previous: int) -> int:
    """Return a signed delta across an int32 counter rollover."""
    delta = int(current) - int(previous)
    if delta > 0x7FFFFFFF:
        delta -= 0x100000000
    elif delta < -0x80000000:
        delta += 0x100000000
    return delta


class EncoderFeedbackNode(Node):
    """Publish wheel odometry decoded from MDROBOT CAN position frames."""

    def __init__(self):
        super().__init__('encoder_feedback')

        # =========================
        # 파라미터
        # =========================
        self.declare_parameter('can_iface', 'can1')
        self.declare_parameter('driver_id', 0x001)
        self.declare_parameter('driver_response_id', 0x701)

        # CAN C5 위치값 기준 바퀴 1회전당 tick 수.
        self.declare_parameter('ticks_per_rev', 61.2)  # 기본 55에서 실측으로 올린 값
        self.declare_parameter('wheel_radius_m', 0.065)
        self.declare_parameter('wheel_base_m', 0.37)

        self.declare_parameter('right_sign', 1.0)
        self.declare_parameter('left_sign', 1.0)

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_topic', '/wheel/odom')

        self.declare_parameter('publish_tf', False)
        # 주행 통합 중에는 모터 노드가 CAN 명령을 담당하므로 encoder_feedback은
        # 기본적으로 수신만 수행하고, 단독 테스트 때만 요청 프레임을 켭니다.
        self.declare_parameter('request_position_feedback', False)  # 기본 False
        # 인코더 CAN 요청 부하를 줄이기 위해 피드백 요청 빈도를 낮춥니다.
        # 30Hz는 이 시스템에 과도했습니다. odom에는 10Hz 정도면 충분합니다.
        self.declare_parameter('request_hz', 20.0)

        self.can_iface = self.get_parameter('can_iface').value
        self.driver_id = int(self.get_parameter('driver_id').value)
        self.driver_response_id = int(
            self.get_parameter('driver_response_id').value
        )

        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.wheel_radius_m = float(self.get_parameter('wheel_radius_m').value)
        self.wheel_base_m = float(self.get_parameter('wheel_base_m').value)

        self.right_sign = float(self.get_parameter('right_sign').value)
        self.left_sign = float(self.get_parameter('left_sign').value)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.request_position_feedback = bool(
            self.get_parameter('request_position_feedback').value
        )
        self.request_hz = float(self.get_parameter('request_hz').value)

        if self.ticks_per_rev <= 0.0:
            raise ValueError('ticks_per_rev must be greater than zero')
        if self.wheel_radius_m <= 0.0:
            raise ValueError('wheel_radius_m must be greater than zero')
        if self.wheel_base_m <= 0.0:
            raise ValueError('wheel_base_m must be greater than zero')
        if self.request_hz <= 0.0:
            raise ValueError('request_hz must be greater than zero')

        # =========================
        # 런타임 상태
        # =========================
        self.right_pos = None
        self.left_pos = None
        self.right_updated = False
        self.left_updated = False

        self.prev_right_pos = None
        self.prev_left_pos = None

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_odom_time = None

        self.wheel_circumference = 2.0 * math.pi * self.wheel_radius_m

        # =========================
        # CAN 설정
        # =========================
        self.bus = can.interface.Bus(
            channel=self.can_iface,
            interface='socketcan'
        )

        self.get_logger().info(f'CAN opened: {self.can_iface}')
        self.get_logger().info(f'driver_id: 0x{self.driver_id:03X}')
        self.get_logger().info(
            f'driver_response_id: 0x{self.driver_response_id:03X}'
        )
        self.get_logger().info(
            f'ticks_per_rev={self.ticks_per_rev}, '
            f'wheel_radius={self.wheel_radius_m}, '
            f'wheel_base={self.wheel_base_m}'
        )
        mode = 'request+read' if self.request_position_feedback else 'read-only'
        self.get_logger().info(f'position feedback mode: {mode}')

        # =========================
        # ROS 퍼블리셔
        # =========================
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_tf else None
        )
        self.get_logger().info(
            f'Publishing raw wheel odometry: {self.odom_topic} '
            f'(publish_tf={self.publish_tf})'
        )

        # 타이머 주기가 오도메트리 갱신 루프 빈도를 결정합니다.
        self.timer = self.create_timer(
            1.0 / self.request_hz,
            self.loop
        )

    def send_position_request(self):
        """
        CAN 위치 피드백 요청 메시지를 생성합니다.

        동등한 CAN 명령:
          cansend can1 001#CF01000001000005

        의미:
          CF = PID_PNT_VEL_CMD
          MOT1 목표 rpm = 0
          MOT2 목표 rpm = 0
          ret_type = 5, 위치 피드백 요청
        """
        msg = can.Message(
            arbitration_id=self.driver_id,
            is_extended_id=False,
            data=bytes([
                PID_PNT_VEL_CMD,
                1, 0, 0,
                1, 0, 0,
                5
            ])
        )
        self.bus.send(msg)

    def read_can_frames(self):
        """
        C5 위치 프레임을 읽어 옵니다.

        예상 프레임:
          701#C500........ -> MOT1 / 오른쪽 바퀴
          701#C501........ -> MOT2 / 왼쪽 바퀴
        """
        for _ in range(100):
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                break

            if msg.arbitration_id != self.driver_response_id:
                continue

            data = list(msg.data)

            if len(data) < 6:
                continue

            pid = data[0]

            if pid != PID_POSI_DATA:
                continue

            motor_index = data[1]
            pos = int32_le(data[2:6])

            if motor_index == 0:
                self.right_pos = pos
                self.right_updated = True
            elif motor_index == 1:
                self.left_pos = pos
                self.left_updated = True

        return self.right_updated and self.left_updated

    def loop(self):
        now = self.get_clock().now()

        if self.request_position_feedback:
            self.send_position_request()
            time.sleep(0.001)

        if not self.read_can_frames():
            return

        # 새 좌우 위치값 한 쌍을 한 번만 소비합니다. 다음 C5 프레임이
        # 들어오기 전에는 이전 값을 재사용해 odometry를 발행하지 않습니다.
        self.right_updated = False
        self.left_updated = False

        if self.prev_right_pos is None or self.prev_left_pos is None:
            self.prev_right_pos = self.right_pos
            self.prev_left_pos = self.left_pos
            self.last_odom_time = now
            self.get_logger().info(
                f'Initial position set: right={self.right_pos}, left={self.left_pos}'
            )
            return

        dt = 0.0
        if self.last_odom_time is not None:
            dt = (now - self.last_odom_time).nanoseconds / 1e9

        if dt <= 0.0:
            dt = 1.0 / self.request_hz

        delta_right_count = int32_delta(
            self.right_pos,
            self.prev_right_pos,
        ) * self.right_sign
        delta_left_count = int32_delta(
            self.left_pos,
            self.prev_left_pos,
        ) * self.left_sign

        self.prev_right_pos = self.right_pos
        self.prev_left_pos = self.left_pos
        self.last_odom_time = now

        right_dist = (
            delta_right_count / self.ticks_per_rev
        ) * self.wheel_circumference

        left_dist = (
            delta_left_count / self.ticks_per_rev
        ) * self.wheel_circumference

        d_center = (right_dist + left_dist) / 2.0
        d_theta = (right_dist - left_dist) / self.wheel_base_m

        theta_mid = self.theta + d_theta / 2.0

        self.x += d_center * math.cos(theta_mid)
        self.y += d_center * math.sin(theta_mid)
        self.theta = normalize_angle(self.theta + d_theta)

        vx = d_center / dt
        vth = d_theta / dt

        self.publish_odom(now, vx, vth)

    def publish_odom(self, stamp, vx, vth):
        qx, qy, qz, qw = yaw_to_quaternion(self.theta)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = vth

        odom_msg.pose.covariance = [
            0.04, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.04, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1000.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1000.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1000.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.03,
        ]
        odom_msg.twist.covariance = [
            0.0025, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0001, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1000.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1000.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1000.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0025,
        ]

        self.odom_pub.publish(odom_msg)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame

            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0

            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderFeedbackNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.bus.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
