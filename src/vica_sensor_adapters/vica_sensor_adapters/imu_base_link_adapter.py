#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformListener

from vica_sensor_adapters.gyro_bias import GyroBiasEstimator


def _quat_to_matrix(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return None

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    return [[
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y - z * w),
        2.0 * (x * z + y * w),
    ], [
        2.0 * (x * y + z * w),
        1.0 - 2.0 * (x * x + z * z),
        2.0 * (y * z - x * w),
    ], [
        2.0 * (x * z - y * w),
        2.0 * (y * z + x * w),
        1.0 - 2.0 * (x * x + y * y),
    ]]


def _copy_vector(vector):
    out = Vector3()
    out.x = vector.x
    out.y = vector.y
    out.z = vector.z
    return out


def _rotate_vector(matrix, vector):
    values = [vector.x, vector.y, vector.z]
    rotated = [
        sum(matrix[row][col] * values[col] for col in range(3))
        for row in range(3)
    ]
    vector.x = rotated[0]
    vector.y = rotated[1]
    vector.z = rotated[2]


def _rotate_covariance(matrix, covariance):
    if covariance[0] < 0.0:
        return list(covariance)

    rotated = [0.0] * 9
    for row in range(3):
        for col in range(3):
            value = 0.0
            for i in range(3):
                for j in range(3):
                    value += (
                        matrix[row][i]
                        * covariance[i * 3 + j]
                        * matrix[col][j]
                    )
            rotated[row * 3 + col] = value
    return rotated


class ImuBaseLinkAdapter(Node):
    def __init__(self):
        super().__init__("imu_base_link_adapter")

        self.declare_parameter("input_topic", "/camera/camera/imu")
        self.declare_parameter("output_topic", "/imu/base_link")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("transform_timeout_sec", 0.05)

        # 정지 중 자이로 편향 보정. 0이면 끈다.
        #
        # 4000표본은 200 Hz에서 약 20초다. 표본 수는 잔차를 좌우한다 — 편향 추정의
        # 표준오차가 sigma/sqrt(N)이기 때문이다. 2026-08-01 Jetson 실측:
        #
        #   표본 없음   161 deg/hour   (수정 전)
        #   400  (2초)   38 deg/hour
        #   4000 (20초)   4.7 deg/hour
        #
        # 20초는 bringup 시간에 비하면 짧고, 그 사이에도 원값이 그대로 나가므로
        # 손해가 없다. 더 늘려도 편향 자체가 온도에 따라 변하므로 이득이 줄어든다.
        #
        # 임계 0.05 rad/s(약 2.9°/s)는 실측 잡음 표준편차 0.0079 rad/s의 6배 이상이라
        # 정지 중 오탐이 나지 않는다.
        self.declare_parameter("gyro_bias_sample_count", 4000)
        self.declare_parameter("gyro_bias_max_rate", 0.05)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.target_frame = self.get_parameter("target_frame").value

        self.bias = GyroBiasEstimator(
            sample_count=self.get_parameter("gyro_bias_sample_count").value,
            max_abs_rate=self.get_parameter("gyro_bias_max_rate").value,
        )
        self._bias_reported = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self.pub = self.create_publisher(Imu, output_topic, 10)
        self.sub = self.create_subscription(Imu, input_topic, self.imu_callback, qos)

        self.get_logger().info(
            f"{input_topic} -> {output_topic} in {self.target_frame}"
        )

    def imu_callback(self, msg: Imu):
        timeout = Duration(
            seconds=float(self.get_parameter("transform_timeout_sec").value)
        )

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                msg.header.stamp,
                timeout,
            )
        except TransformException as exc:
            message = (
                f"Waiting for TF {self.target_frame} "
                f"<- {msg.header.frame_id}: {exc}"
            )
            self.get_logger().warn(
                message,
                throttle_duration_sec=2.0,
            )
            return

        matrix = _quat_to_matrix(transform.transform.rotation)
        if matrix is None:
            self.get_logger().warn("Received invalid TF rotation quaternion")
            return

        out = Imu()
        out.header = msg.header
        out.header.frame_id = self.target_frame

        out.orientation = msg.orientation
        out.orientation_covariance = list(msg.orientation_covariance)

        out.angular_velocity = _copy_vector(msg.angular_velocity)
        _rotate_vector(matrix, out.angular_velocity)

        # 편향은 회전 뒤 base_link 기준으로 다룬다. EKF가 쓰는 축이 그것이다.
        # 선가속도에는 적용하지 않는다 — 중력은 실제 값이다.
        self._apply_gyro_bias(out.angular_velocity)

        out.angular_velocity_covariance = _rotate_covariance(
            matrix,
            msg.angular_velocity_covariance,
        )

        out.linear_acceleration = _copy_vector(msg.linear_acceleration)
        _rotate_vector(matrix, out.linear_acceleration)
        out.linear_acceleration_covariance = _rotate_covariance(
            matrix,
            msg.linear_acceleration_covariance,
        )

        self.pub.publish(out)

    def _apply_gyro_bias(self, gyro):
        """정지 중 추정한 편향을 뺀다. 확정 전이면 원값이 그대로 나간다."""
        self.bias.add(gyro.x, gyro.y, gyro.z)
        gyro.x, gyro.y, gyro.z = self.bias.correct(gyro.x, gyro.y, gyro.z)

        if self._bias_reported:
            return

        if self.bias.ready:
            bx, by, bz = self.bias.bias
            drift = math.degrees(bz) * 3600.0
            self.get_logger().info(
                f"Gyro bias calibrated over {self.bias.collected} samples: "
                f"({bx:+.6f}, {by:+.6f}, {bz:+.6f}) rad/s. "
                f"Removed yaw drift of {drift:+.1f} deg/hour."
            )
            self._bias_reported = True
        elif self.bias.aborted:
            self.get_logger().warn(
                "Gyro bias calibration aborted: motion detected during startup. "
                "Publishing uncorrected rates - yaw will drift. "
                "Restart this node while the robot is stationary."
            )
            self._bias_reported = True


def main():
    rclpy.init()
    node = ImuBaseLinkAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
