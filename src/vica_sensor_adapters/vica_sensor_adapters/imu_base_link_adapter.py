#!/usr/bin/env python3

import math
import time

from geometry_msgs.msg import Vector3
import rclpy
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
    """Republish IMU data in the base_link frame with the gyro bias removed."""

    def __init__(self):
        super().__init__('imu_base_link_adapter')

        self.declare_parameter('input_topic', '/camera/camera/imu')
        self.declare_parameter('output_topic', '/imu/base_link')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('transform_timeout_sec', 0.05)

        self.declare_parameter('gyro_bias_sample_count', 1000)
        self.declare_parameter('gyro_bias_max_rate', 0.05)

        self.declare_parameter('gyro_bias_refresh_count', 120)
        self.declare_parameter('gyro_bias_refresh_alpha', 0.2)
        self.declare_parameter('gyro_bias_max_dev', 0.02)
        self.declare_parameter('gyro_bias_max_jump', 0.01)

        self.declare_parameter('publish_rate_hz', 40.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value

        self.bias = GyroBiasEstimator(
            sample_count=self.get_parameter('gyro_bias_sample_count').value,
            max_abs_rate=self.get_parameter('gyro_bias_max_rate').value,
            refresh_sample_count=self.get_parameter(
                'gyro_bias_refresh_count').value,
            refresh_alpha=self.get_parameter('gyro_bias_refresh_alpha').value,
            max_abs_dev=self.get_parameter('gyro_bias_max_dev').value,
            max_refresh_jump=self.get_parameter('gyro_bias_max_jump').value,
        )
        self._bias_reported = False
        self._bias_refresh_seen = 0

        rate = float(self.get_parameter('publish_rate_hz').value)
        self._min_period_sec = (1.0 / rate) if rate > 0.0 else 0.0
        self._last_publish_monotonic = 0.0

        self._acc_n = 0
        self._acc_gyro = [0.0, 0.0, 0.0]
        self._acc_lin = [0.0, 0.0, 0.0]

        self._tf_matrix = None
        self._tf_frame_id = None

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
            f'{input_topic} -> {output_topic} in {self.target_frame}'
        )

    def imu_callback(self, msg: Imu):
        self._acc_gyro[0] += msg.angular_velocity.x
        self._acc_gyro[1] += msg.angular_velocity.y
        self._acc_gyro[2] += msg.angular_velocity.z
        self._acc_lin[0] += msg.linear_acceleration.x
        self._acc_lin[1] += msg.linear_acceleration.y
        self._acc_lin[2] += msg.linear_acceleration.z
        self._acc_n += 1

        if self._min_period_sec > 0.0:
            now = time.monotonic()
            if now - self._last_publish_monotonic < self._min_period_sec:
                return
            self._last_publish_monotonic = now

        n = self._acc_n
        avg_gyro = [v / n for v in self._acc_gyro]
        avg_lin = [v / n for v in self._acc_lin]
        self._acc_n = 0
        self._acc_gyro = [0.0, 0.0, 0.0]
        self._acc_lin = [0.0, 0.0, 0.0]

        matrix = self._matrix_for(msg.header.frame_id, msg.header.stamp)
        if matrix is None:
            return

        out = Imu()
        out.header = msg.header
        out.header.frame_id = self.target_frame

        out.orientation = msg.orientation
        out.orientation_covariance = list(msg.orientation_covariance)

        out.angular_velocity = Vector3()
        out.angular_velocity.x = avg_gyro[0]
        out.angular_velocity.y = avg_gyro[1]
        out.angular_velocity.z = avg_gyro[2]
        _rotate_vector(matrix, out.angular_velocity)

        self._apply_gyro_bias(out.angular_velocity)

        out.angular_velocity_covariance = _rotate_covariance(
            matrix,
            msg.angular_velocity_covariance,
        )

        out.linear_acceleration = Vector3()
        out.linear_acceleration.x = avg_lin[0]
        out.linear_acceleration.y = avg_lin[1]
        out.linear_acceleration.z = avg_lin[2]
        _rotate_vector(matrix, out.linear_acceleration)
        out.linear_acceleration_covariance = _rotate_covariance(
            matrix,
            msg.linear_acceleration_covariance,
        )

        self.pub.publish(out)

    def _matrix_for(self, frame_id, stamp):
        """캐시된 회전 행렬을 준다. frame_id 가 바뀌면 다시 조회한다."""
        if self._tf_matrix is not None and frame_id == self._tf_frame_id:
            return self._tf_matrix

        timeout = Duration(
            seconds=float(self.get_parameter('transform_timeout_sec').value)
        )
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                frame_id,
                stamp,
                timeout,
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Waiting for TF {self.target_frame} <- {frame_id}: {exc}',
                throttle_duration_sec=2.0,
            )
            return None

        matrix = _quat_to_matrix(transform.transform.rotation)
        if matrix is None:
            self.get_logger().warn('Received invalid TF rotation quaternion')
            return None

        self._tf_matrix = matrix
        self._tf_frame_id = frame_id
        return matrix

    def _apply_gyro_bias(self, gyro):
        """정지 중 추정한 편향을 뺀다. 확정 전이면 원값이 그대로 나간다."""
        self.bias.add(gyro.x, gyro.y, gyro.z)
        gyro.x, gyro.y, gyro.z = self.bias.correct(gyro.x, gyro.y, gyro.z)

        if self.bias.refresh_count != self._bias_refresh_seen:
            self._bias_refresh_seen = self.bias.refresh_count
            bz = self.bias.bias[2]
            self.get_logger().info(
                f'Gyro bias refreshed (#{self.bias.refresh_count}) at stop: '
                f'yaw bias {bz:+.6f} rad/s '
                f'({math.degrees(bz) * 3600.0:+.1f} deg/hour)'
            )

        if self._bias_reported:
            return

        if self.bias.ready:
            bx, by, bz = self.bias.bias
            drift = math.degrees(bz) * 3600.0
            self.get_logger().info(
                f'Gyro bias calibrated over {self.bias.collected} samples: '
                f'({bx:+.6f}, {by:+.6f}, {bz:+.6f}) rad/s. '
                f'Removed yaw drift of {drift:+.1f} deg/hour.'
            )
            self._bias_reported = True
        elif self.bias.aborted:
            self.get_logger().warn(
                'Gyro bias calibration aborted: motion detected during startup. '
                'Publishing uncorrected rates - yaw will drift. '
                'Restart this node while the robot is stationary.'
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


if __name__ == '__main__':
    main()
