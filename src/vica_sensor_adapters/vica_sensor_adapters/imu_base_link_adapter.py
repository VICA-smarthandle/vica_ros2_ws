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
        # 편향 표본 수. publish_rate_hz 로 스로틀하면 그만큼 표본이 느리게
        # 쌓이므로 함께 줄인다. 50 Hz x 1000개 = 20초로, 200 Hz x 4000개와 같다.
        self.declare_parameter('gyro_bias_sample_count', 1000)
        self.declare_parameter('gyro_bias_max_rate', 0.05)

        # 출력 주파수 상한. 0 이하면 제한하지 않는다(입력 그대로).
        #
        # 2026-08-02 주행 중 CPU 실측에서 이 노드가 **1위(34.7 %)** 였다. 하는 일은
        # IMU 메시지의 좌표계를 바꿔 다시 내보내는 것뿐인데, D455 가 200 Hz 로
        # 보내고 EKF 는 30 Hz 만 쓰므로 6배를 헛돌고 있었다.
        #
        # 비용의 대부분은 메시지당 tf_buffer.lookup_transform 호출이다. 스로틀을
        # 콜백 맨 앞에 두어 TF 조회 자체를 건너뛴다.
        #
        # 같은 주행에서 load average 가 17.1(코어 8개)인데 CPU 는 461 %(4.6 코어)
        # 였다. 계산이 많아서가 아니라 **실행 대기 줄이 길어서** controller 가
        # 20 Hz 제어 주기를 382회 놓쳤고, 그것이 "주행이 매끄럽지 못하다"로
        # 나타났다. 이 노드를 줄이면 그 줄이 짧아진다.
        #
        # 50 Hz 는 EKF 사용 주기(30 Hz)의 1.7배다. 더 낮추면 EKF 입력이 부족해진다.
        #
        # 2026-08-15: 50 -> 40. 여전히 EKF(30 Hz)의 1.33배라 입력이 모자라지
        # 않는다. 그날 주행 실측에서 이 노드가 32.6 %로 3위였고, 실제 출력은
        # 41.3 Hz 여서 상한 50 은 사실상 걸리지도 않고 있었다. 40 으로 내리면
        # 상한이 실제로 작동한다.
        #
        # 30 으로 더 내리지 않는 이유: EKF 주기와 같아져 여유가 사라진다.
        # 지터 때문에 어떤 주기는 표본 0개, 어떤 주기는 2개를 받게 된다.
        #
        # 2026-08-18: 스로틀을 '버리기'에서 '평균 내기'로 바꿨다. 카메라(와 그
        # 안의 IMU)가 핸들 기둥 마스트(지면 1.075 m)로 올라가면서 진동 잡음이
        # 늘 것이기 때문이다. 200 Hz 표본을 버리지 않고 발행 주기(40 Hz)마다
        # 평균 내면 5표본 평균이 되어 40 Hz 성분은 완전 상쇄, 백색잡음은
        # sqrt(5)=2.2배 감쇠한다. **수 Hz 의 느린 흔들림은 평균으로 못 지운다**
        # - 자이로에게 그건 진짜 회전이다. 그쪽은 기구 강성이 담당한다.
        # 비용은 콜백당 덧셈 6개다. 콜백 깨움·역직렬화는 구독하는 이상 어차피
        # 200 Hz 로 일어나므로 버리기 대비 추가 비용이 사실상 없다.
        self.declare_parameter('publish_rate_hz', 40.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value

        self.bias = GyroBiasEstimator(
            sample_count=self.get_parameter('gyro_bias_sample_count').value,
            max_abs_rate=self.get_parameter('gyro_bias_max_rate').value,
        )
        self._bias_reported = False

        rate = float(self.get_parameter('publish_rate_hz').value)
        # monotonic 기준이다. 시스템 시계가 바뀌어도 간격 판정이 흔들리지 않는다.
        self._min_period_sec = (1.0 / rate) if rate > 0.0 else 0.0
        self._last_publish_monotonic = 0.0

        # 발행 사이에 도착한 원본 표본의 합. 발행 시점에 평균 내고 비운다.
        self._acc_n = 0
        self._acc_gyro = [0.0, 0.0, 0.0]
        self._acc_lin = [0.0, 0.0, 0.0]

        # TF 캐시. base_link <- IMU frame 은 URDF 고정 변환이라 한 번만
        # 조회하면 된다. lookup_transform 이 이 노드 비용의 대부분이었다
        # (주행 실측 25~33 %). frame_id 가 바뀌면 다시 조회한다 - 변환이
        # 동적으로 바뀌는 구성으로 옮겼을 때 조용히 틀리는 것을 막는
        # 안전장치다.
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
        # 모든 표본을 합산에 넣는다(평균 내기). 스로틀에 걸려도 버리지 않는다.
        self._acc_gyro[0] += msg.angular_velocity.x
        self._acc_gyro[1] += msg.angular_velocity.y
        self._acc_gyro[2] += msg.angular_velocity.z
        self._acc_lin[0] += msg.linear_acceleration.x
        self._acc_lin[1] += msg.linear_acceleration.y
        self._acc_lin[2] += msg.linear_acceleration.z
        self._acc_n += 1

        # 스로틀은 TF·회전보다 **먼저** 한다. 뒤에 두면 CPU 를 아끼지 못한다.
        if self._min_period_sec > 0.0:
            now = time.monotonic()
            if now - self._last_publish_monotonic < self._min_period_sec:
                return
            self._last_publish_monotonic = now

        # 평균을 꺼내고 합산을 비운다. TF 실패로 이번 발행을 건너뛰더라도
        # 비운다 - 기동 초기 TF 대기 동안 쌓인 낡은 표본이 첫 발행에 한꺼번에
        # 평균되는 것을 막는다.
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

        # 편향은 회전 뒤 base_link 기준으로 다룬다. EKF가 쓰는 축이 그것이다.
        # 선가속도에는 적용하지 않는다 — 중력은 실제 값이다.
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
