#!/usr/bin/env python3

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class VslamCovarianceAdapter(Node):
    """Fill in the covariance VSLAM odometry leaves empty."""

    def __init__(self):
        super().__init__('vslam_covariance_adapter')

        self.declare_parameter('input_topic', '/visual_slam/tracking/odometry')
        self.declare_parameter('output_topic', '/visual_slam/tracking/odometry_cov')

        self.declare_parameter('pose_xy_cov', 0.20)
        self.declare_parameter('pose_z_cov', 1000.0)
        self.declare_parameter('pose_roll_pitch_cov', 1000.0)
        self.declare_parameter('pose_yaw_cov', 0.01)

        self.declare_parameter('twist_linear_cov', 1000.0)
        self.declare_parameter('twist_angular_cov', 1000.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(Odometry, output_topic, 10)
        self.sub = self.create_subscription(
            Odometry,
            input_topic,
            self.odom_callback,
            sub_qos,
        )

        self.get_logger().info(f'{input_topic} -> {output_topic}')

    def odom_callback(self, msg: Odometry):
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose = msg.pose.pose
        out.twist.twist = msg.twist.twist

        pose_xy_cov = float(self.get_parameter('pose_xy_cov').value)
        pose_z_cov = float(self.get_parameter('pose_z_cov').value)
        pose_roll_pitch_cov = float(self.get_parameter('pose_roll_pitch_cov').value)
        pose_yaw_cov = float(self.get_parameter('pose_yaw_cov').value)
        twist_linear_cov = float(self.get_parameter('twist_linear_cov').value)
        twist_angular_cov = float(self.get_parameter('twist_angular_cov').value)

        out.pose.covariance = [
            pose_xy_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, pose_xy_cov, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, pose_z_cov, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, pose_roll_pitch_cov, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, pose_roll_pitch_cov, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, pose_yaw_cov,
        ]

        out.twist.covariance = [
            twist_linear_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, twist_linear_cov, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, twist_linear_cov, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, twist_angular_cov, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, twist_angular_cov, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, twist_angular_cov,
        ]

        self.pub.publish(out)


def main():
    rclpy.init()
    node = VslamCovarianceAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
