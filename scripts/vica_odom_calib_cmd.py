#!/usr/bin/env python3

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class OdomCalibCmd(Node):
    def __init__(self):
        super().__init__("vica_odom_calib_cmd")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def publish_for(self, twist, duration, rate_hz):
        dt = 1.0 / rate_hz
        count = int(duration * rate_hz)

        for _ in range(count):
            self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)


def build_twist(args):
    twist = Twist()

    if args.mode == "forward":
        twist.linear.x = abs(args.linear)
    elif args.mode == "backward":
        twist.linear.x = -abs(args.linear)
    elif args.mode == "left":
        twist.angular.z = abs(args.angular)
    elif args.mode == "right":
        twist.angular.z = -abs(args.angular)
    elif args.mode == "stop":
        pass
    else:
        raise ValueError(f"unsupported mode: {args.mode}")

    return twist


def parse_args():
    parser = argparse.ArgumentParser(
        description="Publish repeatable /cmd_vel commands for VICA odom calibration."
    )
    parser.add_argument(
        "mode",
        choices=["forward", "backward", "left", "right", "stop"],
        help="Motion command to publish.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Command duration in seconds.",
    )
    parser.add_argument(
        "--linear",
        type=float,
        default=0.2,
        help="Linear speed in m/s for forward/backward.",
    )
    parser.add_argument(
        "--angular",
        type=float,
        default=0.3698,
        help="Angular speed in rad/s for left/right.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="Publish rate in Hz.",
    )
    parser.add_argument(
        "--stop-duration",
        type=float,
        default=1.0,
        help="Stop command duration after motion.",
    )
    parser.add_argument(
        "--startup-wait",
        type=float,
        default=1.0,
        help="Wait time before publishing, allowing subscribers to match.",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    if args.duration < 0.0 or args.rate <= 0.0 or args.stop_duration < 0.0:
        print("duration must be >= 0, rate must be > 0, stop-duration must be >= 0")
        return 2

    rclpy.init()
    node = OdomCalibCmd()

    try:
        time.sleep(args.startup_wait)

        twist = build_twist(args)
        stop = Twist()

        node.get_logger().info(
            f"mode={args.mode}, duration={args.duration:.3f}s, "
            f"linear={twist.linear.x:.4f}m/s, angular={twist.angular.z:.4f}rad/s"
        )

        node.publish_for(twist, args.duration, args.rate)
        node.publish_for(stop, args.stop_duration, args.rate)
        node.get_logger().info("stop command published")
    except KeyboardInterrupt:
        node.get_logger().warn("interrupted, publishing stop")
        node.publish_for(Twist(), args.stop_duration, args.rate)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
