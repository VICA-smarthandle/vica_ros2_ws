#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanRearFilter(Node):
    def __init__(self):
        super().__init__("scan_rear_filter")

        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_nav2")

        # base_link 기준 후방 자기 구조물 제거 영역.
        self.declare_parameter("rear_x_min", -1.5)
        self.declare_parameter("rear_x_max", 0.05)
        self.declare_parameter("rear_y_abs", 0.3)

        # tf_vica 기준 base_link -> laser_frame translation.
        self.declare_parameter("laser_x_in_base", 0.00995)
        self.declare_parameter("laser_y_in_base", 0.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.pub = self.create_publisher(
            LaserScan,
            output_topic,
            qos_profile_sensor_data,
        )
        self.sub = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"{input_topic} -> {output_topic}, "
            "rear box filter: "
            "x=[-1.5, 0.05], y=[-0.3, 0.3]"
        )

    def scan_callback(self, msg: LaserScan):
        rear_x_min = float(self.get_parameter("rear_x_min").value)
        rear_x_max = float(self.get_parameter("rear_x_max").value)
        rear_y_abs = float(self.get_parameter("rear_y_abs").value)
        laser_x = float(self.get_parameter("laser_x_in_base").value)
        laser_y = float(self.get_parameter("laser_y_in_base").value)

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = list(msg.ranges)
        out.intensities = list(msg.intensities)

        for index, distance in enumerate(out.ranges):
            if not math.isfinite(distance):
                continue

            if distance < msg.range_min or distance > msg.range_max:
                continue

            angle = msg.angle_min + index * msg.angle_increment
            x_base = laser_x + distance * math.cos(angle)
            y_base = laser_y + distance * math.sin(angle)

            inside_rear_box = (
                rear_x_min <= x_base <= rear_x_max
                and abs(y_base) <= rear_y_abs
            )

            if inside_rear_box:
                out.ranges[index] = math.inf

        self.pub.publish(out)


def main():
    rclpy.init()
    node = ScanRearFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
