#!/usr/bin/env python3

import math
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node


class VelocityRelay(Node):

    def __init__(self):
        super().__init__('cmd_vel_republisher')

        # Declare parameters
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('publish_rate', 15.0)

        self.latest_msg = None
        self.last_msg_time = None

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )

        self.publisher = self.create_publisher(
            TwistStamped,
            '/autonav_controller/cmd_vel',
            10
        )

        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        timer_period = 1.0 / publish_rate if publish_rate > 0.0 else 0.1

        # Republish command at fixed rate
        self.timer = self.create_timer(
            timer_period,
            self.timer_callback
        )

        self.get_logger().info(
            f"cmd_vel_republisher started at {publish_rate} Hz (timeout: {self.get_parameter('cmd_vel_timeout').value}s)"
        )

    def listener_callback(self, msg: Twist):
        # Store the latest command and record timestamp
        self.latest_msg = msg
        self.last_msg_time = self.get_clock().now()

    def timer_callback(self):
        now = self.get_clock().now()
        timeout = self.get_parameter('cmd_vel_timeout').get_parameter_value().double_value

        stamped = TwistStamped()
        stamped.header.stamp = now.to_msg()
        stamped.header.frame_id = "base_footprint"

        # Check if timed out or no command received yet (e.g. at startup)
        is_timed_out = (self.last_msg_time is None) or (
            (now - self.last_msg_time).nanoseconds / 1e9 > timeout
        )

        if not is_timed_out and self.latest_msg is not None:
            msg = self.latest_msg
            stamped.twist.linear.x = (
                float(msg.linear.x) if math.isfinite(msg.linear.x) else 0.0
            )
            stamped.twist.linear.y = (
                float(msg.linear.y) if math.isfinite(msg.linear.y) else 0.0
            )
            stamped.twist.linear.z = (
                float(msg.linear.z) if math.isfinite(msg.linear.z) else 0.0
            )

            stamped.twist.angular.x = (
                float(msg.angular.x) if math.isfinite(msg.angular.x) else 0.0
            )
            stamped.twist.angular.y = (
                float(msg.angular.y) if math.isfinite(msg.angular.y) else 0.0
            )
            stamped.twist.angular.z = (
                float(msg.angular.z) if math.isfinite(msg.angular.z) else 0.0
            )
        else:
            # Publish 0 velocity at start or when timed out
            stamped.twist.linear.x = 0.0
            stamped.twist.linear.y = 0.0
            stamped.twist.linear.z = 0.0
            stamped.twist.angular.x = 0.0
            stamped.twist.angular.y = 0.0
            stamped.twist.angular.z = 0.0

        self.publisher.publish(stamped)


def main(args=None):
    rclpy.init(args=args)

    node = VelocityRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()