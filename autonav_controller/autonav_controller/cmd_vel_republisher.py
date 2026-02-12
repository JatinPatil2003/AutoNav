#!/usr/bin/env python3

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node


class VelocityRelay(Node):

    def __init__(self):
        super().__init__('cmd_vel_republisher')

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

    def listener_callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = "base_footprint"   # not mandatory but good practice
        stamped.twist = msg

        self.publisher.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
