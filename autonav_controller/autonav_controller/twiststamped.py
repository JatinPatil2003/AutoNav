#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__('twist_to_twist_stamped_converter')
        self.subscriber = self.create_subscription(
            Twist, '/cmd_vel', self.twist_callback, 10)
        self.publisher = self.create_publisher(
            TwistStamped, '/autonav_controller/cmd_vel', 10)

    def twist_callback(self, msg):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.twist = msg
        self.publisher.publish(twist_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStamped()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
