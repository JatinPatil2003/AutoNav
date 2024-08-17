#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelocityRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_republisher')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(
            Twist,
            '/autonav_controller/cmd_vel_unstamped',
            10)

    def listener_callback(self, msg):
        self.publisher.publish(msg)
        # self.get_logger().info('Relaying velocity command: '%s'' % msg)


def main(args=None):
    rclpy.init(args=args)
    velocity_relay = VelocityRelay()
    rclpy.spin(velocity_relay)
    velocity_relay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
