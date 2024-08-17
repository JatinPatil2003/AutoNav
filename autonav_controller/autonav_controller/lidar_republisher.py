#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarRelay(Node):

    def __init__(self):
        super().__init__('lidar_relay')
        self.subscription = self.create_subscription(
            LaserScan,
            'ydlidar/scan',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)

    def listener_callback(self, msg):
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    lidar_relay = LidarRelay()

    try:
        rclpy.spin(lidar_relay)
    except KeyboardInterrupt:
        pass

    lidar_relay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
