#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf_transformations


class OdomLogger(Node):
    def __init__(self):
        super().__init__('odom_logger')
        self.subscription = self.create_subscription(
            Odometry,
            '/autonav_controller/odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        yaw = tf_transformations.euler_from_quaternion(orientation_list)[2]

        self.get_logger().info(
            'Position: ({:.2f}, {:.2f}), Theta: {:.2f}'.format(position.x, position.y, yaw))


def main(args=None):
    rclpy.init(args=args)
    node = OdomLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
