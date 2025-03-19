#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import tf2_ros
from geometry_msgs.msg import TransformStamped

class IMUToTFPublisher(Node):
    def __init__(self):
        super().__init__('imu_tf_publisher')
        self.subscription = self.create_subscription(
            Imu,
            '/bno055/imu',
            self.imu_callback,
            10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info("IMU to TF Publisher Node Started")
    
    def imu_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'  # Parent frame
        t.child_frame_id = 'imu_link'  # IMU frame
        
        # Set the orientation from IMU message
        t.transform.translation.z = 0.5
        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w
        
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info("Published TF from odom to imu_link")


def main(args=None):
    rclpy.init(args=args)
    node = IMUToTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
