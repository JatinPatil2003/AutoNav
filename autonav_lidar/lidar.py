#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class ScanAnalyzer(Node):
    def __init__(self):
        super().__init__('scan_analyzer')
        # Subscribe to the /scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        # Convert intensities tuple to a numpy array for easier processing
        intensities = np.array(msg.intensities)
        num_points = len(intensities)
        
        if num_points > 0:
            unique_vals, counts = np.unique(intensities, return_counts=True)
            max_intensity = np.max(intensities)
            min_intensity = np.min(intensities)
            
            # Combine the values and counts for clean formatting
            intensity_counts = dict(zip(unique_vals.tolist(), counts.tolist()))
            
            self.get_logger().info(f"Number of points: {num_points}")
            self.get_logger().info(f"Min Intensity: {min_intensity}")
            self.get_logger().info(f"Max Intensity: {max_intensity}")
            self.get_logger().info(f"Unique Intensities and Counts:\n{intensity_counts}")
            self.get_logger().info("-" * 40)
        else:
            self.get_logger().warning("Received scan with 0 intensity points. Your LiDAR might not output intensity data.")

def main(args=None):
    rclpy.init(args=args)
    node = ScanAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
