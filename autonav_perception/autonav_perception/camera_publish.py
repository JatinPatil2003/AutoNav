#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera', 10)
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.publish_image)
        self.bridge = CvBridge()
        self.capture = cv2.VideoCapture(0) 

        self.fixed_width = 720
        self.fixed_height = 480

    def publish_image(self):
        ret, frame = self.capture.read()
        if ret:
            frame_resized = cv2.resize(frame, (self.fixed_width, self.fixed_height))
            ros_image = self.bridge.cv2_to_imgmsg(frame_resized, encoding="bgr8")
            self.publisher_.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
