#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from geometry_msgs.msg import Twist
from pynput import keyboard

class TeleopPublisher(Node):
    def __init__(self):
        super().__init__('teleop_publisher')
        self.publisher_ = self.create_publisher(Twist, 'autonav_controller/cmd_vel_unstamped', 10)
        self.twist = Twist()
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release, suppress=True)
        self.listener.start()
        self.timer = self.create_timer(0.05, self.timer_callback)

    def on_press(self, key):
        if key == keyboard.Key.esc:
            return False
        if key in [keyboard.KeyCode.from_char('w'), keyboard.KeyCode.from_char('W')]:
            self.twist.linear.x = 0.5
        elif key in [keyboard.KeyCode.from_char('s'), keyboard.KeyCode.from_char('S')]:
            self.twist.linear.x = -0.5
        elif key in [keyboard.KeyCode.from_char('a'), keyboard.KeyCode.from_char('A')]:
            self.twist.angular.z = 2.0
        elif key in [keyboard.KeyCode.from_char('d'), keyboard.KeyCode.from_char('D')]:
            self.twist.angular.z = -2.0

    def on_release(self, key):
        if key in [keyboard.KeyCode.from_char('w'), keyboard.KeyCode.from_char('W'), keyboard.KeyCode.from_char('s'), keyboard.KeyCode.from_char('S')]:
            self.twist.linear.x = 0.0
        elif key in [keyboard.KeyCode.from_char('a'), keyboard.KeyCode.from_char('A'), keyboard.KeyCode.from_char('d'), keyboard.KeyCode.from_char('D')]:
            self.twist.angular.z = 0.0

    def timer_callback(self):
        self.publisher_.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    teleop_publisher = TeleopPublisher()
    rclpy.spin(teleop_publisher)

    teleop_publisher.listener.stop()
    teleop_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


