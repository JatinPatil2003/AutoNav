#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import smbus
import time

wheel_separation = 0.23
wheel_radius = 0.083

bus = smbus.SMBus(1)
address = 0x09

def send_data(data):
    arrdata = []
    for i in str(data):
        arrdata.append(ord(i))
    bus.write_i2c_block_data(address, 0, arrdata)
    return

def cmdvelcallback(data):
    try:
        linear_x = data.linear.x
        angular_z = data.angular.z
        right_wheel_velocity = (2 * linear_x + angular_z * wheel_separation) / 2
        send_data(right_wheel_velocity)
        datar = (right_wheel_velocity / (3.14159 * wheel_radius)) * 60
        print("data sent", datar)
    except Exception as e:
        print("cmdvel Error - right_cmdvel.py", e)
        time.sleep(0.75)
        pass

rospy.init_node('right_cmdvel')  # Initialize your ROS node
rospy.Subscriber('cmd_vel', Twist, cmdvelcallback)  # Create a subscriber

rospy.spin()




