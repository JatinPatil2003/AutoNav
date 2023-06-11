#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import smbus
import time

wheel_separation = 0.5  
wheel_radius = 0.1

bus = smbus.SMBus(1)
address = 0x08

def send_data(data):
    arrdata = []
    for i in str(data):
        arrdata.append(ord(i))
    bus.write_i2c_block_data(address, 0, arrdata)
    return

def receive_data():
    byte = bus.read_byte(address)
    data = bus.read_i2c_block_data(address, 0, byte+1)
    strdata = ""
    for i in data:
        strdata += chr(i)
    return int(strdata[1:])

def cmdvelcallback(data):
    linear_x = data.linear.x
    angular_z = data.angular.z
    right_wheel_velocity = (2 * linear_x + angular_z * wheel_separation) / (2 * wheel_radius)
    send_data(right_wheel_velocity)

rospy.init_node('right')  # Initialize your ROS node
rospy.Subscriber('cmd_vel', Twist, cmdvelcallback)  # Create a subscriber
pub = rospy.Publisher('right_wheel_encoder_ticks', Int32, queue_size=10) 
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    try:
        right_ticks = receive_data()
        pub.pubish(right_ticks)
        rate.sleep()
    except OSError:
        continue