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

def receive_data():
    byte = bus.read_byte(address)
    data = bus.read_i2c_block_data(address, 0, byte+1)
    strdata = ""
    for i in data:
        strdata += chr(i)
    return int(strdata[1:])

rospy.init_node('right_encoder')
pub = rospy.Publisher('right_wheel_encoder_ticks', Int32, queue_size=10) 
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    try:
        byte = bus.read_byte(address)
        data = bus.read_i2c_block_data(address, 0, byte+1)
        strdata = ""
        for i in data:
            strdata += chr(i)
        right_ticks = int(strdata[1:])
        print("right: ",right_ticks)
        pub.publish(right_ticks)
        time.sleep(0.05)
        rate.sleep()
    except:
        print("Hello OSError - right.py")
        continue
