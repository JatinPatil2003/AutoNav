#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def left_wheel_callback(msg):
    # rospy.loginfo("Left wheel encoder ticks: %d", msg.data)
    print(msg.data)

rospy.init_node('encoder_subscriber_left')

left_wheel_sub = rospy.Subscriber('left_wheel_encoder_ticks', Int32, left_wheel_callback)

rospy.spin()
