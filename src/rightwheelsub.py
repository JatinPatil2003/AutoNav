#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def right_wheel_callback(msg):
    # rospy.loginfo("Right wheel encoder ticks: %d", msg.data)
    print(msg.data)

rospy.init_node('encoder_subscriber_right')

right_wheel_sub = rospy.Subscriber('right_wheel_encoder_ticks', Int32, right_wheel_callback)

rospy.spin()
