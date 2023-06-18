#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

def odom_callback(msg):

    # Create a TransformStamped message
    odom_to_base_link = TransformStamped()
    odom_to_base_link.header.stamp = rospy.Time.now()
    odom_to_base_link.header.frame_id = 'odom'
    odom_to_base_link.child_frame_id = 'base_link'

    # Set the translation
    odom_to_base_link.transform.translation.x = msg.pose.pose.position.x
    odom_to_base_link.transform.translation.y = msg.pose.pose.position.y
    odom_to_base_link.transform.translation.z = msg.pose.pose.position.z

    # Set the rotation
    odom_to_base_link.transform.rotation = msg.pose.pose.orientation

    # Publish the transform
    tf_broadcaster.sendTransform(odom_to_base_link)

if __name__ == '__main__':
    rospy.init_node('odom_base_link_broadcaster')

    tf_broadcaster = tf2_ros.TransformBroadcaster()
    rospy.Subscriber('odom', Odometry, odom_callback)

    rospy.spin()
