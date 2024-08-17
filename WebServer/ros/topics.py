from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import rclpy.time
from tf2_ros import Buffer, TransformListener
import json
import os
import math

from models.model import Velocity
from ros.node import ros_node
import rclpy

map_msg = {}
location_msg = {}

def map_callback(msg):
    # print('\n\n\n\n\n\n\nRunning Map Callback\n\n\n\n\n')
    global map_msg
    map_msg = {
            'info': {
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin': {
                    'position': {
                        'x': msg.info.origin.position.x,
                        'y': msg.info.origin.position.y,
                        'z': msg.info.origin.position.z,
                    },
                    'orientation': {
                        'x': msg.info.origin.orientation.x,
                        'y': msg.info.origin.orientation.y,
                        'z': msg.info.origin.orientation.z,
                        'w': msg.info.origin.orientation.w,
                    }
                }
            },
            'data': list(msg.data)
        }
    # print(map_msg)

def get_map_msg():
    global map_msg
    return map_msg

def location_callback(msg):
    global location_msg
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    # Quaternion to Euler conversion (yaw)
    q = msg.pose.pose.orientation
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    theta = math.atan2(siny_cosp, cosy_cosp)

    location_msg = {
            'x': x,
            'y': y,
            'theta': theta
        }
    # print(location_msg)
    
def get_location_msg():
    global location_msg
    return location_msg

def get_location_mapping_msg():
    global location_buffer
    try:
        trans = location_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        x = trans.transform.translation.x
        y = trans.transform.translation.y

        # Quaternion to Euler conversion (yaw)
        q = trans.transform.rotation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        location_msg = {
                'x': x,
                'y': y,
                'theta': theta
            }
        return location_msg
    except: 
        return None

def set_joystick_velocity(velocity: Velocity):
    global twist_publisher
    msg = Twist()
    msg.linear.x = velocity.linear
    msg.angular.z = velocity.angular
    print(velocity.linear, velocity.angular)
    twist_publisher.publish(msg)

ros_node.create_subscription(OccupancyGrid, 
                             '/map', map_callback, 10)

ros_node.create_subscription(PoseWithCovarianceStamped,
                             '/amcl_pose', location_callback, 10)

twist_publisher = ros_node.create_publisher(Twist, 'cmd_vel', 10)

location_buffer = Buffer()

location_listner = TransformListener(location_buffer, ros_node)

