from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Bool, Int64
import rclpy.time
from tf2_ros import Buffer, TransformListener
import json
import os
import math

import time as time_module

from models.model import Velocity
from ros.node import ros_node
import rclpy
from routes.websocket import broadcast_message
import asyncio
from enum import Enum

class LED_STATUS(Enum):
    OFF = 0
    ON = 1
    BLINK = 2

map_msg = {}
location_msg = {}
pub = False
twist_msg = Twist()
emergency_msg = Bool()
emergency_msg.data = False
prev_pub = None
localization_cov = None

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
    asyncio.run(broadcast_message({"type": "map", "data": map_msg}))
    # print(map_msg)

def get_map_msg():
    global map_msg
    return map_msg

def location_callback(msg):
    global location_msg, localization_cov
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    localization_cov = msg.pose.covariance

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
    global twist_msg, pub
    twist_msg.linear.x = velocity.linear
    twist_msg.angular.z = velocity.angular
    if velocity.linear == 0.0 and velocity.angular == 0.0:
        pub = False
    else:
        pub = True
    
def set_emergency_status(status: bool):
    global emergency_msg
    emergency_msg.data = status
    emergency_publisher.publish(emergency_msg)

def timer_fuction():
    global twist_publisher, twist_msg, prev_pub
    if prev_pub is not pub:
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        # ros_node.get_logger().info(f"{twist_msg.linear.x}, {twist_msg.angular.z}")
        twist_publisher.publish(twist_msg)
        prev_pub = pub
        return
        
    if pub:
        # ros_node.get_logger().info(f"Publishing {twist_msg.linear.x}, {twist_msg.angular.z}")
        twist_publisher.publish(twist_msg)
    
    prev_pub = pub

    if True:
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

            asyncio.run(broadcast_message({"type": "location", "data": location_msg}))

            # emergency_publisher.publish(emergency_msg)

        except: 
            pass

def set_led_status(status: int):
    global led_publisher
    led_publisher.publish(Int64(data=status))

def get_cov_threshold():
    global localization_cov
    if localization_cov is not None and len(localization_cov) > 0:
        cov_x = localization_cov[0]  # variance in x
        cov_y = localization_cov[7]  # variance in y
        threshold = 0.15  # Example threshold value
        print(f"Covariance X: {cov_x}, Covariance Y: {cov_y}, Threshold: {threshold}")
        if cov_x < threshold and cov_y < threshold:
            return True
    return False

def rotate_n_times(n_rotations: int):
    global twist_publisher, twist_msg

    angular_speed = 0.6  # rad/s
    rotation_time = (2 * math.pi + (math.pi / 3)) / angular_speed   # time for 1 full rotation

    twist_msg.linear.x = 0.0
    twist_msg.angular.z = angular_speed
    print(f"Rotating robot at angular speed: {angular_speed} rad/s for {n_rotations} rotations.")

    for _ in range(n_rotations):
        start = rclpy.clock.Clock().now()

        while (rclpy.clock.Clock().now() - start).nanoseconds < rotation_time * 1e9:
            twist_publisher.publish(twist_msg)
            time_module.sleep(0.05)  # publish at 20 Hz

    # stop robot
    twist_msg.angular.z = 0.0
    twist_publisher.publish(twist_msg)

ros_node.create_subscription(OccupancyGrid, 
                             '/map', map_callback, 10)

ros_node.create_subscription(PoseWithCovarianceStamped,
                             '/amcl_pose', location_callback, 10)

ros_node.create_timer(0.05, timer_fuction)

twist_publisher = ros_node.create_publisher(Twist, 'cmd_vel', 10)

emergency_publisher = ros_node.create_publisher(Bool, '/motor/emergency', 10)

led_publisher = ros_node.create_publisher(Int64, '/led_status', 10)

# led_publisher.publish(Int64(data=5))

location_buffer = Buffer()

location_listner = TransformListener(location_buffer, ros_node)

