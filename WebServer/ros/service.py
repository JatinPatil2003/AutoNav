from nav2_msgs.srv import SetInitialPose
from action_msgs.msg import GoalStatus
import tf_transformations
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from models.model import Pose, Goal
from nav2_msgs.srv import LoadMap

import os

from ros.node import ros_node

initial_pose_client = ros_node.create_client(SetInitialPose, '/set_initial_pose')

load_map_client = ros_node.create_client(LoadMap, '/map_server/load_map')

def set_initial_pose(pose: Goal):
    request = SetInitialPose.Request()
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.stamp = ros_node.get_clock().now().to_msg()
    initial_pose.header.frame_id = 'map'
    initial_pose.pose.pose.position.x = pose.x
    initial_pose.pose.pose.position.y = pose.y
    quaternion = tf_transformations.quaternion_from_euler(0, 0, pose.theta)
    initial_pose.pose.pose.orientation.z = quaternion[2]
    initial_pose.pose.pose.orientation.w = quaternion[3]

    request.pose = initial_pose

    if initial_pose_client.wait_for_service(timeout_sec=1.0):
        response = initial_pose_client.call_async(request)

def loadMapService(map_name: str):
    request = LoadMap.Request()
    request.map_url = os.path.join(os.getcwd(), "maps", f"{map_name}.yaml")
    # request.map_url = os.path.join("/home/jatin/AutoNav/WebServer/maps", f"{map_name}.yaml")

    if load_map_client.wait_for_service(timeout_sec=1.0):
        response = load_map_client.call(request)
        return response
    else:
        print("LoadMap service not available.")
        return None