from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
import tf_transformations
from geometry_msgs.msg import PoseStamped, Quaternion
from models.model import Pose

from ros.node import ros_node

goal_client = ActionClient(ros_node, NavigateToPose, 'navigate_to_pose')

goal_send = None
navigation_feedback = 0.0

def send_goal(pose: Pose):
    global goal_send
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = pose.x
    goal.pose.pose.position.y = pose.y
    quat = tf_transformations.quaternion_from_euler(0, 0, pose.theta)
    goal.pose.pose.orientation = Quaternion()
    goal.pose.pose.orientation.z = quat[2]
    goal.pose.pose.orientation.w = quat[3]

    if not goal_send:
        if goal_client.wait_for_server(timeout_sec=1.0):
            goal_send =goal_client.send_goal_async(goal, feedback_callback=feedback_callback)
            goal_send.add_done_callback(goal_accept_callback)
    else:
        cancel_goal()
        send_goal(pose)

def goal_accept_callback(future):
    goal_result = future.result().get_result_async()
    goal_result.add_done_callback(result_callback)

def result_callback(future):
    global goal_send, navigation_feedback
    result = future.result()
    goal_send = None
    # print(result)
    navigation_feedback = 0.0

def feedback_callback(feedback_msg):
    # print(feedback_msg)
    global navigation_feedback
    navigation_feedback = feedback_msg.feedback.distance_remaining

def cancel_goal():
    global goal_send, navigation_feedback
    if goal_send:
        navigation_feedback = 0.0
        goal_send.result().cancel_goal_async()
        goal_send = None

def get_navigation_feedback():
    global navigation_feedback
    return navigation_feedback


# nav2_msgs.action.NavigateToPose_FeedbackMessage(
#     goal_id=unique_identifier_msgs.msg.UUID(
#         uuid=array([ 69,  59,  89, 142,  10, 208,  64, 216, 185, 207, 174,  54,  94,
#         51,  41, 102], dtype=uint8)),
#     feedback=nav2_msgs.action.NavigateToPose_Feedback(
#         current_pose=geometry_msgs.msg.PoseStamped(
#             header=std_msgs.msg.Header(
#                 stamp=builtin_interfaces.msg.Time(sec=2343, nanosec=314000000), frame_id='map'), 
#             pose=geometry_msgs.msg.Pose(
#                 position=geometry_msgs.msg.Point(x=2.3435522637581943, y=-1.353488348729547, z=0.00012951278530743803), 
#                 orientation=geometry_msgs.msg.Quaternion(x=0.0002765754628599896, y=-0.00015303712413239332, z=-0.606170694839948, w=0.7953345137757487))), 
#         navigation_time=builtin_interfaces.msg.Duration(sec=5, nanosec=930308261), 
#         estimated_time_remaining=builtin_interfaces.msg.Duration(sec=0, nanosec=0), 
#         number_of_recoveries=0, 
#         distance_remaining=0.015159981325268745))
# [component_container_isolated-1] [INFO] [1722442055.722920927] [bt_navigator]: Goal succeeded
# nav2_msgs.action.NavigateToPose_GetResult_Response(
#     status=4, 
#     result=nav2_msgs.action.NavigateToPose_Result(
#         result=std_msgs.msg.Empty()))