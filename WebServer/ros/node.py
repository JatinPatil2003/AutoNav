from rclpy.node import Node
import rclpy

ros_node = None

class ROSNode(Node):
    def __init__(self):
        super().__init__('web_server')

def start_ros_node():
    global ros_node
    rclpy.init(args=None)
    ros_node = ROSNode()
    try:
        rclpy.spin(ros_node)
        # ros_node.destroy_node()
        # rclpy.shutdown()
    except:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()