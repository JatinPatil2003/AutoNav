import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

MAP_FRAME = 'map'
BASE_FRAME = 'base_footprint'
POSE_TOPIC = '/pose_bridge/map_pose'
LOOKUP_PERIOD_SEC = 0.05  # 20Hz, matching RobotServer's telemetry.broadcaster.ts


def transform_to_pose_stamped(transform: TransformStamped) -> PoseStamped:
    """Pure conversion from a looked-up TF to the PoseStamped this node
    publishes — kept free of any ROS node/buffer state so it's directly
    unit-testable."""
    pose = PoseStamped()
    pose.header = transform.header
    pose.pose.position.x = transform.transform.translation.x
    pose.pose.position.y = transform.transform.translation.y
    pose.pose.position.z = transform.transform.translation.z
    pose.pose.orientation = transform.transform.rotation
    return pose


class TfPoseBridge(Node):
    """Republishes map->base_footprint as a plain topic during SLAM.

    rclnodejs (RobotServer's ROS binding) has no TF2 lookup of its own, and
    AMCL — which already gives RobotServer a pose during navigation — isn't
    running during SLAM. slam_toolbox/Cartographer both publish the TF
    regardless; this node is the bridge from that TF to something
    RobotServer's generic RosTopicGateway can subscribe to.
    """

    def __init__(self) -> None:
        super().__init__('tf_pose_bridge')
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._publisher = self.create_publisher(PoseStamped, POSE_TOPIC, 10)
        self.create_timer(LOOKUP_PERIOD_SEC, self._on_timer)

    def _on_timer(self) -> None:
        pose = self.lookup_pose()
        if pose is not None:
            self._publisher.publish(pose)

    def lookup_pose(self) -> PoseStamped | None:
        """Looks up map->base_footprint and converts it. Returns None —
        never raises — on LookupException/ConnectivityException/
        ExtrapolationException: expected early in a SLAM session before the
        map frame exists yet, or during a transient extrapolation gap, and
        callers (the timer, and MappingService.getCurrentLocation() on the
        RobotServer side) already treat "no pose yet" as a normal state."""
        try:
            transform = self._buffer.lookup_transform(MAP_FRAME, BASE_FRAME, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().debug(f'tf lookup skipped: {exc}')
            return None
        return transform_to_pose_stamped(transform)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TfPoseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
