from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
from rclpy.node import Node

HEARTBEAT_TOPIC = '/autonav/system_heartbeat'
PUBLISH_PERIOD_SEC = 1.0


class SystemHeartbeat(Node):
    """Publishes a fixed-rate liveness beat for the always-on utils
    container, independent of any per-sensor health already covered by
    RobotServer's TopicFrequencyMonitor. Absence of this topic — not a
    DEGRADED level in it — is the signal that the container itself is down.
    """

    def __init__(self) -> None:
        super().__init__('system_heartbeat')
        self._publisher = self.create_publisher(DiagnosticStatus, HEARTBEAT_TOPIC, 10)
        self.create_timer(PUBLISH_PERIOD_SEC, self._on_timer)

    def _on_timer(self) -> None:
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = 'autonav_utils'
        status.message = 'alive'
        self._publisher.publish(status)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SystemHeartbeat()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
