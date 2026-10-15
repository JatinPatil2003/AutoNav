import { eventBus } from "../../core/eventBus";
import { RosTopicGateway } from "../../ros/rosTopicGateway";
import { NavigationService } from "../navigation/navigation.service";
import { mapOccupancyGridToMapUpdate, mapPoseToLocationUpdate, OccupancyGridMsg, PoseMsg } from "./telemetry.mappers";

const BROADCAST_PERIOD_MS = 50; // 20Hz
const BROADCAST_PERIOD_NS = BigInt(BROADCAST_PERIOD_MS) * 1_000_000n;
const AMCL_POSE_TOPIC = "/amcl_pose";
const MAP_TOPIC = "/map";

/**
 * Application wiring, not a ROS-primitive gateway: subscribes /map and
 * /amcl_pose on the shared RosTopicGateway (so NavigationService's and
 * MappingService's getCurrentMap()/getCurrentLocation() have data to read
 * via getLatest), and republishes onto core/eventBus for
 * modules/telemetry's WebSocket to pick up.
 *
 * Known gap carried over unchanged: AMCL-only pose, no TF2 lookup during
 * SLAM (see mapping.service.ts's captureWaypoint).
 */
export function registerTelemetryBroadcast(topicGateway: RosTopicGateway, navigationService: NavigationService): void {
  topicGateway.subscribe<OccupancyGridMsg>("nav_msgs/msg/OccupancyGrid", MAP_TOPIC, (msg) => {
    eventBus.emit("map", mapOccupancyGridToMapUpdate(msg));
  });

  topicGateway.subscribe<PoseMsg>("geometry_msgs/msg/PoseWithCovarianceStamped", AMCL_POSE_TOPIC);

  topicGateway.createTimer(BROADCAST_PERIOD_NS, () => {
    const poseMsg = topicGateway.getLatest<PoseMsg>(AMCL_POSE_TOPIC);
    if (poseMsg) {
      eventBus.emit("location", mapPoseToLocationUpdate(poseMsg));
    }

    eventBus.emit("navStatus", { status: navigationService.getStatus() });
  });
}
