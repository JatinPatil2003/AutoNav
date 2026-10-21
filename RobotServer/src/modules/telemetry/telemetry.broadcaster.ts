import { IPlatformLink } from "../../clients/platformLink.interface";
import { eventBus } from "../../core/eventBus";
import { DeviceIdentityService } from "../../ros/deviceIdentity";
import { LaunchManager } from "../../ros/launchManager";
import { RosTopicGateway } from "../../ros/rosTopicGateway";
import { MappingService } from "../mapping/mapping.service";
import { NavigationService } from "../navigation/navigation.service";
import { NavStatus } from "../navigation/navigation.types";
import { TeleopService } from "../teleop/teleop.service";
import {
  MapSnapshotFrame,
  NavStatusLinkFrame,
  PoseUpdateFrame,
  RobotStatusFrame,
} from "../teleop/teleopLink.protocol";
import {
  isPoseConfident,
  mapOccupancyGridToMapUpdate,
  mapPoseStampedToLocationUpdate,
  mapPoseToLocationUpdate,
  OccupancyGridMsg,
  PoseMsg,
  PoseStampedMsg,
} from "./telemetry.mappers";

const BROADCAST_PERIOD_MS = 50; // 20Hz, local eventBus/WS feed
const BROADCAST_PERIOD_NS = BigInt(BROADCAST_PERIOD_MS) * 1_000_000n;
const AMCL_POSE_TOPIC = "/amcl_pose";
const SLAM_BRIDGE_POSE_TOPIC = "/pose_bridge/map_pose";
const MAPPING_LAUNCH_NAME = "mapping";
const MAP_TOPIC = "/map";

/**
 * Every local-broadcast tick is 20Hz; pushing every tick over the
 * robot-link WAN/VPN hop would be wasteful, so pose/status frames to
 * PlatformServer are sent every LINK_PUSH_TICKS-th tick instead (~5Hz).
 */
const LINK_PUSH_TICKS = 4;
/** Map snapshots are large and mostly static tick-to-tick during SLAM — pushed far less often than pose. No delta frames yet (see teleopLink.protocol.ts). */
const MAP_PUSH_TICKS = 40; // ~2s at 20Hz

export interface TelemetryBroadcastDeps {
  topicGateway: RosTopicGateway;
  navigationService: NavigationService;
  mappingService: MappingService;
  teleopService: TeleopService;
  deviceIdentity: DeviceIdentityService;
  launchManager: LaunchManager;
  link: IPlatformLink;
}

/**
 * Application wiring, not a ROS-primitive gateway: subscribes /map,
 * /amcl_pose, and the SLAM-only /pose_bridge/map_pose (published by
 * autonav_utils's tf_pose_bridge node) on the shared
 * RosTopicGateway (so NavigationService's and MappingService's
 * getCurrentMap()/getCurrentLocation() have data to read via getLatest),
 * and republishes onto core/eventBus for modules/telemetry's WebSocket to
 * pick up. Mirrors MappingService.getCurrentLocation()'s AMCL-vs-bridge
 * preference by launch state, so the live telemetry feed shows a moving
 * pose during SLAM too, not just during navigation.
 *
 * Also pushes the same data (at a lower rate) over the already-open
 * platformLink connection, unconditionally — regardless of whether any
 * PlatformServer dashboard viewer is currently watching, same as this
 * function's own local 20Hz loop. PlatformServer's telemetry-relay module
 * is what actually fans these out to interested viewers; this side never
 * needs to know whether anyone's watching.
 */
export function registerTelemetryBroadcast(deps: TelemetryBroadcastDeps): void {
  const { topicGateway, navigationService, mappingService, teleopService, deviceIdentity, launchManager, link } = deps;

  topicGateway.subscribe<OccupancyGridMsg>("nav_msgs/msg/OccupancyGrid", MAP_TOPIC, (msg) => {
    eventBus.emit("map", mapOccupancyGridToMapUpdate(msg));
  });

  topicGateway.subscribe<PoseMsg>("geometry_msgs/msg/PoseWithCovarianceStamped", AMCL_POSE_TOPIC);
  topicGateway.subscribe<PoseStampedMsg>("geometry_msgs/msg/PoseStamped", SLAM_BRIDGE_POSE_TOPIC);

  let tick = 0;

  topicGateway.createTimer(BROADCAST_PERIOD_NS, () => {
    tick += 1;
    const isMapping = launchManager.status().name === MAPPING_LAUNCH_NAME;
    const bridgePose = isMapping ? topicGateway.getLatest<PoseStampedMsg>(SLAM_BRIDGE_POSE_TOPIC) : null;
    const amclPoseMsg = topicGateway.getLatest<PoseMsg>(AMCL_POSE_TOPIC);

    if (bridgePose) {
      eventBus.emit("location", mapPoseStampedToLocationUpdate(bridgePose));
    } else if (amclPoseMsg) {
      eventBus.emit("location", mapPoseToLocationUpdate(amclPoseMsg));
    }

    eventBus.emit("navStatus", { status: navigationService.getStatus() });

    if (tick % LINK_PUSH_TICKS === 0) {
      pushPoseAndStatus({ bridgePose, amclPoseMsg, navigationService, mappingService, teleopService, deviceIdentity, topicGateway, link });
    }

    if (isMapping && tick % MAP_PUSH_TICKS === 0) {
      pushMapSnapshot(topicGateway, link);
    }
  });
}

interface PushPoseAndStatusParams {
  bridgePose: PoseStampedMsg | null;
  amclPoseMsg: PoseMsg | null;
  navigationService: NavigationService;
  mappingService: MappingService;
  teleopService: TeleopService;
  deviceIdentity: DeviceIdentityService;
  topicGateway: RosTopicGateway;
  link: IPlatformLink;
}

function pushPoseAndStatus(params: PushPoseAndStatusParams): void {
  const { bridgePose, amclPoseMsg, navigationService, mappingService, teleopService, deviceIdentity, topicGateway, link } = params;

  if (bridgePose) {
    const { x, y, theta } = mapPoseStampedToLocationUpdate(bridgePose);
    const frame: PoseUpdateFrame = { type: "pose.update", x, y, theta, source: "slam_bridge", confident: true };
    link.send(frame);
  } else if (amclPoseMsg) {
    const { x, y, theta } = mapPoseToLocationUpdate(amclPoseMsg);
    const frame: PoseUpdateFrame = {
      type: "pose.update",
      x,
      y,
      theta,
      source: "amcl",
      confident: isPoseConfident(amclPoseMsg),
    };
    link.send(frame);
  }

  const navStatus = navigationService.getStatus();
  const navStatusFrame: NavStatusLinkFrame = {
    type: "nav.status",
    status: navStatus,
    feedbackDistanceRemaining: navStatus === NavStatus.ACTIVE ? navigationService.getFeedback() : null,
    activeGoalName: navigationService.getActiveGoalName(),
  };
  link.send(navStatusFrame);

  const limits = teleopService.getLimits();
  const robotStatusFrame: RobotStatusFrame = {
    type: "robot.status",
    rclOk: topicGateway.isNodeAlive() && deviceIdentity.getRobotId() !== null,
    navigationActive: navStatus === NavStatus.ACTIVE,
    mappingActive: mappingService.getActiveBackend() !== null,
    activeBackend: mappingService.getActiveBackend(),
    maxLinear: limits.maxLinear,
    maxAngular: limits.maxAngular,
    estopEngaged: teleopService.isEmergencyEngaged(),
  };
  link.send(robotStatusFrame);
}

function pushMapSnapshot(topicGateway: RosTopicGateway, link: IPlatformLink): void {
  const mapMsg = topicGateway.getLatest<OccupancyGridMsg>(MAP_TOPIC);
  if (!mapMsg) return;

  const update = mapOccupancyGridToMapUpdate(mapMsg);
  const frame: MapSnapshotFrame = { type: "map.snapshot", info: update.info, data: update.data };
  link.send(frame);
}
