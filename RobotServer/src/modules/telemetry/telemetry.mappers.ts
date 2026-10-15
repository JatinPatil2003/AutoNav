import type { geometry_msgs, nav_msgs } from "rclnodejs";
import { LocationUpdate, MapUpdate } from "../../core/eventBus";
import { yawFromQuaternion } from "../../ros/quaternion";

const COVARIANCE_THRESHOLD = 0.15;

export type OccupancyGridMsg = nav_msgs.msg.OccupancyGrid;
export type PoseMsg = geometry_msgs.msg.PoseWithCovarianceStamped;

export function mapOccupancyGridToMapUpdate(msg: OccupancyGridMsg): MapUpdate {
  return {
    info: {
      width: msg.info.width,
      height: msg.info.height,
      resolution: msg.info.resolution,
      origin: {
        position: msg.info.origin.position,
        orientation: msg.info.origin.orientation,
      },
    },
    data: Array.from(msg.data),
  };
}

export function mapPoseToLocationUpdate(msg: PoseMsg): LocationUpdate {
  return {
    x: msg.pose.pose.position.x,
    y: msg.pose.pose.position.y,
    theta: yawFromQuaternion(msg.pose.pose.orientation),
  };
}

/** `null` (no /amcl_pose message received yet) is treated as "not confident". */
export function isPoseConfident(msg: PoseMsg | null): boolean {
  if (!msg) {
    return false;
  }
  const covariance = msg.pose.covariance;
  if (!covariance || covariance.length === 0) {
    return false;
  }
  return covariance[0] < COVARIANCE_THRESHOLD && covariance[7] < COVARIANCE_THRESHOLD;
}
