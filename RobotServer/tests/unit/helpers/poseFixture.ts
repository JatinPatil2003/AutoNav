import { quaternionFromYaw } from "../../../src/ros/quaternion";
import { PoseMsg } from "../../../src/modules/telemetry/telemetry.mappers";

interface MakePoseMessageOptions {
  x: number;
  y: number;
  theta: number;
  /** true (default) seeds a low covariance (AMCL "confident"); false seeds a high one. */
  confident?: boolean;
}

/**
 * Builds a PoseWithCovarianceStamped-shaped fixture for tests, since the
 * generic RosTopicGateway only caches raw ROS messages (not each feature's
 * simplified DTOs) — tests seed this instead of a plain {x,y,theta} object.
 */
export function makePoseMessage({ x, y, theta, confident = true }: MakePoseMessageOptions): PoseMsg {
  const quat = quaternionFromYaw(theta);
  const covarianceValue = confident ? 0.01 : 1.0;
  const covariance = new Array(36).fill(0);
  covariance[0] = covarianceValue;
  covariance[7] = covarianceValue;

  return {
    header: { frame_id: "map", stamp: { sec: 0, nanosec: 0 } },
    pose: {
      pose: {
        position: { x, y, z: 0 },
        orientation: { x: 0, y: 0, z: quat.z, w: quat.w },
      },
      covariance,
    },
  } as PoseMsg;
}
