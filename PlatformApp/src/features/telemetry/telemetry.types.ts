/**
 * Mirrors PlatformServer's wire contract by hand — PlatformServer and
 * PlatformApp don't share a types package (see
 * PlatformServer/src/modules/robot-link/robotLink.protocol.ts and
 * telemetry-relay/telemetryRelay.protocol.ts for the source of truth).
 */

export type SlamBackend = "cartographer" | "slam_toolbox";
export type NavStatusValue = "IDLE" | "ACTIVE" | "SUCCEEDED" | "CANCELED" | "FAILED";

export interface PoseUpdateFrame {
  type: "pose.update";
  x: number;
  y: number;
  theta: number;
  source: "amcl" | "slam_bridge";
  confident: boolean;
}

export interface OccupancyGridInfo {
  width: number;
  height: number;
  resolution: number;
  origin: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
}

export interface MapSnapshotFrame {
  type: "map.snapshot";
  info: OccupancyGridInfo;
  data: number[];
}

export interface NavStatusFrame {
  type: "nav.status";
  status: NavStatusValue;
  feedbackDistanceRemaining: number | null;
  activeGoalName: string | null;
}

export interface RobotStatusFrame {
  type: "robot.status";
  rclOk: boolean;
  navigationActive: boolean;
  mappingActive: boolean;
  activeBackend: SlamBackend | null;
  maxLinear: number;
  maxAngular: number;
  estopEngaged: boolean;
}

export interface RobotLinkStateFrame {
  type: "robot_link_state";
  online: boolean;
}

export type TelemetryFrame = PoseUpdateFrame | MapSnapshotFrame | NavStatusFrame | RobotStatusFrame | RobotLinkStateFrame;

export interface Pose2D {
  x: number;
  y: number;
  theta: number;
}
