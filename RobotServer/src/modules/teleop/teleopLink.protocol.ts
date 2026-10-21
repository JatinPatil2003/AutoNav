import { z } from "zod";

/**
 * Frames crossing the Part 2 robot-link connection that this module cares
 * about (teleop + health). Shared with PlatformServer only by contract —
 * see the architecture plan — not by import, since the two services don't
 * share a package.
 */

export const teleopVelocityFrameSchema = z.object({
  type: z.literal("teleop.velocity"),
  sessionId: z.string(),
  data: z.object({
    linear: z.number(),
    angular: z.number(),
  }),
});

export const teleopEstopFrameSchema = z.object({
  type: z.literal("teleop.estop"),
  sessionId: z.string(),
  engaged: z.boolean(),
});

export const teleopReleaseFrameSchema = z.object({
  type: z.literal("teleop.release"),
  sessionId: z.string(),
});

export const healthQueryFrameSchema = z.object({
  type: z.literal("health.query"),
  id: z.string(),
});

export const inboundLinkFrameSchema = z.discriminatedUnion("type", [
  teleopVelocityFrameSchema,
  teleopEstopFrameSchema,
  teleopReleaseFrameSchema,
  healthQueryFrameSchema,
]);

export type InboundLinkFrame = z.infer<typeof inboundLinkFrameSchema>;

export interface HealthResultFrame {
  type: "health.result";
  id: string;
  rclOk: boolean;
  navigationActive: boolean;
  /** This robot's real velocity envelope, so clients can scale their full joystick travel onto it instead of assuming a value. */
  maxLinear: number;
  maxAngular: number;
}

/**
 * Phase 3 streaming frames — mirrors robotLink.protocol.ts's zod schemas on
 * PlatformServer's side field-for-field (contract by convention, same as
 * everything else in this file). Pushed unconditionally on a timer by
 * telemetry.broadcaster.ts, regardless of whether any dashboard viewer is
 * currently watching — PlatformServer's telemetry-relay fans each one out
 * to whoever's subscribed, it never asks this side to start/stop.
 *
 * map.delta (a changed-cells/bounding-box patch, cheaper than a full
 * snapshot every tick) is deliberately not implemented yet — map.snapshot
 * alone is correct, just not yet bandwidth-optimized for a long SLAM
 * session. Noted as a follow-up, not a gap in this pass.
 */
export interface PoseUpdateFrame {
  type: "pose.update";
  x: number;
  y: number;
  theta: number;
  source: "amcl" | "slam_bridge";
  confident: boolean;
}

export interface MapSnapshotFrame {
  type: "map.snapshot";
  info: {
    width: number;
    height: number;
    resolution: number;
    origin: {
      position: { x: number; y: number; z: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
  };
  data: number[];
}

export interface NavStatusLinkFrame {
  type: "nav.status";
  status: "IDLE" | "ACTIVE" | "SUCCEEDED" | "CANCELED" | "FAILED";
  feedbackDistanceRemaining: number | null;
  activeGoalName: string | null;
}

export interface RobotStatusFrame {
  type: "robot.status";
  rclOk: boolean;
  navigationActive: boolean;
  mappingActive: boolean;
  activeBackend: "cartographer" | "slam_toolbox" | null;
  maxLinear: number;
  maxAngular: number;
  estopEngaged: boolean;
}

export type TelemetryStreamFrame = PoseUpdateFrame | MapSnapshotFrame | NavStatusLinkFrame | RobotStatusFrame;
