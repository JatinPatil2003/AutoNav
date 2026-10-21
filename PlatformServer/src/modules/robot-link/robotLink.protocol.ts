import { z } from "zod";

/**
 * Frame shapes crossing the persistent `/ws/robot-link` connection between
 * RobotServer and PlatformServer. Split by direction: PlatformServer only
 * ever sends the teleop/health.query frames (it is the relay, never the
 * driver), RobotServer only ever sends health.result back. Shared 1:1 with
 * RobotServer's `teleopLink.handler.ts` (Part 3 of the plan) — the field
 * names and literal `type` strings here are the actual wire contract.
 */

export const velocityDataSchema = z.object({
  linear: z.number().finite(),
  angular: z.number().finite(),
});

export const teleopVelocityFrameSchema = z.object({
  type: z.literal("teleop.velocity"),
  sessionId: z.string().min(1),
  data: velocityDataSchema,
});

export const teleopEstopFrameSchema = z.object({
  type: z.literal("teleop.estop"),
  sessionId: z.string().min(1),
  engaged: z.boolean(),
});

export const teleopReleaseFrameSchema = z.object({
  type: z.literal("teleop.release"),
  sessionId: z.string().min(1),
});

export const healthQueryFrameSchema = z.object({
  type: z.literal("health.query"),
  id: z.string().min(1),
});

/** Frames PlatformServer sends to RobotServer over the link. */
export const robotLinkOutboundFrameSchema = z.discriminatedUnion("type", [
  teleopVelocityFrameSchema,
  teleopEstopFrameSchema,
  teleopReleaseFrameSchema,
  healthQueryFrameSchema,
]);

export type RobotLinkOutboundFrame = z.infer<typeof robotLinkOutboundFrameSchema>;

export const healthResultFrameSchema = z.object({
  type: z.literal("health.result"),
  id: z.string().min(1),
  rclOk: z.boolean(),
  navigationActive: z.boolean(),
  // Optional for rollout safety — an older RobotServer that hasn't picked
  // up this field yet still produces a valid frame, just without limits.
  maxLinear: z.number().optional(),
  maxAngular: z.number().optional(),
});

export type HealthResult = Pick<
  z.infer<typeof healthResultFrameSchema>,
  "rclOk" | "navigationActive" | "maxLinear" | "maxAngular"
>;

/**
 * Phase 3 streaming frames — RobotServer pushes these unconditionally
 * (same as its own local 20Hz telemetry.broadcaster.ts loop), regardless
 * of whether any dashboard viewer is currently watching. PlatformServer's
 * telemetry-relay module fans each one out to every viewer subscribed to
 * that robot; it never asks the robot to start/stop streaming.
 */
export const poseUpdateFrameSchema = z.object({
  type: z.literal("pose.update"),
  x: z.number(),
  y: z.number(),
  theta: z.number(),
  source: z.enum(["amcl", "slam_bridge"]),
  confident: z.boolean(),
});

export const occupancyGridInfoSchema = z.object({
  width: z.number(),
  height: z.number(),
  resolution: z.number(),
  origin: z.object({
    position: z.object({ x: z.number(), y: z.number(), z: z.number() }),
    orientation: z.object({ x: z.number(), y: z.number(), z: z.number(), w: z.number() }),
  }),
});

export const mapSnapshotFrameSchema = z.object({
  type: z.literal("map.snapshot"),
  info: occupancyGridInfoSchema,
  data: z.array(z.number()),
});

export const navStatusLinkFrameSchema = z.object({
  type: z.literal("nav.status"),
  status: z.enum(["IDLE", "ACTIVE", "SUCCEEDED", "CANCELED", "FAILED"]),
  feedbackDistanceRemaining: z.number().nullable(),
  activeGoalName: z.string().nullable(),
});

export const robotStatusFrameSchema = z.object({
  type: z.literal("robot.status"),
  rclOk: z.boolean(),
  navigationActive: z.boolean(),
  mappingActive: z.boolean(),
  activeBackend: z.enum(["cartographer", "slam_toolbox"]).nullable(),
  maxLinear: z.number(),
  maxAngular: z.number(),
  estopEngaged: z.boolean(),
});

/** Frames RobotServer sends to PlatformServer over the link. */
export const robotLinkInboundFrameSchema = z.discriminatedUnion("type", [
  healthResultFrameSchema,
  poseUpdateFrameSchema,
  mapSnapshotFrameSchema,
  navStatusLinkFrameSchema,
  robotStatusFrameSchema,
]);

export type RobotLinkInboundFrame = z.infer<typeof robotLinkInboundFrameSchema>;
export type PoseUpdateFrame = z.infer<typeof poseUpdateFrameSchema>;
export type MapSnapshotFrame = z.infer<typeof mapSnapshotFrameSchema>;
export type NavStatusLinkFrame = z.infer<typeof navStatusLinkFrameSchema>;
export type RobotStatusFrame = z.infer<typeof robotStatusFrameSchema>;
export type TelemetryStreamFrame = PoseUpdateFrame | MapSnapshotFrame | NavStatusLinkFrame | RobotStatusFrame;
