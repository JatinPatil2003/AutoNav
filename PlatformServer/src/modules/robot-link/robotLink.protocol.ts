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

/** Frames RobotServer sends to PlatformServer over the link. */
export const robotLinkInboundFrameSchema = z.discriminatedUnion("type", [healthResultFrameSchema]);

export type RobotLinkInboundFrame = z.infer<typeof robotLinkInboundFrameSchema>;
