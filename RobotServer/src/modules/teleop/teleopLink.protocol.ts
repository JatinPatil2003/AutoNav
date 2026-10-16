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
