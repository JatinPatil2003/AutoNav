import { z } from "zod";

/**
 * Frame shapes crossing `/ws/control` between the Android/web client and
 * PlatformServer. Kept deliberately close to the legacy
 * `PlatformApp/src/legacy-robot-ui` shape (`{"type":"joystick","data":{...}}`)
 * for continuity, with `seq` added so dropped/out-of-order frames are
 * detectable client-side if ever needed.
 */

export const clientVelocityDataSchema = z.object({
  linear: z.number().finite(),
  angular: z.number().finite(),
});

export const joystickFrameSchema = z.object({
  type: z.literal("joystick"),
  seq: z.number().int().nonnegative(),
  data: clientVelocityDataSchema,
});

export const estopFrameSchema = z.object({
  type: z.literal("estop"),
  data: z.object({ engaged: z.boolean() }),
});

/** Frames the client sends to PlatformServer. */
export const clientFrameSchema = z.discriminatedUnion("type", [joystickFrameSchema, estopFrameSchema]);

export type ClientFrame = z.infer<typeof clientFrameSchema>;

export type SessionState = "ready" | "robot_offline" | "preempted" | "revoked";

export interface SessionServerFrame {
  type: "session";
  state: SessionState;
}

export interface EstopAckServerFrame {
  type: "estop_ack";
  engaged: boolean;
}

export interface ErrorServerFrame {
  type: "error";
  code: string;
  message: string;
}

/** Frames PlatformServer sends back to the client. */
export type ServerFrame = SessionServerFrame | EstopAckServerFrame | ErrorServerFrame;
