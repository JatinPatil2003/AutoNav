import { z } from "zod";

/**
 * Client-side mirror of PlatformServer's
 * src/modules/control/controlSession.protocol.ts — the exact frame shapes
 * crossing `/ws/control`. Outbound types are kept as plain TS interfaces
 * (we construct them, no need to validate our own output); inbound frames
 * are validated with zod as defense in depth against a malformed/garbled
 * message, mirroring the same posture the server takes on the other leg.
 */

export interface ClientVelocityData {
  linear: number;
  angular: number;
}

export interface JoystickClientFrame {
  type: "joystick";
  seq: number;
  data: ClientVelocityData;
}

export interface EstopClientFrame {
  type: "estop";
  data: { engaged: boolean };
}

export type ClientFrame = JoystickClientFrame | EstopClientFrame;

export type SessionState = "ready" | "robot_offline" | "preempted" | "revoked";

const sessionServerFrameSchema = z.object({
  type: z.literal("session"),
  state: z.enum(["ready", "robot_offline", "preempted", "revoked"]),
});

const estopAckServerFrameSchema = z.object({
  type: z.literal("estop_ack"),
  engaged: z.boolean(),
});

const errorServerFrameSchema = z.object({
  type: z.literal("error"),
  code: z.string(),
  message: z.string(),
});

export const serverFrameSchema = z.discriminatedUnion("type", [
  sessionServerFrameSchema,
  estopAckServerFrameSchema,
  errorServerFrameSchema,
]);

export type ServerFrame = z.infer<typeof serverFrameSchema>;
