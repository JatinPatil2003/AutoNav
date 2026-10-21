import { TelemetryStreamFrame } from "../robot-link/robotLink.protocol";

/**
 * Frames PlatformServer sends to a dashboard viewer on `/ws/telemetry`.
 * The stream frames themselves are forwarded byte-for-byte from whatever
 * the robot sent over `/ws/robot-link` (TelemetryRelayService never
 * inspects/transforms their contents — see its class doc comment), plus
 * one connection-state frame this relay adds itself so a viewer can tell
 * "no data yet because nothing's changed" apart from "no data because the
 * robot is offline".
 */
export interface RobotLinkStateFrame {
  type: "robot_link_state";
  online: boolean;
}

export type TelemetryRelayFrame = TelemetryStreamFrame | RobotLinkStateFrame;
