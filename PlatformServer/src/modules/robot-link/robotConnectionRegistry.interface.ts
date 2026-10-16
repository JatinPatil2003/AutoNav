import { WebSocket } from "ws";
import { HealthResult, RobotLinkOutboundFrame } from "./robotLink.protocol";

export interface IRobotConnectionRegistry {
  /** Registers a newly-authenticated robot-link socket, closing out any older connection for the same robotId (code 4409). */
  register(robotId: string, socket: WebSocket): void;
  unregister(robotId: string, socket: WebSocket): void;
  get(robotId: string): WebSocket | undefined;
  isOnline(robotId: string): boolean;
  getRttMs(robotId: string): number | undefined;
  getConnectedSince(robotId: string): Date | undefined;
  /** Validates and sends a frame to the robot's current connection. Returns false if offline or the socket isn't open. */
  send(robotId: string, frame: RobotLinkOutboundFrame): boolean;
  /** One-shot round trip over the link; resolves null on timeout or if the robot is offline. */
  queryHealth(robotId: string, timeoutMs: number): Promise<HealthResult | null>;
  /** Fires with the robotId whenever a connection is torn down (close, replaced, or watchdog timeout). */
  on(event: "disconnect", listener: (robotId: string) => void): void;
  /** Routes an inbound `health.result` frame to whichever queryHealth() call is awaiting that id, if any. */
  resolveHealthResult(robotId: string, id: string, result: HealthResult): void;
}
