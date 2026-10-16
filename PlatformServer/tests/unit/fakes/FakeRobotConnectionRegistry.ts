import { EventEmitter } from "node:events";
import { WebSocket } from "ws";
import { IRobotConnectionRegistry } from "../../../src/modules/robot-link/robotConnectionRegistry.interface";
import { RobotLinkOutboundFrame } from "../../../src/modules/robot-link/robotLink.protocol";

/**
 * In-memory stand-in for RobotConnectionRegistry — records every frame
 * forwarded to each robot so tests can assert on exactly what
 * ControlSessionService chose to send, without a real WebSocket or robot
 * on the other end. `disconnect(robotSlug)`/`setOnline` let tests drive
 * the registry's view of link state directly.
 */
export class FakeRobotConnectionRegistry extends EventEmitter implements IRobotConnectionRegistry {
  public readonly sentFrames: Array<{ robotId: string; frame: RobotLinkOutboundFrame }> = [];
  private online = new Set<string>();
  private healthResults = new Map<string, { rclOk: boolean; navigationActive: boolean } | null>();

  register(_robotId: string, _socket: WebSocket): void {
    // Not exercised by ControlSessionService tests — robot-link connections
    // register themselves directly against the real registry.
  }

  unregister(_robotId: string, _socket: WebSocket): void {
    // See register().
  }

  get(_robotId: string): WebSocket | undefined {
    return undefined;
  }

  setOnline(robotId: string, online: boolean): void {
    if (online) this.online.add(robotId);
    else this.online.delete(robotId);
  }

  isOnline(robotId: string): boolean {
    return this.online.has(robotId);
  }

  getRttMs(_robotId: string): number | undefined {
    return undefined;
  }

  getConnectedSince(_robotId: string): Date | undefined {
    return undefined;
  }

  send(robotId: string, frame: RobotLinkOutboundFrame): boolean {
    if (!this.online.has(robotId)) return false;
    this.sentFrames.push({ robotId, frame });
    return true;
  }

  setHealthResult(robotId: string, result: { rclOk: boolean; navigationActive: boolean } | null): void {
    this.healthResults.set(robotId, result);
  }

  async queryHealth(
    robotId: string,
    _timeoutMs: number,
  ): Promise<{ rclOk: boolean; navigationActive: boolean } | null> {
    if (!this.online.has(robotId)) return null;
    return this.healthResults.get(robotId) ?? null;
  }

  resolveHealthResult(
    _robotId: string,
    _id: string,
    _result: { rclOk: boolean; navigationActive: boolean },
  ): void {
    // Not exercised here — queryHealth() above is driven directly via setHealthResult.
  }

  /** Test helper: simulates the robot-link connection for this robot dropping. */
  simulateDisconnect(robotId: string): void {
    this.online.delete(robotId);
    this.emit("disconnect", robotId);
  }
}
