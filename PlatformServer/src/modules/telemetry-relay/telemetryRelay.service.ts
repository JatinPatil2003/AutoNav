import { WebSocket } from "ws";
import { logger } from "../../common/utils/logger.util";
import { IRobotConnectionRegistry } from "../robot-link/robotConnectionRegistry.interface";
import { IRobotRepository } from "../robots/robot.repository.interface";
import { TelemetryStreamFrame } from "../robot-link/robotLink.protocol";
import { TelemetryRelayFrame } from "./telemetryRelay.protocol";

function send(ws: WebSocket, frame: TelemetryRelayFrame): void {
  if (ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(frame));
  }
}

/**
 * The client<->PlatformServer leg of Phase 3's streaming telemetry (pose/
 * map/nav-status/robot-status). Unlike ControlSessionService, this is
 * fan-out, not exclusive ownership: any number of dashboard viewers may
 * watch the same robot at once, so the invariant here is "every open
 * socket subscribed to a robot gets every frame for that robot" rather
 * than "one session per robot, newer wins". A frame coming in from the
 * robot-link is forwarded byte-for-byte to every viewer of that robot —
 * this class never inspects or transforms its contents, keeping
 * occupancy-grid/pose semantics entirely in RobotServer/PlatformApp,
 * exactly as ControlSessionGateway's forwardClientFrame already does for
 * teleop payloads.
 */
export class TelemetryRelayService {
  /** Keyed by robotMongoId — the dashboard-facing id, same key ControlSessionService uses. */
  private readonly viewersByRobot = new Map<string, Set<WebSocket>>();

  constructor(
    private readonly registry: IRobotConnectionRegistry,
    private readonly robotRepository: IRobotRepository,
  ) {
    // The registry's disconnect event is keyed by robotSlug (the device-level
    // id robot-link/robotConnectionRegistry use), not robotMongoId (the key
    // this fan-out and the dashboard use) — resolved here once per
    // disconnect rather than making every caller do the lookup.
    this.registry.on("disconnect", (robotSlug) => void this.handleRobotDisconnect(robotSlug));
  }

  /** robotSlug is passed in (from the gateway's already-resolved auth context) purely to report accurate online/offline state in the initial frame — subscribe() never needs to look it up itself. */
  subscribe(robotMongoId: string, robotSlug: string, ws: WebSocket): void {
    let viewers = this.viewersByRobot.get(robotMongoId);
    if (!viewers) {
      viewers = new Set();
      this.viewersByRobot.set(robotMongoId, viewers);
    }
    viewers.add(ws);

    ws.on("close", () => this.unsubscribe(robotMongoId, ws));
    ws.on("error", (err) => {
      logger.warn({ err, robotMongoId }, "TelemetryRelayService: viewer socket error");
    });

    send(ws, { type: "robot_link_state", online: this.registry.isOnline(robotSlug) });
  }

  private unsubscribe(robotMongoId: string, ws: WebSocket): void {
    const viewers = this.viewersByRobot.get(robotMongoId);
    if (!viewers) return;
    viewers.delete(ws);
    if (viewers.size === 0) {
      this.viewersByRobot.delete(robotMongoId);
    }
  }

  /** Called by RobotLinkGateway for every inbound streaming frame — forwarded as-is to every current viewer of this robot. */
  handleRobotFrame(robotMongoId: string, frame: TelemetryStreamFrame): void {
    const viewers = this.viewersByRobot.get(robotMongoId);
    if (!viewers || viewers.size === 0) return;
    for (const ws of viewers) {
      send(ws, frame);
    }
  }

  private async handleRobotDisconnect(robotSlug: string): Promise<void> {
    const robot = await this.robotRepository.findByRobotId(robotSlug).catch((err) => {
      logger.warn({ err, robotSlug }, "TelemetryRelayService: failed to resolve robotSlug on disconnect");
      return null;
    });
    if (!robot) return;

    const viewers = this.viewersByRobot.get(robot._id.toString());
    if (!viewers) return;
    for (const ws of viewers) {
      send(ws, { type: "robot_link_state", online: false });
    }
  }
}
