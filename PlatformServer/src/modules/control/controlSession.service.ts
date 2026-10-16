import { randomUUID } from "node:crypto";
import { WebSocket } from "ws";
import { logger } from "../../common/utils/logger.util";
import { IRobotConnectionRegistry } from "../robot-link/robotConnectionRegistry.interface";
import { RobotAssignmentService } from "../robot-assignments/robotAssignment.service";
import { ClientFrame, clientFrameSchema, ServerFrame, SessionState } from "./controlSession.protocol";

const COALESCE_INTERVAL_MS = Math.floor(1000 / 30); // forward to the robot at most 30x/s
const ASSIGNMENT_REVALIDATE_INTERVAL_MS = 30_000;

export interface StartSessionParams {
  userId: string;
  /** Mongo _id of the robot — the same id RobotAssignmentService's query methods use. */
  robotMongoId: string;
  /** Device-level robotId slug — the key robotConnectionRegistry and the robot-link protocol use. */
  robotSlug: string;
  ws: WebSocket;
}

interface Session {
  sessionId: string;
  userId: string;
  robotMongoId: string;
  robotSlug: string;
  ws: WebSocket;
  pendingVelocity?: { linear: number; angular: number };
  coalesceTimer: ReturnType<typeof setInterval>;
  revalidateTimer: ReturnType<typeof setInterval>;
}

type TerminationReason = "client_disconnect" | "preempted" | "robot_offline" | "revoked";

function send(ws: WebSocket, frame: ServerFrame): void {
  if (ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(frame));
  }
}

/**
 * Owns the client<->PlatformServer leg of the teleop relay: exactly one
 * active session per robot (a newer session preempts the older one),
 * inbound frames validated/clamped/coalesced before being forwarded over
 * the robot-link connection, and the underlying assignment re-checked
 * periodically so a mid-session revocation doesn't leave a session
 * running on stale authorization.
 */
export class ControlSessionService {
  /** Keyed by robotMongoId — the invariant this enforces is "one session per robot". */
  private readonly sessionsByRobot = new Map<string, Session>();

  constructor(
    private readonly registry: IRobotConnectionRegistry,
    private readonly assignmentService: RobotAssignmentService,
  ) {
    this.registry.on("disconnect", (robotSlug) => this.handleRobotDisconnect(robotSlug));
  }

  startSession(params: StartSessionParams): void {
    const existing = this.sessionsByRobot.get(params.robotMongoId);
    if (existing) {
      this.terminateSession(existing, "preempted");
    }

    const sessionId = randomUUID();
    // Safe to reference inside the timers below despite being defined in the same
    // statement — they're async callbacks that won't run until after this completes.
    const session: Session = {
      sessionId,
      userId: params.userId,
      robotMongoId: params.robotMongoId,
      robotSlug: params.robotSlug,
      ws: params.ws,
      coalesceTimer: setInterval(() => this.flushPendingVelocity(session), COALESCE_INTERVAL_MS),
      revalidateTimer: setInterval(
        () => void this.revalidateAssignment(session),
        ASSIGNMENT_REVALIDATE_INTERVAL_MS,
      ),
    };

    this.sessionsByRobot.set(params.robotMongoId, session);

    params.ws.on("message", (raw) => this.handleClientMessage(session, raw));
    params.ws.on("close", () => this.handleClientDisconnect(session));
    params.ws.on("error", (err) => {
      logger.warn({ err, sessionId }, "ControlSessionService: client socket error");
    });

    send(params.ws, { type: "session", state: "ready" });
  }

  private handleClientMessage(session: Session, raw: unknown): void {
    let parsed: unknown;
    try {
      parsed = JSON.parse((raw as Buffer).toString());
    } catch {
      send(session.ws, { type: "error", code: "bad_frame", message: "Frame was not valid JSON" });
      return;
    }

    const result = clientFrameSchema.safeParse(parsed);
    if (!result.success) {
      send(session.ws, { type: "error", code: "bad_frame", message: "Frame failed validation" });
      return;
    }

    this.forwardClientFrame(session, result.data);
  }

  private forwardClientFrame(session: Session, frame: ClientFrame): void {
    if (frame.type === "joystick") {
      // RobotServer is the single source of truth for velocity limits
      // (TeleopService clamps every setVelocity call against its own
      // real maxLinear/maxAngular) — clamping again here against a
      // static PlatformServer env constant would just reintroduce a
      // second, independently-configured ceiling that silently caps
      // commands below whatever the robot was actually raised to.
      session.pendingVelocity = { linear: frame.data.linear, angular: frame.data.angular };
      return;
    }

    // estop is safety-critical — forwarded immediately, never coalesced.
    logger.info(
      { sessionId: session.sessionId, robotSlug: session.robotSlug, engaged: frame.data.engaged },
      "ControlSessionService: estop",
    );
    const delivered = this.registry.send(session.robotSlug, {
      type: "teleop.estop",
      sessionId: session.sessionId,
      engaged: frame.data.engaged,
    });
    if (!delivered) {
      send(session.ws, { type: "error", code: "robot_offline", message: "Robot link is offline" });
      return;
    }
    send(session.ws, { type: "estop_ack", engaged: frame.data.engaged });
  }

  private flushPendingVelocity(session: Session): void {
    if (!session.pendingVelocity) return;
    const data = session.pendingVelocity;
    session.pendingVelocity = undefined;
    logger.debug(
      { sessionId: session.sessionId, robotSlug: session.robotSlug, data },
      "ControlSessionService: velocity",
    );
    this.registry.send(session.robotSlug, {
      type: "teleop.velocity",
      sessionId: session.sessionId,
      data,
    });
  }

  private async revalidateAssignment(session: Session): Promise<void> {
    const stillAllowed = await this.assignmentService
      .hasActivePlatformAccess(session.userId, session.robotMongoId)
      .catch((err) => {
        logger.warn({ err, sessionId: session.sessionId }, "ControlSessionService: revalidation check failed");
        return true; // fail open on a transient DB error — don't kick a live session over a blip
      });

    if (!stillAllowed && this.sessionsByRobot.get(session.robotMongoId) === session) {
      this.terminateSession(session, "revoked");
    }
  }

  private handleClientDisconnect(session: Session): void {
    this.terminateSession(session, "client_disconnect");
  }

  private handleRobotDisconnect(robotSlug: string): void {
    for (const session of this.sessionsByRobot.values()) {
      if (session.robotSlug === robotSlug) {
        this.terminateSession(session, "robot_offline");
      }
    }
  }

  /** Centralizes every way a session ends — always releases the robot-side motion and cleans up timers exactly once. */
  private terminateSession(session: Session, reason: TerminationReason): void {
    // Only the map's current owner for this robot may clear it — an already-preempted
    // session terminating late (e.g. a delayed close event) must not clobber the new one.
    if (this.sessionsByRobot.get(session.robotMongoId) === session) {
      this.sessionsByRobot.delete(session.robotMongoId);
    }

    clearInterval(session.coalesceTimer);
    clearInterval(session.revalidateTimer);

    if (reason !== "robot_offline") {
      this.registry.send(session.robotSlug, { type: "teleop.release", sessionId: session.sessionId });
    }

    if (reason !== "client_disconnect") {
      const state: SessionState = reason;
      send(session.ws, { type: "session", state });
      session.ws.close();
    }
  }
}
