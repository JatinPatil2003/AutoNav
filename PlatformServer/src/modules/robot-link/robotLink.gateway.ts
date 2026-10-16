import crypto from "node:crypto";
import { IncomingMessage } from "node:http";
import { WebSocket } from "ws";
import { logger } from "../../common/utils/logger.util";
import { WsAuthenticator, WsConnectionHandler } from "../../core/wsRouter";
import { IRobotDocument } from "../robots/robot.model";
import { IRobotRepository } from "../robots/robot.repository.interface";
import { IRobotConnectionRegistry } from "./robotConnectionRegistry.interface";
import { robotLinkInboundFrameSchema } from "./robotLink.protocol";

interface RobotLinkAuthContext {
  robot: IRobotDocument;
}

/**
 * The robot-link endpoint: a persistent, server-authenticated connection
 * each RobotServer dials and holds open. Auth is a shared secret (not a
 * JWT — RobotServer has no user session), compared with
 * `crypto.timingSafeEqual` against the stored sha256 hash so a timing
 * side-channel can't leak the secret byte-by-byte. Mounted only on the
 * internal HTTP listener — see src/index.ts — never through the public
 * nginx vhost.
 */
export class RobotLinkGateway {
  constructor(
    private readonly robotRepository: IRobotRepository,
    private readonly registry: IRobotConnectionRegistry,
  ) {}

  authenticate: WsAuthenticator<RobotLinkAuthContext> = async (request: IncomingMessage) => {
    const robotId = request.headers["x-robot-id"];
    const authHeader = request.headers["authorization"];

    if (typeof robotId !== "string" || !robotId.trim() || typeof authHeader !== "string") {
      return undefined;
    }

    const match = authHeader.match(/^Robot (.+)$/);
    if (!match) return undefined;
    const presentedSecret = match[1];

    const robot = await this.robotRepository.findByRobotIdWithLinkSecret(robotId);
    if (!robot || !robot.linkSecretHash || robot.status === "retired") {
      return undefined;
    }

    const presentedHash = crypto.createHash("sha256").update(presentedSecret).digest();
    const storedHash = Buffer.from(robot.linkSecretHash, "hex");
    if (presentedHash.length !== storedHash.length || !crypto.timingSafeEqual(presentedHash, storedHash)) {
      return undefined;
    }

    return { robot };
  };

  onConnection: WsConnectionHandler = (ws: WebSocket, request: IncomingMessage) => {
    const context = request.wsAuthContext as RobotLinkAuthContext | undefined;
    if (!context) {
      // Unreachable in practice — wsRouter rejects the upgrade before this
      // handler runs when authenticate() returns undefined — but fail
      // closed rather than operate on an unauthenticated socket.
      ws.close(1011, "missing auth context");
      return;
    }

    const { robotId } = context.robot;
    logger.info({ robotId }, "RobotLinkGateway: robot connected");

    this.registry.register(robotId, ws);

    ws.on("message", (raw) => {
      let parsed: unknown;
      try {
        parsed = JSON.parse(raw.toString());
      } catch {
        logger.warn({ robotId }, "RobotLinkGateway: received non-JSON frame");
        return;
      }

      const result = robotLinkInboundFrameSchema.safeParse(parsed);
      if (!result.success) {
        logger.warn({ robotId, issues: result.error.issues }, "RobotLinkGateway: invalid inbound frame");
        return;
      }

      const frame = result.data;
      if (frame.type === "health.result") {
        this.registry.resolveHealthResult(robotId, frame.id, {
          rclOk: frame.rclOk,
          navigationActive: frame.navigationActive,
          maxLinear: frame.maxLinear,
          maxAngular: frame.maxAngular,
        });
      }
    });

    ws.on("close", () => {
      logger.info({ robotId }, "RobotLinkGateway: robot disconnected");
      this.registry.unregister(robotId, ws);
    });

    ws.on("error", (err) => {
      logger.warn({ err, robotId }, "RobotLinkGateway: socket error");
    });
  };
}
