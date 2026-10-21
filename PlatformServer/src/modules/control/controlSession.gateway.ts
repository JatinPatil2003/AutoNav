import { IncomingMessage } from "node:http";
import { WebSocket } from "ws";
import { extractAccessToken } from "../../common/middleware/auth.middleware";
import { parseCookieHeader } from "../../common/utils/cookie.util";
import { verifyAccessToken } from "../../common/utils/jwt.util";
import { logger } from "../../common/utils/logger.util";
import { WsAuthenticator, WsConnectionHandler } from "../../core/wsRouter";
import { IRobotRepository } from "../robots/robot.repository.interface";
import { RobotAssignmentService } from "../robot-assignments/robotAssignment.service";
import { ControlSessionService } from "./controlSession.service";

interface ControlAuthContext {
  userId: string;
  robotMongoId: string;
  robotSlug: string;
}

/**
 * The client-facing `/ws/control` endpoint. The robot is bound at
 * handshake, not per-frame: `?robotId=<mongo id>` plus a bearer token (or
 * cookie, for parity with the web dashboard) are both required, and the
 * authenticator confirms an active `platformAccess` assignment for that
 * exact (user, robot) pair before the upgrade is allowed to complete — so
 * there is never a connected-but-unbound socket to special-case.
 */
export class ControlSessionGateway {
  constructor(
    private readonly robotRepository: IRobotRepository,
    private readonly assignmentService: RobotAssignmentService,
    private readonly sessionService: ControlSessionService,
  ) {}

  authenticate: WsAuthenticator<ControlAuthContext> = async (request: IncomingMessage) => {
    const url = new URL(request.url ?? "", "http://localhost");
    const robotMongoId = url.searchParams.get("robotId");
    if (!robotMongoId) return undefined;

    const requestWithCookies = request as unknown as Parameters<typeof extractAccessToken>[0];
    requestWithCookies.cookies = parseCookieHeader(request.headers.cookie);

    const token = extractAccessToken(requestWithCookies);
    if (!token) return undefined;

    let userId: string;
    try {
      userId = verifyAccessToken(token).sub;
    } catch {
      return undefined;
    }

    const [robot, hasAccess] = await Promise.all([
      this.robotRepository.findById(robotMongoId),
      this.assignmentService.hasActivePlatformAccess(userId, robotMongoId),
    ]);
    if (!robot || robot.status === "retired" || !hasAccess) return undefined;

    return { userId, robotMongoId, robotSlug: robot.robotId };
  };

  onConnection: WsConnectionHandler = (ws: WebSocket, request: IncomingMessage) => {
    const context = request.wsAuthContext as ControlAuthContext | undefined;
    if (!context) {
      ws.close(1011, "missing auth context");
      return;
    }

    logger.info(
      { userId: context.userId, robotSlug: context.robotSlug },
      "ControlSessionGateway: client connected",
    );

    this.sessionService.startSession({
      userId: context.userId,
      robotMongoId: context.robotMongoId,
      robotSlug: context.robotSlug,
      ws,
    });
  };
}
