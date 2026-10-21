import { IncomingMessage } from "node:http";
import { WebSocket } from "ws";
import { extractAccessToken } from "../../common/middleware/auth.middleware";
import { parseCookieHeader } from "../../common/utils/cookie.util";
import { verifyAccessToken } from "../../common/utils/jwt.util";
import { logger } from "../../common/utils/logger.util";
import { WsAuthenticator, WsConnectionHandler } from "../../core/wsRouter";
import { IRobotRepository } from "../robots/robot.repository.interface";
import { RobotAssignmentService } from "../robot-assignments/robotAssignment.service";
import { TelemetryRelayService } from "./telemetryRelay.service";

interface TelemetryAuthContext {
  robotMongoId: string;
  robotSlug: string;
}

/**
 * The dashboard-facing `/ws/telemetry` endpoint — read-only broadcast of
 * pose/map/nav-status/robot-status for one robot. Auth is identical to
 * ControlSessionGateway's (`?robotId=<mongo id>` + bearer/cookie, active
 * platformAccess assignment required) since watching a robot's telemetry
 * is gated by the same permission as driving it; unlike /ws/control,
 * arbitrarily many viewers may hold this open at once — see
 * TelemetryRelayService's fan-out model.
 */
export class TelemetryRelayGateway {
  constructor(
    private readonly robotRepository: IRobotRepository,
    private readonly assignmentService: RobotAssignmentService,
    private readonly relayService: TelemetryRelayService,
  ) {}

  authenticate: WsAuthenticator<TelemetryAuthContext> = async (request: IncomingMessage) => {
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

    return { robotMongoId, robotSlug: robot.robotId };
  };

  onConnection: WsConnectionHandler = (ws: WebSocket, request: IncomingMessage) => {
    const context = request.wsAuthContext as TelemetryAuthContext | undefined;
    if (!context) {
      ws.close(1011, "missing auth context");
      return;
    }

    logger.info({ robotSlug: context.robotSlug }, "TelemetryRelayGateway: viewer connected");
    this.relayService.subscribe(context.robotMongoId, context.robotSlug, ws);
  };
}
