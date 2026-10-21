import cookieParser from "cookie-parser";
import cors from "cors";
import express, { Express } from "express";
import helmet from "helmet";
import pinoHttp from "pino-http";
import swaggerUi from "swagger-ui-express";

import { config } from "./config/env";
import "./config/passport";
import { swaggerSpec } from "./config/swagger";
import { logger } from "./common/utils/logger.util";
import { errorHandler } from "./common/middleware/errorHandler.middleware";
import { notFoundHandler } from "./common/middleware/notFound.middleware";

import { MongooseUserRepository } from "./modules/users/user.repository";
import { UserService } from "./modules/users/user.service";
import { UserController } from "./modules/users/user.controller";
import { createUserRouter } from "./modules/users/user.routes";

import { MongooseRobotRepository } from "./modules/robots/robot.repository";
import { RobotService } from "./modules/robots/robot.service";
import { RobotController } from "./modules/robots/robot.controller";
import { createPublicRobotRouter, createRobotRouter } from "./modules/robots/robot.routes";

import { MongooseRobotAssignmentRepository } from "./modules/robot-assignments/robotAssignment.repository";
import { RobotAssignmentService } from "./modules/robot-assignments/robotAssignment.service";
import { HttpRobotNotifier } from "./modules/robot-assignments/HttpRobotNotifier";

import { MongooseRobotAccessRequestRepository } from "./modules/robot-access-requests/robotAccessRequest.repository";
import { RobotAccessRequestService } from "./modules/robot-access-requests/robotAccessRequest.service";
import { RobotAccessRequestController } from "./modules/robot-access-requests/robotAccessRequest.controller";
import { createRobotAccessRequestRouter } from "./modules/robot-access-requests/robotAccessRequest.routes";

import { AuthService } from "./modules/auth/auth.service";
import { AuthController } from "./modules/auth/auth.controller";
import { createAuthRouter } from "./modules/auth/auth.routes";
import { MongooseEmailOtpRepository } from "./modules/auth/emailOtp.repository";
import { NodemailerEmailOtpSender } from "./modules/auth/senders/NodemailerEmailOtpSender";

import { AdminService } from "./modules/admin/admin.service";
import { AdminController } from "./modules/admin/admin.controller";
import { createAdminRouter } from "./modules/admin/admin.routes";

import { RobotAuthService } from "./modules/robot-auth/robotAuth.service";
import { RobotAuthController } from "./modules/robot-auth/robotAuth.controller";
import { createRobotAuthRouter } from "./modules/robot-auth/robotAuth.routes";

import { RobotHealthService } from "./modules/control/robotHealth.service";
import { RobotHealthController } from "./modules/control/robotHealth.controller";
import { createRobotHealthRouter } from "./modules/control/robotHealth.routes";
import { ControlSessionService } from "./modules/control/controlSession.service";
import { ControlSessionGateway } from "./modules/control/controlSession.gateway";

import { RobotConnectionRegistry } from "./modules/robot-link/robotConnectionRegistry";
import { RobotLinkGateway } from "./modules/robot-link/robotLink.gateway";

import { TelemetryRelayService } from "./modules/telemetry-relay/telemetryRelay.service";
import { TelemetryRelayGateway } from "./modules/telemetry-relay/telemetryRelay.gateway";

import { HttpRobotApiClient } from "./modules/robot-proxy/robotApiClient";
import { RobotProxyService } from "./modules/robot-proxy/robotProxy.service";
import { RobotProxyController } from "./modules/robot-proxy/robotProxy.controller";
import { createRobotProxyRouter } from "./modules/robot-proxy/robotProxy.routes";

import { WsRouter } from "./core/wsRouter";

export interface CreatedApp {
  app: Express;
  /**
   * Attached to the internal HTTP listener only (see src/index.ts).
   * `/ws/robot-link`, `/ws/control`, and `/ws/telemetry` are all proxied
   * publicly by nginx under its one `/ws/` wildcard block (see
   * AWS/nginx/platform-server.autonav.in.conf) — any new `/ws/*` route is
   * public-by-default under that block. `/ws/robot-link` is guarded by
   * `ROBOT_LINK_SECRET` (robotLinkGateway.authenticate) rather than a VPN
   * trust boundary; `/ws/control` and `/ws/telemetry` both require an
   * active platformAccess assignment.
   */
  wsRouter: WsRouter;
}

/**
 * Composition root: repositories, services, and controllers are wired here
 * via constructor injection so every layer depends on an interface, not a
 * concrete persistence implementation — swapping MongooseUserRepository for
 * an in-memory fake (as the unit tests do) requires no change elsewhere.
 */
export function createApp(): CreatedApp {
  const app = express();

  // nginx proxies every request from loopback (see AWS/nginx/platform-server.autonav.in.conf),
  // setting X-Forwarded-For/-Proto. Trusting only loopback as a proxy hop
  // lets express-rate-limit and req.ip use the real client IP from that
  // header without opening IP spoofing to anyone who can reach this
  // process directly (there is no such direct public path).
  app.set("trust proxy", "loopback");

  app.use(helmet());
  app.use(express.json());
  app.use(cookieParser());

  // CORS is always needed, not just in dev: the web dashboard is same-origin
  // behind nginx in production (so technically wouldn't need this), but the
  // native mobile app (Capacitor WebView) is *always* cross-origin — it has
  // no origin that matches FRONTEND_URL in any environment. Its WebView
  // presents as `https://localhost` (release — Capacitor's default secure
  // scheme) or `http://localhost` (local dev builds against a plain-HTTP
  // dev backend, see AndroidApp/capacitor.config.ts's androidScheme switch),
  // or `capacitor://localhost` on some configs — never the real API host —
  // so all three are allow-listed explicitly alongside the configured web
  // origin rather than opening CORS to "*". AndroidApp's own Vite dev
  // server (`npm run dev`, a plain browser tab, not the WebView) runs on a
  // separate port from PlatformApp's — also allow-listed so the web-only
  // dev loop works without needing the full Capacitor build/install cycle.
  const allowedOrigins = new Set([
    config.frontendUrl,
    "https://localhost",
    "http://localhost",
    "http://localhost:5190",
    "capacitor://localhost",
  ]);
  app.use(
    cors({
      origin: (origin, callback) => {
        if (!origin || allowedOrigins.has(origin)) {
          callback(null, true);
        } else {
          callback(new Error("Not allowed by CORS"));
        }
      },
      credentials: true,
    }),
  );

  app.use(
    pinoHttp({
      logger,
      autoLogging: { ignore: (req) => req.url === "/health" },
      customProps: (req) => ({ userId: req.user?.id }),
      customSuccessMessage: (req, res) => `${req.method} ${req.url} -> ${res.statusCode}`,
      customErrorMessage: (req, res, err) =>
        `${req.method} ${req.url} -> ${res.statusCode} (${err.message})`,
      // pino-http's defaults attach the full req/res objects (headers and
      // all) to every log line — keep only what's actually useful for an
      // access log; the one-line customSuccessMessage above covers the rest.
      serializers: {
        req: (req) => ({ method: req.method, url: req.url }),
        res: (res) => ({ statusCode: res.statusCode }),
      },
    }),
  );

  // --- Repositories ---
  const userRepository = new MongooseUserRepository();
  const robotRepository = new MongooseRobotRepository();
  const robotAssignmentRepository = new MongooseRobotAssignmentRepository();
  const requestRepository = new MongooseRobotAccessRequestRepository();
  const otpRepository = new MongooseEmailOtpRepository();

  // --- Services ---
  const robotNotifier = new HttpRobotNotifier(robotRepository);
  const robotAssignmentService = new RobotAssignmentService(
    robotAssignmentRepository,
    robotRepository,
    robotNotifier,
  );
  const userService = new UserService(userRepository, robotAssignmentService);
  const robotService = new RobotService(robotRepository, robotAssignmentRepository, requestRepository);
  const requestService = new RobotAccessRequestService(
    requestRepository,
    robotRepository,
    robotAssignmentService,
  );
  const emailOtpSender = new NodemailerEmailOtpSender();
  const authService = new AuthService(userRepository, otpRepository, emailOtpSender);
  const adminService = new AdminService(requestRepository, userRepository, robotAssignmentService);
  const robotAuthService = new RobotAuthService(
    robotAssignmentRepository,
    userRepository,
    otpRepository,
    emailOtpSender,
  );

  // --- Controllers ---
  const userController = new UserController(userService);
  const robotController = new RobotController(robotService);
  const requestController = new RobotAccessRequestController(requestService);
  const authController = new AuthController(authService);
  const adminController = new AdminController(adminService, requestService);
  const robotAuthController = new RobotAuthController(robotAuthService, robotService);

  // --- WebSocket infra (robot-link + client control relay) ---
  const robotConnectionRegistry = new RobotConnectionRegistry(robotRepository);
  const telemetryRelayService = new TelemetryRelayService(robotConnectionRegistry, robotRepository);
  const robotLinkGateway = new RobotLinkGateway(robotRepository, robotConnectionRegistry, telemetryRelayService);
  const telemetryRelayGateway = new TelemetryRelayGateway(robotRepository, robotAssignmentService, telemetryRelayService);
  const controlSessionService = new ControlSessionService(robotConnectionRegistry, robotAssignmentService);
  const controlSessionGateway = new ControlSessionGateway(
    robotRepository,
    robotAssignmentService,
    controlSessionService,
  );
  const robotHealthService = new RobotHealthService(robotRepository, robotConnectionRegistry);
  const robotHealthController = new RobotHealthController(robotHealthService, robotAssignmentService);

  const robotApiClient = new HttpRobotApiClient();
  const robotProxyService = new RobotProxyService(robotRepository, robotApiClient);
  const robotProxyController = new RobotProxyController(robotProxyService, robotAssignmentService);

  const wsRouter = new WsRouter();
  wsRouter.register("/ws/robot-link", robotLinkGateway.onConnection, robotLinkGateway.authenticate);
  wsRouter.register("/ws/control", controlSessionGateway.onConnection, controlSessionGateway.authenticate);
  wsRouter.register("/ws/telemetry", telemetryRelayGateway.onConnection, telemetryRelayGateway.authenticate);

  app.get("/health", (_req, res) => res.status(200).json({ status: "ok" }));
  app.use("/api/docs", swaggerUi.serve, swaggerUi.setup(swaggerSpec));

  app.use("/api/auth", createAuthRouter(authController));
  app.use("/api/users", createUserRouter(userController));
  app.use("/api/robots", createPublicRobotRouter(robotController));
  app.use("/api/robots", createRobotHealthRouter(robotHealthController));
  app.use("/api/robots", createRobotProxyRouter(robotProxyController));
  app.use("/api/robot-access-requests", createRobotAccessRequestRouter(requestController));
  app.use("/api/admin/robots", createRobotRouter(robotController));
  app.use("/api/admin", createAdminRouter(adminController));
  app.use("/api/robot-auth", createRobotAuthRouter(robotAuthController, robotRepository));

  app.use(notFoundHandler);
  app.use(errorHandler);

  return { app, wsRouter };
}
