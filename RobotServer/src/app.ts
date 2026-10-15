import cookieParser from "cookie-parser";
import cors from "cors";
import express, { Express } from "express";
import helmet from "helmet";
import path from "node:path";
import pinoHttp from "pino-http";
import swaggerUi from "swagger-ui-express";
import * as rclnodejs from "rclnodejs";

import { config } from "./config/env";
import { swaggerSpec } from "./config/swagger";
import { logger } from "./common/utils/logger.util";
import { errorHandler } from "./common/middleware/errorHandler.middleware";
import { notFoundHandler } from "./common/middleware/notFound.middleware";
import { WsRouter } from "./core/wsRouter";

import { LaunchManager } from "./ros/launchManager";
import { RosTopicGateway } from "./ros/rosTopicGateway";
import { RosServiceGateway } from "./ros/rosServiceGateway";
import { RosActionGateway } from "./ros/rosActionGateway";

import { MongooseGoalRepository } from "./modules/goals/goal.repository";
import { GridFsMapRepository } from "./modules/maps/map.repository";

import { TeleopService } from "./modules/teleop/teleop.service";
import { TeleopController } from "./modules/teleop/teleop.controller";
import { createTeleopRouter } from "./modules/teleop/teleop.routes";
import { registerTeleopWebSocket } from "./modules/teleop/teleop.wsServer";

import { NavigationService } from "./modules/navigation/navigation.service";
import { NavigationController } from "./modules/navigation/navigation.controller";
import { createNavigationRouter } from "./modules/navigation/navigation.routes";

import { MappingService } from "./modules/mapping/mapping.service";
import { MappingController } from "./modules/mapping/mapping.controller";
import { createMappingRouter } from "./modules/mapping/mapping.routes";

import { registerTelemetryWebSocket } from "./modules/telemetry/telemetry.wsServer";
import { registerTelemetryBroadcast } from "./modules/telemetry/telemetry.broadcaster";

import { FfmpegVideoSource } from "./modules/video/ffmpegVideoSource";
import { registerVideoWebSocket } from "./modules/video/video.wsServer";

import { FirmwareService } from "./modules/firmware/firmware.service";
import { FirmwareController } from "./modules/firmware/firmware.controller";
import { createFirmwareRouter } from "./modules/firmware/firmware.routes";

import { createNetworkRouter } from "./modules/system/network/network.routes";
import { createDiagnosticsRouter } from "./modules/system/diagnostics/diagnostics.routes";
import { createMaintenanceRouter } from "./modules/system/maintenance/maintenance.routes";

import { requireAuth } from "./common/middleware/auth.middleware";
import { PlatformClient } from "./clients/platformClient";
import { MongooseLocalUserRepository } from "./modules/robot-auth/localUser.repository";
import { RosterSyncService } from "./modules/robot-auth/rosterSync.service";
import { RobotAuthService } from "./modules/robot-auth/robotAuth.service";
import { RobotAuthController } from "./modules/robot-auth/robotAuth.controller";
import { createHmiAuthRouter, createPlatformAuthRouter } from "./modules/robot-auth/robotAuth.routes";

export interface RobotApp {
  app: Express;
  registerWebSockets: (wsRouter: WsRouter) => void;
  rosterSyncService: RosterSyncService;
}

/**
 * Composition root — mirrors PlatformServer/src/app.ts: every layer is
 * constructed here and wired by constructor injection, so a unit test can
 * swap any gateway/repository for a fake without touching the layer that
 * uses it (see tests/unit).
 */
export function createApp(rosNode: rclnodejs.Node): RobotApp {
  const app = express();

  app.use(helmet());
  app.use(express.json());
  app.use(cookieParser());

  if (config.corsOrigins.length > 0) {
    app.use(cors({ origin: config.corsOrigins }));
  }

  app.use(
    pinoHttp({
      logger,
      autoLogging: { ignore: (req) => req.url === "/health" },
      customSuccessMessage: (req, res) => `${req.method} ${req.url} -> ${res.statusCode}`,
      serializers: {
        req: (req) => ({ method: req.method, url: req.url }),
        res: (res) => ({ statusCode: res.statusCode }),
      },
    }),
  );

  // --- ROS gateways (generic primitives, shared by every feature) ---
  const topicGateway = new RosTopicGateway(rosNode);
  const serviceGateway = new RosServiceGateway(rosNode);
  const actionGateway = new RosActionGateway(rosNode);
  const launchManager = new LaunchManager();

  // --- Repositories ---
  const goalRepository = new MongooseGoalRepository();
  const mapRepository = new GridFsMapRepository();

  // --- Services ---
  const teleopService = new TeleopService(topicGateway);
  const navigationService = new NavigationService(
    actionGateway,
    serviceGateway,
    topicGateway,
    teleopService,
    goalRepository,
    mapRepository,
    launchManager,
    config.mapsDir,
  );
  const mappingService = new MappingService(topicGateway, goalRepository, mapRepository, launchManager, config.mapsDir);
  const firmwareService = new FirmwareService(serviceGateway);

  registerTelemetryBroadcast(topicGateway, navigationService);

  // --- Robot-auth (human HMI session + inter-service auth with PlatformServer) ---
  const platformClient = new PlatformClient();
  const localUserRepository = new MongooseLocalUserRepository();
  const rosterSyncService = new RosterSyncService(platformClient, localUserRepository);
  const robotAuthService = new RobotAuthService(localUserRepository, platformClient);

  // --- Controllers ---
  const teleopController = new TeleopController(teleopService);
  const navigationController = new NavigationController(navigationService);
  const mappingController = new MappingController(mappingService);
  const firmwareController = new FirmwareController(firmwareService);
  const robotAuthController = new RobotAuthController(robotAuthService);

  app.get("/health", (_req, res) => res.status(200).json({ status: "ok" }));
  app.use("/maps", requireAuth, express.static(config.mapsDir));
  app.use("/api/docs", swaggerUi.serve, swaggerUi.setup(swaggerSpec));

  app.use("/api/hmi-auth", createHmiAuthRouter(robotAuthController));
  app.use("/api/platform-auth", createPlatformAuthRouter(rosterSyncService));

  // Every operational endpoint below requires a logged-in RobotApp session.
  app.use("/api/teleop", requireAuth, createTeleopRouter(teleopController));
  app.use("/api/navigation", requireAuth, createNavigationRouter(navigationController));
  app.use("/api/mapping", requireAuth, createMappingRouter(mappingController));
  app.use("/api/firmware", requireAuth, createFirmwareRouter(firmwareController));
  app.use("/api/system/network", requireAuth, createNetworkRouter());
  app.use("/api/system/diagnostics", requireAuth, createDiagnosticsRouter());
  app.use("/api/system/maintenance", requireAuth, createMaintenanceRouter());

  // RobotApp (the local HMI PWA) is built separately and copied into the
  // image at ROBOTAPP_STATIC_DIR — served same-origin so its cookies need
  // no CORS/SameSite gymnastics. Mounted after every /api and /maps route
  // so those always win; only unmatched paths fall through to the SPA.
  app.use(express.static(config.robotAppStaticDir));
  app.get(/^(?!\/api|\/maps|\/health).*/, (_req, res) => {
    res.sendFile(path.join(config.robotAppStaticDir, "index.html"), (err) => {
      if (err) res.status(404).json({ error: "NotFound", message: "RobotApp build not found" });
    });
  });

  app.use(notFoundHandler);
  app.use(errorHandler);

  const registerWebSockets = (wsRouter: WsRouter): void => {
    registerTelemetryWebSocket(wsRouter);
    registerTeleopWebSocket(wsRouter, teleopService);
    registerVideoWebSocket(wsRouter, new FfmpegVideoSource(config.cameraDevice));
  };

  return { app, registerWebSockets, rosterSyncService };
}
