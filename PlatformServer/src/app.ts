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

/**
 * Composition root: repositories, services, and controllers are wired here
 * via constructor injection so every layer depends on an interface, not a
 * concrete persistence implementation — swapping MongooseUserRepository for
 * an in-memory fake (as the unit tests do) requires no change elsewhere.
 */
export function createApp(): Express {
  const app = express();

  app.use(helmet());
  app.use(express.json());
  app.use(cookieParser());

  if (config.isDevelopment) {
    app.use(cors({ origin: config.frontendUrl, credentials: true }));
  }

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
  const robotAuthController = new RobotAuthController(robotAuthService);

  app.get("/health", (_req, res) => res.status(200).json({ status: "ok" }));
  app.use("/api/docs", swaggerUi.serve, swaggerUi.setup(swaggerSpec));

  app.use("/api/auth", createAuthRouter(authController));
  app.use("/api/users", createUserRouter(userController));
  app.use("/api/robots", createPublicRobotRouter(robotController));
  app.use("/api/robot-access-requests", createRobotAccessRequestRouter(requestController));
  app.use("/api/admin/robots", createRobotRouter(robotController));
  app.use("/api/admin", createAdminRouter(adminController));
  app.use("/api/robot-auth", createRobotAuthRouter(robotAuthController, robotRepository));

  app.use(notFoundHandler);
  app.use(errorHandler);

  return app;
}
