import { Router } from "express";
import { createResolveRobot } from "../../common/middleware/resolveRobot.middleware";
import { validate } from "../../common/middleware/validate.middleware";
import { robotAuthRateLimiter, robotOtpRateLimiter } from "../../common/middleware/rateLimiter.middleware";
import { asyncHandler } from "../../common/utils/asyncHandler.util";
import { IRobotRepository } from "../robots/robot.repository.interface";
import { RobotAuthController } from "./robotAuth.controller";
import {
  checkUserExistsSchema,
  confirmPinSetupSchema,
  requestPinSetupOtpSchema,
  verifyPinSetupOtpSchema,
} from "./robotAuth.validation";

/**
 * Mounted at /api/robot-auth — the robot-facing service-to-service API,
 * publicly reachable (see AWS/nginx/platform-server.autonav.in.conf).
 * Every route here requires the calling robot's `x-robot-id` header plus
 * its ROBOT_LINK_SECRET as `Authorization: Robot <secret>` (see
 * resolveRobot.middleware.ts) — the same credential /ws/robot-link uses.
 */
export function createRobotAuthRouter(
  controller: RobotAuthController,
  robotRepository: IRobotRepository,
): Router {
  const router = Router();

  router.use(robotAuthRateLimiter, createResolveRobot(robotRepository));

  /**
   * @openapi
   * /api/robot-auth/check-user:
   *   post:
   *     summary: Check whether a platform account exists for this email (not scoped to this robot)
   *     tags: [Robot Auth]
   *     security: [{ robotHeaderAuth: [] }]
   *     responses:
   *       200:
   *         description: Whether an account exists for that email
   */
  router.post(
    "/check-user",
    validate({ body: checkUserExistsSchema }),
    asyncHandler(controller.checkUserExists),
  );

  /**
   * @openapi
   * /api/robot-auth/roster:
   *   get:
   *     summary: Fetch the roster of users with active access to the calling robot
   *     tags: [Robot Auth]
   *     security: [{ robotHeaderAuth: [] }]
   *     responses:
   *       200:
   *         description: Active assignments for this robot, including pinHash for local caching
   */
  router.get("/roster", asyncHandler(controller.getRoster));

  /**
   * @openapi
   * /api/robot-auth/pin-setup/request-otp:
   *   post:
   *     summary: Relay a PIN-setup OTP email request to the user, scoped to the calling robot
   *     tags: [Robot Auth]
   *     security: [{ robotHeaderAuth: [] }]
   *     responses:
   *       200:
   *         description: Always 200 (anti-enumeration) — sends only if an active assignment exists
   */
  router.post(
    "/pin-setup/request-otp",
    robotOtpRateLimiter,
    validate({ body: requestPinSetupOtpSchema }),
    asyncHandler(controller.requestPinSetupOtp),
  );

  /**
   * @openapi
   * /api/robot-auth/pin-setup/verify-otp:
   *   post:
   *     summary: Verify a PIN-setup OTP, returning a short-lived pinSetupToken
   *     tags: [Robot Auth]
   *     security: [{ robotHeaderAuth: [] }]
   *     responses:
   *       200:
   *         description: pinSetupToken issued
   *       401:
   *         description: Invalid or expired code
   */
  router.post(
    "/pin-setup/verify-otp",
    robotOtpRateLimiter,
    validate({ body: verifyPinSetupOtpSchema }),
    asyncHandler(controller.verifyPinSetupOtp),
  );

  /**
   * @openapi
   * /api/robot-auth/pin-setup/confirm:
   *   post:
   *     summary: Confirm a new 4-digit PIN using a verified pinSetupToken
   *     tags: [Robot Auth]
   *     security: [{ robotHeaderAuth: [] }]
   *     responses:
   *       200:
   *         description: PIN stored; returns the updated roster entry for local cache update
   */
  router.post(
    "/pin-setup/confirm",
    validate({ body: confirmPinSetupSchema }),
    asyncHandler(controller.confirmPinSetup),
  );

  /**
   * @openapi
   * /api/robot-auth/rotate-link-secret:
   *   post:
   *     summary: Rotate the calling robot's own ROBOT_LINK_SECRET, proven by presenting the current one
   *     description: Distinct from the admin-only rotate-link-secret route — a robot never has a human admin session. The response's linkSecret is shown once; RobotServer must start using it immediately and persist it so a restart doesn't regress to the old value.
   *     tags: [Robot Auth]
   *     security: [{ robotHeaderAuth: [] }]
   *     responses:
   *       200:
   *         description: New plaintext linkSecret
   */
  router.post("/rotate-link-secret", asyncHandler(controller.rotateLinkSecret));

  return router;
}
