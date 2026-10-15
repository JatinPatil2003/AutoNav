import { Router } from "express";
import { createRequireRobotAuth } from "../../common/middleware/requireRobotAuth.middleware";
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
 * Mounted at /api/robot-auth — the robot-facing service-to-service API.
 * Every route here requires the robot's provisioned `x-robot-id`/
 * `x-robot-secret` headers (see requireRobotAuth.middleware.ts) — there's
 * no separate token-issuance step to call first.
 */
export function createRobotAuthRouter(
  controller: RobotAuthController,
  robotRepository: IRobotRepository,
): Router {
  const router = Router();

  router.use(robotAuthRateLimiter, createRequireRobotAuth(robotRepository));

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

  return router;
}
