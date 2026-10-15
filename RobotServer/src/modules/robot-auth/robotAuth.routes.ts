import { Router } from "express";
import { requireAuth } from "../../common/middleware/auth.middleware";
import { validate } from "../../common/middleware/validate.middleware";
import { asyncHandler } from "../../common/utils/asyncHandler.util";
import { RobotAuthController } from "./robotAuth.controller";
import { RosterSyncService } from "./rosterSync.service";
import {
  confirmPinSetupSchema,
  identifySchema,
  loginSchema,
  requestPinSetupOtpSchema,
  verifyPinSetupOtpSchema,
} from "./robotAuth.validation";

/**
 * Mounted at /api/hmi-auth — the human-facing PIN login + pin-setup wizard
 * consumed by RobotApp (served same-origin by this process, so cookies
 * just work with no CORS complication). Kept entirely separate from
 * /api/platform-auth below so the two trust domains (human session vs.
 * inter-service) can never be confused by a handler checking the wrong
 * thing.
 */
export function createHmiAuthRouter(controller: RobotAuthController): Router {
  const router = Router();

  router.get("/users", asyncHandler(controller.listUsers));
  router.post("/identify", validate({ body: identifySchema }), asyncHandler(controller.identify));
  router.post("/login", validate({ body: loginSchema }), asyncHandler(controller.login));
  router.post("/refresh", asyncHandler(controller.refresh));
  router.post("/logout", asyncHandler(controller.logout));
  router.post(
    "/pin-setup/request-otp",
    validate({ body: requestPinSetupOtpSchema }),
    asyncHandler(controller.requestPinSetupOtp),
  );
  router.post(
    "/pin-setup/verify-otp",
    validate({ body: verifyPinSetupOtpSchema }),
    asyncHandler(controller.verifyPinSetupOtp),
  );
  router.post(
    "/pin-setup/confirm",
    validate({ body: confirmPinSetupSchema }),
    asyncHandler(controller.confirmPinSetup),
  );

  router.use(requireAuth);
  router.get("/session", asyncHandler(controller.session));

  return router;
}

/**
 * Mounted at /api/platform-auth — called only by PlatformServer itself,
 * never by RobotApp or any browser. No auth: the VPN is the trust
 * boundary (see root README), same as every other robot<->platform call.
 */
export function createPlatformAuthRouter(rosterSyncService: RosterSyncService): Router {
  const router = Router();

  /**
   * @openapi
   * /api/platform-auth/roster/invalidate:
   *   post:
   *     summary: Re-sync the local roster cache now (pushed by PlatformServer after a grant/revoke)
   *     tags: [Platform Auth]
   *     security: [{ platformBearerAuth: [] }]
   *     responses:
   *       200:
   *         description: Roster re-synced
   */
  router.post(
    "/roster/invalidate",
    asyncHandler(async (_req, res) => {
      await rosterSyncService.sync();
      res.status(200).json({ status: "synced" });
    }),
  );

  return router;
}
