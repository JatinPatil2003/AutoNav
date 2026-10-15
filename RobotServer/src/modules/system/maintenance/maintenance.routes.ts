import { Router } from "express";
import { NotImplementedError } from "../../../common/errors";
import { asyncHandler } from "../../../common/utils/asyncHandler.util";

/**
 * Mounted at /api/system/maintenance — reboot, NTP sync, DB reset,
 * credentials. Stubbed (501) in this pass — see network.routes.ts for
 * why. Note the old WebServer's /reboot and /credentials were already
 * non-functional stubs that answered `success: true` for nothing; this
 * at least says so honestly instead of lying about it.
 */
export function createMaintenanceRouter(): Router {
  const router = Router();

  const notImplemented = asyncHandler(async () => {
    throw new NotImplementedError("Maintenance actions are not implemented yet");
  });

  /**
   * @openapi
   * /api/system/maintenance/reboot:
   *   post:
   *     summary: Reboot the robot's onboard computer
   *     tags: [System/Maintenance]
   *     responses:
   *       501: { $ref: '#/components/responses/NotImplemented' }
   */
  router.post("/reboot", notImplemented);

  /**
   * @openapi
   * /api/system/maintenance/sync-datetime:
   *   post:
   *     summary: Force an NTP time sync
   *     tags: [System/Maintenance]
   *     responses:
   *       501: { $ref: '#/components/responses/NotImplemented' }
   */
  router.post("/sync-datetime", notImplemented);

  /**
   * @openapi
   * /api/system/maintenance/db/reset:
   *   post:
   *     summary: Reset the local database (maps, saved goals)
   *     tags: [System/Maintenance]
   *     responses:
   *       501: { $ref: '#/components/responses/NotImplemented' }
   */
  router.post("/db/reset", notImplemented);

  /**
   * @openapi
   * /api/system/maintenance/credentials:
   *   post:
   *     summary: Update robot credentials
   *     tags: [System/Maintenance]
   *     responses:
   *       501: { $ref: '#/components/responses/NotImplemented' }
   */
  router.post("/credentials", notImplemented);

  return router;
}
