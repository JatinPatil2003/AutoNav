import { Router } from "express";
import { asyncHandler } from "../../../common/utils/asyncHandler.util";
import { MaintenanceController } from "./maintenance.controller";

/**
 * Mounted at /api/system/maintenance — reboot and credential rotation.
 * The old stub also listed /sync-datetime and /db/reset; both were dropped
 * entirely (not just left stubbed) rather than implemented — NTP sync and
 * a destructive local DB wipe weren't judged worth the risk/complexity for
 * this pass. If either is wanted later, it's a fresh addition, not a
 * revival of dead code.
 */
export function createMaintenanceRouter(controller: MaintenanceController): Router {
  const router = Router();

  /**
   * @openapi
   * /api/system/maintenance/reboot:
   *   post:
   *     summary: Request a reboot of the robot's onboard computer
   *     description: RobotServer itself has no host-reboot privilege — this writes a trigger file a host-level systemd unit (deploy/maintenance/autonav-reboot.path) watches and acts on. 202 means the request was recorded, not that the reboot has happened yet.
   *     tags: [System/Maintenance]
   *     responses:
   *       202:
   *         description: Reboot requested
   *         content:
   *           application/json:
   *             schema: { type: object, properties: { status: { type: string, example: reboot_requested } } }
   */
  router.post("/reboot", asyncHandler(controller.reboot));

  /**
   * @openapi
   * /api/system/maintenance/credentials:
   *   post:
   *     summary: Rotate this robot's ROBOT_LINK_SECRET
   *     description: Asks PlatformServer for a new secret (proving identity with the current one), applies it immediately, persists it so a restart doesn't regress to the old value, and forces the robot-link connection to reconnect with it right away.
   *     tags: [System/Maintenance]
   *     responses:
   *       200:
   *         description: New plaintext linkSecret (shown once)
   *         content:
   *           application/json:
   *             schema: { type: object, properties: { linkSecret: { type: string } } }
   */
  router.post("/credentials", asyncHandler(controller.rotateCredentials));

  return router;
}
