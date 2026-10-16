import { Router } from "express";
import { requireAuth } from "../../common/middleware/auth.middleware";
import { requireAdmin } from "../../common/middleware/requireAdmin.middleware";
import { validate } from "../../common/middleware/validate.middleware";
import { asyncHandler } from "../../common/utils/asyncHandler.util";
import { RobotController } from "./robot.controller";
import { createRobotSchema } from "./robot.validation";

/**
 * Mounted at /api/admin/robots — robot provisioning is an admin-only
 * operation in this milestone (no self-registration from the robot side
 * yet). A robot identifies itself to PlatformServer with its own device
 * id (robotId), not a secret — the VPN is the trust boundary (see root
 * README and the robot-auth module).
 */
export function createRobotRouter(robotController: RobotController): Router {
  const router = Router();

  router.use(requireAuth, requireAdmin);

  /**
   * @openapi
   * /api/admin/robots:
   *   get:
   *     summary: List robots by status (admin only)
   *     tags: [Admin, Robots]
   *     security: [{ cookieAuth: [] }]
   *     parameters:
   *       - in: query
   *         name: status
   *         schema: { type: string, enum: [provisioned, active, retired] }
   *     responses:
   *       200:
   *         description: Robots matching the requested status
   */
  router.get("/", asyncHandler(robotController.listRobots));

  /**
   * @openapi
   * /api/admin/robots:
   *   post:
   *     summary: Provision a new robot record (admin only)
   *     tags: [Admin, Robots]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       201:
   *         description: >
   *           Robot created. Also generates and returns the plaintext
   *           robot-link secret this robot authenticates with on
   *           `/ws/robot-link` (`Authorization: Robot <secret>`) — shown
   *           exactly once, only the sha256 hash is persisted.
   */
  router.post("/", validate({ body: createRobotSchema }), asyncHandler(robotController.createRobot));

  /**
   * @openapi
   * /api/admin/robots/{id}/rotate-link-secret:
   *   post:
   *     summary: Rotate a robot's robot-link secret (admin only)
   *     tags: [Admin, Robots]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       200:
   *         description: >
   *           New plaintext robot-link secret, shown exactly once. The
   *           robot's existing /ws/robot-link connection (if any) keeps
   *           working until it next reconnects and is rejected with the
   *           old secret — update the robot's env and restart it promptly.
   *       404:
   *         description: Robot not found
   */
  router.post("/:id/rotate-link-secret", asyncHandler(robotController.rotateLinkSecret));

  /**
   * @openapi
   * /api/admin/robots/{id}:
   *   delete:
   *     summary: Permanently delete a robot, its assignments, and its access requests (admin only)
   *     tags: [Admin, Robots]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       204:
   *         description: Robot deleted
   *       404:
   *         description: Robot not found
   */
  router.delete("/:id", asyncHandler(robotController.deleteRobot));

  return router;
}

/**
 * Mounted at /api/robots — a read-only, non-admin listing (requireAuth only)
 * used by the request-access robot picker. Never exposes vpnHostname/status.
 */
export function createPublicRobotRouter(robotController: RobotController): Router {
  const router = Router();

  router.use(requireAuth);

  /**
   * @openapi
   * /api/robots:
   *   get:
   *     summary: List non-retired robots (minimal fields, any authenticated user)
   *     tags: [Robots]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       200:
   *         description: Robots available to request access to
   */
  router.get("/", asyncHandler(robotController.listRobotSummaries));

  return router;
}
