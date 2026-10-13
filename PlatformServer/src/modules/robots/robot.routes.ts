import { Router } from "express";
import { requireAuth } from "../../common/middleware/auth.middleware";
import { requireAdmin } from "../../common/middleware/requireAdmin.middleware";
import { validate } from "../../common/middleware/validate.middleware";
import { asyncHandler } from "../../common/utils/asyncHandler.util";
import { RobotController } from "./robot.controller";
import { createRobotSchema } from "./robot.validation";

/**
 * Mounted at /api/admin/robots — robot provisioning is an admin-only
 * operation in this milestone (no self-registration from the robot side yet).
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
   *         schema: { type: string, enum: [unassigned, assigned, retired] }
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
   *         description: Robot created
   */
  router.post("/", validate({ body: createRobotSchema }), asyncHandler(robotController.createRobot));

  return router;
}
