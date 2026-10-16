import { Router } from "express";
import { requireAuth } from "../../common/middleware/auth.middleware";
import { asyncHandler } from "../../common/utils/asyncHandler.util";
import { RobotHealthController } from "./robotHealth.controller";

/**
 * Mounted at /api/robots — alongside the existing public robot-summary
 * router. Any authenticated user may call this, but the controller itself
 * enforces the real gate: an active platformAccess assignment for the
 * specific :robotId requested (same check /ws/control's handshake makes).
 */
export function createRobotHealthRouter(controller: RobotHealthController): Router {
  const router = Router();

  router.use(requireAuth);

  /**
   * @openapi
   * /api/robots/{robotId}/health:
   *   get:
   *     summary: One-shot health check for a robot (link reachability + ROS liveness)
   *     tags: [Robots]
   *     security: [{ cookieAuth: [] }]
   *     parameters:
   *       - in: path
   *         name: robotId
   *         required: true
   *         schema: { type: string }
   *         description: Robot's Mongo id (the same id used for /ws/control's robotId query param)
   *     responses:
   *       200:
   *         description: Health snapshot
   *       403:
   *         description: No active platformAccess assignment for this robot
   *       404:
   *         description: Robot not found
   */
  router.get("/:robotId/health", asyncHandler(controller.getHealth));

  return router;
}
