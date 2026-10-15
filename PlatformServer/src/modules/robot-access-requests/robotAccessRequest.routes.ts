import { Router } from "express";
import { requireAuth } from "../../common/middleware/auth.middleware";
import { validate } from "../../common/middleware/validate.middleware";
import { asyncHandler } from "../../common/utils/asyncHandler.util";
import { RobotAccessRequestController } from "./robotAccessRequest.controller";
import { createRequestSchema } from "./robotAccessRequest.validation";

export function createRobotAccessRequestRouter(
  controller: RobotAccessRequestController,
): Router {
  const router = Router();

  router.use(requireAuth);

  /**
   * @openapi
   * /api/robot-access-requests:
   *   post:
   *     summary: Request access to a specific robot (creates a pending request for an admin to review)
   *     tags: [Robot Access Requests]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       201:
   *         description: Request created
   *       404:
   *         description: Robot not found
   *       409:
   *         description: Already has access to this robot, or already has a pending request for it
   */
  router.post("/", validate({ body: createRequestSchema }), asyncHandler(controller.createRequest));

  /**
   * @openapi
   * /api/robot-access-requests/mine:
   *   get:
   *     summary: List the current user's robot access requests (all statuses, newest first)
   *     tags: [Robot Access Requests]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       200:
   *         description: The user's requests, newest first
   */
  router.get("/mine", asyncHandler(controller.listMine));

  return router;
}
