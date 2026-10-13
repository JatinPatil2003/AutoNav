import { Router } from "express";
import { requireAuth } from "../../common/middleware/auth.middleware";
import { asyncHandler } from "../../common/utils/asyncHandler.util";
import { RobotAccessRequestController } from "./robotAccessRequest.controller";

export function createRobotAccessRequestRouter(
  controller: RobotAccessRequestController,
): Router {
  const router = Router();

  router.use(requireAuth);

  /**
   * @openapi
   * /api/robot-access-requests:
   *   post:
   *     summary: Request robot access (creates a pending request for an admin to review)
   *     tags: [Robot Access Requests]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       201:
   *         description: Request created
   *       409:
   *         description: Already assigned a robot, or already has a pending request
   */
  router.post("/", asyncHandler(controller.createRequest));

  /**
   * @openapi
   * /api/robot-access-requests/me:
   *   get:
   *     summary: Get the current user's latest robot access request
   *     tags: [Robot Access Requests]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       200:
   *         description: The latest request, or null if none exists
   */
  router.get("/me", asyncHandler(controller.getMyLatestRequest));

  return router;
}
