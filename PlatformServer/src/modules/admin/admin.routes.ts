import { Router } from "express";
import { requireAuth } from "../../common/middleware/auth.middleware";
import { requireAdmin } from "../../common/middleware/requireAdmin.middleware";
import { validate } from "../../common/middleware/validate.middleware";
import { asyncHandler } from "../../common/utils/asyncHandler.util";
import { AdminController } from "./admin.controller";
import { updateUserRoleSchema } from "./admin.validation";
import {
  approveRequestSchema,
  rejectRequestSchema,
} from "../robot-access-requests/robotAccessRequest.validation";

export function createAdminRouter(adminController: AdminController): Router {
  const router = Router();

  router.use(requireAuth, requireAdmin);

  /**
   * @openapi
   * /api/admin/robot-access-requests:
   *   get:
   *     summary: List robot access requests by status (admin only)
   *     tags: [Admin]
   *     security: [{ cookieAuth: [] }]
   *     parameters:
   *       - in: query
   *         name: status
   *         schema: { type: string, enum: [pending, approved, rejected] }
   *     responses:
   *       200:
   *         description: Matching requests, each including requester email/name
   */
  router.get("/robot-access-requests", asyncHandler(adminController.listRequests));

  /**
   * @openapi
   * /api/admin/robot-access-requests/{id}/approve:
   *   post:
   *     summary: Approve a pending robot access request and assign a robot
   *     tags: [Admin]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       200:
   *         description: Request approved, robot assigned
   *       409:
   *         description: Request not pending, or robot already assigned
   */
  router.post(
    "/robot-access-requests/:id/approve",
    validate({ body: approveRequestSchema }),
    asyncHandler(adminController.approveRequest),
  );

  /**
   * @openapi
   * /api/admin/robot-access-requests/{id}/reject:
   *   post:
   *     summary: Reject a pending robot access request
   *     tags: [Admin]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       200:
   *         description: Request rejected
   */
  router.post(
    "/robot-access-requests/:id/reject",
    validate({ body: rejectRequestSchema }),
    asyncHandler(adminController.rejectRequest),
  );

  /**
   * @openapi
   * /api/admin/users:
   *   get:
   *     summary: List all platform users (admin only)
   *     tags: [Admin]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       200:
   *         description: All users, including role and assigned robot
   */
  router.get("/users", asyncHandler(adminController.listUsers));

  /**
   * @openapi
   * /api/admin/users/{id}/role:
   *   patch:
   *     summary: Promote or demote a user's role (admin only; cannot target self)
   *     tags: [Admin]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       200:
   *         description: Role updated
   *       403:
   *         description: Attempted to change your own role
   *       404:
   *         description: User not found
   */
  router.patch(
    "/users/:id/role",
    validate({ body: updateUserRoleSchema }),
    asyncHandler(adminController.updateUserRole),
  );

  return router;
}
