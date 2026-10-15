import { Router } from "express";
import { requireAuth } from "../../common/middleware/auth.middleware";
import { requireAdmin } from "../../common/middleware/requireAdmin.middleware";
import { validate } from "../../common/middleware/validate.middleware";
import { asyncHandler } from "../../common/utils/asyncHandler.util";
import { AdminController } from "./admin.controller";
import {
  grantRobotAssignmentSchema,
  revokeRobotAssignmentSchema,
  updateRobotAssignmentAccessSchema,
  updateUserRoleSchema,
} from "./admin.validation";
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
   * /api/admin/robot-assignments:
   *   post:
   *     summary: Directly grant a user access to a robot (admin only), bypassing the request workflow
   *     tags: [Admin]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       201:
   *         description: Assignment created or reactivated
   *       409:
   *         description: User already has active access to this robot
   */
  router.post(
    "/robot-assignments",
    validate({ body: grantRobotAssignmentSchema }),
    asyncHandler(adminController.grantRobotAssignment),
  );

  /**
   * @openapi
   * /api/admin/robot-assignments/{id}/revoke:
   *   post:
   *     summary: Revoke a user's access to a robot (admin only)
   *     tags: [Admin]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       200:
   *         description: Assignment revoked
   */
  router.post(
    "/robot-assignments/:id/revoke",
    validate({ body: revokeRobotAssignmentSchema }),
    asyncHandler(adminController.revokeRobotAssignment),
  );

  /**
   * @openapi
   * /api/admin/robot-assignments/{id}/access:
   *   patch:
   *     summary: Toggle a user's HMI (RobotApp) and/or PlatformApp access on an existing assignment (admin only)
   *     tags: [Admin]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       200:
   *         description: Access flags updated
   */
  router.patch(
    "/robot-assignments/:id/access",
    validate({ body: updateRobotAssignmentAccessSchema }),
    asyncHandler(adminController.updateRobotAssignmentAccess),
  );

  /**
   * @openapi
   * /api/admin/robots/{robotId}/assignments:
   *   get:
   *     summary: List all assignments (any status) for a robot (admin only)
   *     tags: [Admin]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       200:
   *         description: Assignments for the robot
   */
  router.get("/robots/:robotId/assignments", asyncHandler(adminController.listAssignmentsForRobot));

  /**
   * @openapi
   * /api/admin/users/{userId}/assignments:
   *   get:
   *     summary: List all assignments (any status) for a user (admin only)
   *     tags: [Admin]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       200:
   *         description: Assignments for the user
   */
  router.get("/users/:userId/assignments", asyncHandler(adminController.listAssignmentsForUser));

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

  /**
   * @openapi
   * /api/admin/users/{id}:
   *   delete:
   *     summary: Permanently delete a user, their robot assignments, and their access requests (admin only; cannot target self)
   *     tags: [Admin]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       204:
   *         description: User deleted
   *       403:
   *         description: Attempted to delete your own account
   *       404:
   *         description: User not found
   */
  router.delete("/users/:id", asyncHandler(adminController.deleteUser));

  return router;
}
