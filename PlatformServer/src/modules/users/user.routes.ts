import { Router } from "express";
import { requireAuth } from "../../common/middleware/auth.middleware";
import { asyncHandler } from "../../common/utils/asyncHandler.util";
import { UserController } from "./user.controller";

export function createUserRouter(userController: UserController): Router {
  const router = Router();

  /**
   * @openapi
   * /api/users/me:
   *   get:
   *     summary: Get the current authenticated user's profile
   *     tags: [Users]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       200:
   *         description: The current user, including assigned robot (if any)
   *       401:
   *         description: Not authenticated
   */
  router.get("/me", requireAuth, asyncHandler(userController.getMe));

  return router;
}
