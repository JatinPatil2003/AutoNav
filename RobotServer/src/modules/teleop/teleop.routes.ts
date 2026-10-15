import { Router } from "express";
import { validate } from "../../common/middleware/validate.middleware";
import { asyncHandler } from "../../common/utils/asyncHandler.util";
import { TeleopController } from "./teleop.controller";
import { emergencySchema, velocitySchema } from "./teleop.validation";

/** Mounted at /api/teleop — manual drive and emergency stop. */
export function createTeleopRouter(teleopController: TeleopController): Router {
  const router = Router();

  /**
   * @openapi
   * /api/teleop/velocity:
   *   post:
   *     summary: Set the robot's commanded linear/angular velocity
   *     description: Republished on `cmd_vel` at 20Hz until changed. Call with `{linear:0,angular:0}` to stop.
   *     tags: [Teleop]
   *     requestBody:
   *       required: true
   *       content:
   *         application/json:
   *           schema:
   *             type: object
   *             properties:
   *               linear: { type: number, description: "m/s", example: 0.3 }
   *               angular: { type: number, description: "rad/s", example: 0 }
   *             required: [linear, angular]
   *     responses:
   *       200:
   *         description: Velocity accepted
   *         content:
   *           application/json:
   *             schema:
   *               type: object
   *               properties:
   *                 status: { type: string, example: set }
   *       400: { $ref: '#/components/responses/ValidationError' }
   */
  router.post("/velocity", validate({ body: velocitySchema }), asyncHandler(teleopController.setVelocity));

  /**
   * @openapi
   * /api/teleop/emergency:
   *   post:
   *     summary: Trigger or clear the emergency stop
   *     description: Publishes to `/motor/emergency`. `autonav_firmware`'s AutonavInterface brakes both wheels and refuses velocity commands while `true`.
   *     tags: [Teleop]
   *     requestBody:
   *       required: true
   *       content:
   *         application/json:
   *           schema:
   *             type: object
   *             properties:
   *               status: { type: boolean, example: true }
   *             required: [status]
   *     responses:
   *       200:
   *         description: Emergency status applied
   *         content:
   *           application/json:
   *             schema:
   *               type: object
   *               properties:
   *                 status: { type: boolean, example: true }
   *       400: { $ref: '#/components/responses/ValidationError' }
   */
  router.post("/emergency", validate({ body: emergencySchema }), asyncHandler(teleopController.setEmergency));

  return router;
}
