import { Router } from "express";
import { NotImplementedError } from "../../../common/errors";
import { asyncHandler } from "../../../common/utils/asyncHandler.util";
import { DiagnosticsController } from "./diagnostics.controller";

/**
 * Mounted at /api/system/diagnostics — per-sensor topic-hz health,
 * ESP32/launch/identity status, software version, and battery (currently
 * always a labeled placeholder — see DiagnosticsService.getBatteryStatus).
 * /update stays a 501 stub: triggering an OTA image update is a
 * substantially bigger feature than diagnostics reporting and wasn't in
 * scope for this pass.
 */
export function createDiagnosticsRouter(controller: DiagnosticsController): Router {
  const router = Router();

  const notImplemented = asyncHandler(async () => {
    throw new NotImplementedError("Triggering an update is not implemented yet");
  });

  /**
   * @openapi
   * /api/system/diagnostics/sensors/{sensorName}:
   *   get:
   *     summary: Get topic-hz health for a named sensor
   *     tags: [System/Diagnostics]
   *     parameters:
   *       - in: path
   *         name: sensorName
   *         required: true
   *         schema: { type: string, enum: [lidar, imu, odom] }
   *         example: lidar
   *     responses:
   *       200:
   *         description: Rolling-average Hz and time since the last message, or null for either if no message has arrived yet
   *       404: { $ref: '#/components/responses/NotFound' }
   */
  router.get("/sensors/:sensorName", asyncHandler(controller.getSensorHealth));

  /**
   * @openapi
   * /api/system/diagnostics/status:
   *   get:
   *     summary: Device identity, ROS liveness, ESP32 link, and active launch/SLAM-backend status
   *     tags: [System/Diagnostics]
   *     responses:
   *       200:
   *         description: System status snapshot
   */
  router.get("/status", asyncHandler(controller.getStatus));

  /**
   * @openapi
   * /api/system/diagnostics/version:
   *   get:
   *     summary: Get RobotServer's own software version (the ESP32 reports no firmware version — see response note)
   *     tags: [System/Diagnostics]
   *     responses:
   *       200:
   *         description: Version info
   */
  router.get("/version", asyncHandler(controller.getVersion));

  /**
   * @openapi
   * /api/system/diagnostics/battery:
   *   get:
   *     summary: Battery status — currently always a labeled placeholder (isDefault=true) until the board has a real voltage-sense ADC
   *     tags: [System/Diagnostics]
   *     responses:
   *       200:
   *         description: Battery status
   *       503: { $ref: '#/components/responses/RosUnavailable' }
   */
  router.get("/battery", asyncHandler(controller.getBatteryStatus));

  /**
   * @openapi
   * /api/system/diagnostics/update:
   *   post:
   *     summary: Trigger a firmware/image update
   *     tags: [System/Diagnostics]
   *     responses:
   *       501: { $ref: '#/components/responses/NotImplemented' }
   */
  router.post("/update", notImplemented);

  return router;
}
