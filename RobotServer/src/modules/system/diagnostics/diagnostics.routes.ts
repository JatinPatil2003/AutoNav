import { Router } from "express";
import { NotImplementedError } from "../../../common/errors";
import { asyncHandler } from "../../../common/utils/asyncHandler.util";

/**
 * Mounted at /api/system/diagnostics — per-sensor topic-hz health checks
 * and firmware/image version info. Stubbed (501) in this pass — see
 * network.routes.ts for why.
 */
export function createDiagnosticsRouter(): Router {
  const router = Router();

  const notImplemented = asyncHandler(async () => {
    throw new NotImplementedError("Diagnostics are not implemented yet");
  });

  /**
   * @openapi
   * /api/system/diagnostics/sensors/{sensorName}:
   *   get:
   *     summary: Get health (e.g. topic hz) for a named sensor
   *     tags: [System/Diagnostics]
   *     parameters:
   *       - in: path
   *         name: sensorName
   *         required: true
   *         schema: { type: string }
   *         example: lidar
   *     responses:
   *       501: { $ref: '#/components/responses/NotImplemented' }
   */
  router.get("/sensors/:sensorName", notImplemented);

  /**
   * @openapi
   * /api/system/diagnostics/version:
   *   get:
   *     summary: Get firmware/image version info
   *     tags: [System/Diagnostics]
   *     responses:
   *       501: { $ref: '#/components/responses/NotImplemented' }
   */
  router.get("/version", notImplemented);

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
