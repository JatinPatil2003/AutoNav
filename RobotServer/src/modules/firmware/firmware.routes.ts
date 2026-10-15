import { Router } from "express";
import { validate } from "../../common/middleware/validate.middleware";
import { asyncHandler } from "../../common/utils/asyncHandler.util";
import { FirmwareController } from "./firmware.controller";
import { brightnessSchema, ledPatternSchema } from "./firmware.validation";

/** Mounted at /api/firmware — the ESP32 peripheral board (LED patterns, brightness) via esp32_bridge_node's ROS2 services. */
export function createFirmwareRouter(firmwareController: FirmwareController): Router {
  const router = Router();

  /**
   * @openapi
   * /api/firmware/led/pattern:
   *   post:
   *     summary: Set the ESP32's active LED pattern
   *     description: >
   *       Pattern ids (see autonav_firmware/firmware/Esp-32-Autonav-4.0): 0=off,
   *       1=fade+wait (standby), 2=breathe (idle), 3=comet (active), 4=blink (alert).
   *       Omit hue/sat/val/speedMs entirely to use that pattern's hardcoded default
   *       color/speed on the ESP32 (see DefaultParamsForPattern in led_pattern.h);
   *       to override them, all four must be given together -- a partial set is
   *       rejected since the wire protocol only supports "defaults" (1-byte payload)
   *       or "fully custom" (6-byte payload), not a mix.
   *     tags: [Firmware]
   *     requestBody:
   *       required: true
   *       content:
   *         application/json:
   *           schema:
   *             type: object
   *             properties:
   *               patternId: { type: integer, minimum: 0, maximum: 255, example: 2 }
   *               hue: { type: integer, minimum: 0, maximum: 255, example: 86, description: "Optional; must be given together with sat, val, and speedMs" }
   *               sat: { type: integer, minimum: 0, maximum: 255, example: 255 }
   *               val: { type: integer, minimum: 0, maximum: 255, example: 255 }
   *               speedMs: { type: integer, minimum: 0, maximum: 65535, description: "Pattern-dependent tick/step interval", example: 50 }
   *             required: [patternId]
   *             example: { patternId: 2 }
   *     responses:
   *       200:
   *         description: ESP32 acknowledged the pattern change
   *         content:
   *           application/json:
   *             schema:
   *               type: object
   *               properties:
   *                 success: { type: boolean, example: true }
   *                 message: { type: string, example: OK }
   *       400: { $ref: '#/components/responses/ValidationError' }
   *       502:
   *         description: ESP32 rejected or did not acknowledge the command (board disconnected, CRC/length error, etc.)
   *         content:
   *           application/json:
   *             schema:
   *               type: object
   *               properties:
   *                 success: { type: boolean, example: false }
   *                 message: { type: string, example: "No ACK from ESP32 (not connected?)" }
   */
  router.post("/led/pattern", validate({ body: ledPatternSchema }), asyncHandler(firmwareController.setLedPattern));

  /**
   * @openapi
   * /api/firmware/led/brightness:
   *   post:
   *     summary: Set the ESP32 LED strip's global brightness
   *     tags: [Firmware]
   *     requestBody:
   *       required: true
   *       content:
   *         application/json:
   *           schema:
   *             type: object
   *             properties:
   *               brightness: { type: integer, minimum: 0, maximum: 255, example: 180 }
   *             required: [brightness]
   *     responses:
   *       200:
   *         description: ESP32 acknowledged the brightness change
   *         content:
   *           application/json:
   *             schema:
   *               type: object
   *               properties:
   *                 success: { type: boolean, example: true }
   *                 message: { type: string, example: OK }
   *       400: { $ref: '#/components/responses/ValidationError' }
   *       502:
   *         description: ESP32 rejected or did not acknowledge the command
   *         content:
   *           application/json:
   *             schema:
   *               type: object
   *               properties:
   *                 success: { type: boolean, example: false }
   *                 message: { type: string, example: "No ACK from ESP32 (not connected?)" }
   */
  router.post("/led/brightness", validate({ body: brightnessSchema }), asyncHandler(firmwareController.setBrightness));

  return router;
}
