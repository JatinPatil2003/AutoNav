import { Router } from "express";
import { NotImplementedError } from "../../../common/errors";
import { asyncHandler } from "../../../common/utils/asyncHandler.util";

/**
 * Mounted at /api/system/network — WiFi + VPN control. Deliberately
 * stubbed (501) in this pass: the old WebServer's `nmcli`/`systemctl`
 * shell-outs are being redesigned, not ported as-is — see
 * RobotServer/README.md. The route shape is fixed now so clients have a
 * stable contract to build against ahead of the real implementation.
 */
export function createNetworkRouter(): Router {
  const router = Router();

  const notImplemented = asyncHandler(async () => {
    throw new NotImplementedError("Network management is not implemented yet");
  });

  /**
   * @openapi
   * /api/system/network/wifi:
   *   get:
   *     summary: List visible WiFi networks
   *     tags: [System/Network]
   *     responses:
   *       501: { $ref: '#/components/responses/NotImplemented' }
   */
  router.get("/wifi", notImplemented);

  /**
   * @openapi
   * /api/system/network/wifi/current:
   *   get:
   *     summary: Get the currently connected WiFi network
   *     tags: [System/Network]
   *     responses:
   *       501: { $ref: '#/components/responses/NotImplemented' }
   */
  router.get("/wifi/current", notImplemented);

  /**
   * @openapi
   * /api/system/network/wifi/connect:
   *   post:
   *     summary: Connect to a WiFi network
   *     tags: [System/Network]
   *     responses:
   *       501: { $ref: '#/components/responses/NotImplemented' }
   */
  router.post("/wifi/connect", notImplemented);

  /**
   * @openapi
   * /api/system/network/vpn:
   *   post:
   *     summary: Control the OpenVPN connection
   *     tags: [System/Network]
   *     responses:
   *       501: { $ref: '#/components/responses/NotImplemented' }
   */
  router.post("/vpn", notImplemented);

  return router;
}
