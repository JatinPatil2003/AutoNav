import { Router } from "express";
import { validate } from "../../../common/middleware/validate.middleware";
import { asyncHandler } from "../../../common/utils/asyncHandler.util";
import { NetworkController } from "./network.controller";
import { vpnControlSchema, wifiConnectSchema } from "./network.validation";

/** Mounted at /api/system/network — WiFi + VPN control via nmcli/systemctl. */
export function createNetworkRouter(networkController: NetworkController): Router {
  const router = Router();

  /**
   * @openapi
   * /api/system/network/wifi:
   *   get:
   *     summary: List visible WiFi networks
   *     tags: [System/Network]
   *     responses:
   *       200:
   *         description: Visible networks, strongest signal not guaranteed to be first
   */
  router.get("/wifi", asyncHandler(networkController.listWifiNetworks));

  /**
   * @openapi
   * /api/system/network/wifi/current:
   *   get:
   *     summary: Get the currently connected WiFi network, if any
   *     tags: [System/Network]
   *     responses:
   *       200:
   *         description: Current connection state
   */
  router.get("/wifi/current", asyncHandler(networkController.getCurrentWifi));

  /**
   * @openapi
   * /api/system/network/wifi/connect:
   *   post:
   *     summary: Connect to a WiFi network
   *     tags: [System/Network]
   *     requestBody:
   *       required: true
   *       content:
   *         application/json:
   *           schema:
   *             type: object
   *             properties:
   *               ssid: { type: string }
   *               password: { type: string, description: "Omit for an open network" }
   *             required: [ssid]
   *     responses:
   *       200:
   *         description: Connected
   *       400: { $ref: '#/components/responses/ValidationError' }
   *       502:
   *         description: nmcli rejected the connection (wrong password, network out of range, etc.)
   */
  router.post(
    "/wifi/connect",
    validate({ body: wifiConnectSchema }),
    asyncHandler(networkController.connectWifi),
  );

  /**
   * @openapi
   * /api/system/network/vpn:
   *   post:
   *     summary: Start or stop the OpenVPN connection
   *     tags: [System/Network]
   *     requestBody:
   *       required: true
   *       content:
   *         application/json:
   *           schema:
   *             type: object
   *             properties:
   *               action: { type: string, enum: [connect, disconnect] }
   *             required: [action]
   *     responses:
   *       200:
   *         description: VPN state changed
   *       400: { $ref: '#/components/responses/ValidationError' }
   *       502:
   *         description: systemctl rejected the request
   */
  router.post("/vpn", validate({ body: vpnControlSchema }), asyncHandler(networkController.controlVpn));

  return router;
}
