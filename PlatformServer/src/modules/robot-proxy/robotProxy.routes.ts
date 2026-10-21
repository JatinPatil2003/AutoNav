import { Router } from "express";
import { requireAuth } from "../../common/middleware/auth.middleware";
import { validate } from "../../common/middleware/validate.middleware";
import { asyncHandler } from "../../common/utils/asyncHandler.util";
import { RobotProxyController } from "./robotProxy.controller";
import { goalSchema, mapNameSchema, startGoalSchema, startMappingSchema, waypointSchema } from "./robotProxy.validation";

/**
 * Mounted at /api/robots — request/response navigation & mapping commands,
 * proxied straight through to the robot's own RobotServer REST API over
 * the VPN (see RobotProxyService). The robot-link WebSocket relay
 * (telemetry-relay module) is reserved for streaming data (pose/map/
 * nav-status); this covers everything that's naturally request/response.
 * Every route requires the same active platformAccess assignment /ws/control
 * and /api/robots/:robotId/health already require.
 */
export function createRobotProxyRouter(controller: RobotProxyController): Router {
  const router = Router();

  router.use(requireAuth);

  /**
   * @openapi
   * /api/robots/{robotId}/mapping/start:
   *   post:
   *     summary: Launch SLAM on the robot (defaults to Cartographer)
   *     tags: [Robot Proxy]
   *     security: [{ cookieAuth: [] }]
   *     parameters:
   *       - in: path
   *         name: robotId
   *         required: true
   *         schema: { type: string }
   *     requestBody:
   *       content:
   *         application/json:
   *           schema:
   *             type: object
   *             properties:
   *               backend: { type: string, enum: [cartographer, slam_toolbox] }
   *     responses:
   *       200: { description: SLAM launch started }
   *       403: { description: No active platformAccess assignment for this robot }
   *       502: { description: Robot did not respond }
   */
  router.post("/:robotId/mapping/start", validate({ body: startMappingSchema }), asyncHandler(controller.startMapping));

  /**
   * @openapi
   * /api/robots/{robotId}/mapping/stop:
   *   post:
   *     summary: Stop the robot's SLAM launch
   *     tags: [Robot Proxy]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       200: { description: SLAM launch stopped }
   */
  router.post("/:robotId/mapping/stop", asyncHandler(controller.stopMapping));

  /**
   * @openapi
   * /api/robots/{robotId}/mapping/waypoints:
   *   post:
   *     summary: Capture the robot's current live pose as a named waypoint
   *     tags: [Robot Proxy]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       201: { description: Waypoint captured }
   */
  router.post("/:robotId/mapping/waypoints", validate({ body: waypointSchema }), asyncHandler(controller.captureWaypoint));

  /**
   * @openapi
   * /api/robots/{robotId}/mapping/waypoints/commit:
   *   post:
   *     summary: Move all temp waypoints onto a named map
   *     tags: [Robot Proxy]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       200: { description: Waypoints committed }
   */
  router.post(
    "/:robotId/mapping/waypoints/commit",
    validate({ body: mapNameSchema }),
    asyncHandler(controller.commitWaypoints),
  );

  /**
   * @openapi
   * /api/robots/{robotId}/mapping/save:
   *   post:
   *     summary: Save the robot's in-progress SLAM map to disk
   *     tags: [Robot Proxy]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       200: { description: Map saved }
   */
  router.post("/:robotId/mapping/save", validate({ body: mapNameSchema }), asyncHandler(controller.saveMap));

  /**
   * @openapi
   * /api/robots/{robotId}/navigation/maps:
   *   get:
   *     summary: List this robot's saved maps
   *     tags: [Robot Proxy]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       200: { description: Saved map names }
   */
  router.get("/:robotId/navigation/maps", asyncHandler(controller.listMaps));

  /**
   * @openapi
   * /api/robots/{robotId}/navigation/maps/{mapName}/select:
   *   post:
   *     summary: Load a saved map for navigation
   *     tags: [Robot Proxy]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       200: { description: Map selected }
   */
  router.post("/:robotId/navigation/maps/:mapName/select", asyncHandler(controller.selectMap));

  /**
   * @openapi
   * /api/robots/{robotId}/navigation/maps/{mapName}/goals:
   *   get:
   *     summary: List this robot's saved goals for a map, categorized
   *     tags: [Robot Proxy]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       200: { description: Categorized goal names }
   */
  router.get("/:robotId/navigation/maps/:mapName/goals", asyncHandler(controller.listGoals));

  /**
   * @openapi
   * /api/robots/{robotId}/navigation/goals:
   *   post:
   *     summary: Save a named goal pose on a map
   *     tags: [Robot Proxy]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       201: { description: Goal saved }
   */
  router.post("/:robotId/navigation/goals", validate({ body: goalSchema }), asyncHandler(controller.saveGoal));

  /**
   * @openapi
   * /api/robots/{robotId}/navigation/goal/start:
   *   post:
   *     summary: Send the robot to a named, saved goal
   *     tags: [Robot Proxy]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       200: { description: Goal started }
   */
  router.post("/:robotId/navigation/goal/start", validate({ body: startGoalSchema }), asyncHandler(controller.startGoal));

  /**
   * @openapi
   * /api/robots/{robotId}/navigation/goal/cancel:
   *   post:
   *     summary: Cancel the robot's active navigation goal
   *     tags: [Robot Proxy]
   *     security: [{ cookieAuth: [] }]
   *     responses:
   *       200: { description: Goal canceled }
   */
  router.post("/:robotId/navigation/goal/cancel", asyncHandler(controller.cancelGoal));

  return router;
}
