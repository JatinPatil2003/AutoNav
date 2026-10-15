import { Router } from "express";
import { validate } from "../../common/middleware/validate.middleware";
import { asyncHandler } from "../../common/utils/asyncHandler.util";
import { NavigationController } from "./navigation.controller";
import { goalLookupSchema, goalSchema, mapNameSchema, poseSchema } from "./navigation.validation";

/**
 * Mounted at /api/navigation. Consolidates what the old WebServer spread
 * across `/navigation/*` and a parallel `*_linux` set of near-duplicate
 * routes (list/maps vs list/maps_linux, use_map vs use_map_linux, etc.)
 * into one contract — see RobotServer/README.md for the migration notes
 * clients need when they move off the old API.
 */
export function createNavigationRouter(navigationController: NavigationController): Router {
  const router = Router();

  /**
   * @openapi
   * /api/navigation/start:
   *   post:
   *     summary: Launch Nav2 against a saved map
   *     tags: [Navigation]
   *     requestBody:
   *       required: true
   *       content:
   *         application/json:
   *           schema:
   *             type: object
   *             properties:
   *               mapName: { type: string, example: cafe }
   *             required: [mapName]
   *     responses:
   *       200:
   *         description: Navigation launch started
   *         content:
   *           application/json:
   *             schema: { type: object, properties: { status: { type: string, example: started } } }
   *       400: { $ref: '#/components/responses/ValidationError' }
   *       404: { $ref: '#/components/responses/NotFound' }
   */
  router.post("/start", validate({ body: mapNameSchema }), asyncHandler(navigationController.start));

  /**
   * @openapi
   * /api/navigation/stop:
   *   post:
   *     summary: Stop the navigation launch
   *     tags: [Navigation]
   *     responses:
   *       200:
   *         description: Navigation launch stopped
   *         content:
   *           application/json:
   *             schema: { type: object, properties: { status: { type: string, example: stopped } } }
   */
  router.post("/stop", asyncHandler(navigationController.stop));

  /**
   * @openapi
   * /api/navigation/status:
   *   get:
   *     summary: Get the current navigation goal status
   *     tags: [Navigation]
   *     responses:
   *       200:
   *         description: Current status
   *         content:
   *           application/json:
   *             schema:
   *               type: object
   *               properties:
   *                 status: { type: string, enum: [IDLE, ACTIVE, SUCCEEDED, CANCELED, FAILED], example: ACTIVE }
   */
  router.get("/status", asyncHandler(navigationController.getStatus));

  /**
   * @openapi
   * /api/navigation/maps:
   *   get:
   *     summary: List saved maps
   *     tags: [Navigation]
   *     responses:
   *       200:
   *         description: Map names
   *         content:
   *           application/json:
   *             schema:
   *               type: object
   *               properties:
   *                 maps: { type: array, items: { type: string }, example: [cafe, warehouse] }
   */
  router.get("/maps", asyncHandler(navigationController.listMaps));

  /**
   * @openapi
   * /api/navigation/maps/{mapName}/select:
   *   post:
   *     summary: Reload Nav2's map_server with a different saved map
   *     tags: [Navigation]
   *     parameters:
   *       - in: path
   *         name: mapName
   *         required: true
   *         schema: { type: string }
   *         example: cafe
   *     responses:
   *       200:
   *         description: Map reloaded
   *         content:
   *           application/json:
   *             schema: { type: object, properties: { status: { type: string, example: selected } } }
   *       404: { $ref: '#/components/responses/NotFound' }
   *       503: { $ref: '#/components/responses/RosUnavailable' }
   */
  router.post("/maps/:mapName/select", asyncHandler(navigationController.selectMap));

  /**
   * @openapi
   * /api/navigation/maps/{mapName}/goals:
   *   get:
   *     summary: List saved goals on a map, categorized by name convention
   *     description: "Buckets by substring match on the goal name: 'localization' -> localization, 'charger' -> charging, 'standby' -> standby, else -> goals."
   *     tags: [Navigation]
   *     parameters:
   *       - in: path
   *         name: mapName
   *         required: true
   *         schema: { type: string }
   *         example: cafe
   *     responses:
   *       200:
   *         description: Categorized goal names
   *         content:
   *           application/json:
   *             schema:
   *               type: object
   *               properties:
   *                 localization: { type: array, items: { type: string } }
   *                 charging: { type: array, items: { type: string } }
   *                 standby: { type: array, items: { type: string } }
   *                 goals: { type: array, items: { type: string } }
   *               example:
   *                 localization: [localization_1]
   *                 charging: [charger_1]
   *                 standby: [standby_1]
   *                 goals: [goal_1, goal_2]
   */
  router.get("/maps/:mapName/goals", asyncHandler(navigationController.listGoals));

  /**
   * @openapi
   * /api/navigation/goals:
   *   post:
   *     summary: Save a named goal pose on a map
   *     tags: [Navigation]
   *     requestBody:
   *       required: true
   *       content:
   *         application/json:
   *           schema:
   *             allOf:
   *               - $ref: '#/components/schemas/Pose'
   *               - type: object
   *                 properties:
   *                   mapName: { type: string, example: cafe }
   *                   name: { type: string, example: goal_1 }
   *                 required: [mapName, name]
   *     responses:
   *       201:
   *         description: Goal saved
   *         content:
   *           application/json:
   *             schema: { type: object, properties: { status: { type: string, example: saved } } }
   *       400: { $ref: '#/components/responses/ValidationError' }
   */
  router.post("/goals", validate({ body: goalSchema }), asyncHandler(navigationController.saveGoal));

  /**
   * @openapi
   * /api/navigation/goals/lookup:
   *   post:
   *     summary: Look up a saved goal's pose by name
   *     tags: [Navigation]
   *     requestBody:
   *       required: true
   *       content:
   *         application/json:
   *           schema:
   *             type: object
   *             properties:
   *               mapName: { type: string, example: cafe }
   *               name: { type: string, example: goal_1 }
   *             required: [mapName, name]
   *     responses:
   *       200:
   *         description: The goal's pose
   *         content:
   *           application/json:
   *             schema: { $ref: '#/components/schemas/Pose' }
   *       400: { $ref: '#/components/responses/ValidationError' }
   *       404: { $ref: '#/components/responses/NotFound' }
   */
  router.post("/goals/lookup", validate({ body: goalLookupSchema }), asyncHandler(navigationController.getGoal));

  /**
   * @openapi
   * /api/navigation/goals/localize:
   *   post:
   *     summary: Drive to a localization goal, set the initial pose there, and spin in place to help AMCL converge
   *     description: Blocks for roughly two in-place rotations (~25s at the current angular speed) before responding.
   *     tags: [Navigation]
   *     requestBody:
   *       required: true
   *       content:
   *         application/json:
   *           schema:
   *             type: object
   *             properties:
   *               mapName: { type: string, example: cafe }
   *               name: { type: string, example: localization_1 }
   *             required: [mapName, name]
   *     responses:
   *       200:
   *         description: Localization attempt result (success reflects AMCL confidence, not an HTTP-level failure)
   *         content:
   *           application/json:
   *             schema:
   *               type: object
   *               properties:
   *                 success: { type: boolean, example: true }
   *                 message: { type: string, example: "Localization to localization_1 successful" }
   *       400: { $ref: '#/components/responses/ValidationError' }
   *       404: { $ref: '#/components/responses/NotFound' }
   */
  router.post("/goals/localize", validate({ body: goalLookupSchema }), asyncHandler(navigationController.localize));

  /**
   * @openapi
   * /api/navigation/goal/start:
   *   post:
   *     summary: Send the robot to a saved goal via Nav2's NavigateToPose action
   *     tags: [Navigation]
   *     requestBody:
   *       required: true
   *       content:
   *         application/json:
   *           schema:
   *             type: object
   *             properties:
   *               mapName: { type: string, example: cafe }
   *               name: { type: string, example: goal_1 }
   *             required: [mapName, name]
   *     responses:
   *       200:
   *         description: Goal sent, navigation now active
   *         content:
   *           application/json:
   *             schema: { type: object, properties: { status: { type: string, example: navigating } } }
   *       400: { $ref: '#/components/responses/ValidationError' }
   *       404: { $ref: '#/components/responses/NotFound' }
   */
  router.post("/goal/start", validate({ body: goalLookupSchema }), asyncHandler(navigationController.startGoal));

  /**
   * @openapi
   * /api/navigation/goal/cancel:
   *   post:
   *     summary: Cancel the active navigation goal
   *     tags: [Navigation]
   *     responses:
   *       200:
   *         description: Goal cancelled (a no-op if nothing was active)
   *         content:
   *           application/json:
   *             schema: { type: object, properties: { status: { type: string, example: cancelled } } }
   */
  router.post("/goal/cancel", asyncHandler(navigationController.cancelGoal));

  /**
   * @openapi
   * /api/navigation/goal/feedback:
   *   get:
   *     summary: Get the active goal's remaining distance
   *     tags: [Navigation]
   *     responses:
   *       200:
   *         description: Distance remaining in meters (0 if no goal is active)
   *         content:
   *           application/json:
   *             schema: { type: object, properties: { feedback: { type: number, example: 2.4 } } }
   */
  router.get("/goal/feedback", asyncHandler(navigationController.getFeedback));

  /**
   * @openapi
   * /api/navigation/initial-pose:
   *   post:
   *     summary: Set AMCL's initial pose estimate directly
   *     tags: [Navigation]
   *     requestBody:
   *       required: true
   *       content:
   *         application/json:
   *           schema: { $ref: '#/components/schemas/Pose' }
   *     responses:
   *       200:
   *         description: Initial pose set
   *         content:
   *           application/json:
   *             schema: { type: object, properties: { status: { type: string, example: set } } }
   *       400: { $ref: '#/components/responses/ValidationError' }
   */
  router.post("/initial-pose", validate({ body: poseSchema }), asyncHandler(navigationController.setInitialPose));

  /**
   * @openapi
   * /api/navigation/current/map:
   *   get:
   *     summary: Get the latest occupancy grid received on /map
   *     tags: [Navigation]
   *     responses:
   *       200:
   *         description: The current map, or null if none received yet
   *         content:
   *           application/json:
   *             schema:
   *               type: object
   *               nullable: true
   *               properties:
   *                 info:
   *                   type: object
   *                   properties:
   *                     width: { type: integer, example: 384 }
   *                     height: { type: integer, example: 384 }
   *                     resolution: { type: number, example: 0.05 }
   *                     origin:
   *                       type: object
   *                       properties:
   *                         position: { type: object, properties: { x: { type: number }, y: { type: number }, z: { type: number } } }
   *                         orientation: { type: object, properties: { x: { type: number }, y: { type: number }, z: { type: number }, w: { type: number } } }
   *                 data: { type: array, items: { type: integer }, description: "Row-major occupancy values, -1..100" }
   */
  router.get("/current/map", asyncHandler(navigationController.getCurrentMap));

  /**
   * @openapi
   * /api/navigation/current/location:
   *   get:
   *     summary: Get the latest AMCL pose received on /amcl_pose
   *     tags: [Navigation]
   *     responses:
   *       200:
   *         description: The current location, or null if no AMCL pose received yet
   *         content:
   *           application/json:
   *             schema:
   *               oneOf:
   *                 - $ref: '#/components/schemas/Pose'
   *                 - type: object
   *                   nullable: true
   */
  router.get("/current/location", asyncHandler(navigationController.getCurrentLocation));

  return router;
}
